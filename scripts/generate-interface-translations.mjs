import { mkdir, readFile, writeFile } from "node:fs/promises";
import path from "node:path";
import process from "node:process";
import { fileURLToPath } from "node:url";

const scriptDirectory = path.dirname(fileURLToPath(import.meta.url));
const projectRoot = path.resolve(scriptDirectory, "..");
const translationsFile = path.join(
  projectRoot,
  "src/apps/center-frontend/src/i18n/translations.ts",
);
const localesDirectory = path.join(
  projectRoot,
  "src/apps/center-frontend/src/i18n/locales",
);
const force = process.argv.includes("--force");
const targetLanguages = process.argv.slice(2).filter((argument) => argument !== "--force");
const languages = targetLanguages.length
  ? targetLanguages
  : ["zh", "ko", "ja", "th", "fr", "de", "es", "ru"];
const supportedLanguages = new Set(["zh", "ko", "ja", "th", "fr", "de", "es", "ru"]);

for (const language of languages) {
  if (!supportedLanguages.has(language)) {
    throw new Error(`Unsupported target language: ${language}`);
  }
}

const source = await readFile(translationsFile, "utf8");
const start = source.indexOf("{", source.indexOf("const ENGLISH"));
const end = source.indexOf("\n};", start) + 2;
if (start < 0 || end < 2) throw new Error("Could not find the English catalogue");

// The catalogue is a data-only object literal maintained in this repository.
const englishCatalogue = Function(`"use strict"; return (${source.slice(start, end)});`)();
const entries = Object.entries(englishCatalogue);

function protectVariables(value) {
  const variables = [];
  const text = value.replace(/\{[^}]+\}/g, (variable) => {
    const index = variables.push(variable) - 1;
    return `⟪${index}⟫`;
  });
  return { text, variables };
}

function restoreVariables(value, variables) {
  return variables.reduce(
    (result, variable, index) => result.replaceAll(`⟪${index}⟫`, variable),
    value,
  );
}

async function translateText(text, targetLanguage, attempt = 1) {
  const url = new URL("https://translate.googleapis.com/translate_a/single");
  url.search = new URLSearchParams({
    client: "gtx",
    sl: "en",
    tl: targetLanguage,
    dt: "t",
    q: text,
  });

  try {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const payload = await response.json();
    return payload[0].map((part) => part[0]).join("");
  } catch (error) {
    if (attempt >= 4) throw error;
    await new Promise((resolve) => setTimeout(resolve, 300 * (2 ** attempt)));
    return translateText(text, targetLanguage, attempt + 1);
  }
}

async function mapWithConcurrency(values, concurrency, mapper) {
  const results = new Array(values.length);
  let nextIndex = 0;
  async function worker() {
    while (nextIndex < values.length) {
      const index = nextIndex;
      nextIndex += 1;
      results[index] = await mapper(values[index], index);
    }
  }
  await Promise.all(Array.from({ length: concurrency }, () => worker()));
  return results;
}

await mkdir(localesDirectory, { recursive: true });
for (const language of languages) {
  let existingCatalogue = {};
  try {
    existingCatalogue = JSON.parse(
      await readFile(path.join(localesDirectory, `${language}.json`), "utf8"),
    );
  } catch {
    // A missing catalogue is created from scratch.
  }
  const pendingEntries = entries.filter(([key]) => force || !existingCatalogue[key]);
  process.stdout.write(`Generating ${language} (${pendingEntries.length}/${entries.length} messages)...\n`);
  const translatedEntries = await mapWithConcurrency(pendingEntries, 12, async ([key, value]) => {
    const { text, variables } = protectVariables(value);
    const translation = restoreVariables(await translateText(text, language), variables);
    return [key, translation];
  });
  const translatedCatalogue = { ...existingCatalogue, ...Object.fromEntries(translatedEntries) };
  const orderedCatalogue = Object.fromEntries(
    entries.map(([key]) => [key, translatedCatalogue[key]]),
  );
  const output = `${JSON.stringify(orderedCatalogue, null, 2)}\n`;
  await writeFile(path.join(localesDirectory, `${language}.json`), output, "utf8");
}
