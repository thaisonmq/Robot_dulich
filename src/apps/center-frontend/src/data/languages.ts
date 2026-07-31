export interface LanguageOption {
  code: string;
  /** International ISO language name, displayed as the primary label. */
  label: string;
  /** Name written in the language itself, when available. */
  nativeLabel: string;
}

// ISO 639-1 languages. English is used as the international display standard;
// the endonym is secondary and never replaces the stable international label.
const LANGUAGE_CODES = [
  "aa", "ab", "ae", "af", "ak", "am", "an", "ar", "as", "av", "ay", "az",
  "ba", "be", "bg", "bh", "bi", "bm", "bn", "bo", "br", "bs",
  "ca", "ce", "ch", "co", "cr", "cs", "cu", "cv", "cy",
  "da", "de", "dv", "dz",
  "ee", "el", "en", "eo", "es", "et", "eu",
  "fa", "ff", "fi", "fj", "fo", "fr", "fy",
  "ga", "gd", "gl", "gn", "gu", "gv",
  "ha", "he", "hi", "ho", "hr", "ht", "hu", "hy", "hz",
  "ia", "id", "ie", "ig", "ii", "ik", "io", "is", "it", "iu",
  "ja", "jv",
  "ka", "kg", "ki", "kj", "kk", "kl", "km", "kn", "ko", "kr", "ks", "ku", "kv", "kw", "ky",
  "la", "lb", "lg", "li", "ln", "lo", "lt", "lu", "lv",
  "mg", "mh", "mi", "mk", "ml", "mn", "mr", "ms", "mt", "my",
  "na", "nb", "nd", "ne", "ng", "nl", "nn", "no", "nr", "nv", "ny",
  "oc", "oj", "om", "or", "os",
  "pa", "pi", "pl", "ps", "pt",
  "qu",
  "rm", "rn", "ro", "ru", "rw",
  "sa", "sc", "sd", "se", "sg", "si", "sk", "sl", "sm", "sn", "so", "sq", "sr", "ss", "st", "su", "sv", "sw",
  "ta", "te", "tg", "th", "ti", "tk", "tl", "tn", "to", "tr", "ts", "tt", "tw", "ty",
  "ug", "uk", "ur", "uz",
  "ve", "vi", "vo",
  "wa", "wo",
  "xh",
  "yi", "yo",
  "za", "zh", "zu",
] as const;

const PRIORITY_CODES = ["vi", "en", "zh", "ko", "ja", "th", "fr", "de", "es", "ru"] as const;

function languageName(code: string, locale: string): string {
  try {
    const localizedName = new Intl.DisplayNames([locale], { type: "language" }).of(code);
    if (localizedName && localizedName.toLocaleLowerCase() !== code) {
      return localizedName;
    }
    const englishName = new Intl.DisplayNames(["en"], { type: "language" }).of(code);
    if (englishName && englishName.toLocaleLowerCase() !== code) {
      return englishName;
    }
    return code.toUpperCase();
  } catch {
    return code.toUpperCase();
  }
}

const priority = new Set<string>(PRIORITY_CODES);
const collator = new Intl.Collator("en", { sensitivity: "base" });

const allLanguages = LANGUAGE_CODES.map((code) => ({
  code,
  label: languageName(code, "en"),
  nativeLabel: languageName(code, code),
}));

export const LANGUAGE_OPTIONS: LanguageOption[] = [
  ...PRIORITY_CODES.map((code) => allLanguages.find((language) => language.code === code)!),
  ...allLanguages
    .filter((language) => !priority.has(language.code))
    .sort((left, right) => collator.compare(left.label, right.label)),
];

export function getLanguage(code: string): LanguageOption {
  return LANGUAGE_OPTIONS.find((language) => language.code === code) ?? LANGUAGE_OPTIONS[1];
}

export function normalizeLanguageSearch(value: string): string {
  return value
    .normalize("NFD")
    .replace(/\p{Diacritic}/gu, "")
    .toLocaleLowerCase("vi")
    .trim();
}
