export interface LanguageOption {
  code: string;
  /** Compact country flag used to identify the language in selectors. */
  flag: string;
  /** International ISO language name, displayed as the primary label. */
  label: string;
  /** Name written in the language itself, when available. */
  nativeLabel: string;
}

// Keep this list aligned with the catalogues registered in translations.ts.
// Offering an ISO language without UI copy makes the selection appear broken.
export const SUPPORTED_LANGUAGE_CODES = [
  "vi", "en", "zh", "ko", "ja", "th", "fr", "de", "es", "ru",
] as const;

const LANGUAGE_FLAGS: Record<(typeof SUPPORTED_LANGUAGE_CODES)[number], string> = {
  vi: "🇻🇳",
  en: "🇬🇧",
  zh: "🇨🇳",
  ko: "🇰🇷",
  ja: "🇯🇵",
  th: "🇹🇭",
  fr: "🇫🇷",
  de: "🇩🇪",
  es: "🇪🇸",
  ru: "🇷🇺",
};

export function getLanguageDisplayName(code: string, locale: string): string {
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

export const LANGUAGE_OPTIONS: LanguageOption[] = SUPPORTED_LANGUAGE_CODES.map((code) => ({
  code,
  flag: LANGUAGE_FLAGS[code],
  label: getLanguageDisplayName(code, "en"),
  nativeLabel: getLanguageDisplayName(code, code),
}));

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
