import {
  createContext, useCallback, useContext, useEffect, useMemo, useRef, useState,
} from "react";
import { getLanguage } from "../data/languages";
import { useAppStore } from "../state/appStore";
import { translate, type TranslationVariables } from "./translations";

const GUEST_LANGUAGE_KEY = "rovera:interface-language:guest";
const ACCOUNT_LANGUAGE_KEY = "rovera:interface-language:";
const LEGACY_ACCOUNT_LANGUAGE_KEY = "rovera:account-language:";
const RTL_LANGUAGES = new Set(["ar", "dv", "fa", "he", "ku", "ps", "sd", "ug", "ur", "yi"]);

interface I18nContextValue {
  language: string;
  setLanguage: (language: string) => void;
  t: (source: string, variables?: TranslationVariables) => string;
}

const I18nContext = createContext<I18nContextValue | null>(null);

function validLanguage(language: string | null): string | null {
  if (!language) return null;
  return getLanguage(language).code === language ? language : null;
}

function browserLanguage(): string {
  const language = navigator.language.split("-")[0].toLocaleLowerCase();
  return validLanguage(language) ?? "en";
}

function accountLanguage(userId: string): string | null {
  return validLanguage(localStorage.getItem(`${ACCOUNT_LANGUAGE_KEY}${userId}`))
    ?? validLanguage(localStorage.getItem(`${LEGACY_ACCOUNT_LANGUAGE_KEY}${userId}`));
}

function initialLanguage(userId?: string): string {
  return (userId ? accountLanguage(userId) : null)
    ?? validLanguage(localStorage.getItem(GUEST_LANGUAGE_KEY))
    ?? browserLanguage();
}

export function I18nProvider({ children }: { children: React.ReactNode }) {
  const userId = useAppStore((state) => state.user?.id);
  const [language, setLanguageState] = useState(() => initialLanguage(userId));
  const previousUserId = useRef(userId);

  useEffect(() => {
    if (previousUserId.current === userId) return;
    previousUserId.current = userId;
    if (!userId) return;
    const savedLanguage = accountLanguage(userId);
    if (savedLanguage) {
      setLanguageState(savedLanguage);
    } else {
      localStorage.setItem(`${ACCOUNT_LANGUAGE_KEY}${userId}`, language);
    }
  }, [language, userId]);

  const setLanguage = useCallback((nextLanguage: string) => {
    const normalizedLanguage = validLanguage(nextLanguage) ?? "en";
    setLanguageState(normalizedLanguage);
    localStorage.setItem(
      userId ? `${ACCOUNT_LANGUAGE_KEY}${userId}` : GUEST_LANGUAGE_KEY,
      normalizedLanguage,
    );
  }, [userId]);

  const t = useCallback(
    (source: string, variables?: TranslationVariables) =>
      translate(language, source, variables),
    [language],
  );

  useEffect(() => {
    document.documentElement.lang = language;
    document.documentElement.dir = RTL_LANGUAGES.has(language) ? "rtl" : "ltr";
    document.title = language === "ja"
      ? "Rovera・ロボット運用センター"
      : language === "vi"
        ? "Rovera · Trung tâm vận hành robot"
        : "Rovera · Robot operations center";
  }, [language]);

  const value = useMemo(() => ({ language, setLanguage, t }), [language, setLanguage, t]);
  return <I18nContext.Provider value={value}>{children}</I18nContext.Provider>;
}

export function useI18n(): I18nContextValue {
  const context = useContext(I18nContext);
  if (!context) throw new Error("useI18n must be used inside I18nProvider");
  return context;
}
