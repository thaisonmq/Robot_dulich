import { Languages } from "lucide-react";
import { useI18n } from "../i18n/I18nProvider";
import { LanguageSelect } from "./LanguageSelect";

export function GlobalLanguageSelect({ onDark = false }: { onDark?: boolean }) {
  const { language, setLanguage, t } = useI18n();
  return (
    <div className={`global-language-select ${onDark ? "on-dark" : ""}`}>
      <Languages size={16} aria-hidden="true" />
      <span>{t("Ngôn ngữ")}</span>
      <LanguageSelect value={language} onChange={setLanguage} compact />
    </div>
  );
}
