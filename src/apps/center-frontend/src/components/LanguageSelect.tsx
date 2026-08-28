import { useEffect, useMemo, useRef, useState } from "react";
import { Check, ChevronDown, Search, X } from "lucide-react";
import {
  getLanguage, getLanguageDisplayName, LANGUAGE_OPTIONS, normalizeLanguageSearch,
} from "../data/languages";
import { useI18n } from "../i18n/I18nProvider";

interface LanguageSelectProps {
  value: string;
  onChange: (languageCode: string) => void;
  compact?: boolean;
}

function showNativeLabel(language: { label: string; nativeLabel: string }): boolean {
  return language.nativeLabel !== language.label
    && !/^tiếng\s/iu.test(language.nativeLabel);
}

export function LanguageSelect({ value, onChange, compact = false }: LanguageSelectProps) {
  const { language: interfaceLanguage, t } = useI18n();
  const [open, setOpen] = useState(false);
  const [query, setQuery] = useState("");
  const rootRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);
  const selected = getLanguage(value);
  const selectedLabel = getLanguageDisplayName(selected.code, interfaceLanguage);

  const filteredLanguages = useMemo(() => {
    const normalizedQuery = normalizeLanguageSearch(query);
    if (!normalizedQuery) return LANGUAGE_OPTIONS;
    return LANGUAGE_OPTIONS.filter((language) => normalizeLanguageSearch(
      `${language.label} ${language.nativeLabel} ${getLanguageDisplayName(language.code, interfaceLanguage)} ${language.code}`,
    ).includes(normalizedQuery));
  }, [interfaceLanguage, query]);

  useEffect(() => {
    if (!open) return;
    inputRef.current?.focus();
    const closeOnOutsideClick = (event: PointerEvent) => {
      if (!rootRef.current?.contains(event.target as Node)) {
        setOpen(false);
        setQuery("");
      }
    };
    document.addEventListener("pointerdown", closeOnOutsideClick);
    return () => document.removeEventListener("pointerdown", closeOnOutsideClick);
  }, [open]);

  function selectLanguage(code: string) {
    onChange(code);
    setOpen(false);
    setQuery("");
  }

  return (
    <div className={`language-select ${compact ? "is-compact" : ""}`} ref={rootRef}>
      <button
        type="button"
        className="language-select__trigger"
        aria-haspopup="listbox"
        aria-expanded={open}
        onClick={() => setOpen((current) => !current)}
        onKeyDown={(event) => {
          if (event.key === "ArrowDown") {
            event.preventDefault();
            setOpen(true);
          }
        }}
      >
        <span className="language-select__flag" aria-hidden="true">{selected.flag}</span>
        <span>
          <strong>{selectedLabel}</strong>
          {selected.nativeLabel !== selectedLabel && <small>{selected.nativeLabel}</small>}
        </span>
        <ChevronDown size={16} className={open ? "is-open" : ""} />
      </button>

      {open && (
        <div className="language-select__popover">
          <div className="language-select__search">
            <Search size={15} />
            <input
              ref={inputRef}
              type="search"
              value={query}
              onChange={(event) => setQuery(event.target.value)}
              onKeyDown={(event) => {
                if (event.key === "Escape") {
                  setOpen(false);
                  setQuery("");
                }
                if (event.key === "Enter" && filteredLanguages.length === 1) {
                  selectLanguage(filteredLanguages[0].code);
                }
              }}
              placeholder={t("Tìm ngôn ngữ…")}
              aria-label={t("Tìm ngôn ngữ")}
            />
            {query && (
              <button type="button" onClick={() => setQuery("")} aria-label={t("Xóa tìm kiếm")}>
                <X size={14} />
              </button>
            )}
          </div>
          <div className="language-select__list" role="listbox" aria-label={t("Danh sách ngôn ngữ")}>
            {filteredLanguages.length ? filteredLanguages.map((language) => (
              <button
                type="button"
                role="option"
                aria-selected={language.code === value}
                className={language.code === value ? "is-selected" : ""}
                key={language.code}
                onClick={() => selectLanguage(language.code)}
              >
                <span className="language-select__flag" aria-hidden="true">{language.flag}</span>
                <span>
                  <strong>{getLanguageDisplayName(language.code, interfaceLanguage)}</strong>
                  {language.nativeLabel !== getLanguageDisplayName(language.code, interfaceLanguage)
                    && showNativeLabel(language) && <small>{language.nativeLabel}</small>}
                </span>
                <code>{language.code.toUpperCase()}</code>
                {language.code === value && <Check size={15} />}
              </button>
            )) : (
              <p>{t("Không tìm thấy ngôn ngữ phù hợp.")}</p>
            )}
          </div>
        </div>
      )}
    </div>
  );
}
