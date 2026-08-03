import { fireEvent, render, screen } from "@testing-library/react";
import { GlobalLanguageSelect } from "../src/components/GlobalLanguageSelect";
import { LanguageSelect } from "../src/components/LanguageSelect";
import { LANGUAGE_OPTIONS } from "../src/data/languages";
import { I18nProvider, useI18n } from "../src/i18n/I18nProvider";

function TranslatedLoginLabel() {
  const { t } = useI18n();
  return <output>{t("Đăng nhập")}</output>;
}

describe("LanguageSelect", () => {
  afterEach(() => localStorage.clear());

  it("only provides languages that have an interface catalogue", () => {
    expect(LANGUAGE_OPTIONS.map((language) => language.code)).toEqual([
      "vi", "en", "zh", "ko", "ja", "th", "fr", "de", "es", "ru",
    ]);
  });

  it("searches international language names and selects a language", () => {
    const onChange = vi.fn();
    render(
      <I18nProvider>
        <LanguageSelect value="en" onChange={onChange} />
      </I18nProvider>,
    );

    fireEvent.click(screen.getByRole("button", { name: /English/ }));
    expect(screen.queryByText(/^Tiếng\s/u)).not.toBeInTheDocument();
    fireEvent.change(screen.getByRole("searchbox", { name: "Search languages" }), {
      target: { value: "japan" },
    });
    fireEvent.click(screen.getByRole("option", { name: /Japanese/ }));

    expect(onChange).toHaveBeenCalledWith("ja");
  });

  it("updates the interface when another supported language is selected", () => {
    render(
      <I18nProvider>
        <GlobalLanguageSelect />
        <TranslatedLoginLabel />
      </I18nProvider>,
    );

    fireEvent.click(screen.getByRole("button", { name: /English/ }));
    fireEvent.change(screen.getByRole("searchbox", { name: "Search languages" }), {
      target: { value: "french" },
    });
    fireEvent.click(screen.getByRole("option", { name: /French/ }));

    expect(screen.getByText("Se connecter")).toBeInTheDocument();
    expect(document.documentElement).toHaveAttribute("lang", "fr");
  });
});
