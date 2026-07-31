import { fireEvent, render, screen } from "@testing-library/react";
import { LanguageSelect } from "../src/components/LanguageSelect";
import { LANGUAGE_OPTIONS } from "../src/data/languages";
import { I18nProvider } from "../src/i18n/I18nProvider";

describe("LanguageSelect", () => {
  it("provides the full ISO 639-1 language list", () => {
    expect(LANGUAGE_OPTIONS).toHaveLength(184);
    expect(LANGUAGE_OPTIONS.slice(0, 3).map((language) => language.code))
      .toEqual(["vi", "en", "zh"]);
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
});
