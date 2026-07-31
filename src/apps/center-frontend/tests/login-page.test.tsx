import { render, waitFor } from "@testing-library/react";
import { LoginPage } from "../src/pages/LoginPage";
import { I18nProvider } from "../src/i18n/I18nProvider";

vi.mock("../src/components/GoogleAuthButton", () => ({
  GoogleAuthButton: () => <button type="button">Google</button>,
}));

describe("LoginPage", () => {
  afterEach(() => {
    localStorage.clear();
    sessionStorage.clear();
  });

  it("does not remember, prefill, or display the bootstrap admin account", async () => {
    localStorage.setItem("rovera_identifier", "admin");

    const { container } = render(
      <I18nProvider>
        <LoginPage />
      </I18nProvider>,
    );

    expect(container.querySelector<HTMLInputElement>("#identifier")).toHaveValue("");
    expect(container.querySelector<HTMLInputElement>("#password")).toHaveValue("");
    expect(container.querySelector('input[type="checkbox"]')).not.toBeInTheDocument();
    expect(container).not.toHaveTextContent("admin123");

    await waitFor(() => {
      expect(localStorage.getItem("rovera_identifier")).toBeNull();
    });
  });
});
