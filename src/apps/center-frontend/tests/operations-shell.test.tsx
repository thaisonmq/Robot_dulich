import { act, fireEvent, render, screen, within } from "@testing-library/react";
import { I18nProvider } from "../src/i18n/I18nProvider";
import { OperationsShell } from "../src/components/OperationsShell";
import { useAppStore } from "../src/state/appStore";
import type { User } from "../src/types";

const SIDEBAR_KEY = "rovera:operations-sidebar-collapsed";

function operator(): User {
  return {
    id: "operations-shell-user",
    username: "operator",
    email: "operator@example.com",
    name: "Nhân viên vận hành",
    full_name: "Nhân viên vận hành",
    role: "operator",
    active: true,
    email_verified: true,
    avatar_url: null,
    must_change_password: false,
    password_enabled: true,
    auth_providers: [],
    permissions: ["maps.view", "maps.manage"],
    created_by_id: null,
    last_login_at: null,
    created_at: new Date().toISOString(),
    updated_at: new Date().toISOString(),
  };
}

function renderShell() {
  return render(
    <I18nProvider>
      <OperationsShell title="Không gian vận hành"><div>Nội dung</div></OperationsShell>
    </I18nProvider>,
  );
}

describe("OperationsShell", () => {
  beforeEach(() => {
    window.history.replaceState(null, "", "/robots");
    const user = operator();
    localStorage.setItem(`rovera:interface-language:${user.id}`, "vi");
    act(() => useAppStore.getState().setUser(user));
  });

  afterEach(() => {
    act(() => useAppStore.getState().setUser(null));
    localStorage.clear();
    sessionStorage.clear();
  });

  it("marks the current navigation item and follows route changes", () => {
    renderShell();
    const navigation = screen.getByRole("navigation", { name: "Điều hướng chính" });
    const robotItem = within(navigation).getByRole("button", { name: "Danh sách robot" });
    const mapItem = within(navigation).getByRole("button", { name: "Bản đồ" });

    expect(robotItem).toHaveAttribute("aria-current", "page");
    expect(mapItem).not.toHaveAttribute("aria-current");

    fireEvent.click(mapItem);

    expect(window.location.pathname).toBe("/maps");
    expect(mapItem).toHaveAttribute("aria-current", "page");
    expect(robotItem).not.toHaveAttribute("aria-current");
  });

  it("keeps map creation inside the map area without a separate sidebar tab", () => {
    window.history.replaceState(null, "", "/maps/create");
    renderShell();
    const navigation = screen.getByRole("navigation", { name: "Điều hướng chính" });

    expect(within(navigation).getByRole("button", { name: "Bản đồ" }))
      .toHaveAttribute("aria-current", "page");
    expect(within(navigation).queryByRole("button", { name: "Tạo bản đồ" }))
      .not.toBeInTheDocument();
  });

  it("collapses accessibly and restores the persisted preference", () => {
    const firstRender = renderShell();
    const toggle = screen.getByRole("button", { name: "Thu gọn menu" });
    const topbar = document.querySelector(".operations-topbar") as HTMLElement;
    const sidebar = document.querySelector(".operations-sidebar") as HTMLElement;

    expect(within(topbar).getByRole("button", { name: "Về danh sách robot" })).toBeInTheDocument();
    expect(within(sidebar).queryByRole("button", { name: "Về danh sách robot" })).not.toBeInTheDocument();

    expect(toggle).toHaveAttribute("aria-expanded", "true");
    expect(toggle).toHaveAttribute("aria-controls", "operations-sidebar-navigation");
    fireEvent.click(toggle);
    expect(localStorage.getItem(SIDEBAR_KEY)).toBe("true");
    expect(screen.getByRole("button", { name: "Mở rộng menu" })).toHaveAttribute("aria-expanded", "false");
    expect(document.querySelector(".operations-shell")).toHaveClass("is-sidebar-collapsed");
    expect(within(topbar).getByRole("button", { name: "Về danh sách robot" })).toBeInTheDocument();

    firstRender.unmount();
    renderShell();

    expect(screen.getByRole("button", { name: "Mở rộng menu" })).toHaveAttribute("aria-expanded", "false");
    expect(document.querySelector(".operations-shell")).toHaveClass("is-sidebar-collapsed");
  });
});
