import { act, fireEvent, render, screen } from "@testing-library/react";
import { AccountMenu } from "../src/components/AccountMenu";
import { I18nProvider } from "../src/i18n/I18nProvider";
import { useAppStore } from "../src/state/appStore";
import type { User } from "../src/types";

function account(role: User["role"]): User {
  return {
    id: `user-${role}`,
    username: role,
    email: `${role}@example.com`,
    name: role === "admin" ? "Quản trị hệ thống" : "Khách tham quan",
    full_name: role === "admin" ? "Quản trị hệ thống" : "Khách tham quan",
    role,
    active: true,
    email_verified: true,
    avatar_url: null,
    must_change_password: false,
    password_enabled: true,
    auth_providers: [],
    permissions: [],
    created_by_id: null,
    last_login_at: null,
    created_at: new Date().toISOString(),
    updated_at: new Date().toISOString(),
  };
}

describe("AccountMenu", () => {
  afterEach(() => {
    act(() => useAppStore.getState().setUser(null));
    sessionStorage.clear();
  });

  it("shows full account administration for admin", () => {
    act(() => useAppStore.getState().setUser(account("admin")));
    render(<I18nProvider><AccountMenu /></I18nProvider>);

    fireEvent.click(screen.getByRole("button", { name: /Quản trị hệ thống/ }));

    expect(screen.getByText("Account management")).toBeInTheDocument();
    expect(screen.getByText("My account")).toBeInTheDocument();
  });

  it("shows guest account management for operator", () => {
    act(() => useAppStore.getState().setUser(account("operator")));
    render(<I18nProvider><AccountMenu /></I18nProvider>);

    fireEvent.click(screen.getByRole("button", { name: /Khách tham quan/ }));

    expect(screen.getByText("Account management")).toBeInTheDocument();
    expect(screen.getByText("Manage guest accounts")).toBeInTheDocument();
  });

  it("keeps guest account menu limited to self-service", () => {
    act(() => useAppStore.getState().setUser(account("guest")));
    render(<I18nProvider><AccountMenu /></I18nProvider>);

    fireEvent.click(screen.getByRole("button", { name: /Khách tham quan/ }));

    expect(screen.queryByText("Account management")).not.toBeInTheDocument();
    expect(screen.getByText("My account")).toBeInTheDocument();
  });
});
