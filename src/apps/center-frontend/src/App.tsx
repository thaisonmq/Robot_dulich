import { useEffect, useState } from "react";
import { api, AUTH_EXPIRED_EVENT, authStorage, userStorage } from "./api/client";
import { AccountPage } from "./pages/AccountPage";
import { DashboardPage } from "./pages/DashboardPage";
import { GoogleCallbackPage } from "./pages/GoogleCallbackPage";
import { LoginPage } from "./pages/LoginPage";
import { RegisterPage } from "./pages/RegisterPage";
import { RobotConfigurationPage } from "./pages/RobotConfigurationPage";
import { RobotEditorPage } from "./pages/RobotEditorPage";
import { RobotListPage } from "./pages/RobotListPage";
import { UserManagementPage } from "./pages/UserManagementPage";
import { useI18n } from "./i18n/I18nProvider";
import { navigate, usePathname } from "./router";
import { useAppStore } from "./state/appStore";

export function App() {
  const { t } = useI18n();
  const pathname = usePathname();
  const setUser = useAppStore((state) => state.setUser);
  const user = useAppStore((state) => state.user);
  const resetSession = useAppStore((state) => state.resetSession);
  const [checkingSession, setCheckingSession] = useState(Boolean(authStorage.get()));

  useEffect(() => {
    const handleExpiredSession = () => {
      setUser(null);
      resetSession();
      navigate("/", { replace: true });
    };
    window.addEventListener(AUTH_EXPIRED_EVENT, handleExpiredSession);
    return () => window.removeEventListener(AUTH_EXPIRED_EVENT, handleExpiredSession);
  }, [resetSession, setUser]);

  useEffect(() => {
    if (!authStorage.get()) {
      setCheckingSession(false);
      return;
    }
    let cancelled = false;
    void api.me()
      .then((account) => {
        if (cancelled) return;
        setUser(account);
        userStorage.set(account);
      })
      .finally(() => {
        if (!cancelled) setCheckingSession(false);
      });
    return () => {
      cancelled = true;
    };
  }, [setUser]);

  if (pathname === "/") return <LoginPage />;
  if (pathname === "/register") return <RegisterPage />;
  if (pathname === "/auth/google/callback") return <GoogleCallbackPage />;
  if (!authStorage.get()) {
    queueMicrotask(() => navigate("/", { replace: true }));
    return null;
  }
  if (checkingSession || !user) {
    return <main className="app-auth-loading"><span /><p>{t("Đang tải phiên đăng nhập…")}</p></main>;
  }
  if (pathname === "/account") return <AccountPage />;
  if (pathname === "/admin/users") {
    if (user.role === "admin" || user.role === "operator") return <UserManagementPage />;
    queueMicrotask(() => navigate("/robots", { replace: true }));
    return null;
  }
  if (pathname === "/robots") return <RobotListPage />;
  if (
    user.role === "guest"
    && (
      pathname === "/robots/new"
      || /^\/robots\/[^/]+\/(?:edit|configuration)$/.test(pathname)
    )
  ) {
    queueMicrotask(() => navigate("/robots", { replace: true }));
    return null;
  }
  if (pathname === "/robots/new") return <RobotEditorPage mode="create" />;
  if (/^\/robots\/[^/]+\/edit$/.test(pathname)) return <RobotEditorPage mode="edit" />;
  if (/^\/robots\/[^/]+\/configuration$/.test(pathname)) return <RobotConfigurationPage />;
  if (/^\/control\/[^/]+$/.test(pathname)) return <DashboardPage />;
  queueMicrotask(() => navigate("/", { replace: true }));
  return null;
}
