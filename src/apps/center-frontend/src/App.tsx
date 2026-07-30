import { useEffect } from "react";
import { AUTH_EXPIRED_EVENT, authStorage } from "./api/client";
import { DashboardPage } from "./pages/DashboardPage";
import { LoginPage } from "./pages/LoginPage";
import { RobotConfigurationPage } from "./pages/RobotConfigurationPage";
import { RobotEditorPage } from "./pages/RobotEditorPage";
import { RobotListPage } from "./pages/RobotListPage";
import { navigate, usePathname } from "./router";
import { useAppStore } from "./state/appStore";

export function App() {
  const pathname = usePathname();
  const setUser = useAppStore((state) => state.setUser);
  const resetSession = useAppStore((state) => state.resetSession);

  useEffect(() => {
    const handleExpiredSession = () => {
      setUser(null);
      resetSession();
      navigate("/", { replace: true });
    };
    window.addEventListener(AUTH_EXPIRED_EVENT, handleExpiredSession);
    return () => window.removeEventListener(AUTH_EXPIRED_EVENT, handleExpiredSession);
  }, [resetSession, setUser]);

  if (pathname === "/") return <LoginPage />;
  if (!authStorage.get()) {
    queueMicrotask(() => navigate("/", { replace: true }));
    return null;
  }
  if (pathname === "/robots") return <RobotListPage />;
  if (pathname === "/robots/new") return <RobotEditorPage mode="create" />;
  if (/^\/robots\/[^/]+\/edit$/.test(pathname)) return <RobotEditorPage mode="edit" />;
  if (/^\/robots\/[^/]+\/configuration$/.test(pathname)) return <RobotConfigurationPage />;
  if (/^\/control\/[^/]+$/.test(pathname)) return <DashboardPage />;
  queueMicrotask(() => navigate("/", { replace: true }));
  return null;
}
