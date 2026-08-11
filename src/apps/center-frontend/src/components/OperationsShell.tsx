import { useState, type ReactNode } from "react";
import {
  Bot, ChevronsLeft, ChevronsRight, MapPinned, Settings, UsersRound,
} from "lucide-react";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate, usePathname } from "../router";
import { useAppStore } from "../state/appStore";
import { hasPermission } from "../utils/permissions";
import { AccountMenu } from "./AccountMenu";
import { Brand } from "./Brand";
import { GlobalLanguageSelect } from "./GlobalLanguageSelect";

const SIDEBAR_KEY = "rovera:operations-sidebar-collapsed";
const SIDEBAR_ID = "operations-sidebar-navigation";

function initialSidebarCollapsed(): boolean {
  const savedPreference = localStorage.getItem(SIDEBAR_KEY);
  if (savedPreference != null) return savedPreference === "true";
  return typeof window.matchMedia === "function"
    && window.matchMedia("(max-width: 1100px)").matches;
}

interface Props {
  title: string;
  className?: string;
  headerStatus?: ReactNode;
  children: ReactNode;
}

export function OperationsShell({ title, className = "", headerStatus, children }: Props) {
  const { t } = useI18n();
  const navigate = useNavigate();
  const pathname = usePathname();
  const user = useAppStore((state) => state.user);
  const [collapsed, setCollapsed] = useState(initialSidebarCollapsed);
  const canViewMaps = hasPermission(user, "maps.view");
  const canManageAccounts = user?.role === "admin" || user?.role === "operator";

  const toggleSidebar = () => setCollapsed((value) => {
    localStorage.setItem(SIDEBAR_KEY, String(!value));
    return !value;
  });
  const active = (path: string) => path === "/maps"
    ? pathname === "/maps" || /^\/maps\/[^/]+$/.test(pathname)
    : pathname === path || pathname.startsWith(`${path}/`);

  return <main className={`roster-page operations-shell${collapsed ? " is-sidebar-collapsed" : ""} ${className}`.trim()}>
    <header className="operations-topbar">
      <button type="button" className="operations-topbar__brand" onClick={() => navigate("/robots")} aria-label={t("Về danh sách robot")}>
        <Brand compact />
      </button>
      <span className="operations-topbar__divider" aria-hidden="true" />
      <div className="operations-topbar__title"><small>{t("Không gian vận hành")}</small><strong>{t(title)}</strong></div>
      <div className="operations-topbar__spacer" />
      {headerStatus}
      <GlobalLanguageSelect />
      <AccountMenu />
    </header>

    <aside className="operations-sidebar" aria-label={t("Danh mục vận hành")}>
      <nav id={SIDEBAR_ID} className="operations-sidebar__nav" aria-label={t("Điều hướng chính")}>
        <p>{t("Vận hành")}</p>
        <button type="button" className={active("/robots") ? "is-active" : ""}
          aria-current={active("/robots") ? "page" : undefined}
          onClick={() => navigate("/robots")} title={collapsed ? t("Danh sách robot") : undefined}>
          <span><Bot size={19} /></span><strong>{t("Danh sách robot")}</strong>
        </button>
        {canViewMaps && <button type="button" className={active("/maps") ? "is-active" : ""}
          aria-current={active("/maps") ? "page" : undefined}
          onClick={() => navigate("/maps")} title={collapsed ? t("Bản đồ") : undefined}>
          <span><MapPinned size={19} /></span><strong>{t("Bản đồ")}</strong>
        </button>}
        <p>{t("Tài khoản")}</p>
        {canManageAccounts && <button type="button" className={active("/admin/users") ? "is-active" : ""}
          aria-current={active("/admin/users") ? "page" : undefined}
          onClick={() => navigate("/admin/users")} title={collapsed ? t("Quản lý tài khoản") : undefined}>
          <span><UsersRound size={19} /></span><strong>{t("Quản lý tài khoản")}</strong>
        </button>}
        <button type="button" className={active("/account") ? "is-active" : ""}
          aria-current={active("/account") ? "page" : undefined}
          onClick={() => navigate("/account")} title={collapsed ? t("Cài đặt tài khoản") : undefined}>
          <span><Settings size={19} /></span><strong>{t("Cài đặt tài khoản")}</strong>
        </button>
      </nav>

      <button type="button" className="operations-sidebar__toggle" onClick={toggleSidebar}
        aria-label={t(collapsed ? "Mở rộng menu" : "Thu gọn menu")}
        aria-controls={SIDEBAR_ID} aria-expanded={!collapsed}>
        {collapsed ? <ChevronsRight size={19} aria-hidden="true" /> : <ChevronsLeft size={19} aria-hidden="true" />}
        <strong>{t(collapsed ? "Mở rộng menu" : "Thu gọn menu")}</strong>
      </button>
    </aside>

    <section className="operations-workspace">
      {children}
    </section>
  </main>;
}
