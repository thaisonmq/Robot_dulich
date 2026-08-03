import { useEffect, useRef, useState } from "react";
import {
  ChevronDown, LogOut, Settings, ShieldCheck, UserRound,
  UsersRound,
} from "lucide-react";
import { clearSession } from "../api/client";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";

const ROLE_LABELS = {
  admin: "Quản trị viên",
  operator: "Nhân viên vận hành",
  guest: "Tài khoản khách",
} as const;

export function AccountMenu() {
  const { t } = useI18n();
  const navigate = useNavigate();
  const user = useAppStore((state) => state.user);
  const setUser = useAppStore((state) => state.setUser);
  const resetSession = useAppStore((state) => state.resetSession);
  const [open, setOpen] = useState(false);
  const rootRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    function closeOnOutsideClick(event: MouseEvent) {
      if (!rootRef.current?.contains(event.target as Node)) setOpen(false);
    }
    function closeOnEscape(event: KeyboardEvent) {
      if (event.key === "Escape") setOpen(false);
    }
    document.addEventListener("mousedown", closeOnOutsideClick);
    document.addEventListener("keydown", closeOnEscape);
    return () => {
      document.removeEventListener("mousedown", closeOnOutsideClick);
      document.removeEventListener("keydown", closeOnEscape);
    };
  }, []);

  if (!user) return null;

  function go(path: string) {
    setOpen(false);
    navigate(path);
  }

  function logout() {
    clearSession();
    setUser(null);
    resetSession();
    navigate("/", { replace: true });
  }

  return (
    <div className="account-menu" ref={rootRef}>
      <button
        type="button"
        className="account-menu__trigger"
        aria-haspopup="menu"
        aria-expanded={open}
        onClick={() => setOpen((value) => !value)}
      >
        <span className="account-avatar">
          {user.avatar_url ? (
            <img src={user.avatar_url} alt={t(user.full_name)} referrerPolicy="no-referrer" />
          ) : (
            t(user.full_name).slice(0, 1).toLocaleUpperCase()
          )}
        </span>
        <span className="account-menu__identity">
          <strong>{t(user.full_name)}</strong>
          <small>{t(ROLE_LABELS[user.role])}</small>
        </span>
        <ChevronDown size={16} className={open ? "is-open" : ""} />
      </button>

      {open && (
        <div className="account-menu__popover" role="menu">
          <div className="account-menu__summary">
            <span><UserRound size={17} /></span>
            <div>
              <strong>@{user.username}</strong>
              <small>{user.email}</small>
            </div>
            {user.role === "admin" && <ShieldCheck size={17} />}
          </div>
          <button type="button" role="menuitem" onClick={() => go("/account")}>
            <Settings size={17} />
            <span><strong>{t("Tài khoản của tôi")}</strong><small>{t("Hồ sơ và bảo mật")}</small></span>
          </button>
          {(user.role === "admin" || user.role === "operator") && (
            <button type="button" role="menuitem" onClick={() => go("/admin/users")}>
              <UsersRound size={17} />
              <span>
                <strong>{t("Quản lý tài khoản")}</strong>
                <small>{user.role === "admin" ? t("Phân quyền nhân sự") : t("Quản lý tài khoản khách")}</small>
              </span>
            </button>
          )}
          <button type="button" role="menuitem" className="account-menu__logout" onClick={logout}>
            <LogOut size={17} />
            <span><strong>{t("Đăng xuất")}</strong><small>{t("Kết thúc phiên hiện tại")}</small></span>
          </button>
        </div>
      )}
    </div>
  );
}
