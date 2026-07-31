import { useState } from "react";
import { ArrowLeft, AtSign, KeyRound, ShieldCheck, UserRound } from "lucide-react";
import { api, userStorage } from "../api/client";
import { AccountMenu } from "../components/AccountMenu";
import { Brand } from "../components/Brand";
import { GlobalLanguageSelect } from "../components/GlobalLanguageSelect";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";

const ROLE_LABELS = {
  admin: "Quản trị viên",
  operator: "Nhân viên vận hành",
  guest: "Tài khoản khách",
} as const;

export function AccountPage() {
  const { t } = useI18n();
  const navigate = useNavigate();
  const user = useAppStore((state) => state.user)!;
  const setUser = useAppStore((state) => state.setUser);
  const [fullName, setFullName] = useState(user.full_name);
  const [currentPassword, setCurrentPassword] = useState("");
  const [newPassword, setNewPassword] = useState("");
  const [confirmation, setConfirmation] = useState("");
  const [profileMessage, setProfileMessage] = useState("");
  const [passwordMessage, setPasswordMessage] = useState("");
  const [savingProfile, setSavingProfile] = useState(false);
  const [savingPassword, setSavingPassword] = useState(false);

  async function saveProfile(event: React.FormEvent) {
    event.preventDefault();
    setSavingProfile(true);
    setProfileMessage("");
    try {
      const updated = await api.updateProfile(fullName);
      setUser(updated);
      userStorage.set(updated);
      setProfileMessage(t("Đã lưu hồ sơ"));
    } catch (reason) {
      setProfileMessage(reason instanceof Error ? reason.message : t("Không thể lưu hồ sơ"));
    } finally {
      setSavingProfile(false);
    }
  }

  async function savePassword(event: React.FormEvent) {
    event.preventDefault();
    setPasswordMessage("");
    if (newPassword !== confirmation) {
      setPasswordMessage(t("Mật khẩu xác nhận chưa khớp"));
      return;
    }
    setSavingPassword(true);
    try {
      await api.changePassword(currentPassword, newPassword);
      const refreshed = await api.me();
      setUser(refreshed);
      userStorage.set(refreshed);
      setCurrentPassword("");
      setNewPassword("");
      setConfirmation("");
      setPasswordMessage(t("Đã đổi mật khẩu"));
    } catch (reason) {
      setPasswordMessage(reason instanceof Error ? reason.message : t("Không thể đổi mật khẩu"));
    } finally {
      setSavingPassword(false);
    }
  }

  return (
    <main className="account-page">
      <header className="account-header">
        <div>
          <Brand compact />
          <span className="header-divider" />
          <button type="button" onClick={() => navigate("/robots")}><ArrowLeft size={18} /> {t("Quản lý robot")}</button>
        </div>
        <GlobalLanguageSelect />
        <AccountMenu />
      </header>

      <div className="account-page__content">
        <section className="account-hero">
          <div>
            <p className="eyebrow">IDENTITY · ACCESS</p>
            <h1>{t("Tài khoản của tôi")}</h1>
            <p>{t("Cập nhật thông tin cá nhân và kiểm soát phương thức đăng nhập.")}</p>
          </div>
          <div className="account-role-card">
            <span><ShieldCheck size={21} /></span>
            <div><small>{t("Vai trò hiện tại")}</small><strong>{t(ROLE_LABELS[user.role])}</strong></div>
          </div>
        </section>

        {user.must_change_password && (
          <div className="notice notice--warning account-password-notice">
            <KeyRound size={18} />
            <span><strong>{t("Bạn đang dùng mật khẩu tạm thời.")}</strong> {t("Hãy đổi mật khẩu trước khi bàn giao tài khoản.")}</span>
          </div>
        )}

        <div className="account-settings-grid">
          <section className="account-setting-panel">
            <div className="account-setting-panel__title">
              <span><UserRound size={20} /></span>
              <div><h2>{t("Hồ sơ cá nhân")}</h2><p>{t("Thông tin hiển thị trong trung tâm vận hành.")}</p></div>
            </div>
            <form onSubmit={saveProfile}>
              <label className="form-field"><span>{t("Họ và tên")}</span><input value={fullName} onChange={(event) => setFullName(event.target.value)} required minLength={2} /></label>
              <div className="account-readonly-field"><AtSign size={16} /><span><small>{t("Tên đăng nhập")}</small><strong>@{user.username}</strong></span></div>
              <div className="account-readonly-field"><span className="account-readonly-field__mark">E</span><span><small>Email</small><strong>{user.email}</strong></span></div>
              {profileMessage && <p className="account-form-message">{profileMessage}</p>}
              <button className="button button--primary" disabled={savingProfile}>{savingProfile ? t("Đang lưu…") : t("Lưu hồ sơ")}</button>
            </form>
          </section>

          <section className="account-setting-panel">
            <div className="account-setting-panel__title">
              <span><KeyRound size={20} /></span>
              <div><h2>{t("Bảo mật đăng nhập")}</h2><p>{t("Mật khẩu được lưu bằng hash PBKDF2 trong PostgreSQL.")}</p></div>
            </div>
            <form onSubmit={savePassword}>
              {user.password_enabled && <label className="form-field"><span>{t("Mật khẩu hiện tại")}</span><input type="password" value={currentPassword} autoComplete="current-password" onChange={(event) => setCurrentPassword(event.target.value)} required /></label>}
              <label className="form-field"><span>{t("Mật khẩu mới")}</span><input type="password" value={newPassword} minLength={8} autoComplete="new-password" onChange={(event) => setNewPassword(event.target.value)} required /></label>
              <label className="form-field"><span>{t("Xác nhận mật khẩu")}</span><input type="password" value={confirmation} minLength={8} autoComplete="new-password" onChange={(event) => setConfirmation(event.target.value)} required /></label>
              {passwordMessage && <p className="account-form-message">{passwordMessage}</p>}
              <button className="button button--outline" disabled={savingPassword}>{savingPassword ? t("Đang cập nhật…") : user.password_enabled ? t("Đổi mật khẩu") : t("Tạo mật khẩu")}</button>
            </form>
            <div className="linked-identities">
              <small>{t("Phương thức đã liên kết")}</small>
              <span className={user.password_enabled ? "is-linked" : ""}>Password</span>
              <span className={user.auth_providers.includes("google") ? "is-linked" : ""}>Google</span>
            </div>
          </section>
        </div>
      </div>
    </main>
  );
}
