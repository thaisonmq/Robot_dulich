import { useState } from "react";
import { Eye, EyeOff, UserPlus } from "lucide-react";
import { api, persistSession } from "../api/client";
import { AuthShell } from "../components/AuthShell";
import { GoogleAuthButton } from "../components/GoogleAuthButton";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";

export function RegisterPage() {
  const { t } = useI18n();
  const navigate = useNavigate();
  const setUser = useAppStore((state) => state.setUser);
  const [fullName, setFullName] = useState("");
  const [username, setUsername] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [confirmation, setConfirmation] = useState("");
  const [showPassword, setShowPassword] = useState(false);
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  async function submit(event: React.FormEvent) {
    event.preventDefault();
    setError("");
    if (password !== confirmation) {
      setError(t("Mật khẩu xác nhận chưa khớp"));
      return;
    }
    setLoading(true);
    try {
      const result = await api.register({
        username,
        email,
        full_name: fullName,
        password,
      });
      persistSession(result.access_token, result.user);
      setUser(result.user);
      navigate("/robots");
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : t("Không thể đăng ký tài khoản"));
    } finally {
      setLoading(false);
    }
  }

  return (
    <AuthShell wide>
      <div className="login-security"><UserPlus size={18} /><span>Guest registration</span></div>
      <div className="login-copy">
        <h1>{t("Tạo tài khoản")}</h1>
        <p>{t("Tài khoản mới bắt đầu với quyền khách: được kết nối robot nhưng không được sửa cấu hình kỹ thuật.")}</p>
      </div>
      <form className="login-form register-form" onSubmit={submit}>
        <div className="auth-form-grid">
          <label className="form-field">
            <span>{t("Họ và tên")}</span>
            <input type="text" value={fullName} autoComplete="name" onChange={(event) => setFullName(event.target.value)} required />
          </label>
          <label className="form-field">
            <span>{t("Tên đăng nhập")}</span>
            <input type="text" value={username} autoComplete="username" pattern="[a-z0-9][a-z0-9._-]{2,31}" onChange={(event) => setUsername(event.target.value.toLocaleLowerCase())} required />
          </label>
        </div>
        <label className="form-field">
          <span>Email</span>
          <input type="email" value={email} autoComplete="email" onChange={(event) => setEmail(event.target.value)} required />
        </label>
        <div className="auth-form-grid">
          <div className="form-field">
            <label htmlFor="register-password">{t("Mật khẩu")}</label>
            <span className="password-field">
              <input
                id="register-password"
                type={showPassword ? "text" : "password"}
                value={password}
                minLength={8}
                autoComplete="new-password"
                onChange={(event) => setPassword(event.target.value)}
                required
              />
              <button type="button" onClick={() => setShowPassword((value) => !value)} aria-label={showPassword ? t("Ẩn mật khẩu") : t("Hiện mật khẩu")}>
                {showPassword ? <EyeOff size={18} /> : <Eye size={18} />}
              </button>
            </span>
          </div>
          <label className="form-field">
            <span>{t("Xác nhận mật khẩu")}</span>
            <input type={showPassword ? "text" : "password"} value={confirmation} minLength={8} autoComplete="new-password" onChange={(event) => setConfirmation(event.target.value)} required />
          </label>
        </div>
        <small className="form-hint">{t("Dùng ít nhất 8 ký tự. Tên đăng nhập không chứa khoảng trắng.")}</small>
        {error && <p role="alert" className="form-error">{error}</p>}
        <button className="button button--primary button--large" disabled={loading}>
          {loading ? t("Đang tạo tài khoản…") : t("Tạo tài khoản khách")}
        </button>
        <div className="auth-divider"><span>{t("hoặc")}</span></div>
        <GoogleAuthButton label="Đăng ký bằng Google" />
      </form>
      <p className="auth-switch">
        {t("Đã có tài khoản?")}{" "}
        <button type="button" onClick={() => navigate("/")}>{t("Quay lại đăng nhập")}</button>
      </p>
    </AuthShell>
  );
}
