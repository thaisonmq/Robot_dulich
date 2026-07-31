import { useEffect, useState } from "react";
import { Eye, EyeOff, LockKeyhole } from "lucide-react";
import { api, persistSession } from "../api/client";
import { AuthShell } from "../components/AuthShell";
import { GoogleAuthButton } from "../components/GoogleAuthButton";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";

export function LoginPage() {
  const navigate = useNavigate();
  const { t } = useI18n();
  const setUser = useAppStore((state) => state.setUser);
  const [identifier, setIdentifier] = useState("");
  const [password, setPassword] = useState("");
  const [showPassword, setShowPassword] = useState(false);
  const oauthError = new URLSearchParams(window.location.search).get("oauth_error");
  const [error, setError] = useState(oauthError ?? "");
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    localStorage.removeItem("rovera_identifier");
  }, []);

  async function submit(event: React.FormEvent) {
    event.preventDefault();
    setLoading(true);
    setError("");
    try {
      const result = await api.login(identifier, password);
      persistSession(result.access_token, result.user);
      setUser(result.user);
      navigate("/robots");
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : t("Đăng nhập thất bại"));
    } finally {
      setLoading(false);
    }
  }

  return (
    <AuthShell>
      <div className="login-security"><LockKeyhole size={18} /><span>Secure access</span></div>
      <div className="login-copy">
        <h1>{t("Đăng nhập")}</h1>
        <p>{t("Dùng tên đăng nhập hoặc email của bạn.")}</p>
      </div>
      <form className="login-form" onSubmit={submit}>
        <label className="form-field">
          <span>{t("Tên đăng nhập hoặc email")}</span>
          <input
            id="identifier"
            type="text"
            value={identifier}
            autoComplete="username"
            onChange={(event) => setIdentifier(event.target.value)}
            required
          />
        </label>
        <div className="form-field">
          <label htmlFor="password">{t("Mật khẩu")}</label>
          <span className="password-field">
            <input
              id="password"
              type={showPassword ? "text" : "password"}
              value={password}
              autoComplete="current-password"
              onChange={(event) => setPassword(event.target.value)}
              required
            />
            <button
              type="button"
              onClick={() => setShowPassword((value) => !value)}
              aria-label={showPassword ? t("Ẩn mật khẩu") : t("Hiện mật khẩu")}
            >
              {showPassword ? <EyeOff size={19} /> : <Eye size={19} />}
            </button>
          </span>
        </div>
        <div className="form-row form-row--end">
          <button type="button" className="text-button">{t("Quên mật khẩu?")}</button>
        </div>
        {error && <p role="alert" className="form-error">{error}</p>}
        <button className="button button--primary button--large" disabled={loading}>
          {loading ? t("Đang xác thực…") : t("Đăng nhập")}
        </button>
        <div className="auth-divider"><span>{t("hoặc")}</span></div>
        <GoogleAuthButton />
      </form>
      <p className="auth-switch">
        {t("Chưa có tài khoản?")}{" "}
        <button type="button" onClick={() => navigate("/register")}>{t("Đăng ký tài khoản khách")}</button>
      </p>
    </AuthShell>
  );
}
