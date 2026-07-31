import { useEffect, useRef, useState } from "react";
import { LoaderCircle, ShieldCheck } from "lucide-react";
import { api, persistSession } from "../api/client";
import { AuthShell } from "../components/AuthShell";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";

export function GoogleCallbackPage() {
  const { t } = useI18n();
  const navigate = useNavigate();
  const setUser = useAppStore((state) => state.setUser);
  const started = useRef(false);
  const [error, setError] = useState("");

  useEffect(() => {
    if (started.current) return;
    started.current = true;
    const code = new URLSearchParams(window.location.search).get("code") ?? "";
    if (!code) {
      setError(t("Thiếu mã xác thực Google"));
      return;
    }
    void api.exchangeGoogleCode(code)
      .then((result) => {
        persistSession(result.access_token, result.user);
        setUser(result.user);
        navigate("/robots", { replace: true });
      })
      .catch((reason) => {
        setError(reason instanceof Error ? reason.message : t("Không thể đăng nhập bằng Google"));
      });
  }, [navigate, setUser, t]);

  return (
    <AuthShell>
      <div className="oauth-callback">
        <span className={error ? "oauth-callback__icon is-error" : "oauth-callback__icon"}>
          {error ? <ShieldCheck size={30} /> : <LoaderCircle size={30} />}
        </span>
        <h1>{error ? t("Không thể xác thực") : t("Đang hoàn tất đăng nhập")}</h1>
        <p>{error || t("Hệ thống đang xác minh tài khoản Google và tạo phiên đăng nhập.")}</p>
        {error && <button type="button" className="button button--primary" onClick={() => navigate("/", { replace: true })}>{t("Quay lại đăng nhập")}</button>}
      </div>
    </AuthShell>
  );
}
