import { useQuery } from "@tanstack/react-query";
import { api, googleLoginUrl } from "../api/client";
import { useI18n } from "../i18n/I18nProvider";

export function GoogleAuthButton({
  label = "Tiếp tục với Google",
}: {
  label?: string;
}) {
  const { t } = useI18n();
  const status = useQuery({
    queryKey: ["google-oauth-status"],
    queryFn: api.googleStatus,
    staleTime: 60_000,
    retry: false,
  });
  const enabled = status.data?.enabled === true;

  return (
    <button
      type="button"
      className="google-auth-button"
      disabled={!enabled || status.isLoading}
      title={!status.isLoading && !enabled ? t("Google OAuth chưa được cấu hình") : undefined}
      onClick={() => window.location.assign(googleLoginUrl())}
    >
      <span aria-hidden="true">G</span>
      {status.isLoading ? t("Đang kiểm tra Google…") : t(label)}
    </button>
  );
}
