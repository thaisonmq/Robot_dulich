import { useEffect, useState } from "react";
import { AlertTriangle, Check, Info, X } from "lucide-react";
import { useI18n } from "../i18n/I18nProvider";

export type ToastTone = "success" | "error" | "info";

interface ToastDetail {
  id: number;
  message: string;
  tone: ToastTone;
}

const TOAST_EVENT = "rovera:toast";

export function showToast(message: string, tone: ToastTone = "success"): void {
  window.dispatchEvent(new CustomEvent<ToastDetail>(TOAST_EVENT, {
    detail: { id: Date.now(), message, tone },
  }));
}

export function ToastViewport() {
  const { t } = useI18n();
  const [toast, setToast] = useState<ToastDetail | null>(null);

  useEffect(() => {
    const show = (event: Event) => setToast((event as CustomEvent<ToastDetail>).detail);
    window.addEventListener(TOAST_EVENT, show);
    return () => window.removeEventListener(TOAST_EVENT, show);
  }, []);

  useEffect(() => {
    if (!toast) return;
    const timer = window.setTimeout(() => setToast(null), 2600);
    return () => window.clearTimeout(timer);
  }, [toast]);

  if (!toast) return null;
  const Icon = toast.tone === "success" ? Check : toast.tone === "error" ? AlertTriangle : Info;

  return <div className="toast-viewport" aria-live="polite" aria-atomic="true">
    <div className={`app-toast app-toast--${toast.tone}`} role={toast.tone === "error" ? "alert" : "status"}>
      <span className="app-toast__icon"><Icon size={17} /></span>
      <strong>{toast.message}</strong>
      <button type="button" aria-label={t("Đóng")} onClick={() => setToast(null)}><X size={15} /></button>
    </div>
  </div>;
}
