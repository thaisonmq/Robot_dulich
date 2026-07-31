import {
  Activity, Images, MapPinned, MonitorPlay, Navigation, ScanFace,
} from "lucide-react";
import { Brand } from "./Brand";
import { GlobalLanguageSelect } from "./GlobalLanguageSelect";
import { useI18n } from "../i18n/I18nProvider";

export function AuthShell({
  children,
  wide = false,
}: {
  children: React.ReactNode;
  wide?: boolean;
}) {
  const { t } = useI18n();
  return (
    <main className="login-page">
      <section className="login-image" aria-label={t("Không gian du lịch được số hoá")}>
        <img
          src="/assets/login-spatial-tourism-v4.png"
          alt={t("Không gian số với các màn hình bản đồ và khu du lịch")}
        />
        <div className="login-image__veil" aria-hidden="true" />
        <div className="login-vision" aria-hidden="true">
          <header className="login-vision__header">
            <div className="login-vision__title">
              <Activity size={20} />
              <span>
                <strong>KHÔNG GIAN DU LỊCH SỐ</strong>
                <small><i /> VISION · AR · LIVE · MAP</small>
              </span>
            </div>
          </header>

          <div className="spatial-screen-labels">
            <div className="spatial-screen-label spatial-screen-label--route">
              <Navigation size={15} />
              <span><strong>LỘ TRÌNH DI CHUYỂN</strong><small>Bản đồ trực quan</small></span>
            </div>
            <div className="spatial-screen-label spatial-screen-label--live">
              <MonitorPlay size={15} />
              <span><strong>MÀN HÌNH TRỰC TIẾP</strong><small><i /> Khu du lịch</small></span>
            </div>
            <div className="spatial-screen-label spatial-screen-label--ar">
              <ScanFace size={15} />
              <span><strong>HƯỚNG DẪN THAM QUAN AR</strong><small>Điều hướng tại điểm đến</small></span>
            </div>
            <div className="spatial-screen-label spatial-screen-label--map">
              <MapPinned size={15} />
              <span><strong>BẢN ĐỒ ĐIỂM ĐẾN</strong><small>Khám phá khu du lịch</small></span>
            </div>
            <div className="spatial-screen-label spatial-screen-label--places">
              <Images size={15} />
              <span><strong>KHU DU LỊCH</strong><small>Không gian nổi bật</small></span>
            </div>
          </div>
        </div>
      </section>
      <section className="login-panel">
        <div className={`login-panel__inner${wide ? " login-panel__inner--wide" : ""}`}>
          <div className="login-panel__top">
            <Brand />
            <GlobalLanguageSelect />
          </div>
          <div className="login-panel__flow">{children}</div>
        </div>
      </section>
    </main>
  );
}
