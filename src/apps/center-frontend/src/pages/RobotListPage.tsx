import { useState } from "react";
import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";
import {
  Battery, Bot, ChevronLeft, ChevronRight, Clock3, MapPin,
  MonitorPlay, OctagonX, Plus, PlugZap, RadioTower, Search, Server,
  Settings2, SlidersHorizontal, UserRound, Wifi,
} from "lucide-react";
import { api } from "../api/client";
import { AccountMenu } from "../components/AccountMenu";
import { Brand } from "../components/Brand";
import { GlobalLanguageSelect } from "../components/GlobalLanguageSelect";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";
import type { Robot } from "../types";

export function RobotListPage() {
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const { language, t } = useI18n();
  const user = useAppStore((state) => state.user);
  const selectRobot = useAppStore((state) => state.selectRobot);
  const setSession = useAppStore((state) => state.setSession);
  const setConnectionState = useAppStore((state) => state.setConnectionState);
  const [connectingId, setConnectingId] = useState("");
  const [watchingId, setWatchingId] = useState("");
  const [error, setError] = useState("");
  const [page, setPage] = useState(1);
  const [search, setSearch] = useState("");
  const [status, setStatus] = useState("all");
  const canOperate = user?.role === "admin" || user?.role === "operator";

  function statusLabel(robot: Robot) {
    if (robot.enrollment_status === "pending") return t("Chờ robot chạy");
    if (!robot.enabled) return t("Đã vô hiệu hoá");
    if (robot.status === "offline") return t("Ngoại tuyến");
    if (robot.availability === "busy") return t("Đang bận");
    if (robot.status === "error") return t("Có lỗi");
    return t("Sẵn sàng");
  }

  const robotsQuery = useQuery({
    queryKey: ["robots", page, search, status],
    queryFn: () => api.robots({ page, pageSize: 6, search, status }),
    refetchInterval: 2000,
  });
  const activeSessionsQuery = useQuery({
    queryKey: ["active-guest-sessions"],
    queryFn: api.activeGuestSessions,
    enabled: canOperate,
    refetchInterval: 2000,
  });
  const mySessionsQuery = useQuery({
    queryKey: ["my-active-sessions"],
    queryFn: api.myActiveSessions,
    refetchInterval: 2000,
  });
  const forceEndMutation = useMutation({
    mutationFn: ({ sessionId, owned }: { sessionId: string; owned: boolean }) => (
      owned ? api.deleteSession(sessionId) : api.forceEndSession(sessionId)
    ),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: ["my-active-sessions"] });
      void queryClient.invalidateQueries({ queryKey: ["active-guest-sessions"] });
      void queryClient.invalidateQueries({ queryKey: ["robots"] });
    },
    onError: (reason) => {
      setError(reason instanceof Error ? reason.message : t("Không thể kết thúc phiên"));
    },
  });
  const robots = robotsQuery.data?.items ?? [];
  const mySessionIds = new Set(
    (mySessionsQuery.data ?? []).map((activeSession) => activeSession.session_id),
  );
  const visibleSessions = [
    ...(mySessionsQuery.data ?? []),
    ...(activeSessionsQuery.data ?? []).filter(
      (activeSession) => !mySessionIds.has(activeSession.session_id),
    ),
  ];
  const summary = robotsQuery.data?.summary ?? {
    total: 0, online: 0, available: 0, pending: 0,
  };

  async function connect(robot: Robot) {
    setConnectingId(robot.robot_id);
    setError("");
    selectRobot(robot);
    setConnectionState("connecting");
    try {
      const session = await api.createSession(robot.robot_id);
      setSession(session);
      setConnectionState("connected");
      navigate(`/control/${robot.robot_id}`);
    } catch (reason) {
      setConnectionState(robot.status === "offline" ? "offline" : "error");
      setError(reason instanceof Error ? reason.message : t("Không thể kết nối robot"));
    } finally {
      setConnectingId("");
    }
  }

  async function watchSession(sessionId: string, robotId: string) {
    setWatchingId(sessionId);
    setError("");
    try {
      const [robot, spectatorSession] = await Promise.all([
        api.robot(robotId),
        api.spectateSession(sessionId),
      ]);
      selectRobot(robot);
      setSession(spectatorSession);
      setConnectionState("connected");
      navigate(`/control/${robotId}`);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : t("Không thể xem phiên điều khiển"));
    } finally {
      setWatchingId("");
    }
  }

  return (
    <main className="roster-page fleet-manager">
      <header className="roster-header">
        <div className="roster-header__title">
          <Brand compact />
          <span className="header-divider" />
          <h1>{t("Quản lý robot")}</h1>
        </div>
        <div className="system-summary">
          <span><i className="status-dot online" /><small>Gateway</small><strong>{t("Hoạt động")}</strong></span>
          <span><Server size={20} /><small>{t("Đang online")}</small><strong>{summary.online} / {summary.total}</strong></span>
          <span><RadioTower size={20} /><small>{t("Chờ kết nối")}</small><strong>{summary.pending}</strong></span>
        </div>
        <GlobalLanguageSelect />
        <AccountMenu />
      </header>

      <div className="fleet-manager__content">
        <section className="fleet-manager__heading">
          <div>
            <p className="eyebrow">{t("ĐĂNG KÝ THIẾT BỊ · TRẠNG THÁI TRỰC TIẾP")}</p>
            <h2>{t("Danh sách robot")}</h2>
            <p>{t("Đăng ký thiết bị, theo dõi kết nối và mở phiên điều khiển.")}</p>
          </div>
          {canOperate && (
            <button type="button" className="button button--primary" onClick={() => navigate("/robots/new")}>
              <Plus size={19} /> {t("Thêm robot")}
            </button>
          )}
        </section>

        {!canOperate && (
          <div className="guest-access-notice">
            <span><PlugZap size={18} /></span>
            <div>
              <strong>{t("Quyền điều khiển tiêu chuẩn")}</strong>
              <p>{t("Tài khoản khách được kết nối và điều khiển robot; cấu hình kỹ thuật được bảo vệ ở cấp vận hành.")}</p>
            </div>
          </div>
        )}

        {Boolean(visibleSessions.length) && (
          <section className="active-session-strip" aria-label={t("Phiên đang hoạt động")}>
            <header>
              <span><i className="status-dot online" /> {t("Phiên đang hoạt động")}</span>
              <small>{visibleSessions.length} {t("phiên hoạt động")}</small>
            </header>
            <div>
              {visibleSessions.map((activeSession) => {
                const owned = activeSession.controller.id === user?.id;
                return <article key={activeSession.session_id}>
                  <span className="active-session-strip__user"><UserRound size={17} /></span>
                  <span>
                    <strong>{owned ? t("Phiên điều khiển của bạn") : t(activeSession.controller.name)}</strong>
                    <small>@{activeSession.controller.username} · {activeSession.robot_name}</small>
                  </span>
                  <time>{Math.max(1, Math.ceil(activeSession.duration_seconds / 60))} {t("phút")}</time>
                  {!owned && (
                    <button
                      type="button"
                      className="button button--outline"
                      disabled={Boolean(watchingId)}
                      onClick={() => void watchSession(activeSession.session_id, activeSession.robot_id)}
                    >
                      <MonitorPlay size={16} />
                      {watchingId === activeSession.session_id ? t("Đang mở…") : t("Xem cùng")}
                    </button>
                  )}
                  <button
                    type="button"
                    className="button button--danger-outline"
                    disabled={forceEndMutation.isPending}
                    onClick={() => forceEndMutation.mutate({
                      sessionId: activeSession.session_id,
                      owned,
                    })}
                  >
                    <OctagonX size={16} /> {owned ? t("Ngắt phiên") : t("Kết thúc")}
                  </button>
                </article>;
              })}
            </div>
          </section>
        )}

        <section className="fleet-toolbar" aria-label={t("Bộ lọc robot")}>
          <label className="fleet-search">
            <Search size={17} />
            <input
              value={search}
              onChange={(event) => {
                setSearch(event.target.value);
                setPage(1);
              }}
              placeholder={t("Tìm theo mã, tên hoặc khu vực…")}
            />
          </label>
          <label className="fleet-filter">
            <SlidersHorizontal size={17} />
            <select
              value={status}
              onChange={(event) => {
                setStatus(event.target.value);
                setPage(1);
              }}
            >
              <option value="all">{t("Tất cả trạng thái")}</option>
              <option value="online">{t("Đang online")}</option>
              <option value="offline">{t("Ngoại tuyến")}</option>
              <option value="pending">{t("Chờ robot chạy")}</option>
            </select>
          </label>
          <div className="fleet-toolbar__metrics">
            <span><strong>{summary.available}</strong> {t("sẵn sàng")}</span>
            <span><strong>{summary.online}</strong> {t("đang kết nối")}</span>
          </div>
        </section>

        {error && <div role="alert" className="notice notice--error">{t(error)}</div>}

        <section className="managed-robot-grid" aria-label={t("Danh sách robot")}>
          {robotsQuery.isLoading ? (
            Array.from({ length: 6 }, (_, index) => (
              <div className="managed-robot-card robot-card--loading" key={index} />
            ))
          ) : robots.length === 0 ? (
            <div className="fleet-empty">
              <span><Bot size={34} /></span>
              <h3>{t("Chưa có robot phù hợp")}</h3>
              <p>{t("Thay đổi bộ lọc hoặc đăng ký robot đầu tiên.")}</p>
              {canOperate && (
                <button type="button" className="button button--primary" onClick={() => navigate("/robots/new")}>
                  <Plus size={18} /> {t("Thêm robot")}
                </button>
              )}
            </div>
          ) : robots.map((robot) => {
            const canConnect = (
              robot.enabled
              && robot.enrollment_status === "enrolled"
              && robot.status === "online"
              && robot.availability === "available"
            );
            return (
              <article className="managed-robot-card robot-row" key={robot.robot_id}>
                <div className="managed-robot-card__head">
                  <span className={`availability-tag availability-tag--${robot.status}`}>
                    <i /> {statusLabel(robot)}
                  </span>
                  {canOperate && (
                    <button
                      type="button"
                      className="robot-card__settings"
                      aria-label={t("Sửa {name}", { name: robot.name })}
                      onClick={() => navigate(`/robots/${robot.robot_id}/edit`)}
                    >
                      <Settings2 size={17} />
                    </button>
                  )}
                </div>

                <div className="managed-robot-card__identity">
                  <span className="robot-avatar"><Bot size={27} /></span>
                  <span>
                    <small>{robot.robot_id}</small>
                    <strong>{robot.name}</strong>
                    <em><MapPin size={13} /> {t(robot.site_id)}</em>
                  </span>
                </div>

                <div className="managed-robot-card__telemetry">
                  <span><Wifi size={15} /><small>{t("Độ trễ")}</small><strong>{robot.status === "online" ? `${robot.network_rtt_ms} ms` : "—"}</strong></span>
                  <span><Battery size={15} /><small>{t("Pin")}</small><strong>{robot.status === "online" ? `${Math.round(robot.battery_percent)}%` : "—"}</strong></span>
                  <span><Clock3 size={15} /><small>{t("Cập nhật")}</small><strong>{robot.last_seen_at ? new Date(robot.last_seen_at).toLocaleTimeString(language, { hour: "2-digit", minute: "2-digit" }) : t("Chưa có")}</strong></span>
                </div>

                <div className="managed-robot-card__actions">
                  {canOperate ? (
                    <>
                      <button
                        type="button"
                        className="button button--outline"
                        aria-label={t("Cấu hình {name}", { name: robot.name })}
                        disabled={robot.enrollment_status === "pending"}
                        onClick={() => navigate(`/robots/${robot.robot_id}/configuration`)}
                      >
                        {t("Cấu hình")}
                      </button>
                      <button
                        type="button"
                        className="button button--primary"
                        disabled={!canConnect || Boolean(connectingId)}
                        onClick={() => void connect(robot)}
                      >
                        <PlugZap size={17} />
                        {connectingId === robot.robot_id ? t("Đang kết nối…") : t("Kết nối")}
                      </button>
                    </>
                  ) : (
                    <button
                      type="button"
                      className="button button--primary managed-robot-card__guest-connect"
                      disabled={!canConnect || Boolean(connectingId)}
                      onClick={() => void connect(robot)}
                    >
                      <PlugZap size={17} />
                      {connectingId === robot.robot_id ? t("Đang kết nối…") : t("Kết nối")}
                    </button>
                  )}
                </div>
              </article>
            );
          })}
        </section>

        <footer className="fleet-pagination">
          <span>
            {t("Hiển thị {shown} trong tổng số {total} robot", {
              shown: robots.length,
              total: robotsQuery.data?.total ?? 0,
            })}
          </span>
          <div>
            <button
              type="button"
              aria-label={t("Trang trước")}
              disabled={page <= 1}
              onClick={() => setPage((value) => Math.max(1, value - 1))}
            >
              <ChevronLeft size={18} />
            </button>
            <strong>{page} / {robotsQuery.data?.total_pages ?? 1}</strong>
            <button
              type="button"
              aria-label={t("Trang sau")}
              disabled={page >= (robotsQuery.data?.total_pages ?? 1)}
              onClick={() => setPage((value) => value + 1)}
            >
              <ChevronRight size={18} />
            </button>
          </div>
        </footer>
      </div>
    </main>
  );
}
