import { useState } from "react";
import { useQuery } from "@tanstack/react-query";
import {
  Battery, Bot, ChevronLeft, ChevronRight, Clock3, LogOut, MapPin,
  Plus, PlugZap, RadioTower, Search, Server, Settings2, SlidersHorizontal,
  Wifi,
} from "lucide-react";
import { api, authStorage } from "../api/client";
import { Brand } from "../components/Brand";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";
import type { Robot } from "../types";

function statusLabel(robot: Robot) {
  if (robot.enrollment_status === "pending") return "Chờ robot chạy";
  if (!robot.enabled) return "Đã vô hiệu hoá";
  if (robot.status === "offline") return "Ngoại tuyến";
  if (robot.availability === "busy") return "Đang bận";
  if (robot.status === "error") return "Có lỗi";
  return "Sẵn sàng";
}

export function RobotListPage() {
  const navigate = useNavigate();
  const user = useAppStore((state) => state.user);
  const setUser = useAppStore((state) => state.setUser);
  const selectRobot = useAppStore((state) => state.selectRobot);
  const setSession = useAppStore((state) => state.setSession);
  const setConnectionState = useAppStore((state) => state.setConnectionState);
  const [connectingId, setConnectingId] = useState("");
  const [error, setError] = useState("");
  const [page, setPage] = useState(1);
  const [search, setSearch] = useState("");
  const [status, setStatus] = useState("all");

  const robotsQuery = useQuery({
    queryKey: ["robots", page, search, status],
    queryFn: () => api.robots({ page, pageSize: 6, search, status }),
    refetchInterval: 2000,
  });
  const robots = robotsQuery.data?.items ?? [];
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
      setError(reason instanceof Error ? reason.message : "Không thể kết nối robot");
    } finally {
      setConnectingId("");
    }
  }

  function logout() {
    authStorage.clear();
    sessionStorage.removeItem("rovera_user");
    setUser(null);
    navigate("/");
  }

  return (
    <main className="roster-page fleet-manager">
      <header className="roster-header">
        <div className="roster-header__title">
          <Brand compact />
          <span className="header-divider" />
          <h1>Quản lý robot</h1>
        </div>
        <div className="system-summary">
          <span><i className="status-dot online" /><small>Gateway</small><strong>Hoạt động</strong></span>
          <span><Server size={20} /><small>Đang online</small><strong>{summary.online} / {summary.total}</strong></span>
          <span><RadioTower size={20} /><small>Chờ kết nối</small><strong>{summary.pending}</strong></span>
        </div>
        <div className="operator">
          <span className="operator__avatar">{user?.name?.slice(0, 1) ?? "N"}</span>
          <span><strong>{user?.name ?? "Nguyễn Minh"}</strong><small>Operator</small></span>
          <button type="button" onClick={logout}><LogOut size={19} /> Đăng xuất</button>
        </div>
      </header>

      <div className="fleet-manager__content">
        <section className="fleet-manager__heading">
          <div>
            <p className="eyebrow">DEVICE REGISTRY · LIVE STATUS</p>
            <h2>Danh sách robot</h2>
            <p>Đăng ký thiết bị, theo dõi kết nối và mở phiên điều khiển.</p>
          </div>
          <button type="button" className="button button--primary" onClick={() => navigate("/robots/new")}>
            <Plus size={19} /> Thêm robot
          </button>
        </section>

        <section className="fleet-toolbar" aria-label="Bộ lọc robot">
          <label className="fleet-search">
            <Search size={17} />
            <input
              value={search}
              onChange={(event) => {
                setSearch(event.target.value);
                setPage(1);
              }}
              placeholder="Tìm theo mã, tên hoặc khu vực…"
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
              <option value="all">Tất cả trạng thái</option>
              <option value="online">Đang online</option>
              <option value="offline">Ngoại tuyến</option>
              <option value="pending">Chờ robot chạy</option>
            </select>
          </label>
          <div className="fleet-toolbar__metrics">
            <span><strong>{summary.available}</strong> sẵn sàng</span>
            <span><strong>{summary.online}</strong> đang kết nối</span>
          </div>
        </section>

        {error && <div role="alert" className="notice notice--error">{error}</div>}

        <section className="managed-robot-grid" aria-label="Danh sách robot">
          {robotsQuery.isLoading ? (
            Array.from({ length: 6 }, (_, index) => (
              <div className="managed-robot-card robot-card--loading" key={index} />
            ))
          ) : robots.length === 0 ? (
            <div className="fleet-empty">
              <span><Bot size={34} /></span>
              <h3>Chưa có robot phù hợp</h3>
              <p>Thay đổi bộ lọc hoặc đăng ký robot đầu tiên.</p>
              <button type="button" className="button button--primary" onClick={() => navigate("/robots/new")}>
                <Plus size={18} /> Thêm robot
              </button>
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
                  <button
                    type="button"
                    className="robot-card__settings"
                    aria-label={`Sửa ${robot.name}`}
                    onClick={() => navigate(`/robots/${robot.robot_id}/edit`)}
                  >
                    <Settings2 size={17} />
                  </button>
                </div>

                <div className="managed-robot-card__identity">
                  <span className="robot-avatar"><Bot size={27} /></span>
                  <span>
                    <small>{robot.robot_id}</small>
                    <strong>{robot.name}</strong>
                    <em><MapPin size={13} /> {robot.site_id}</em>
                  </span>
                </div>

                <div className="managed-robot-card__telemetry">
                  <span><Wifi size={15} /><small>Độ trễ</small><strong>{robot.status === "online" ? `${robot.network_rtt_ms} ms` : "—"}</strong></span>
                  <span><Battery size={15} /><small>Pin</small><strong>{robot.status === "online" ? `${Math.round(robot.battery_percent)}%` : "—"}</strong></span>
                  <span><Clock3 size={15} /><small>Cập nhật</small><strong>{robot.last_seen_at ? new Date(robot.last_seen_at).toLocaleTimeString("vi-VN", { hour: "2-digit", minute: "2-digit" }) : "Chưa có"}</strong></span>
                </div>

                <div className="managed-robot-card__actions">
                  <button
                    type="button"
                    className="button button--outline"
                    aria-label={`Cấu hình ${robot.name}`}
                    disabled={robot.enrollment_status === "pending"}
                    onClick={() => navigate(`/robots/${robot.robot_id}/configuration`)}
                  >
                    Cấu hình
                  </button>
                  <button
                    type="button"
                    className="button button--primary"
                    disabled={!canConnect || Boolean(connectingId)}
                    onClick={() => void connect(robot)}
                  >
                    <PlugZap size={17} />
                    {connectingId === robot.robot_id ? "Đang kết nối…" : "Kết nối"}
                  </button>
                </div>
              </article>
            );
          })}
        </section>

        <footer className="fleet-pagination">
          <span>
            Hiển thị {robots.length} trong tổng số {robotsQuery.data?.total ?? 0} robot
          </span>
          <div>
            <button
              type="button"
              aria-label="Trang trước"
              disabled={page <= 1}
              onClick={() => setPage((value) => Math.max(1, value - 1))}
            >
              <ChevronLeft size={18} />
            </button>
            <strong>{page} / {robotsQuery.data?.total_pages ?? 1}</strong>
            <button
              type="button"
              aria-label="Trang sau"
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
