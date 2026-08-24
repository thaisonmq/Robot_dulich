import { useEffect, useMemo, useState } from "react";
import { useQuery } from "@tanstack/react-query";
import {
  AlertTriangle, ArrowLeft, ArrowRight, Battery, Bot, Check, LoaderCircle,
  MapPinned, RadioTower, RefreshCw, Search, Wifi,
} from "lucide-react";
import { api } from "../api/client";
import { MappingPosePicker } from "../components/MappingPosePicker";
import { OperationsShell } from "../components/OperationsShell";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";
import type { MapData, MappingInitialPose, Robot } from "../types";

interface MappingIntent {
  map_id?: string;
  source_version?: number;
  session_id?: string;
  robot_id?: string;
  name?: string;
  site_id?: string;
  floor_id?: string;
  notes?: string;
  initial_pose?: MappingInitialPose;
  initial_pose_confirmed?: boolean;
}

function readMappingIntent(): MappingIntent {
  try {
    return JSON.parse(sessionStorage.getItem("rovera:mapping-intent") ?? "{}") as MappingIntent;
  } catch {
    sessionStorage.removeItem("rovera:mapping-intent");
    return {};
  }
}

function canStartMapping(robot: Robot): boolean {
  return robot.enabled
    && robot.enrollment_status === "enrolled"
    && robot.status === "online"
    && robot.availability === "available"
    && robot.capabilities.mapping !== false
    && !robot.capabilities.mapping_blockers?.length;
}

function robotStatus(robot: Robot, t: (source: string) => string): string {
  if (!robot.enabled) return t("Đã vô hiệu hóa");
  if (robot.enrollment_status !== "enrolled") return t("Chưa hoàn tất đăng ký");
  if (robot.status !== "online") return t("Đang offline");
  if (robot.availability === "busy") return t("Đang có phiên điều khiển");
  if (robot.capabilities.mapping === false) return t("Không hỗ trợ mapping");
  if (robot.capabilities.mapping_blockers?.length) return robot.capabilities.mapping_blockers.join(" · ");
  return t("Sẵn sàng mapping");
}

export function CreateMapPage() {
  const { t } = useI18n();
  const navigate = useNavigate();
  const selectRobot = useAppStore((state) => state.selectRobot);
  const setSession = useAppStore((state) => state.setSession);
  const setConnectionState = useAppStore((state) => state.setConnectionState);
  const [intent] = useState(readMappingIntent);
  const [selectedRobotId, setSelectedRobotId] = useState(intent.robot_id ?? "");
  const [name, setName] = useState(intent.name ?? "");
  const [siteId, setSiteId] = useState(intent.site_id ?? "");
  const [floorId, setFloorId] = useState(intent.floor_id ?? "");
  const [notes, setNotes] = useState(intent.notes ?? "");
  const [initialPose, setInitialPose] = useState<MappingInitialPose | null>(intent.initial_pose ?? null);
  const [initialPoseConfirmed, setInitialPoseConfirmed] = useState(Boolean(intent.initial_pose_confirmed));
  const [search, setSearch] = useState("");
  const [connecting, setConnecting] = useState(false);
  const [error, setError] = useState("");

  const robotsQuery = useQuery({
    queryKey: ["mapping-robots"],
    queryFn: () => api.robots({ page: 1, pageSize: 50, status: "all" }),
    refetchInterval: 5000,
  });
  const mappingQuery = useQuery({
    queryKey: ["mapping-session", intent.session_id],
    queryFn: () => api.mappingSession(intent.session_id!),
    enabled: Boolean(intent.session_id),
    retry: 1,
  });
  const sourceMapQuery = useQuery({
    queryKey: ["mapping-source-map", intent.map_id],
    queryFn: () => api.map(intent.map_id!),
    enabled: Boolean(intent.map_id && !intent.session_id),
    retry: 1,
  });
  const robots = robotsQuery.data?.items ?? [];
  const visibleRobots = useMemo(() => {
    const query = search.trim().toLocaleLowerCase();
    return robots.filter((robot) => !query || [robot.name, robot.robot_id, robot.site_id]
      .some((value) => value.toLocaleLowerCase().includes(query)));
  }, [robots, search]);
  const selectedRobot = robots.find((robot) => robot.robot_id === selectedRobotId);
  const isContinuation = Boolean(intent.map_id || intent.session_id);
  const requiresInitialPose = Boolean(intent.map_id && !intent.session_id);
  const continuationMap = useMemo<MapData | null>(() => {
    const detail = sourceMapQuery.data;
    if (!detail) return null;
    const version = detail.versions?.find((item) => item.version === intent.source_version);
    if (!version) return null;
    return {
      ...detail,
      active_version: version.version,
      image_url: version.preview_url,
      width_pixels: version.width_pixels,
      height_pixels: version.height_pixels,
      resolution_m_per_pixel: version.resolution,
      origin: version.origin,
    };
  }, [intent.source_version, sourceMapQuery.data]);
  const initialPoseReady = !requiresInitialPose
    || Boolean(continuationMap && initialPose && initialPoseConfirmed);

  useEffect(() => {
    const mapping = mappingQuery.data;
    if (!mapping) return;
    setSelectedRobotId(mapping.robot_id);
    setName(mapping.metadata.name);
    setSiteId(mapping.metadata.site_id);
    setFloorId(mapping.metadata.floor_id);
  }, [mappingQuery.data]);

  useEffect(() => {
    if (selectedRobotId || robots.length !== 1 || !canStartMapping(robots[0])) return;
    setSelectedRobotId(robots[0].robot_id);
  }, [robots, selectedRobotId]);

  const openControl = async () => {
    if (!selectedRobot || !canStartMapping(selectedRobot)) return;
    if (!initialPoseReady) {
      setError(t("Hãy chọn vùng và hướng gần đúng của robot trên map cũ."));
      return;
    }
    setConnecting(true);
    setError("");
    const nextIntent: MappingIntent = {
      ...intent,
      robot_id: selectedRobot.robot_id,
      name: name.trim(),
      site_id: siteId.trim(),
      floor_id: floorId.trim(),
      notes: notes.trim(),
      ...(requiresInitialPose && initialPose ? {
        initial_pose: initialPose,
        initial_pose_confirmed: true,
      } : {}),
    };
    sessionStorage.setItem("rovera:mapping-intent", JSON.stringify(nextIntent));
    selectRobot(selectedRobot);
    setSession(null);
    setConnectionState("connecting");
    try {
      const controlSession = await api.createSession(selectedRobot.robot_id);
      setSession(controlSession);
      setConnectionState("connected");
      navigate(`/control/${selectedRobot.robot_id}`);
    } catch (reason) {
      setConnectionState(selectedRobot.status === "offline" ? "offline" : "error");
      setError(reason instanceof Error ? reason.message : t("Không thể kết nối robot"));
    } finally {
      setConnecting(false);
    }
  };

  return <OperationsShell title="Tạo bản đồ" className="maps-page map-create-page">
    <div className="maps-shell map-create-shell">
      <div className="map-create-topline">
        <button type="button" className="map-back-link" onClick={() => navigate("/maps")}>
          <ArrowLeft size={18} /> {t("Quay lại thư viện bản đồ")}
        </button>
        <ol className="map-create-steps" aria-label={t("Tiến trình tạo bản đồ")}>
          <li className="is-active"><span>1</span> {t("Thiết lập")}</li>
          <li><span>2</span> {t("Mapping trên Control")}</li>
          <li><span>3</span> {t("Lưu phiên bản")}</li>
        </ol>
      </div>

      <section className="map-create-heading">
        <div>
          <h2>{t(isContinuation ? "Chọn robot để tiếp tục mapping" : "Thiết lập phiên mapping")}</h2>
        </div>
        <span className="map-create-mode"><RadioTower size={18} /> SLAM Toolbox</span>
      </section>

      <form className="map-create-workspace" onSubmit={(event) => { event.preventDefault(); void openControl(); }}>
        <section className="map-create-card map-robot-picker">
          <header>
            <span className="map-create-card__number">01</span>
            <div><h3>{t("Chọn robot")}</h3><p>{t("Chỉ robot online, rảnh và hỗ trợ mapping mới có thể bắt đầu.")}</p></div>
          </header>
          <label className="map-robot-search"><Search size={18} /><input value={search}
            onChange={(event) => setSearch(event.target.value)} placeholder={t("Tìm tên, mã robot hoặc khu vực…")} /></label>

          <div className="map-robot-list" role="radiogroup" aria-label={t("Robot dùng để mapping")}>
            {robotsQuery.isLoading ? [0, 1, 2].map((item) => <span className="map-robot-skeleton" key={item} />)
              : robotsQuery.isError ? <div className="map-robot-message is-error"><AlertTriangle /><div><strong>{t("Không tải được robot")}</strong><p>{t("Kiểm tra kết nối Center rồi thử lại.")}</p></div><button type="button" onClick={() => void robotsQuery.refetch()}><RefreshCw size={16} /> {t("Thử lại")}</button></div>
                : visibleRobots.length ? visibleRobots.map((robot) => {
                  const ready = canStartMapping(robot);
                  const selected = selectedRobotId === robot.robot_id;
                  return <button type="button" role="radio" aria-checked={selected} key={robot.robot_id}
                    className={`map-robot-option${selected ? " is-selected" : ""}`} disabled={!ready}
                    onClick={() => { setSelectedRobotId(robot.robot_id); setError(""); }}>
                    <span className="map-robot-option__icon"><Bot size={24} /></span>
                    <span className="map-robot-option__identity"><strong>{robot.name}</strong><small>{robot.robot_id} · {robot.site_id === "Chưa phân khu" ? t("Chưa phân khu") : robot.site_id || t("Chưa đặt khu vực")}</small></span>
                    <span className="map-robot-option__telemetry"><small><Battery size={14} /> {Math.round(robot.battery_percent)}%</small><small><Wifi size={14} /> {robot.status === "online" ? `${robot.network_rtt_ms} ms` : "—"}</small></span>
                    <span className={`map-robot-option__status${ready ? " is-ready" : ""}`}>{robotStatus(robot, t)}</span>
                    <span className="map-robot-option__check">{selected && <Check size={16} />}</span>
                  </button>;
                }) : <div className="map-robot-message"><Bot /><div><strong>{t("Không có robot phù hợp")}</strong><p>{t("Đổi từ khóa hoặc bật robot cần dùng để mapping.")}</p></div></div>}
          </div>
        </section>

        <section className="map-create-card map-create-metadata">
          <header>
            <span className="map-create-card__number">02</span>
            <div><h3>{t("Thông tin bản đồ")}</h3><p>{t("Dùng tên và vị trí dễ nhận biết khi vận hành.")}</p></div>
          </header>
          <label><span>{t("Tên bản đồ")}</span><input required minLength={2} value={name} onChange={(event) => setName(event.target.value)} placeholder={t("Ví dụ: Sảnh trưng bày tầng 1")} /><small>{t("Tên hiển thị trong thư viện và màn Control.")}</small></label>
          <div className="map-create-field-row">
            <label><span>{t("Site / tòa nhà")}</span><input required value={siteId} onChange={(event) => setSiteId(event.target.value)} placeholder="MQ ICT Solutions" /></label>
            <label><span>{t("Tầng / khu vực")}</span><input required value={floorId} onChange={(event) => setFloorId(event.target.value)} placeholder={t("Tầng 1")} /></label>
          </div>
          <label><span>{t("Ghi chú")} <em>{t("không bắt buộc")}</em></span><textarea value={notes} onChange={(event) => setNotes(event.target.value)} placeholder={t("Mô tả phạm vi mapping, lối đi cần kiểm tra…")} /></label>

          {requiresInitialPose && <section className="mapping-current-pose" aria-labelledby="mapping-current-pose-title">
            <header>
              <div><strong id="mapping-current-pose-title">{t("Chỉ vùng robot đang đứng gần đó")}</strong>
                <p>{t("Không cần đặt chính xác. Vị trí và hướng chỉ là gợi ý ban đầu; SLAM phải tự khớp và xác minh trước khi cập nhật map.")}</p></div>
              <span>v{intent.source_version}</span>
            </header>
            {sourceMapQuery.isLoading && <p className="mapping-current-pose__message">{t("Đang tải Saved Map…")}</p>}
            {sourceMapQuery.isError && <p className="mapping-current-pose__message is-error" role="alert">{t("Không tải được Saved Map")}</p>}
            {!sourceMapQuery.isLoading && !sourceMapQuery.isError && !continuationMap
              && <p className="mapping-current-pose__message is-error" role="alert">{t("Không tìm thấy version map cần tiếp tục.")}</p>}
            {continuationMap && <MappingPosePicker map={continuationMap} value={initialPose}
              onChange={(pose) => { setInitialPose(pose); setInitialPoseConfirmed(false); setError(""); }} />}
            <label className="mapping-current-pose__confirm">
              <input type="checkbox" checked={initialPoseConfirmed} disabled={!initialPose}
                onChange={(event) => { setInitialPoseConfirmed(event.target.checked); setError(""); }} />
              <span>{t("Tôi đã chọn vùng và hướng gần đúng; robot đang đứng yên để SLAM xác minh.")}</span>
            </label>
          </section>}

          <aside className="map-create-safety-note">
            <MapPinned size={20} />
            <div><strong>{t("Trước khi bắt đầu")}</strong><p>{t("Đặt robot tại vị trí an toàn, kiểm tra LiDAR và đảm bảo E-stop đã nhả. Bạn vẫn điều khiển robot trực tiếp trong Control.")}</p></div>
          </aside>

          {error && <p className="map-create-error" role="alert"><AlertTriangle size={17} /> {error}</p>}
          <footer>
            <div>{selectedRobot ? <><strong>{selectedRobot.name}</strong><small>{robotStatus(selectedRobot, t)}</small></> : <><strong>{t("Chưa chọn robot")}</strong><small>{t("Chọn một robot sẵn sàng ở danh sách bên trái.")}</small></>}</div>
            <button type="submit" className="button button--primary" disabled={!selectedRobot || !canStartMapping(selectedRobot) || connecting || !initialPoseReady}>
              {connecting ? <LoaderCircle className="is-spinning" size={18} /> : <ArrowRight size={18} />}
              {t(connecting ? "Đang mở Control…" : "Mở Control để mapping")}
            </button>
          </footer>
        </section>
      </form>
    </div>
  </OperationsShell>;
}
