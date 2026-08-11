import { useEffect, useState } from "react";
import { useMutation } from "@tanstack/react-query";
import { CircleStop, RotateCcw, Save, Trash2 } from "lucide-react";
import { api } from "../api/client";
import { useI18n } from "../i18n/I18nProvider";
import type { Health, MappingSession } from "../types";
import { createUuid } from "../utils/uuid";

type Translate = ReturnType<typeof useI18n>["t"];

interface Props {
  robotId: string;
  health: Health;
  expectedState?: string;
  disabled?: boolean;
  onMappingChanged?: (mapping: MappingSession | null) => void;
}

const TERMINAL = new Set(["FINISHED", "CANCELED", "MAPPING_ERROR", "FAULT"]);

const MAPPING_STATE_LABELS: Record<string, string> = {
  IDLE: "Chưa bắt đầu",
  MAPPING_STARTING: "Đang khởi động mapping",
  STARTING: "Đang khởi động mapping",
  MAPPING_RUNNING: "Đang mapping",
  MAPPING: "Đang mapping",
  MAPPING_STOPPED_UNSAVED: "Đã dừng, chưa lưu",
  PAUSED: "Đã tạm dừng",
  MAPPING_SAVING: "Đang lưu bản đồ",
  SAVING: "Đang lưu bản đồ",
  SAVED_DRAFT: "Đã lưu bản nháp",
  FINISHING: "Đang hoàn tất",
  FINISHED: "Đã hoàn tất",
  CANCELED: "Đã hủy",
  MAPPING_ERROR: "Mapping gặp lỗi",
  FAULT: "Mapping gặp lỗi",
};

const LOCAL_STATUS_LABELS: Record<string, string> = {
  AVAILABLE: "Có sẵn trên robot",
  MISSING: "Không có trên robot",
  LOCAL_ONLY: "Chỉ có trên robot",
};

const SYNC_STATUS_LABELS: Record<string, string> = {
  SYNCED: "Đã đồng bộ",
  SYNC_PENDING: "Chờ đồng bộ",
  LOCAL_ONLY: "Chưa đồng bộ",
  DELETION_PENDING: "Chờ xóa đồng bộ",
  DELETED: "Đã xóa đồng bộ",
  SYNC_FAILED: "Đồng bộ thất bại",
};

function statusLabel(status: string, labels: Record<string, string>, t: Translate): string {
  return t(labels[status] ?? status);
}

export function MappingControlPanel({ robotId, health, expectedState = "IDLE", disabled = false, onMappingChanged }: Props) {
  const { t } = useI18n();
  const [mapping, setMapping] = useState<MappingSession | null>(null);
  const [name, setName] = useState("");
  const [siteId, setSiteId] = useState("");
  const [floorId, setFloorId] = useState("");
  const [notes, setNotes] = useState("");
  const [error, setError] = useState("");
  const [continuation, setContinuation] = useState<{ map_id?: string; source_version?: number }>({});

  useEffect(() => onMappingChanged?.(mapping), [mapping, onMappingChanged]);
  useEffect(() => {
    try {
      const raw = sessionStorage.getItem("rovera:mapping-intent");
      if (!raw) return;
      const intent = JSON.parse(raw) as {
        map_id?: string; source_version?: number; session_id?: string;
        name?: string; site_id?: string; floor_id?: string; notes?: string;
      };
      sessionStorage.removeItem("rovera:mapping-intent");
      setContinuation({ map_id: intent.map_id, source_version: intent.source_version });
      setName(intent.name ?? "");
      setSiteId(intent.site_id ?? "");
      setFloorId(intent.floor_id ?? "");
      setNotes(intent.notes ?? "");
      if (intent.session_id) {
        void api.mappingSession(intent.session_id).then((session) => {
          if (session.robot_id === robotId) {
            setMapping(session);
            setName(session.metadata.name);
            setSiteId(session.metadata.site_id);
            setFloorId(session.metadata.floor_id);
          }
        }).catch((reason) => setError(reason instanceof Error ? reason.message : t("Không tải được phiên mapping")));
      }
    } catch {
      sessionStorage.removeItem("rovera:mapping-intent");
    }
  }, [robotId, t]);

  const start = useMutation({
    mutationFn: () => api.startMapping({
      request_id: createUuid(),
      robot_id: robotId,
      expected_state: expectedState,
      name,
      site_id: siteId,
      floor_id: floorId,
      notes,
      ...continuation,
    }),
    onMutate: () => setError(""),
    onSuccess: setMapping,
    onError: (reason) => setError(reason instanceof Error ? reason.message : t("SLAM không khởi động được.")),
  });
  const action = useMutation({
    mutationFn: (actionName: "stop" | "save" | "discard") => api.mappingAction(
      mapping!.session_id,
      actionName,
      createUuid(),
      mapping!.status,
    ),
    onMutate: () => setError(""),
    onSuccess: setMapping,
    onError: (reason) => setError(reason instanceof Error ? reason.message : t("Lệnh mapping thất bại")),
  });

  const mappingHealth = health.mapping;
  const state = mapping?.status ?? "IDLE";
  const running = state === "MAPPING_RUNNING";
  const stopped = state === "MAPPING_STOPPED_UNSAVED";
  const terminal = TERMINAL.has(state);
  const elapsed = Number(mappingHealth?.elapsedSeconds ?? 0);
  const elapsedText = [Math.floor(elapsed / 3600), Math.floor(elapsed / 60) % 60, elapsed % 60]
    .map((value) => String(value).padStart(2, "0")).join(":");
  const requiresRosHealth = health.motion_backend === "ros2" || health.navigation_backend === "ros2";
  // IDLE intentionally stops both Nav2 and SLAM. Starting mapping is the
  // operation that brings SLAM up, where the edge performs the real preflight.
  const startsFreshRuntime = health.mode === "IDLE";
  const startBlockers = requiresRosHealth && !startsFreshRuntime ? [
    !health.scan_fresh && t("Không nhận được LiDAR."),
    !health.odometry_ready && t("Odometry không hoạt động."),
    !health.lidar_tf_ready && t("TF không hợp lệ."),
    health.safety !== "HEALTHY" && t("Motion safety chưa sẵn sàng."),
    health.estop && t("E-stop đang bật."),
  ].filter(Boolean) as string[] : [];

  return <section className="mapping-control-panel" aria-label={t("Tạo bản đồ")}> 
    <header>
      <div><p className="eyebrow">{t("SLAM Toolbox · RViz2")}</p><h2>{t("Tạo bản đồ")}</h2></div>
      <strong className={`map-status map-status--${state.toLowerCase()}`}>{statusLabel(state, MAPPING_STATE_LABELS, t)}</strong>
    </header>
    {!mapping || terminal ? <form onSubmit={(event) => { event.preventDefault(); start.mutate(); }}>
      <label><span>{t("Tên map")}</span><input required minLength={2} value={name} onChange={(event) => setName(event.target.value)} /></label>
      <div className="mapping-control-panel__row">
        <label><span>{t("Site / tòa nhà")}</span><input required value={siteId} onChange={(event) => setSiteId(event.target.value)} /></label>
        <label><span>{t("Tầng")}</span><input required value={floorId} onChange={(event) => setFloorId(event.target.value)} /></label>
      </div>
      <button className="button button--primary" type="submit" title={startBlockers.join(" ")}
        disabled={disabled || start.isPending || startBlockers.length > 0}>
        <RotateCcw size={16} /> {start.isPending ? t("Đang khởi động…") : t("Bắt đầu mapping")}
      </button>
      {startBlockers.length > 0 && <small className="mapping-start-blockers">{startBlockers.join(" ")}</small>}
    </form> : <>
      <div className="mapping-health-grid">
        <span>{t("LiDAR")} <i className={mappingHealth?.scanHealthy ? "is-ok" : "is-fault"} /> {mappingHealth?.scanHealthy ? t("Tốt") : t("Lỗi")}</span>
        <span>{t("Odometry")} <i className={mappingHealth?.odomHealthy ? "is-ok" : "is-fault"} /> {mappingHealth?.odomHealthy ? t("Tốt") : t("Lỗi")}</span>
        <span>{t("TF")} <i className={mappingHealth?.tfHealthy ? "is-ok" : "is-fault"} /> {mappingHealth?.tfHealthy ? t("Tốt") : t("Lỗi")}</span>
        <span>{t("SLAM")} <i className={mappingHealth?.slamHealthy ? "is-ok" : "is-fault"} /> {mappingHealth?.slamHealthy ? t("Đang chạy") : t("Lỗi")}</span>
      </div>
      <div className="mapping-timer"><small>{t("Thời gian")}</small><strong>{elapsedText}</strong></div>
      <p className="mapping-rviz-note">{t("Mở RViz2 trên máy Ubuntu kỹ thuật để xem /map, /scan, TF và odometry realtime.")}</p>
      <div className="mapping-control-panel__actions">
        {running && <button type="button" onClick={() => action.mutate("stop")} disabled={action.isPending}><CircleStop /> {t("Dừng mapping")}</button>}
        {stopped && <button type="button" className="button--primary" onClick={() => action.mutate("save")} disabled={action.isPending}><Save /> {t("Lưu bản đồ")}</button>}
        {stopped && <button type="button" className="is-danger" onClick={() => action.mutate("discard")} disabled={action.isPending}><Trash2 /> {t("Hủy bản nháp")}</button>}
      </div>
      <small>{t("Dữ liệu cục bộ")}: {statusLabel(mapping.local_status, LOCAL_STATUS_LABELS, t)} · {t("Đồng bộ")}: {statusLabel(mapping.sync_status, SYNC_STATUS_LABELS, t)}</small>
    </>}
    {error && <p className="navigation-inline-error" role="alert">{t(error)}</p>}
  </section>;
}
