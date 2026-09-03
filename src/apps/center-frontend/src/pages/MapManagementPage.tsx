import { useEffect, useMemo, useRef, useState } from "react";
import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";
import {
  AlertTriangle, Archive, ArrowLeft, CheckCircle2, ChevronLeft, ChevronRight, Database,
  Download, FilePenLine, HardDrive, Layers3, MapPinned, Pencil, PlayCircle,
  Plus, RefreshCw, Save, Search, Settings2, Trash2,
} from "lucide-react";
import { api, authenticatedAsset } from "../api/client";
import { OperationsShell } from "../components/OperationsShell";
import { showToast } from "../components/ToastViewport";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate, usePathname } from "../router";
import { useAppStore } from "../state/appStore";
import type { MapData } from "../types";
import { hasPermission } from "../utils/permissions";
import { createUuid } from "../utils/uuid";

type MapFilter = "ALL" | "ACTIVE" | "PENDING" | "ARCHIVED";
type MapDetailTab = "OVERVIEW" | "VERSIONS" | "SETTINGS";
const MAPS_PER_PAGE = 4;

function formatDate(value: string | undefined, language: string): string {
  return value ? new Intl.DateTimeFormat(language, {
    dateStyle: "medium",
    timeStyle: "short",
  }).format(new Date(value)) : "—";
}

function statusLabel(status: string | undefined, t: (source: string) => string): string {
  const label = ({
    ACTIVE: "Sẵn sàng",
    DRAFT: "Bản nháp",
    SYNC_PENDING: "Chờ đồng bộ",
    VALIDATING: "Đang xác minh",
    ARCHIVED: "Đã lưu trữ",
    DELETED: "Đã xóa",
  } as Record<string, string>)[status ?? ""];
  return label ? t(label) : status ?? t("Chưa xác định");
}

function dataStatusLabel(status: string | undefined, t: (source: string) => string): string {
  const label = ({
    AVAILABLE: "Có sẵn",
    MISSING: "Bị thiếu",
    LOCAL_ONLY: "Chỉ có ở robot",
    SYNCED: "Đã đồng bộ",
    SYNC_PENDING: "Chờ đồng bộ",
    UPLOADING: "Đang tải lên",
    FAILED: "Đồng bộ lỗi",
    SYNC_FAILED: "Đồng bộ thất bại",
    RUNNING: "Đang chạy",
    MAPPING_RUNNING: "Đang mapping",
    MAPPING_LOCALIZING: "Đang khớp vị trí",
  } as Record<string, string>)[status ?? ""];
  return label ? t(label) : status ?? t("Chưa xác định");
}

function isActivatedMap(map: MapData): boolean {
  return map.active_version != null || map.active_status === "ACTIVE";
}

function MapPreviewImage({ url, alt }: { url: string; alt: string }) {
  const { t } = useI18n();
  const [source, setSource] = useState(url.startsWith("/api/") ? "" : url);
  const [failed, setFailed] = useState(false);

  useEffect(() => {
    let active = true;
    let objectUrl = "";
    setFailed(false);
    if (!url.startsWith("/api/")) {
      setSource(url);
      return () => { active = false; };
    }
    setSource("");
    void authenticatedAsset(url).then((blob) => {
      if (!active) return;
      objectUrl = URL.createObjectURL(blob);
      setSource(objectUrl);
    }).catch(() => {
      if (active) setFailed(true);
    });
    return () => {
      active = false;
      if (objectUrl) URL.revokeObjectURL(objectUrl);
    };
  }, [url]);

  if (source) return <img src={source} alt={alt} />;
  return <span className={`map-preview-placeholder${failed ? " is-failed" : ""}`}>
    <MapPinned size={30} />
    <small>{failed ? t("Không đọc được preview") : t("Đang tải preview")}</small>
  </span>;
}

function RegistrySkeleton() {
  const { t } = useI18n();
  return <div className="map-registry-skeleton" aria-label={t("Đang tải danh sách bản đồ")}>
    {[0, 1, 2].map((item) => <span key={item} />)}
  </div>;
}

export function MapManagementPage() {
  const { language, t } = useI18n();
  const navigate = useNavigate();
  const pathname = usePathname();
  const queryClient = useQueryClient();
  const user = useAppStore((state) => state.user);
  const [detailTab, setDetailTab] = useState<MapDetailTab>("OVERVIEW");
  const [deleteConfirmationOpen, setDeleteConfirmationOpen] = useState(false);
  const [search, setSearch] = useState("");
  const [filter, setFilter] = useState<MapFilter>("ALL");
  const [page, setPage] = useState(1);
  const deleteCancelRef = useRef<HTMLButtonElement>(null);
  const [draft, setDraft] = useState({ name: "", site_id: "", floor_id: "", notes: "" });
  const selectedMapId = pathname.match(/^\/maps\/([^/]+)$/)?.[1];
  const mapsQuery = useQuery({ queryKey: ["maps"], queryFn: () => api.maps() });
  const detailQuery = useQuery({
    queryKey: ["map", selectedMapId],
    queryFn: () => api.map(selectedMapId!),
    enabled: Boolean(selectedMapId),
    refetchInterval: (query) => query.state.data?.versions?.some(
      (version) => version.sync_status === "SYNC_PENDING",
    ) ? 2_000 : false,
  });
  const refreshRegistry = async () => {
    await queryClient.invalidateQueries({ queryKey: ["maps"] });
    if (selectedMapId) await queryClient.invalidateQueries({ queryKey: ["map", selectedMapId] });
  };
  const activate = useMutation({
    mutationFn: ({ mapId, version }: { mapId: string; version: number }) => api.activateMap(mapId, version),
    onSuccess: async () => {
      await refreshRegistry();
      showToast(t("Đã kích hoạt"));
    },
  });
  const archive = useMutation({
    mutationFn: api.archiveMap,
    onSuccess: async () => {
      await refreshRegistry();
      showToast(t("Đã lưu trữ"));
    },
  });
  const resync = useMutation({
    mutationFn: ({ mapId, version }: { mapId: string; version: number }) => api.resyncMapVersion(mapId, version),
    onSuccess: async (result) => {
      await refreshRegistry();
      showToast(
        t(result.sync_status === "SYNCED" ? "Đã đồng bộ" : "Chờ đồng bộ"),
        result.sync_status === "SYNCED" ? "success" : "info",
      );
    },
  });
  const update = useMutation({
    mutationFn: () => api.updateMap(selectedMapId!, draft),
    onSuccess: async () => {
      await refreshRegistry();
      showToast(t("Đã lưu thay đổi"));
    },
  });
  const remove = useMutation({
    // The DELETE endpoint owns the complete lifecycle: stop mapping/Nav2,
    // deactivate map_server/localization, clear robot refs and tombstone.
    mutationFn: (map: MapData) => api.deleteMap(map.map_id),
    onSuccess: async () => {
      setDeleteConfirmationOpen(false);
      await queryClient.invalidateQueries({ queryKey: ["maps"] });
      showToast(t("Đã xóa"));
      navigate("/maps");
    },
  });
  const recoverMapping = useMutation({
    mutationFn: (session: NonNullable<MapData["recoverable_mapping_session"]>) =>
      api.mappingAction(session.session_id, "recover", createUuid(), "IDLE"),
    onSuccess: async (session) => {
      sessionStorage.setItem("rovera:mapping-intent", JSON.stringify({
        map_id: session.map_id,
        session_id: session.session_id,
        robot_id: session.robot_id,
        name: session.metadata.name,
        site_id: session.metadata.site_id,
        floor_id: session.metadata.floor_id,
        notes: session.metadata.notes,
      }));
      await refreshRegistry();
      showToast(t("Đã khôi phục autosave"));
      navigate("/maps/create");
    },
  });
  const canOperate = hasPermission(user, "maps.manage");
  const maps = mapsQuery.data ?? [];
  const detail = detailQuery.data;
  const activeMapping = detail?.mapping_session;
  const recoverableMapping = detail?.recoverable_mapping_session;
  const continuableVersion = detail?.versions?.find((version) => version.can_continue);
  const activeCount = maps.filter(isActivatedMap).length;
  const pendingCount = maps.filter((item) => ["SYNC_PENDING", "VALIDATING"].includes(item.status ?? "")
    || item.sync_status === "SYNC_PENDING").length;
  const localCount = maps.filter((item) => item.local_status === "AVAILABLE").length;
  const visibleMaps = useMemo(() => {
    const query = search.trim().toLocaleLowerCase();
    return maps.filter((item) => {
      const matchesSearch = !query || [item.name, item.map_id, item.site_id, item.floor_id]
        .some((value) => value?.toLocaleLowerCase().includes(query));
      const matchesFilter = filter === "ALL"
        || (filter === "ACTIVE" && isActivatedMap(item))
        || (filter === "PENDING" && (["SYNC_PENDING", "VALIDATING"].includes(item.status ?? "")
          || item.sync_status === "SYNC_PENDING"))
        || (filter === "ARCHIVED" && item.status === "ARCHIVED");
      return matchesSearch && matchesFilter;
    });
  }, [filter, maps, search]);
  const totalPages = Math.max(1, Math.ceil(visibleMaps.length / MAPS_PER_PAGE));
  const paginatedMaps = useMemo(
    () => visibleMaps.slice((page - 1) * MAPS_PER_PAGE, page * MAPS_PER_PAGE),
    [page, visibleMaps],
  );
  const operationError = activate.error ?? update.error ?? remove.error ?? archive.error
    ?? resync.error ?? recoverMapping.error;

  useEffect(() => {
    setPage((current) => Math.min(current, totalPages));
  }, [totalPages]);

  useEffect(() => {
    if (!detail) return;
    setDraft({
      name: detail.name,
      site_id: detail.site_id ?? "",
      floor_id: detail.floor_id ?? "",
      notes: detail.notes ?? "",
    });
    setDetailTab("OVERVIEW");
    setDeleteConfirmationOpen(false);
  }, [detail?.map_id]);

  useEffect(() => {
    if (!deleteConfirmationOpen) return;
    const focusFrame = window.requestAnimationFrame(() => deleteCancelRef.current?.focus());
    const closeOnEscape = (event: KeyboardEvent) => {
      if (event.key === "Escape" && !remove.isPending) setDeleteConfirmationOpen(false);
    };
    window.addEventListener("keydown", closeOnEscape);
    return () => {
      window.cancelAnimationFrame(focusFrame);
      window.removeEventListener("keydown", closeOnEscape);
    };
  }, [deleteConfirmationOpen, remove.isPending]);

  const continueMapping = () => {
    if (!detail) return;
    sessionStorage.setItem("rovera:mapping-intent", JSON.stringify({
      map_id: detail.map_id,
      source_version: continuableVersion?.version,
      session_id: activeMapping?.session_id,
      robot_id: activeMapping?.robot_id,
      name: detail.name,
      site_id: detail.site_id,
      floor_id: detail.floor_id,
      notes: detail.notes,
    }));
    navigate("/maps/create");
  };

  const startNewMapping = () => {
    sessionStorage.setItem("rovera:mapping-intent", "{}");
    navigate("/maps/create");
  };

  const downloadVersion = async (url: string, filename: string) => {
    const objectUrl = URL.createObjectURL(await authenticatedAsset(url));
    const link = document.createElement("a");
    link.href = objectUrl;
    link.download = filename;
    link.click();
    URL.revokeObjectURL(objectUrl);
  };

  const renderRegistry = () => <>
    <section className="map-registry-overview" aria-label={t("Tổng quan thư viện bản đồ")}>
      <div><Database /><span><small>{t("Tổng số bản đồ")}</small><strong>{maps.length}</strong></span></div>
      <div><CheckCircle2 /><span><small>{t("Đã kích hoạt")}</small><strong>{activeCount}</strong></span></div>
      <div><RefreshCw /><span><small>{t("Chờ đồng bộ")}</small><strong>{pendingCount}</strong></span></div>
      <div><HardDrive /><span><small>{t("Có dữ liệu cục bộ")}</small><strong>{localCount}</strong></span></div>
      <label className="map-registry-search"><Search size={17} /><input value={search}
        onChange={(event) => { setSearch(event.target.value); setPage(1); }} placeholder={t("Tìm tên, Map ID, site hoặc tầng…")} /></label>
    </section>
    <section className="map-registry-toolbar">
      <div role="tablist" aria-label={t("Lọc trạng thái bản đồ")}>
        {(["ALL", "ACTIVE", "PENDING", "ARCHIVED"] as MapFilter[]).map((value) => <button
          type="button" role="tab" aria-selected={filter === value} key={value}
          className={filter === value ? "is-active" : ""} onClick={() => { setFilter(value); setPage(1); }}>
          {t({ ALL: "Tất cả", ACTIVE: "Đã kích hoạt", PENDING: "Cần xử lý", ARCHIVED: "Đã lưu trữ" }[value])}
        </button>)}
      </div>
    </section>
    {mapsQuery.isLoading ? <RegistrySkeleton /> : mapsQuery.isError ? <div className="map-registry-error" role="alert">
      <AlertTriangle /><div><strong>{t("Không tải được thư viện bản đồ")}</strong><p>{mapsQuery.error instanceof Error ? mapsQuery.error.message : t("Center không phản hồi.")}</p></div>
      <button type="button" onClick={() => void mapsQuery.refetch()}>{t("Thử lại")}</button>
    </div> : <section className="map-registry-list" aria-label={t("Danh sách bản đồ")}>
      <header><span>{t("Bản đồ")}</span><span>{t("Thông số")}</span><span>{t("Lưu trữ")}</span><span>{t("Trạng thái")}</span><span /></header>
      {paginatedMaps.map((item) => <button type="button" key={item.map_id} onClick={() => navigate(`/maps/${item.map_id}`)}>
        <span className="map-card-identity">
          <span className="map-card-preview">{item.image_url
            ? <MapPreviewImage url={item.image_url} alt={t("Bản đồ {name}", { name: item.name })} />
            : <span className="map-preview-placeholder"><MapPinned size={30} /><small>{t("Chưa có ảnh xem trước")}</small></span>}</span>
          <span className="map-card-copy"><small>{item.site_id || t("Chưa đặt site")} / {item.floor_id || t("Chưa đặt tầng")}</small>
            <strong>{item.name}</strong><code>{item.map_id}</code></span>
        </span>
        <span className="map-card-metric"><strong>v{item.active_version ?? "—"}</strong><small>{item.width_pixels} × {item.height_pixels}</small><small>{item.resolution_m_per_pixel} m/px</small></span>
        <span className="map-card-sync"><strong>{dataStatusLabel(item.sync_status ?? "LOCAL_ONLY", t)}</strong><small>{dataStatusLabel(item.local_status ?? "MISSING", t)}</small><small>{formatDate(item.updated_at, language)}</small></span>
        <span><span className={`map-status map-status--${isActivatedMap(item) ? "activated" : item.status?.toLowerCase()}`}>
          {isActivatedMap(item) ? t("Đang kích hoạt · v{version}", { version: item.active_version ?? "—" }) : statusLabel(item.status, t)}
        </span></span>
        <ChevronRight size={19} />
      </button>)}
      {!visibleMaps.length && <div className="map-registry-empty"><MapPinned /><h3>{t(maps.length ? "Không có bản đồ phù hợp" : "Chưa có bản đồ")}</h3>
        <p>{t(maps.length ? "Thử từ khóa hoặc bộ lọc khác." : "Tạo phiên SLAM đầu tiên từ một robot online.")}</p></div>}
    </section>}
    {!mapsQuery.isLoading && !mapsQuery.isError && Boolean(visibleMaps.length) && <footer className="map-registry-pagination">
      <span>{t("{count} kết quả", { count: visibleMaps.length })}</span>
      <div>
        <button type="button" aria-label={t("Trang trước")} disabled={page <= 1}
          onClick={() => setPage((current) => Math.max(1, current - 1))}><ChevronLeft size={18} /></button>
        <strong>{page} / {totalPages}</strong>
        <button type="button" aria-label={t("Trang sau")} disabled={page >= totalPages}
          onClick={() => setPage((current) => Math.min(totalPages, current + 1))}><ChevronRight size={18} /></button>
      </div>
    </footer>}
  </>;

  const renderDetail = () => {
    if (detailQuery.isLoading) return <RegistrySkeleton />;
    if (detailQuery.isError || !detail) return <div className="map-registry-error" role="alert">
      <AlertTriangle /><div><strong>{t("Không tải được thông tin bản đồ")}</strong><p>{detailQuery.error instanceof Error ? detailQuery.error.message : t("Bản đồ không tồn tại hoặc đã bị xóa.")}</p></div>
      <button type="button" onClick={() => navigate("/maps")}>{t("Về danh sách")}</button>
    </div>;
    const isActive = isActivatedMap(detail);
    return <section className="map-detail-view">
      <div className="map-detail-breadcrumb">
        <button type="button" onClick={() => navigate("/maps")}><ArrowLeft size={16} /> {t("Thư viện bản đồ")}</button>
        <span>/</span><strong>{detail.name}</strong>
      </div>
      <nav className="map-detail-tabs" role="tablist" aria-label={t("Các mục thông tin bản đồ")}>
        {([
          ["OVERVIEW", "Tổng quan", <MapPinned size={16} />],
          ["VERSIONS", "Phiên bản", <Layers3 size={16} />],
          ["SETTINGS", "Cài đặt", <Settings2 size={16} />],
        ] as const).map(([value, label, icon]) => <button type="button" role="tab" key={value}
          id={`map-tab-${value.toLocaleLowerCase()}`} aria-controls="map-detail-panel"
          aria-selected={detailTab === value} className={detailTab === value ? "is-active" : ""}
          onClick={() => setDetailTab(value)}>{icon}<span>{t(label)}</span>
          {value === "VERSIONS" && <small>{detail.versions?.length ?? 0}</small>}</button>)}
      </nav>
      <div className="map-detail-tab-panel" role="tabpanel" id="map-detail-panel"
        aria-labelledby={`map-tab-${detailTab.toLocaleLowerCase()}`}>
      {detailTab === "OVERVIEW" && <>
        <div className="map-detail-hero">
          <figure className="map-detail-preview">{detail.image_url
            ? <MapPreviewImage url={detail.image_url} alt={t("Bản đồ đã lưu {name}", { name: detail.name })} />
            : <span className="map-preview-placeholder"><MapPinned size={36} /><small>{t("Phiên bản này chưa có ảnh xem trước")}</small></span>}
            <figcaption><span><Layers3 size={15} /> {t("Bản đồ OccupancyGrid")}</span><code>{detail.width_pixels} × {detail.height_pixels} px</code></figcaption>
          </figure>
          <div className="map-detail-summary">
            <div className="map-detail-summary__status"><span className={`map-status map-status--${isActive ? "activated" : detail.status?.toLowerCase()}`}>
              {isActive ? t("Đang kích hoạt") : statusLabel(detail.status, t)}
            </span>
              {isActive && <span className="map-live-indicator"><i /> {t("ĐANG HOẠT ĐỘNG")} · v{detail.active_version}</span>}</div>
            <p className="map-location">{detail.site_id || t("Chưa đặt site")} / {detail.floor_id || t("Chưa đặt tầng")}</p>
            <h2>{detail.name}</h2>
            <p className="map-detail-notes">{detail.notes || t("Chưa có ghi chú vận hành cho bản đồ này.")}</p>
            <div className="map-detail-identity"><small>MAP ID</small><code>{detail.map_id}</code></div>
            {canOperate && <div className="map-detail-primary-actions">
              {recoverableMapping ? <button type="button" className="button button--primary"
                disabled={recoverMapping.isPending} onClick={() => recoverMapping.mutate(recoverableMapping)}>
                <RefreshCw size={16} /> {t(recoverMapping.isPending ? "Đang khôi phục…" : "Khôi phục & tiếp tục mapping")}</button>
                : <button type="button" className="button button--primary" disabled={!activeMapping && !continuableVersion}
                  title={!activeMapping && !continuableVersion ? t("Phiên bản chưa có posegraph hoàn chỉnh") : undefined}
                  onClick={continueMapping}><Pencil size={16} /> {t(activeMapping ? "Mở phiên mapping" : "Tiếp tục mapping")}</button>}
              <button type="button" className="button" onClick={() => setDetailTab("SETTINGS")}><FilePenLine size={16} /> {t("Sửa thông tin")}</button>
            </div>}
            {canOperate && !activeMapping && !continuableVersion && !recoverableMapping && <p className="map-action-hint">{t("Bản đồ này chưa có posegraph để tiếp tục mapping. Bạn vẫn có thể tạo một phiên mapping mới từ thư viện.")}</p>}
            {recoverableMapping && <p className="map-action-hint">{t("Pi còn autosave của phiên bị gián đoạn. Khôi phục để tiếp tục quét, lưu bản nháp hoặc kết thúc và lưu map.")}</p>}
          </div>
        </div>

        <section className="map-detail-facts" aria-label={t("Thông số bản đồ")}>
          <div><small>{t("Độ phân giải")}</small><strong>{detail.resolution_m_per_pixel} m/px</strong></div>
          <div><small>{t("Kích thước")}</small><strong>{detail.width_pixels} × {detail.height_pixels}</strong></div>
          <div><small>{t("Dữ liệu cục bộ")}</small><strong>{dataStatusLabel(detail.local_status ?? "MISSING", t)}</strong></div>
          <div><small>{t("Đồng bộ Center")}</small><strong>{dataStatusLabel(detail.sync_status ?? "LOCAL_ONLY", t)}</strong></div>
          <div><small>{t("Dữ liệu tiếp tục")}</small><strong>{t(detail.posegraph_available ? "Có posegraph" : "Không có posegraph")}</strong></div>
          <div><small>{t("Cập nhật")}</small><strong>{formatDate(detail.updated_at, language)}</strong></div>
        </section>

        {activeMapping && <div className="map-active-mapping"><PlayCircle /><div><strong>{t("Phiên mapping đang hoạt động")}</strong><span>{activeMapping.robot_id} · {dataStatusLabel(activeMapping.status, t)} · v{activeMapping.version}</span></div><button type="button" className="button button--primary" onClick={continueMapping}>{t("Mở Control")}</button></div>}
      </>}

      {detailTab === "VERSIONS" && <section className="map-version-section">
        <header><div><h3>{t("Các phiên bản bản đồ")}</h3></div><span>{t("{count} phiên bản", { count: detail.versions?.length ?? 0 })}</span></header>
        <div className="map-version-list">
          <header><span>{t("Phiên bản")}</span><span>{t("Dữ liệu bản đồ")}</span><span>{t("Ngày tạo")}</span><span>{t("Thao tác")}</span></header>
          {(detail.versions ?? []).map((version) => {
            const requestingThisVersion = resync.isPending
              && resync.variables?.mapId === detail.map_id
              && resync.variables.version === version.version;
            const waitingForUpload = version.sync_status === "SYNC_PENDING";
            return <article key={version.version} className={version.version === detail.active_version ? "is-active" : ""}>
            <span className="map-version-number"><strong>v{version.version}</strong>{version.version === detail.active_version && <small>{t("ĐANG DÙNG")}</small>}</span>
            <div className="map-version-artifact"><strong>{statusLabel(version.status, t)}</strong><code>{version.checksum.slice(0, 16)}…</code><small>{version.width_pixels} × {version.height_pixels} · {version.resolution} m/px · {version.created_by_robot}</small></div>
            <div className="map-version-date"><strong>{formatDate(version.created_at, language)}</strong><small>{dataStatusLabel(version.local_status, t)} · {dataStatusLabel(version.sync_status, t)}</small><small>{t(version.has_posegraph ? "Có dữ liệu posegraph" : "Thiếu dữ liệu posegraph")}</small></div>
            <div className="map-version-actions">
              <button type="button" title={t("Tải bundle")} onClick={() => void downloadVersion(version.download_url, `${detail.map_id}-v${version.version}.tar.gz`)}><Download size={15} /><span>{t("Tải xuống")}</span></button>
              {canOperate && <button type="button"
                title={t(waitingForUpload ? "Chờ đồng bộ" : "Đồng bộ lại")}
                disabled={requestingThisVersion || waitingForUpload}
                onClick={() => resync.mutate({ mapId: detail.map_id, version: version.version })}>
                <RefreshCw size={15} /><span>{t(requestingThisVersion || waitingForUpload ? "Chờ đồng bộ" : "Đồng bộ lại")}</span></button>}
              {canOperate && version.status === "VALIDATING" && <button type="button" className="is-primary" disabled={activate.isPending} onClick={() => activate.mutate({ mapId: detail.map_id, version: version.version })}><CheckCircle2 size={15} /><span>{t("Kích hoạt")}</span></button>}
            </div>
          </article>;
          })}
          {!detail.versions?.length && <div className="map-version-empty"><Layers3 size={24} /><strong>{t("Chưa có phiên bản đã lưu")}</strong><p>{t("Hoàn tất và lưu một phiên mapping để tạo phiên bản đầu tiên.")}</p></div>}
        </div>
      </section>}

      {detailTab === "SETTINGS" && <div className="map-settings-layout">
        <form className="map-edit-form" onSubmit={(event) => { event.preventDefault(); update.mutate(); }}>
          <header><div><small>{t("THÔNG TIN BẢN ĐỒ")}</small><strong>{t("Chỉnh sửa thông tin vận hành")}</strong></div></header>
          <label><span>{t("Tên bản đồ")}</span><input required minLength={2} value={draft.name} onChange={(event) => setDraft({ ...draft, name: event.target.value })} /></label>
          <label><span>{t("Site / tòa nhà")}</span><input required value={draft.site_id} onChange={(event) => setDraft({ ...draft, site_id: event.target.value })} /></label>
          <label><span>{t("Tầng")}</span><input required value={draft.floor_id} onChange={(event) => setDraft({ ...draft, floor_id: event.target.value })} /></label>
          <label className="map-edit-form__notes"><span>{t("Ghi chú")}</span><textarea value={draft.notes} onChange={(event) => setDraft({ ...draft, notes: event.target.value })} /></label>
          <div><button type="submit" className="button button--primary" disabled={update.isPending}><Save size={15} /> {update.isPending ? t("Đang lưu…") : t("Lưu thay đổi")}</button></div>
        </form>
        {canOperate && <aside className="map-danger-zone">
          <header><span><AlertTriangle size={18} /></span><div><small>{t("THAO TÁC QUẢN TRỊ")}</small><strong>{t("Lưu trữ hoặc xóa bản đồ")}</strong></div></header>
          <p>{t("Các thao tác này ảnh hưởng tới dữ liệu bản đồ và runtime điều hướng của robot.")}</p>
          {detail.status !== "ARCHIVED" && <button type="button" className="button" disabled={archive.isPending} onClick={() => archive.mutate(detail.map_id)}><Archive size={15} /> {t("Lưu trữ bản đồ")}</button>}
          <button type="button" className="button map-delete-action" onClick={() => setDeleteConfirmationOpen(true)}>
            <Trash2 size={16} /> {t(isActive ? "Dừng và xóa bản đồ đang kích hoạt" : "Xóa bản đồ")}</button>
        </aside>}
      </div>}
      </div>

      {operationError && <p className="form-error map-operation-error" role="alert">{operationError instanceof Error ? operationError.message : t("Thao tác bản đồ thất bại")}</p>}

      {deleteConfirmationOpen && <div className="map-delete-dialog" role="dialog" aria-modal="true" aria-labelledby="delete-map-title" aria-describedby="delete-map-description">
        <div className="map-delete-dialog__panel">
          <span className="map-delete-dialog__icon"><AlertTriangle /></span>
          <p className="eyebrow">{t("THAO TÁC KHÔNG THỂ HOÀN TÁC")}</p>
          <h3 id="delete-map-title">{t(isActive ? "Dừng robot và xóa bản đồ đang kích hoạt?" : "Xóa bản đồ này?")}</h3>
          <p id="delete-map-description">{t("Bản đồ {name} sẽ bị xóa khỏi thư viện.", { name: detail.name })}</p>
          {isActive ? <ul><li>{t("Hủy goal Nav2 và dừng robot")}</li><li>{t("Dừng localization, map_server và Nav2")}</li><li>{t("Xóa tham chiếu đang kích hoạt và cache cục bộ")}</li><li>{t("Tạo tombstone để bản đồ không tự xuất hiện lại")}</li></ul>
            : <p>{t("Bundle, phiên bản và metadata liên quan sẽ không còn khả dụng.")}</p>}
          {remove.error && <p className="map-delete-dialog__error" role="alert">{remove.error instanceof Error ? remove.error.message : t("Không thể xóa bản đồ.")}</p>}
          <div><button ref={deleteCancelRef} type="button" className="button" disabled={remove.isPending} onClick={() => setDeleteConfirmationOpen(false)}>{t("Giữ lại bản đồ")}</button>
            <button type="button" className="button map-delete-confirm" disabled={remove.isPending} onClick={() => remove.mutate(detail)}><Trash2 size={16} /> {remove.isPending ? t("Đang dừng runtime…") : t("Xác nhận xóa")}</button></div>
        </div>
      </div>}
    </section>;
  };

  return <OperationsShell title={t("Bản đồ")} className="maps-page">
    <div className="maps-shell">
      {!selectedMapId && <section className="maps-heading">
        <div><h2>{t("Bản đồ vận hành")}</h2></div>
        {canOperate && <button className="button button--primary" type="button" onClick={startNewMapping}><Plus size={19} /> {t("Tạo bản đồ")}</button>}
      </section>}
      {selectedMapId ? renderDetail() : renderRegistry()}
    </div>
  </OperationsShell>;
}
