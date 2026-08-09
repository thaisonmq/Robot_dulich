import { useEffect, useState } from "react";
import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";
import { Archive, CheckCircle2, ChevronRight, FilePenLine, Map, MapPinned, Pencil, PlayCircle, Plus, RadioTower, Save, Trash2, X } from "lucide-react";
import { api } from "../api/client";
import { AccountMenu } from "../components/AccountMenu";
import { Brand } from "../components/Brand";
import { GlobalLanguageSelect } from "../components/GlobalLanguageSelect";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate, usePathname } from "../router";
import { useAppStore } from "../state/appStore";
import type { MapData } from "../types";
import { createUuid } from "../utils/uuid";
import { hasPermission } from "../utils/permissions";

export function MapManagementPage() {
  const { t } = useI18n();
  const navigate = useNavigate();
  const pathname = usePathname();
  const queryClient = useQueryClient();
  const user = useAppStore((state) => state.user);
  const [editing, setEditing] = useState(false);
  const [draft, setDraft] = useState({ name: "", site_id: "", floor_id: "", notes: "" });
  const selectedMapId = pathname.match(/^\/maps\/([^/]+)$/)?.[1];
  const mapsQuery = useQuery({ queryKey: ["maps"], queryFn: () => api.maps() });
  const detailQuery = useQuery({
    queryKey: ["map", selectedMapId],
    queryFn: () => api.map(selectedMapId!),
    enabled: Boolean(selectedMapId),
  });
  const activate = useMutation({
    mutationFn: ({ mapId, version }: { mapId: string; version: number }) => api.activateMap(mapId, version),
    onSuccess: () => void queryClient.invalidateQueries({ queryKey: ["maps"] }),
  });
  const archive = useMutation({
    mutationFn: api.archiveMap,
    onSuccess: () => void queryClient.invalidateQueries({ queryKey: ["maps"] }),
  });
  const update = useMutation({
    mutationFn: () => api.updateMap(selectedMapId!, draft),
    onSuccess: async () => {
      setEditing(false);
      await queryClient.invalidateQueries({ queryKey: ["maps"] });
      await queryClient.invalidateQueries({ queryKey: ["map", selectedMapId] });
    },
  });
  const remove = useMutation({
    mutationFn: async (map: MapData) => {
      const active = map.mapping_session;
      if (active && !["FINISHED", "CANCELED", "FAULT"].includes(active.status)) {
        let canceled = await api.mappingAction(
          active.session_id,
          "cancel",
          createUuid(),
          active.status,
        );
        if (["MAPPING", "PAUSED"].includes(canceled.status)) {
          canceled = await api.mappingAction(
            active.session_id,
            "cancel",
            createUuid(),
            canceled.status,
          );
        }
        if (canceled.status !== "CANCELED") {
          throw new Error(canceled.error_message ?? t("Không thể hủy phiên mapping đang hoạt động"));
        }
      }
      return api.deleteMap(map.map_id);
    },
    onSuccess: async () => {
      await queryClient.invalidateQueries({ queryKey: ["maps"] });
      navigate("/maps");
    },
  });
  const canOperate = hasPermission(user, "maps.manage");
  const maps = mapsQuery.data ?? [];
  const detail = detailQuery.data;
  const activeMapping = detail?.mapping_session;
  const continuableVersion = detail?.versions?.find((version) => version.can_continue);
  useEffect(() => {
    if (!detail) return;
    setDraft({ name: detail.name, site_id: detail.site_id ?? "", floor_id: detail.floor_id ?? "", notes: detail.notes ?? "" });
    setEditing(false);
  }, [detail?.map_id]);

  const confirmDelete = () => {
    if (!detail) return;
    const warning = activeMapping
      ? `Map "${detail.name}" đang được mapping. Hủy phiên hiện tại và xóa vĩnh viễn map?`
      : `Xóa vĩnh viễn map "${detail.name}"? Thao tác này không thể hoàn tác.`;
    if (window.confirm(t(warning))) {
      remove.mutate(detail);
    }
  };

  const continueMapping = () => {
    if (!detail) return;
    if (activeMapping) {
      navigate(`/maps/create?session_id=${encodeURIComponent(activeMapping.session_id)}`);
      return;
    }
    if (continuableVersion) {
      navigate(`/maps/create?map_id=${encodeURIComponent(detail.map_id)}&source_version=${continuableVersion.version}`);
    }
  };

  return <main className="roster-page maps-page">
    <header className="roster-header">
      <div className="roster-header__title"><Brand compact /><span className="header-divider" /><h1>{t("Quản lý map")}</h1></div>
      <nav className="maps-header-nav">
        <button type="button" onClick={() => navigate("/robots")}><RadioTower size={16} /> {t("Robot")}</button>
        <button type="button" className="is-active" onClick={() => navigate("/maps")}><Map size={16} /> Maps</button>
      </nav>
      <GlobalLanguageSelect /><AccountMenu />
    </header>
    <div className="maps-shell">
      <section className="maps-heading">
        <div><p className="eyebrow">MAP REGISTRY · VERSIONED</p><h2>{t("Bản đồ vận hành")}</h2>
          <p>{t("Center là registry chính; robot giữ cache cục bộ đã xác minh SHA-256.")}</p></div>
        {canOperate && <button className="button button--primary" type="button" onClick={() => navigate("/maps/create")}>
          <Plus size={18} /> {t("Tạo map")}
        </button>}
      </section>
      {selectedMapId && detail ? <section className="map-detail-view">
        <button type="button" className="text-action" onClick={() => navigate("/maps")}>← {t("Tất cả map")}</button>
        <div className="map-detail-view__head"><span><MapPinned size={26} /></span><div><small>{detail.site_id} · {detail.floor_id}</small><h2>{detail.name}</h2><p>{detail.notes}</p></div>
          <strong className={`map-status map-status--${detail.status?.toLowerCase()}`}>{detail.status}</strong></div>
        {editing && <form className="map-edit-form" onSubmit={(event) => { event.preventDefault(); update.mutate(); }}>
          <label><span>{t("Tên map")}</span><input required minLength={2} value={draft.name} onChange={(event) => setDraft({ ...draft, name: event.target.value })} /></label>
          <label><span>Site / {t("tòa nhà")}</span><input required value={draft.site_id} onChange={(event) => setDraft({ ...draft, site_id: event.target.value })} /></label>
          <label><span>{t("Tầng")}</span><input required value={draft.floor_id} onChange={(event) => setDraft({ ...draft, floor_id: event.target.value })} /></label>
          <label className="map-edit-form__notes"><span>{t("Ghi chú")}</span><textarea value={draft.notes} onChange={(event) => setDraft({ ...draft, notes: event.target.value })} /></label>
          <div><button type="submit" className="button button--primary" disabled={update.isPending}><Save size={15} /> {t("Lưu thay đổi")}</button><button type="button" className="button" onClick={() => setEditing(false)}><X size={15} /> {t("Hủy")}</button></div>
        </form>}
        <div className="map-version-list">
          {(detail.versions ?? []).map((version) => <article key={version.version}>
            <span>v{version.version}</span><div><strong>{version.status}</strong><code>{version.checksum.slice(0, 16)}…</code>
              <small>{version.width_pixels} × {version.height_pixels} · {version.resolution} m/px · {version.created_by_robot}</small></div>
            {canOperate && version.status === "VALIDATING" && <button type="button" className="button button--primary" onClick={() => activate.mutate({ mapId: detail.map_id, version: version.version })}>
              <CheckCircle2 size={15} /> Activate
            </button>}
          </article>)}
        </div>
        {activeMapping && <div className="map-active-mapping"><PlayCircle /><div><strong>{t("Phiên mapping đang hoạt động")}</strong><span>{activeMapping.robot_id} · {activeMapping.status} · v{activeMapping.version}</span></div><button type="button" className="button button--primary" onClick={continueMapping}>{t("Vào lại màn mapping")}</button></div>}
        {(update.error || remove.error || archive.error) && <p className="form-error map-operation-error">{(update.error ?? remove.error ?? archive.error) instanceof Error ? (update.error ?? remove.error ?? archive.error as Error).message : t("Thao tác map thất bại")}</p>}
        {canOperate && <div className="map-detail-actions">
          <button type="button" className="button button--primary" disabled={!activeMapping && !continuableVersion} title={!activeMapping && !continuableVersion ? t("Hãy Save Draft hoặc Finish để có pose-graph tiếp tục") : undefined} onClick={continueMapping}><Pencil size={15} /> {activeMapping ? t("Tiếp tục mapping") : t("Sửa map · tiếp tục SLAM")}</button>
          <button type="button" className="button" onClick={() => setEditing((value) => !value)}><FilePenLine size={15} /> {t("Sửa thông tin")}</button>
          {detail.status !== "ARCHIVED" && <button type="button" className="button button--danger-outline" onClick={() => archive.mutate(detail.map_id)}><Archive size={15} /> {t("Archive map")}</button>}
          {detail.status !== "ACTIVE" && detail.active_version == null && <button type="button" className="button button--danger-outline" disabled={remove.isPending} onClick={confirmDelete}><Trash2 size={15} /> {t("Xóa map")}</button>}
        </div>}
      </section> : <section className="map-registry-grid">
        {maps.map((item) => <button type="button" key={item.map_id} onClick={() => navigate(`/maps/${item.map_id}`)}>
          <span className="map-card-preview">{item.image_url ? <img src={item.image_url.startsWith("/api/") ? "/maps/map-001.svg" : item.image_url} alt="" /> : <MapPinned size={32} />}</span>
          <span className="map-card-copy"><small>{item.site_id || "—"} · {item.floor_id || "—"}</small><strong>{item.name}</strong>
            <em>{item.width_pixels}×{item.height_pixels} · {item.resolution_m_per_pixel} m/px</em>
            <code>{item.checksum ? `${item.checksum.slice(0, 12)}…` : "legacy sample"}</code></span>
          <span className={`map-status map-status--${item.status?.toLowerCase()}`}>{item.status}</span><ChevronRight size={18} />
        </button>)}
        {!maps.length && !mapsQuery.isLoading && <div className="fleet-empty"><span><MapPinned /></span><h3>{t("Chưa có map")}</h3><p>{t("Tạo phiên SLAM đầu tiên từ một robot online.")}</p></div>}
      </section>}
    </div>
  </main>;
}
