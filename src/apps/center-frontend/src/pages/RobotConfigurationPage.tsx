import { useEffect, useRef, useState } from "react";
import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";
import {
  Activity, ArrowLeft, Camera, Check, EthernetPort, MapPinned, Mic2,
  KeyRound, Play, Radar, RefreshCw, Save, ServerCog, Square,
  Volume2, WifiOff, X,
} from "lucide-react";
import { api } from "../api/client";
import { OperationsShell } from "../components/OperationsShell";
import { showToast } from "../components/ToastViewport";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate, useParams } from "../router";
import type {
  MediaSource, OnvifDevice, OnvifScanRequest, RejectedMediaSource,
  RobotConfigurationUpdate,
} from "../types";
import type { LiveKitMediaTransport } from "../transports/MediaTransport";

const EMPTY_CONFIGURATION: RobotConfigurationUpdate = {
  device_ip: "",
  video_source_type: "rtsp",
  video_source: "",
  video_profile: "balanced",
  rtsp_transport: "auto",
  camera_label: "Camera chính",
  audio_source_type: "silent",
  audio_source: "",
  microphone_label: "Microphone chính",
  audio_output_type: "disabled",
  audio_output: "",
  speaker_label: "Loa chính",
};

function configurationForm(
  configuration: RobotConfigurationUpdate,
): RobotConfigurationUpdate {
  const {
    device_ip, video_source_type, video_source, video_profile, rtsp_transport,
    camera_label, audio_source_type, audio_source, microphone_label,
    audio_output_type, audio_output, speaker_label,
  } = configuration;
  return {
    device_ip, video_source_type, video_source, video_profile, rtsp_transport,
    camera_label, audio_source_type, audio_source, microphone_label,
    audio_output_type: audio_output_type ?? "disabled",
    audio_output: audio_output ?? "",
    speaker_label: speaker_label ?? "Loa chính",
  };
}

export function RobotConfigurationPage() {
  const navigate = useNavigate();
  const { t } = useI18n();
  const queryClient = useQueryClient();
  const { robotId = "" } = useParams();
  const [form, setForm] = useState<RobotConfigurationUpdate>(EMPTY_CONFIGURATION);
  const [assignedMapId, setAssignedMapId] = useState("");
  const [tab, setTab] = useState<"connection" | "video" | "audio">("connection");
  const [previewState, setPreviewState] = useState("idle");
  const [previewOpen, setPreviewOpen] = useState(false);
  const [videoSources, setVideoSources] = useState<MediaSource[]>([]);
  const [audioSources, setAudioSources] = useState<MediaSource[]>([]);
  const [speakerSources, setSpeakerSources] = useState<MediaSource[]>([]);
  const [rejectedVideoSources, setRejectedVideoSources] = useState<RejectedMediaSource[]>([]);
  const [rejectedAudioSources, setRejectedAudioSources] = useState<RejectedMediaSource[]>([]);
  const [rejectedSpeakerSources, setRejectedSpeakerSources] = useState<RejectedMediaSource[]>([]);
  const [onvifDevices, setOnvifDevices] = useState<OnvifDevice[]>([]);
  const [onvifScanned, setOnvifScanned] = useState(false);
  const [onvifDialogOpen, setOnvifDialogOpen] = useState(false);
  const [onvifSelectedHost, setOnvifSelectedHost] = useState("");
  const [onvifCredentials, setOnvifCredentials] = useState({
    username: "",
    password: "",
  });
  const [sourcesScanned, setSourcesScanned] = useState({
    video: false,
    audio: false,
    speaker: false,
  });
  const videoRef = useRef<HTMLVideoElement>(null);
  const audioRef = useRef<HTMLAudioElement>(null);
  const previewTransport = useRef<LiveKitMediaTransport | null>(null);
  const previewLeaseId = useRef<string | null>(null);
  const previewRequested = useRef(false);

  const robotQuery = useQuery({
    queryKey: ["robot", robotId],
    queryFn: () => api.robot(robotId),
    enabled: Boolean(robotId),
  });
  const configurationQuery = useQuery({
    queryKey: ["robot-configuration", robotId],
    queryFn: () => api.robotConfiguration(robotId),
    enabled: Boolean(robotId),
    retry: false,
  });
  const mapsQuery = useQuery({
    queryKey: ["maps"],
    queryFn: () => api.maps(),
  });
  const assignableMaps = (mapsQuery.data ?? []).filter(
    (item) => item.status !== "DELETED" && item.active_version != null,
  );
  useEffect(() => {
    if (robotQuery.data) setAssignedMapId(robotQuery.data.map_id);
  }, [robotQuery.data]);
  useEffect(() => {
    if (!configurationQuery.data) return;
    setForm(configurationForm(configurationQuery.data));
  }, [configurationQuery.data]);
  useEffect(() => {
    setVideoSources([]);
    setAudioSources([]);
    setSpeakerSources([]);
    setRejectedVideoSources([]);
    setRejectedAudioSources([]);
    setRejectedSpeakerSources([]);
    setOnvifDevices([]);
    setOnvifScanned(false);
    setOnvifDialogOpen(false);
    setOnvifSelectedHost("");
    setOnvifCredentials({ username: "", password: "" });
    setSourcesScanned({ video: false, audio: false, speaker: false });
  }, [robotId]);
  useEffect(() => {
    const heartbeat = window.setInterval(() => {
      const leaseId = previewLeaseId.current;
      if (leaseId) {
        void api.renewRobotPreview(robotId, leaseId).catch(async () => {
          previewLeaseId.current = null;
          await previewTransport.current?.disconnect();
          previewTransport.current = null;
          setPreviewState("Phiên xem trước đã kết thúc");
        });
      }
    }, 10_000);
    return () => {
      window.clearInterval(heartbeat);
      void previewTransport.current?.disconnect();
      previewTransport.current = null;
      const leaseId = previewLeaseId.current;
      previewLeaseId.current = null;
      if (leaseId) void api.stopRobotPreview(robotId, leaseId).catch(() => undefined);
    };
  }, [robotId]);

  const save = useMutation({
    mutationFn: () => api.updateRobotConfiguration(robotId, form),
    onSuccess: (configuration) => {
      setForm(configurationForm(configuration));
      showToast(t("Đã lưu cấu hình"));
    },
  });
  const assignMap = useMutation({
    mutationFn: () => api.updateRobot(robotId, {
      name: robotQuery.data!.name,
      site_id: robotQuery.data!.site_id,
      map_id: assignedMapId,
      enabled: robotQuery.data!.enabled,
      management_address: robotQuery.data!.management_address ?? undefined,
      management_username: robotQuery.data!.management_username ?? undefined,
    }),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: ["robot", robotId] });
      void queryClient.invalidateQueries({ queryKey: ["robots"] });
      showToast(t("Đã lưu thay đổi"));
    },
  });
  const connectionTest = useMutation({
    mutationFn: () => api.testRobotConnection(robotId),
  });
  const mediaTest = useMutation({
    mutationFn: (mediaKind: "video" | "audio" | "speaker") =>
      api.testRobotMedia(robotId, mediaKind, form),
  });
  const mediaSourceScan = useMutation({
    mutationFn: (mediaKind: "video" | "audio" | "speaker") =>
      api.robotMediaSources(robotId, mediaKind),
    onSuccess: (sources, mediaKind) => {
      if (mediaKind === "video") {
        setVideoSources(sources.video_sources);
        setRejectedVideoSources(sources.rejected_video_sources ?? []);
        if (sources.video_sources.length) {
          setForm((current) => current.video_source_type === "camera"
            ? current
            : { ...current, video_source_type: "camera", video_source: "" });
        }
      } else if (mediaKind === "audio") {
        setAudioSources(sources.audio_sources);
        setRejectedAudioSources(sources.rejected_audio_sources ?? []);
        if (sources.audio_sources.length) {
          setForm((current) => current.audio_source_type === "device"
            ? current
            : { ...current, audio_source_type: "device", audio_source: "" });
        }
      } else {
        const availableSpeakers = sources.speaker_sources ?? [];
        setSpeakerSources(availableSpeakers);
        setRejectedSpeakerSources(sources.rejected_speaker_sources ?? []);
        if (availableSpeakers.length) {
          setForm((current) => current.audio_output_type === "device"
            ? current
            : { ...current, audio_output_type: "device", audio_output: "" });
        }
      }
      setSourcesScanned((current) => ({ ...current, [mediaKind]: true }));
    },
  });
  const onvifScan = useMutation({
    mutationFn: (credentials: OnvifScanRequest) =>
      api.scanRobotOnvifCameras(robotId, credentials),
    onSuccess: (result, credentials) => {
      if (credentials.target_host) {
        setOnvifDevices((current) => {
          const updated = result.devices[0];
          if (!updated) return current;
          const exists = current.some((device) => device.host === updated.host);
          return exists
            ? current.map((device) => device.host === updated.host ? updated : device)
            : [...current, updated];
        });
        if (result.devices[0]?.profiles.length) {
          setOnvifCredentials({ username: "", password: "" });
        }
      } else {
        setOnvifDevices(result.devices);
      }
      setOnvifScanned(true);
    },
  });

  function scanMediaSources(mediaKind: "video" | "audio" | "speaker") {
    mediaSourceScan.reset();
    mediaSourceScan.mutate(mediaKind);
  }

  function selectOnvifProfile(device: OnvifDevice, rtspUrl: string, profileName: string) {
    setForm((current) => ({
      ...current,
      video_source_type: "rtsp",
      video_source: rtspUrl,
      camera_label: `${device.name} · ${device.host} · ${profileName}`,
    }));
    setOnvifDialogOpen(false);
    setOnvifSelectedHost("");
    setOnvifCredentials({ username: "", password: "" });
  }

  function authenticateOnvifDevice(host: string) {
    onvifScan.reset();
    onvifScan.mutate({
      target_host: host,
      username: onvifCredentials.username.trim(),
      password: onvifCredentials.password,
    });
  }

  const selectedOnvifDevice = onvifDevices.find(
    (device) => device.host === onvifSelectedHost,
  );

  async function stopPreview() {
    previewRequested.current = false;
    await previewTransport.current?.disconnect();
    previewTransport.current = null;
    const leaseId = previewLeaseId.current;
    previewLeaseId.current = null;
    if (leaseId) await api.stopRobotPreview(robotId, leaseId).catch(() => undefined);
    setPreviewState("idle");
  }

  async function startPreview() {
    if (!videoRef.current || !audioRef.current) return;
    setPreviewState("connecting");
    let leaseId: string | null = null;
    try {
      const access = await api.robotPreviewToken(robotId);
      leaseId = access.lease_id;
      if (!previewRequested.current || !videoRef.current || !audioRef.current) {
        await api.stopRobotPreview(robotId, leaseId).catch(() => undefined);
        setPreviewState("idle");
        return;
      }
      previewLeaseId.current = leaseId;
      const { LiveKitMediaTransport } = await import("../transports/MediaTransport");
      const transport = new LiveKitMediaTransport(
        videoRef.current,
        audioRef.current,
        setPreviewState,
      );
      previewTransport.current = transport;
      await transport.connect(access.url, access.token);
    } catch (reason) {
      previewTransport.current = null;
      previewLeaseId.current = null;
      if (leaseId) {
        await api.stopRobotPreview(robotId, leaseId).catch(() => undefined);
      }
      setPreviewState(previewRequested.current
        ? reason instanceof Error ? reason.message : "failed"
        : "idle");
    }
  }

  useEffect(() => {
    if (previewOpen && previewState === "idle" && !previewTransport.current) {
      previewRequested.current = true;
      void startPreview();
    }
  }, [previewOpen, previewState]);

  useEffect(() => {
    if (!previewOpen) return;
    const closeOnEscape = (event: KeyboardEvent) => {
      if (event.key !== "Escape") return;
      setPreviewOpen(false);
      void stopPreview();
    };
    window.addEventListener("keydown", closeOnEscape);
    return () => window.removeEventListener("keydown", closeOnEscape);
  }, [previewOpen, robotId]);

  const robot = robotQuery.data;
  const loading = robotQuery.isLoading || configurationQuery.isLoading;

  return (
    <OperationsShell title={robot?.name ? `${t("Cấu hình")}: ${robot.name}` : "Cấu hình thiết bị"} className="robot-configuration-operations">
      <section className="configuration-shell configuration-shell--single">
        <form
          className="configuration-form"
          onSubmit={(event) => {
            event.preventDefault();
            save.mutate();
          }}
        >
          <div className="configuration-form__heading">
            <div>
              <p className="eyebrow">{t("KẾT NỐI & HÌNH ẢNH")}</p>
              <h2>{t("Thông số robot")}</h2>
            </div>
            <span className={robot?.status === "online" ? "connection-chip is-online" : "connection-chip"}>
              <i /> {robot?.status === "online" ? t("Đang kết nối") : t("Chưa kết nối")}
            </span>
          </div>
          <nav className="configuration-tabs" aria-label={t("Nhóm cấu hình")}>
            <button
              type="button"
              className={tab === "connection" ? "is-active" : ""}
              onClick={() => setTab("connection")}
            >
              <EthernetPort size={16} /> {t("Kết nối")}
            </button>
            <button
              type="button"
              className={tab === "video" ? "is-active" : ""}
              onClick={() => setTab("video")}
            >
              <Camera size={16} /> {t("Camera")}
            </button>
            <button
              type="button"
              className={tab === "audio" ? "is-active" : ""}
              onClick={() => setTab("audio")}
            >
              <Volume2 size={16} /> {t("Âm thanh")}
            </button>
          </nav>

          {loading ? (
            <div className="configuration-loading">{t("Đang gọi cấu hình từ simulator…")}</div>
          ) : configurationQuery.isError ? (
            <div className="configuration-error" role="alert">
              <span><WifiOff size={24} /></span>
              <h3>{t("Không kết nối được simulator")}</h3>
              <p>
                {configurationQuery.error instanceof Error
                  ? t(configurationQuery.error.message)
                  : t("Simulator không phản hồi yêu cầu cấu hình")}
              </p>
              <button
                type="button"
                className="button button--outline"
                onClick={() => configurationQuery.refetch()}
                disabled={configurationQuery.isFetching}
              >
                <RefreshCw size={17} />
                {configurationQuery.isFetching ? t("Đang thử lại…") : t("Thử kết nối lại")}
              </button>
            </div>
          ) : (
            <div className="configuration-fields">
              {tab === "connection" && (
                <>
                  <section className="robot-map-assignment config-field--wide">
                    <span className="robot-map-assignment__icon"><MapPinned size={20} /></span>
                    <div className="robot-map-assignment__copy">
                      <strong>{t("Bản đồ điều hướng")}</strong>
                      <small>{t("Chọn bản đồ đã kích hoạt để gắn cho robot này.")}</small>
                    </div>
                    <select
                      value={assignedMapId}
                      aria-label={t("Bản đồ")}
                      disabled={mapsQuery.isLoading || !assignableMaps.length || assignMap.isPending}
                      onChange={(event) => setAssignedMapId(event.target.value)}
                    >
                      {mapsQuery.isLoading && <option value={assignedMapId}>{t("Đang tải danh sách bản đồ…")}</option>}
                      {!mapsQuery.isLoading && !assignableMaps.length && <option value={assignedMapId}>{t("Chưa có bản đồ đã kích hoạt")}</option>}
                      {assignedMapId && !assignableMaps.some((item) => item.map_id === assignedMapId) && (
                        <option value={assignedMapId}>{assignedMapId} · {t("Bản đồ hiện tại")}</option>
                      )}
                      {assignableMaps.map((item) => <option value={item.map_id} key={item.map_id}>
                        {item.name} · {item.site_id || "—"} / {item.floor_id || "—"} · v{item.active_version}
                      </option>)}
                    </select>
                    <button
                      type="button"
                      className="button button--outline"
                      disabled={!robot || !assignedMapId || assignedMapId === robot.map_id || assignMap.isPending}
                      onClick={() => assignMap.mutate()}
                    >
                      <Save size={16} /> {assignMap.isPending ? t("Đang lưu…") : t("Gắn bản đồ")}
                    </button>
                    {(mapsQuery.isError || assignMap.isError) && <p role="alert" className="robot-map-assignment__error">
                      {assignMap.error instanceof Error ? t(assignMap.error.message) : t("Không tải được danh sách bản đồ")}
                    </p>}
                  </section>
                  <label className="config-field config-field--wide">
                    <span><EthernetPort size={17} /> {t("Địa chỉ IP robot")}</span>
                    <input
                      value={form.device_ip}
                      onChange={(event) => setForm({ ...form, device_ip: event.target.value })}
                      placeholder="192.168.1.20"
                      required
                    />
                    <small>{t("Địa chỉ do edge agent báo về; không cần mở cổng inbound.")}</small>
                  </label>
                  <section className="diagnostic-card config-field--wide">
                    <span><Activity size={20} /></span>
                    <div>
                      <strong>{t("Kiểm tra kênh điều khiển")}</strong>
                      <small>{t("Đo phản hồi WSS và trạng thái publisher media trên robot.")}</small>
                    </div>
                    <button
                      type="button"
                      className="button button--outline"
                      disabled={connectionTest.isPending}
                      onClick={() => connectionTest.mutate()}
                    >
                      <RefreshCw size={16} /> {connectionTest.isPending ? t("Đang kiểm tra…") : t("Kiểm tra kết nối")}
                    </button>
                    {connectionTest.data && (
                      <p className={connectionTest.data.ok ? "diagnostic-result is-ok" : "diagnostic-result"}>
                        {connectionTest.data.ok
                          ? `Gateway ${connectionTest.data.gateway} · Media ${connectionTest.data.media}`
                          : t(connectionTest.data.detail ?? "Không thể kết nối trung tâm")}
                      </p>
                    )}
                  </section>
                </>
              )}

              {tab === "video" && (
                <>
                  <label className="config-field">
                    <span><Camera size={17} /> {t("Loại nguồn video")}</span>
                    <select
                      value={form.video_source_type}
                      onChange={(event) => {
                        const video_source_type = event.target.value as RobotConfigurationUpdate["video_source_type"];
                        setForm({
                          ...form,
                          video_source_type,
                          video_source: video_source_type === "camera"
                            ? ""
                            : video_source_type === "rtsp"
                              ? "rtsp://camera.local/live"
                              : video_source_type === "test"
                                ? "generated://test-pattern"
                                : "",
                        });
                      }}
                    >
                      <option value="rtsp">{t("Camera mạng")} · RTSP</option>
                      <option value="camera">Camera USB · V4L2</option>
                      <option value="file">{t("Tệp hoặc HTTP stream")}</option>
                      <option value="test">{t("Ảnh kiểm thử tự động")}</option>
                    </select>
                  </label>
                  <label className="config-field">
                    <span><ServerCog size={17} /> {t("Tên camera")}</span>
                    <input
                      value={form.camera_label}
                      onChange={(event) => setForm({ ...form, camera_label: event.target.value })}
                      required
                    />
                  </label>
                  <label className="config-field config-field--wide">
                    <span><Camera size={17} /> {t("Nguồn phát video")}</span>
                    <div className="source-picker">
                      {form.video_source_type === "camera" ? (
                          <select
                            value={form.video_source}
                            onChange={(event) => setForm({ ...form, video_source: event.target.value })}
                            required
                            aria-describedby="video-source-status"
                          >
                            <option value="" disabled>
                              {sourcesScanned.video
                                ? videoSources.length
                                  ? t("Chọn camera vừa tìm thấy")
                                  : t("Không tìm thấy camera V4L2")
                                : t("Bấm Quét để tìm camera trên robot")}
                            </option>
                            {form.video_source && !videoSources.some((source) => source.value === form.video_source) && (
                              <option value={form.video_source}>
                                {sourcesScanned.video ? t("Không còn kết nối") : t("Cấu hình hiện tại")} · {form.video_source}
                              </option>
                            )}
                            {videoSources.map((source) => (
                              <option value={source.value} key={`${source.type}-${source.value}`}>
                                {source.label} · {source.value}
                              </option>
                            ))}
                          </select>
                      ) : (
                        <input
                          value={form.video_source}
                          disabled={form.video_source_type === "test"}
                          onChange={(event) => setForm({ ...form, video_source: event.target.value })}
                          placeholder={form.video_source_type === "rtsp" ? "rtsp://camera.local/live" : "/media/video.mp4"}
                          required
                        />
                      )}
                      {form.video_source_type === "camera" && (
                        <button
                          type="button"
                          className="source-scan-button"
                          onClick={() => scanMediaSources("video")}
                          disabled={robot?.status !== "online" || mediaSourceScan.isPending}
                        >
                          <RefreshCw
                            size={17}
                            className={mediaSourceScan.isPending && mediaSourceScan.variables === "video" ? "is-spinning" : ""}
                          />
                          {mediaSourceScan.isPending && mediaSourceScan.variables === "video" ? t("Đang quét…") : t("Quét")}
                        </button>
                      )}
                      {form.video_source_type === "rtsp" && (
                        <button
                          type="button"
                          className="source-scan-button source-scan-button--onvif"
                          onClick={() => {
                            onvifScan.reset();
                            setOnvifDialogOpen(true);
                            setOnvifSelectedHost("");
                            setOnvifCredentials({ username: "", password: "" });
                            onvifScan.mutate({});
                          }}
                          disabled={robot?.status !== "online" || onvifScan.isPending}
                        >
                          <Radar size={17} className={onvifScan.isPending ? "is-spinning" : ""} />
                          {onvifScan.isPending ? t("Đang quét…") : t("Quét ONVIF")}
                        </button>
                      )}
                    </div>
                    <small id="video-source-status" className={
                      (form.video_source_type === "rtsp" && onvifScan.isError)
                      || (mediaSourceScan.isError && mediaSourceScan.variables === "video")
                        ? "source-scan-status is-error"
                        : "source-scan-status"
                    }>
                      {form.video_source_type === "rtsp"
                        ? onvifScan.isPending
                          ? t("Robot đang quét camera ONVIF trong cùng mạng LAN…")
                          : onvifScan.isError
                            ? onvifScan.error instanceof Error
                              ? t(onvifScan.error.message)
                              : t("Không quét được camera ONVIF")
                            : onvifScanned
                              ? t("Tìm thấy {devices} camera ONVIF: {ready} đã đọc profile, {locked} chờ đăng nhập.", {
                                  devices: onvifDevices.length,
                                  ready: onvifDevices.filter((device) => device.profiles.length > 0).length,
                                  locked: onvifDevices.filter((device) => device.auth_required).length,
                                })
                              : t("Quét ONVIF để tìm camera cùng dải mạng và chọn đúng RTSP path.")
                        : mediaSourceScan.isPending && mediaSourceScan.variables === "video"
                          ? t("Robot đang dò thiết bị camera trên máy đang chạy…")
                          : mediaSourceScan.isError && mediaSourceScan.variables === "video"
                            ? mediaSourceScan.error instanceof Error
                              ? t(mediaSourceScan.error.message)
                              : t("Không quét được camera trên robot")
                            : sourcesScanned.video
                              ? videoSources.length
                                ? `${t("Đã xác minh {count} camera trả về hình ảnh.", { count: videoSources.length })}${rejectedVideoSources.length ? ` ${t("Loại {count} nguồn không hoạt động.", { count: rejectedVideoSources.length })}` : ""}`
                                : rejectedVideoSources.length
                                  ? `${t("Không có camera hoạt động.")} ${t(rejectedVideoSources[0].reason)}`
                                  : t("Robot không phát hiện camera nào. Kiểm tra kết nối USB và quyền truy cập thiết bị.")
                              : form.video_source_type === "camera"
                                ? t("Bấm Quét để chỉ giữ camera thực sự trả về được frame hình ảnh.")
                                : t("Nhập đường dẫn nguồn video cần phát.")}
                    </small>
                  </label>
                  <label className="config-field">
                    <span>{t("Chất lượng video")}</span>
                    <select
                      value={form.video_profile}
                      onChange={(event) => setForm({
                        ...form,
                        video_profile: event.target.value as RobotConfigurationUpdate["video_profile"],
                      })}
                    >
                      <option value="full_hd">Full HD · 1080p</option>
                      <option value="balanced">{t("Cân bằng")} · 720p</option>
                      <option value="low_bandwidth">{t("Băng thông thấp")} · 480p</option>
                    </select>
                  </label>
                  <label className="config-field">
                    <span>{t("Giao thức RTSP")}</span>
                    <select
                      value={form.rtsp_transport}
                      disabled={form.video_source_type !== "rtsp"}
                      onChange={(event) => setForm({
                        ...form,
                        rtsp_transport: event.target.value as RobotConfigurationUpdate["rtsp_transport"],
                      })}
                    >
                      <option value="auto">Auto · UDP → TCP fallback</option>
                      <option value="tcp">TCP · {t("ưu tiên ổn định")}</option>
                      <option value="udp">UDP · {t("ưu tiên độ trễ")}</option>
                    </select>
                  </label>
                  <div className="media-test-actions config-field--wide">
                    <button type="button" className="button button--outline" disabled={mediaTest.isPending} onClick={() => mediaTest.mutate("video")}>
                      <Activity size={16} /> {t("Kiểm tra nguồn camera")}
                    </button>
                    <button type="button" className="button button--outline" onClick={() => setPreviewOpen(true)}>
                      <Play size={15} /> {t("Xem trước camera")}
                    </button>
                    {mediaTest.data?.diagnostic === "video" && (
                      <span className={mediaTest.data.ok ? "is-ok" : ""}>
                        {mediaTest.data.detail
                          ? t(mediaTest.data.detail)
                          : mediaTest.data.ok ? t("Nguồn video hoạt động") : t("Nguồn video không hoạt động")}
                      </span>
                    )}
                  </div>
                </>
              )}

              {tab === "audio" && (
                <>
                  <div className="audio-section-title config-field--wide">
                    <Volume2 size={18} />
                    <span>
                      <strong>{t("Loa phát đàm thoại")}</strong>
                      <small>{t("Âm thanh microphone của người điều khiển sẽ phát qua loa này.")}</small>
                    </span>
                  </div>
                  <label className="config-field">
                    <span><Volume2 size={17} /> {t("Đầu ra âm thanh")}</span>
                    <select
                      value={form.audio_output_type}
                      onChange={(event) => {
                        const audio_output_type = event.target.value as RobotConfigurationUpdate["audio_output_type"];
                        setForm({ ...form, audio_output_type, audio_output: "" });
                      }}
                    >
                      <option value="device">{t("Loa thiết bị")}</option>
                      <option value="disabled">{t("Không phát loa")}</option>
                    </select>
                  </label>
                  <label className="config-field">
                    <span>{t("Tên loa")}</span>
                    <input
                      value={form.speaker_label}
                      onChange={(event) => setForm({ ...form, speaker_label: event.target.value })}
                      required
                    />
                  </label>
                  <label className="config-field config-field--wide">
                    <span><Volume2 size={17} /> {t("Thiết bị loa")}</span>
                    <div className="source-picker">
                      <select
                        value={form.audio_output}
                        disabled={form.audio_output_type === "disabled"}
                        onChange={(event) => setForm({ ...form, audio_output: event.target.value })}
                        required={form.audio_output_type === "device"}
                        aria-describedby="speaker-source-status"
                      >
                        <option value="" disabled>
                          {sourcesScanned.speaker
                            ? speakerSources.length
                              ? t("Chọn loa vừa tìm thấy")
                              : t("Không tìm thấy loa trên robot")
                            : t("Bấm Quét để tìm loa trên robot")}
                        </option>
                        {form.audio_output && !speakerSources.some((source) => source.value === form.audio_output) && (
                          <option value={form.audio_output}>
                            {sourcesScanned.speaker ? t("Không còn kết nối") : t("Cấu hình hiện tại")} · {form.audio_output}
                          </option>
                        )}
                        {speakerSources.map((source) => (
                          <option value={source.value} key={`${source.type}-${source.value}`}>
                            {source.label} · {source.value}
                          </option>
                        ))}
                      </select>
                      <button
                        type="button"
                        className="source-scan-button"
                        onClick={() => scanMediaSources("speaker")}
                        disabled={robot?.status !== "online" || mediaSourceScan.isPending}
                      >
                        <RefreshCw
                          size={17}
                          className={mediaSourceScan.isPending && mediaSourceScan.variables === "speaker" ? "is-spinning" : ""}
                        />
                        {mediaSourceScan.isPending && mediaSourceScan.variables === "speaker" ? t("Đang quét…") : t("Quét")}
                      </button>
                    </div>
                    <small id="speaker-source-status" className={
                      mediaSourceScan.isError && mediaSourceScan.variables === "speaker"
                        ? "source-scan-status is-error"
                        : "source-scan-status"
                    }>
                      {mediaSourceScan.isPending && mediaSourceScan.variables === "speaker"
                        ? t("Robot đang dò đầu ra ALSA và PipeWire/PulseAudio…")
                        : mediaSourceScan.isError && mediaSourceScan.variables === "speaker"
                          ? mediaSourceScan.error instanceof Error
                            ? t(mediaSourceScan.error.message)
                            : t("Không quét được loa trên robot")
                          : sourcesScanned.speaker
                            ? speakerSources.length
                              ? `${t("Đã xác minh {count} đầu ra loa.", { count: speakerSources.length })}${rejectedSpeakerSources.length ? ` ${t("Loại {count} đầu ra không hoạt động.", { count: rejectedSpeakerSources.length })}` : ""} ${t("Chọn loa, kiểm tra âm báo rồi lưu cấu hình.")}`
                              : rejectedSpeakerSources.length
                                ? `${t("Không có loa hoạt động.")} ${rejectedSpeakerSources[0].label}: ${t(rejectedSpeakerSources[0].reason)}`
                                : t("Robot không phát hiện loa nào. Kiểm tra USB, Bluetooth hoặc dịch vụ PipeWire/PulseAudio.")
                            : form.audio_output_type === "device"
                              ? t("Bấm Quét để tìm loa; quá trình quét không phát âm thanh.")
                              : t("Chọn Loa thiết bị rồi bấm Quét để cấu hình đàm thoại hai chiều.")}
                    </small>
                  </label>
                  <div className="media-test-actions config-field--wide">
                    <button
                      type="button"
                      className="button button--outline"
                      disabled={mediaTest.isPending || form.audio_output_type === "disabled" || !form.audio_output}
                      onClick={() => mediaTest.mutate("speaker")}
                    >
                      <Volume2 size={16} /> {t("Phát âm kiểm tra loa")}
                    </button>
                    {mediaTest.data?.diagnostic === "speaker" && (
                      <span className={mediaTest.data.ok ? "is-ok" : ""}>
                        {mediaTest.data.detail
                          ? t(mediaTest.data.detail)
                          : mediaTest.data.ok ? t("Loa phát âm thanh thành công") : t("Loa không hoạt động")}
                      </span>
                    )}
                  </div>
                </>
              )}

              {tab === "audio" && (
                <>
                  <label className="config-field">
                    <span><Mic2 size={17} /> {t("Loại nguồn âm thanh")}</span>
                    <select
                      value={form.audio_source_type}
                      onChange={(event) => {
                        const audio_source_type = event.target.value as RobotConfigurationUpdate["audio_source_type"];
                        setForm({
                          ...form,
                          audio_source_type,
                          audio_source: "",
                        });
                      }}
                    >
                      <option value="device">Microphone USB/ALSA</option>
                      <option value="file">{t("Tệp hoặc audio stream")}</option>
                      <option value="silent">{t("Không dùng microphone")}</option>
                    </select>
                  </label>
                  <label className="config-field">
                    <span>{t("Tên microphone")}</span>
                    <input
                      value={form.microphone_label}
                      onChange={(event) => setForm({ ...form, microphone_label: event.target.value })}
                      required
                    />
                  </label>
                  <label className="config-field config-field--wide">
                    <span><Mic2 size={17} /> {t("Nguồn microphone")}</span>
                    <div className="source-picker">
                      {form.audio_source_type === "device" ? (
                          <select
                            value={form.audio_source}
                            onChange={(event) => setForm({ ...form, audio_source: event.target.value })}
                            required
                            aria-describedby="audio-source-status"
                          >
                            <option value="" disabled>
                              {sourcesScanned.audio
                                ? audioSources.length
                                  ? t("Chọn microphone vừa tìm thấy")
                                  : t("Không tìm thấy microphone ALSA")
                                : t("Bấm Quét để tìm microphone trên robot")}
                            </option>
                            {form.audio_source && !audioSources.some((source) => source.value === form.audio_source) && (
                              <option value={form.audio_source}>
                                {sourcesScanned.audio ? t("Không còn kết nối") : t("Cấu hình hiện tại")} · {form.audio_source}
                              </option>
                            )}
                            {audioSources.map((source) => (
                              <option value={source.value} key={`${source.type}-${source.value}`}>
                                {source.label} · {source.value}
                              </option>
                            ))}
                          </select>
                      ) : (
                        <input
                          value={form.audio_source}
                          disabled={form.audio_source_type === "silent"}
                          onChange={(event) => setForm({ ...form, audio_source: event.target.value })}
                          placeholder={t("/media/microphone.wav hoặc URL audio")}
                          required={form.audio_source_type === "file"}
                        />
                      )}
                      <button
                        type="button"
                        className="source-scan-button"
                        onClick={() => scanMediaSources("audio")}
                        disabled={robot?.status !== "online" || mediaSourceScan.isPending}
                      >
                        <RefreshCw
                          size={17}
                          className={mediaSourceScan.isPending && mediaSourceScan.variables === "audio" ? "is-spinning" : ""}
                        />
                        {mediaSourceScan.isPending && mediaSourceScan.variables === "audio" ? t("Đang quét…") : t("Quét")}
                      </button>
                    </div>
                    <small id="audio-source-status" className={
                      mediaSourceScan.isError && mediaSourceScan.variables === "audio"
                        ? "source-scan-status is-error"
                        : "source-scan-status"
                    }>
                      {mediaSourceScan.isPending && mediaSourceScan.variables === "audio"
                        ? t("Robot đang dò ALSA, PipeWire và Bluetooth; hãy nói vào microphone…")
                        : mediaSourceScan.isError && mediaSourceScan.variables === "audio"
                          ? mediaSourceScan.error instanceof Error
                            ? t(mediaSourceScan.error.message)
                            : t("Không quét được microphone trên robot")
                          : sourcesScanned.audio
                            ? audioSources.length
                              ? `${t("Đã xác minh {count} microphone có tín hiệu.", { count: audioSources.length })}${rejectedAudioSources.length ? ` ${t("Loại {count} nguồn không hoạt động.", { count: rejectedAudioSources.length })}` : ""} ${t("Cấu hình âm thanh chưa thay đổi; bấm Lưu cấu hình để áp dụng.")}`
                              : rejectedAudioSources.length
                                ? (() => {
                                    const rejectedSource = rejectedAudioSources.find((source) => source.type === "pulse")
                                      ?? rejectedAudioSources[0];
                                    return `${t("Không có microphone hoạt động.")} ${rejectedSource.label}: ${t(rejectedSource.reason)}`;
                                  })()
                                : t("Robot không phát hiện microphone nào. Với Bluetooth, chọn HSP/HFP, bật tiếng rồi quét khi đang nói.")
                            : form.audio_source_type === "device"
                              ? t("Bấm Quét và nói vào microphone; thao tác quét không thay đổi cấu hình âm thanh.")
                              : t("Bấm Quét để dò nguồn đang hoạt động; chỉ nút Lưu cấu hình mới áp dụng thay đổi âm thanh.")}
                    </small>
                  </label>
                  <div className="media-test-actions config-field--wide">
                    <button type="button" className="button button--outline" disabled={mediaTest.isPending} onClick={() => mediaTest.mutate("audio")}>
                      <Mic2 size={16} /> {t("Kiểm tra microphone")}
                    </button>
                    {mediaTest.data?.diagnostic === "audio" && (
                      <span className={mediaTest.data.ok ? "is-ok" : ""}>
                        {mediaTest.data.detail
                          ? t(mediaTest.data.detail)
                          : mediaTest.data.ok ? t("Microphone có tín hiệu") : t("Microphone không hoạt động")}
                      </span>
                    )}
                  </div>
                </>
              )}
            </div>
          )}

          {save.isError && (
            <p role="alert" className="form-error">
              {save.error instanceof Error ? t(save.error.message) : t("Không lưu được cấu hình")}
            </p>
          )}
          <div className="configuration-actions">
            <span />
            <button type="button" className="button button--outline" onClick={() => navigate("/robots")}>{t("Huỷ")}</button>
            <button
              type="submit"
              className="button button--primary"
              disabled={loading || configurationQuery.isError || save.isPending}
            >
              <Save size={18} /> {save.isPending ? t("Đang lưu…") : t("Lưu cấu hình")}
            </button>
          </div>
        </form>
      </section>

      {previewOpen && (
        <div className="video-preview-backdrop" role="presentation" onMouseDown={(event) => {
          if (event.target !== event.currentTarget) return;
          setPreviewOpen(false);
          void stopPreview();
        }}>
          <section className="video-preview-modal" role="dialog" aria-modal="true" aria-labelledby="video-preview-title">
            <header>
              <span className="video-preview-modal__icon"><Camera size={19} /></span>
              <div>
                <h2 id="video-preview-title">{t("Xem trước camera robot")}</h2>
                <p>{robot?.name ?? robotId} · {form.camera_label}</p>
              </div>
              <span className={`video-preview-modal__status${previewState === "connected" ? " is-live" : ""}`}>
                <i /> {previewState === "connected" ? t("Camera trực tiếp") : previewState === "connecting" ? t("Đang mở camera…") : t("Chưa có tín hiệu video")}
              </span>
              <button type="button" autoFocus aria-label={t("Đóng")} onClick={() => {
                setPreviewOpen(false);
                void stopPreview();
              }}><X size={18} /></button>
            </header>
            <div className={`video-preview-modal__stage${previewState === "connected" ? " is-live" : ""}`}>
              <video ref={videoRef} autoPlay playsInline aria-label={t("Xem trước camera robot")} />
              <audio ref={audioRef} autoPlay />
              {previewState !== "connected" && <div className="video-preview-modal__empty">
                <Camera size={32} />
                <strong>{previewState === "connecting" ? t("Đang kết nối camera…") : t("Không nhận được hình ảnh")}</strong>
                <small>{previewState === "idle" || previewState === "connecting" ? t("Vui lòng chờ trong giây lát.") : t(previewState)}</small>
                {previewState !== "idle" && previewState !== "connecting" && <button type="button" onClick={() => {
                  previewRequested.current = true;
                  setPreviewState("idle");
                }}>{t("Thử lại")}</button>}
              </div>}
            </div>
            <footer>
              <span>{form.video_profile === "full_hd" ? "Full HD · 1080p" : form.video_profile === "balanced" ? `${t("Cân bằng")} · 720p` : `${t("Băng thông thấp")} · 480p`}</span>
              <button type="button" className="button button--outline" onClick={() => {
                setPreviewOpen(false);
                void stopPreview();
              }}><Square size={15} /> {t("Dừng xem trước")}</button>
            </footer>
          </section>
        </div>
      )}

      {form.video_source_type === "rtsp" && onvifDialogOpen && (
        <div
          className="onvif-modal-backdrop"
          role="presentation"
          onMouseDown={(event) => {
            if (event.target === event.currentTarget && !onvifScan.isPending) {
              setOnvifDialogOpen(false);
              setOnvifSelectedHost("");
            }
          }}
        >
          <section
            className="onvif-modal"
            role="dialog"
            aria-modal="true"
            aria-labelledby="onvif-modal-title"
            onKeyDown={(event) => {
              if (event.key === "Escape" && !onvifScan.isPending) {
                setOnvifDialogOpen(false);
                setOnvifSelectedHost("");
              }
            }}
          >
            <header className="onvif-modal__header">
              <button
                type="button"
                className={`onvif-modal__back ${selectedOnvifDevice ? "is-visible" : ""}`}
                aria-label={t("Quay lại danh sách camera")}
                onClick={() => {
                  onvifScan.reset();
                  setOnvifSelectedHost("");
                  setOnvifCredentials({ username: "", password: "" });
                }}
              >
                <ArrowLeft size={17} />
              </button>
              <span className="onvif-modal__mark"><Radar size={18} /></span>
              <div>
                <h2 id="onvif-modal-title">
                  {selectedOnvifDevice ? selectedOnvifDevice.name : t("Chọn camera ONVIF")}
                </h2>
                <p>
                  {selectedOnvifDevice
                    ? `${selectedOnvifDevice.host} · ${selectedOnvifDevice.profiles.length
                      ? t("Chọn luồng RTSP")
                      : t("Đăng nhập riêng cho camera này")}`
                    : t("Các camera được phát hiện trong cùng mạng LAN")}
                </p>
              </div>
              <button
                type="button"
                className="onvif-modal__close"
                aria-label={t("Đóng")}
                disabled={onvifScan.isPending}
                onClick={() => {
                  setOnvifDialogOpen(false);
                  setOnvifSelectedHost("");
                }}
              >
                <X size={17} />
              </button>
            </header>

            <div className="onvif-modal__body">
              {onvifScan.isPending && !onvifScan.variables?.target_host ? (
                <div className="onvif-modal__scanning" aria-live="polite">
                  <span><Radar size={24} /></span>
                  <strong>{t("Đang quét camera ONVIF…")}</strong>
                  <small>{t("Robot đang dò các thiết bị trong cùng dải mạng.")}</small>
                  <i /><i /><i />
                </div>
              ) : selectedOnvifDevice ? (
                selectedOnvifDevice.profiles.length ? (
                  <div className="onvif-profile-list onvif-profile-list--modal">
                    {selectedOnvifDevice.profiles.map((profile) => (
                      <button
                        type="button"
                        key={`${selectedOnvifDevice.host}-${profile.token}`}
                        className={form.video_source === profile.rtsp_url ? "is-selected" : ""}
                        onClick={() => selectOnvifProfile(
                          selectedOnvifDevice,
                          profile.rtsp_url,
                          profile.name,
                        )}
                      >
                        <span>
                          <strong>{profile.name}</strong>
                          <small>
                            {profile.width && profile.height
                              ? `${profile.width}×${profile.height}`
                              : profile.encoding || "RTSP"}
                            {profile.fps ? ` · ${profile.fps} fps` : ""}
                            {profile.bitrate_kbps ? ` · ${profile.bitrate_kbps} kbps` : ""}
                            {` · ${profile.encoding || "codec ?"}`}
                            {` · Mức ${profile.support_level} · ${profile.route}`}
                            {profile.ptz ? ` · PTZ` : ""}
                          </small>
                          {profile.warning && <small>{t(profile.warning)}</small>}
                        </span>
                        <code>{profile.path}</code>
                        <Check size={16} />
                      </button>
                    ))}
                  </div>
                ) : (
                  <div className="onvif-login-panel">
                    <div className="onvif-login-panel__intro">
                      <span><KeyRound size={18} /></span>
                      <div>
                        <strong>{t("Nhập tài khoản của camera này")}</strong>
                        <small>{t("Mỗi camera dùng tài khoản riêng; thông tin này không hiển thị trong URL RTSP.")}</small>
                      </div>
                    </div>

                    {selectedOnvifDevice.error && (
                      <p className={
                        selectedOnvifDevice.error === "Tài khoản hoặc mật khẩu ONVIF không đúng"
                          ? "onvif-device__error is-error"
                          : "onvif-device__error"
                      } role={selectedOnvifDevice.error === "Tài khoản hoặc mật khẩu ONVIF không đúng" ? "alert" : undefined}>
                        {selectedOnvifDevice.auth_required
                          && selectedOnvifDevice.error !== "Tài khoản hoặc mật khẩu ONVIF không đúng"
                          ? t("Camera yêu cầu đăng nhập để đọc profile chính xác.")
                          : t(selectedOnvifDevice.error)}
                      </p>
                    )}

                    <div className="onvif-auth-fields onvif-auth-fields--modal">
                      <label>
                        <span>{t("Tài khoản ONVIF")}</span>
                        <input
                          value={onvifCredentials.username}
                          onChange={(event) => setOnvifCredentials((current) => ({
                            ...current,
                            username: event.target.value,
                          }))}
                          onKeyDown={(event) => {
                            if (event.key === "Enter" && onvifCredentials.username.trim()) {
                              event.preventDefault();
                              authenticateOnvifDevice(selectedOnvifDevice.host);
                            }
                          }}
                          placeholder={t("Nhập tài khoản")}
                          autoComplete="username"
                          autoFocus
                        />
                      </label>
                      <label>
                        <span>{t("Mật khẩu ONVIF")}</span>
                        <input
                          type="password"
                          value={onvifCredentials.password}
                          onChange={(event) => setOnvifCredentials((current) => ({
                            ...current,
                            password: event.target.value,
                          }))}
                          onKeyDown={(event) => {
                            if (event.key === "Enter" && onvifCredentials.username.trim()) {
                              event.preventDefault();
                              authenticateOnvifDevice(selectedOnvifDevice.host);
                            }
                          }}
                          placeholder={t("Nhập mật khẩu")}
                          autoComplete="current-password"
                        />
                      </label>
                      <button
                        type="button"
                        className="onvif-auth-submit"
                        disabled={
                          !onvifCredentials.username.trim()
                          || (onvifScan.isPending
                            && onvifScan.variables?.target_host === selectedOnvifDevice.host)
                        }
                        onClick={() => authenticateOnvifDevice(selectedOnvifDevice.host)}
                      >
                        {onvifScan.isPending
                          && onvifScan.variables?.target_host === selectedOnvifDevice.host
                          ? t("Đang đăng nhập…")
                          : t("Đăng nhập và đọc profile")}
                      </button>
                    </div>

                    {Boolean(selectedOnvifDevice.suggested_profiles?.length) && (
                      <div className="onvif-suggested-paths onvif-suggested-paths--modal">
                        <small>{t("Path RTSP phổ biến của hãng")}</small>
                        {selectedOnvifDevice.suggested_profiles!.map((profile) => (
                          <div key={`${selectedOnvifDevice.host}-${profile.token}`}>
                            <span>{t(profile.name)}</span>
                            <code>{profile.path}</code>
                          </div>
                        ))}
                      </div>
                    )}
                  </div>
                )
              ) : onvifDevices.length ? (
                <div className="onvif-camera-list" aria-label={t("Camera ONVIF đã tìm thấy")}>
                  <div className="onvif-camera-list__summary">
                    <strong>{t("Tìm thấy {count} camera", { count: onvifDevices.length })}</strong>
                    <small>{t("Chọn một camera để tiếp tục")}</small>
                  </div>
                  {onvifDevices.map((device) => (
                    <button
                      type="button"
                      className="onvif-camera-row"
                      key={`${device.host}-${device.xaddr}`}
                      onClick={() => {
                        onvifScan.reset();
                        setOnvifSelectedHost(device.host);
                        setOnvifCredentials({ username: "", password: "" });
                      }}
                    >
                      <span className="onvif-camera-row__icon"><Camera size={18} /></span>
                      <span className="onvif-camera-row__identity">
                        <strong>{device.name}</strong>
                        <small>{device.host}</small>
                      </span>
                      <span className={device.profiles.length
                        ? "onvif-camera-row__state is-ready"
                        : "onvif-camera-row__state"}>
                        {device.profiles.length ? <Check size={13} /> : <KeyRound size={13} />}
                        {device.profiles.length ? t("Sẵn sàng") : t("Cần đăng nhập")}
                      </span>
                      <ArrowLeft className="onvif-camera-row__arrow" size={16} />
                    </button>
                  ))}
                </div>
              ) : (
                <div className="onvif-modal__empty">
                  <Radar size={24} />
                  <strong>{onvifScan.isError
                    ? t("Không quét được camera ONVIF")
                    : t("Không tìm thấy camera ONVIF")}</strong>
                  <small>{onvifScan.isError && onvifScan.error instanceof Error
                    ? t(onvifScan.error.message)
                    : t("Kiểm tra kết nối mạng của robot rồi quét lại.")}</small>
                  <button
                    type="button"
                    className="onvif-auth-submit"
                    onClick={() => {
                      onvifScan.reset();
                      onvifScan.mutate({});
                    }}
                  >
                    <RefreshCw size={14} /> {t("Quét lại")}
                  </button>
                </div>
              )}
            </div>
          </section>
        </div>
      )}
    </OperationsShell>
  );
}
