import { useEffect, useRef, useState } from "react";
import { useMutation, useQuery } from "@tanstack/react-query";
import {
  Activity, ArrowLeft, Camera, Check, CircleDot, Cpu, EthernetPort, Mic2,
  Play, RadioTower, RefreshCw, Save, ServerCog, Square, Video, Volume2, WifiOff,
} from "lucide-react";
import { api } from "../api/client";
import { Brand } from "../components/Brand";
import { GlobalLanguageSelect } from "../components/GlobalLanguageSelect";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate, useParams } from "../router";
import type {
  MediaSource, RejectedMediaSource, RobotConfigurationUpdate,
} from "../types";
import type { LiveKitMediaTransport } from "../transports/MediaTransport";

const EMPTY_CONFIGURATION: RobotConfigurationUpdate = {
  device_ip: "",
  video_source_type: "rtsp",
  video_source: "",
  video_profile: "full_hd",
  rtsp_transport: "tcp",
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
  const { robotId = "" } = useParams();
  const [form, setForm] = useState<RobotConfigurationUpdate>(EMPTY_CONFIGURATION);
  const [saved, setSaved] = useState(false);
  const [tab, setTab] = useState<"connection" | "video" | "audio">("video");
  const [previewState, setPreviewState] = useState("idle");
  const [videoSources, setVideoSources] = useState<MediaSource[]>([]);
  const [audioSources, setAudioSources] = useState<MediaSource[]>([]);
  const [speakerSources, setSpeakerSources] = useState<MediaSource[]>([]);
  const [rejectedVideoSources, setRejectedVideoSources] = useState<RejectedMediaSource[]>([]);
  const [rejectedAudioSources, setRejectedAudioSources] = useState<RejectedMediaSource[]>([]);
  const [rejectedSpeakerSources, setRejectedSpeakerSources] = useState<RejectedMediaSource[]>([]);
  const [sourcesScanned, setSourcesScanned] = useState({
    video: false,
    audio: false,
    speaker: false,
  });
  const videoRef = useRef<HTMLVideoElement>(null);
  const audioRef = useRef<HTMLAudioElement>(null);
  const previewTransport = useRef<LiveKitMediaTransport | null>(null);
  const previewLeaseId = useRef<string | null>(null);

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
      setSaved(true);
      window.setTimeout(() => setSaved(false), 2200);
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

  function scanMediaSources(mediaKind: "video" | "audio" | "speaker") {
    mediaSourceScan.reset();
    mediaSourceScan.mutate(mediaKind);
  }

  async function togglePreview() {
    if (previewTransport.current) {
      await previewTransport.current.disconnect();
      previewTransport.current = null;
      const leaseId = previewLeaseId.current;
      previewLeaseId.current = null;
      if (leaseId) {
        await api.stopRobotPreview(robotId, leaseId).catch(() => undefined);
      }
      setPreviewState("idle");
      return;
    }
    if (!videoRef.current || !audioRef.current) return;
    setPreviewState("connecting");
    let leaseId: string | null = null;
    try {
      const access = await api.robotPreviewToken(robotId);
      leaseId = access.lease_id;
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
      setPreviewState(reason instanceof Error ? reason.message : "failed");
    }
  }

  const robot = robotQuery.data;
  const configuration = configurationQuery.data;
  const loading = robotQuery.isLoading || configurationQuery.isLoading;

  return (
    <main className="configuration-page">
      <header className="app-header">
        <Brand compact />
        <div className="app-header__context">
          <span>{t("Cấu hình thiết bị")}</span>
          <strong>{robot?.name ?? robotId}</strong>
        </div>
        <GlobalLanguageSelect />
        <button type="button" className="header-action" onClick={() => navigate("/robots")}>
          <ArrowLeft size={18} /> {t("Danh sách robot")}
        </button>
      </header>

      <section className="configuration-shell">
        <aside className="configuration-summary">
          <div className="device-orbit"><RadioTower size={36} /></div>
          <div>
            <p className="eyebrow">{t("THIẾT BỊ ĐANG CHỌN")}</p>
            <h1>{robot?.name ?? t("Đang tải robot")}</h1>
            <p>{robotId}</p>
          </div>
          <div className="configuration-health">
            <span><CircleDot size={17} /><small>{t("Kết nối")}</small><strong>{robot?.status === "online" ? t("Trực tuyến") : t("Ngoại tuyến")}</strong></span>
            <span><Cpu size={17} /><small>{t("Phiên bản")}</small><strong>{configuration?.software_version ?? "—"}</strong></span>
            <span><Video size={17} /><small>{t("Hồ sơ")}</small><strong>{form.video_profile === "full_hd" ? "Full HD" : form.video_profile === "balanced" ? t("Cân bằng") : t("Băng thông thấp")}</strong></span>
          </div>
          <div className={`config-preview ${previewState === "connected" ? "is-live" : ""}`}>
            <video ref={videoRef} autoPlay playsInline aria-label={t("Xem trước camera robot")} />
            <audio ref={audioRef} autoPlay />
            <span>
              <Camera size={18} />
              {previewState === "connected"
                ? t("Camera trực tiếp")
                : previewState === "connecting"
                  ? t("Đang mở camera…")
                  : t("Chưa xem trước")}
            </span>
          </div>
          <p className="configuration-note">
            {t("Cấu hình được đọc và áp dụng trực tiếp tại simulator. Thông tin đăng nhập RTSP luôn được ẩn khỏi trình duyệt.")}
          </p>
        </aside>

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
                    </div>
                    <small id="video-source-status" className={
                      mediaSourceScan.isError && mediaSourceScan.variables === "video"
                        ? "source-scan-status is-error"
                        : "source-scan-status"
                    }>
                      {mediaSourceScan.isPending && mediaSourceScan.variables === "video"
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
                              : t("Bấm Quét để dò camera USB; nguồn không trả về hình ảnh sẽ bị loại.")}
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
                      <option value="tcp">TCP · {t("ưu tiên ổn định")}</option>
                      <option value="udp">UDP · {t("ưu tiên độ trễ")}</option>
                    </select>
                  </label>
                  <div className="media-test-actions config-field--wide">
                    <button type="button" className="button button--outline" disabled={mediaTest.isPending} onClick={() => mediaTest.mutate("video")}>
                      <Activity size={16} /> {t("Kiểm tra nguồn camera")}
                    </button>
                    <button type="button" className="button button--outline" onClick={() => void togglePreview()}>
                      {previewTransport.current ? <Square size={15} /> : <Play size={15} />}
                      {previewTransport.current ? t("Dừng xem trước") : t("Xem trước camera")}
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
            <span>{saved && <><Check size={17} /> {t("Đã lưu cấu hình")}</>}</span>
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
    </main>
  );
}
