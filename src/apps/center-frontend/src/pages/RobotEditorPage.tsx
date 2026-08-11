import { useEffect, useState } from "react";
import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";
import {
  Eye, EyeOff, KeyRound, LockKeyhole, Network,
  Save, Trash2, UserRound,
} from "lucide-react";
import { api } from "../api/client";
import { OperationsShell } from "../components/OperationsShell";
import { showToast } from "../components/ToastViewport";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate, useParams } from "../router";
import type {
  RobotQuickCreateInput, RobotUpdateInput,
} from "../types";

const EMPTY_QUICK_FORM: RobotQuickCreateInput = {
  management_address: "",
  username: "",
  password: "",
};

const EMPTY_EDIT_FORM: RobotUpdateInput = {
  name: "",
  site_id: "",
  map_id: "",
  enabled: true,
  management_address: "",
  management_username: "",
  management_password: "",
};

export function RobotEditorPage({ mode }: { mode: "create" | "edit" }) {
  const navigate = useNavigate();
  const { t } = useI18n();
  const queryClient = useQueryClient();
  const { robotId = "" } = useParams();
  const [quickForm, setQuickForm] = useState(EMPTY_QUICK_FORM);
  const [editForm, setEditForm] = useState(EMPTY_EDIT_FORM);
  const [showPassword, setShowPassword] = useState(false);
  const [confirmDelete, setConfirmDelete] = useState(false);
  const [changePassword, setChangePassword] = useState(false);

  const robotQuery = useQuery({
    queryKey: ["robot", robotId],
    queryFn: () => api.robot(robotId),
    enabled: mode === "edit" && Boolean(robotId),
  });
  const mapsQuery = useQuery({
    queryKey: ["maps"],
    queryFn: () => api.maps(),
    enabled: mode === "edit",
  });
  const assignableMaps = (mapsQuery.data ?? []).filter(
    (item) => item.status !== "DELETED" && item.active_version != null,
  );

  useEffect(() => {
    if (!robotQuery.data) return;
    setEditForm({
      name: robotQuery.data.name,
      site_id: robotQuery.data.site_id,
      map_id: robotQuery.data.map_id,
      enabled: robotQuery.data.enabled,
      management_address: robotQuery.data.management_address ?? "",
      management_username: robotQuery.data.management_username ?? "",
      management_password: "",
    });
  }, [robotQuery.data]);

  const create = useMutation({
    mutationFn: () => api.quickAddRobot(quickForm),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: ["robots"] });
      showToast(t("ĐÃ THÊM ROBOT"));
      navigate("/robots");
    },
  });
  const update = useMutation({
    mutationFn: () => api.updateRobot(robotId, {
      name: editForm.name,
      site_id: editForm.site_id,
      map_id: editForm.map_id,
      enabled: editForm.enabled,
      management_address: editForm.management_address?.trim() || undefined,
      management_username: editForm.management_username?.trim() || undefined,
      management_password: changePassword
        ? editForm.management_password
        : undefined,
    }),
    onSuccess: () => {
      setChangePassword(false);
      setEditForm((current) => ({ ...current, management_password: "" }));
      void queryClient.invalidateQueries({ queryKey: ["robots"] });
      void queryClient.invalidateQueries({ queryKey: ["robot", robotId] });
      showToast(t("Đã lưu thay đổi"));
    },
  });
  const remove = useMutation({
    mutationFn: () => api.deleteRobot(robotId),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: ["robots"] });
      navigate("/robots");
    },
  });

  const error = create.error ?? update.error ?? remove.error;
  const isOnline = robotQuery.data?.status === "online";

  return (
    <OperationsShell title={mode === "create" ? "Thêm robot" : editForm.name || "Chỉnh sửa robot"} className="robot-editor-operations">
      <section className="robot-editor-shell robot-editor-shell--single">
        <form
          className="robot-editor-form"
          onSubmit={(event) => {
            event.preventDefault();
            if (mode === "create") create.mutate();
            else update.mutate();
          }}
        >
          <div className="editor-form-heading">
            <div>
              <p className="eyebrow">{mode === "create" ? t("KẾT NỐI NHANH") : t("HỒ SƠ ROBOT")}</p>
              <h2>{mode === "create" ? t("Thông tin đăng nhập robot") : t("Chỉnh sửa robot")}</h2>
            </div>
            {mode === "edit" && (
              <span className={`connection-chip ${isOnline ? "is-online" : ""}`}>
                <i /> {isOnline ? t("Đang online") : t("Đang offline")}
              </span>
            )}
          </div>

          {mode === "create" ? (
                <div className="editor-fields editor-fields--quick">
                  <label className="config-field config-field--wide">
                    <span><Network size={17} /> {t("IP hoặc hostname robot")}</span>
                    <input
                      value={quickForm.management_address}
                      onChange={(event) => setQuickForm({
                        ...quickForm,
                        management_address: event.target.value,
                      })}
                      placeholder="192.168.1.20"
                      autoComplete="off"
                      required
                    />
                    <small>{t("Địa chỉ quản lý mà edge agent báo về Center.")}</small>
                  </label>
                  <label className="config-field">
                    <span><UserRound size={17} /> {t("Tài khoản robot")}</span>
                    <input
                      value={quickForm.username}
                      onChange={(event) => setQuickForm({
                        ...quickForm,
                        username: event.target.value,
                      })}
                      placeholder="operator"
                      autoComplete="username"
                      required
                    />
                  </label>
                  <label className="config-field">
                    <span><LockKeyhole size={17} /> {t("Mật khẩu robot")}</span>
                    <span className="password-control">
                      <input
                        type={showPassword ? "text" : "password"}
                        value={quickForm.password}
                        onChange={(event) => setQuickForm({
                          ...quickForm,
                          password: event.target.value,
                        })}
                        placeholder={t("Ít nhất 6 ký tự")}
                        minLength={6}
                        autoComplete="current-password"
                        required
                      />
                      <button
                        type="button"
                        aria-label={showPassword ? t("Ẩn mật khẩu") : t("Hiện mật khẩu")}
                        onClick={() => setShowPassword((value) => !value)}
                      >
                        {showPassword ? <EyeOff size={17} /> : <Eye size={17} />}
                      </button>
                    </span>
                  </label>
                  <div className="quick-add-note config-field--wide">
                    <LockKeyhole size={18} />
                    <span>
                      <strong>{t("Không lưu mật khẩu rõ")}</strong>
                      <small>{t("Center chỉ lưu hash PBKDF2. Robot luôn chủ động kết nối outbound.")}</small>
                    </span>
                  </div>
                </div>
          ) : robotQuery.isLoading ? (
                <div className="configuration-loading">{t("Đang tải hồ sơ robot…")}</div>
              ) : robotQuery.isError ? (
                <div className="configuration-error" role="alert">
                  <h3>{t("Không tải được robot")}</h3>
                  <p>{robotQuery.error instanceof Error ? t(robotQuery.error.message) : t("Robot không tồn tại")}</p>
                </div>
              ) : (
                <div className="editor-fields">
                  <label className="config-field">
                    <span>{t("Tên hiển thị")}</span>
                    <input
                      value={editForm.name}
                      onChange={(event) => setEditForm({ ...editForm, name: event.target.value })}
                      required
                    />
                  </label>
                  <label className="config-field">
                    <span>{t("Khu vực hoạt động")}</span>
                    <input
                      value={editForm.site_id}
                      onChange={(event) => setEditForm({ ...editForm, site_id: event.target.value })}
                      required
                    />
                  </label>
                  <label className="config-field">
                    <span>{t("IP hoặc hostname robot")}</span>
                    <input
                      value={editForm.management_address}
                      onChange={(event) => setEditForm({
                        ...editForm,
                        management_address: event.target.value,
                      })}
                      placeholder="192.168.1.20"
                    />
                  </label>
                  <label className="config-field">
                    <span>{t("Tài khoản robot")}</span>
                    <input
                      value={editForm.management_username}
                      onChange={(event) => setEditForm({
                        ...editForm,
                        management_username: event.target.value,
                      })}
                    />
                  </label>
                  <label className="config-field">
                    <span>{t("Map mặc định")}</span>
                    <select
                      value={editForm.map_id}
                      disabled={mapsQuery.isLoading || !assignableMaps.length}
                      onChange={(event) => setEditForm({ ...editForm, map_id: event.target.value })}
                    >
                      {mapsQuery.isLoading && <option value={editForm.map_id}>{t("Đang tải danh sách bản đồ…")}</option>}
                      {!mapsQuery.isLoading && !assignableMaps.length && <option value={editForm.map_id}>{t("Chưa có bản đồ đã kích hoạt")}</option>}
                      {editForm.map_id && !assignableMaps.some((item) => item.map_id === editForm.map_id) && (
                        <option value={editForm.map_id}>{editForm.map_id} · {t("Bản đồ hiện tại")}</option>
                      )}
                      {assignableMaps.map((item) => <option value={item.map_id} key={item.map_id}>
                        {item.name} · {item.site_id || "—"} / {item.floor_id || "—"} · v{item.active_version}
                      </option>)}
                    </select>
                    <small>{mapsQuery.isError ? t("Không tải được danh sách bản đồ") : t("Chọn bản đồ điều hướng mặc định cho robot.")}</small>
                  </label>
                  {changePassword ? (
                    <label className="config-field">
                      <span>{t("Mật khẩu mới")}</span>
                      <input
                        type="password"
                        name="rovera-robot-password-change"
                        value={editForm.management_password}
                        onChange={(event) => setEditForm({
                          ...editForm,
                          management_password: event.target.value,
                        })}
                        placeholder={t("Ít nhất 6 ký tự")}
                        minLength={6}
                        autoComplete="off"
                        required
                      />
                      <button
                        type="button"
                        className="inline-cancel-action"
                        onClick={() => {
                          setChangePassword(false);
                          setEditForm({ ...editForm, management_password: "" });
                        }}
                      >
                        {t("Không đổi mật khẩu")}
                      </button>
                    </label>
                  ) : (
                    <div className="password-reset-action">
                      <span><KeyRound size={17} /></span>
                      <div>
                        <strong>{t("Mật khẩu robot")}</strong>
                        <small>{t("Chỉ thay đổi khi bạn chủ động yêu cầu.")}</small>
                      </div>
                      <button type="button" onClick={() => setChangePassword(true)}>
                        {t("Đổi mật khẩu")}
                      </button>
                    </div>
                  )}
                  <label className="editor-toggle config-field--wide">
                    <input
                      type="checkbox"
                      checked={editForm.enabled}
                      onChange={(event) => setEditForm({ ...editForm, enabled: event.target.checked })}
                    />
                    <span>
                      <strong>{t("Cho phép robot kết nối")}</strong>
                      <small>{isOnline
                        ? t("Có thể sửa IP, tài khoản và mật khẩu mà không ngắt phiên hiện tại.")
                        : t("Vô hiệu hoá để chặn robot lấy JWT.")}
                      </small>
                    </span>
                  </label>
                </div>
          )}

          {error && (
                <p role="alert" className="form-error">
                  {error instanceof Error ? error.message : t("Không thể lưu robot")}
                </p>
              )}

          <div className="editor-actions">
                {mode === "edit" && (
                  <button
                    type="button"
                    className="button button--danger-outline"
                    disabled={isOnline || remove.isPending}
                    onClick={() => {
                      if (confirmDelete) remove.mutate();
                      else setConfirmDelete(true);
                    }}
                  >
                    <Trash2 size={17} />
                    {confirmDelete ? t("Nhấn lại để xác nhận xoá") : t("Xoá robot")}
                  </button>
                )}
                <span />
                <button type="button" className="button button--outline" onClick={() => navigate("/robots")}>
                  {t("Huỷ")}
                </button>
                <button
                  type="submit"
                  className="button button--primary"
                  disabled={create.isPending || update.isPending || robotQuery.isError || (mode === "edit" && !editForm.map_id)}
                >
                  <Save size={18} />
                  {create.isPending || update.isPending
                    ? t("Đang lưu…")
                    : mode === "create"
                      ? t("Thêm robot")
                      : t("Lưu thay đổi")}
                </button>
          </div>
        </form>
      </section>
    </OperationsShell>
  );
}
