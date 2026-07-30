import { useEffect, useState } from "react";
import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";
import {
  ArrowLeft, Bot, Check, Eye, EyeOff, KeyRound, LockKeyhole, Network,
  Save, Trash2, UserRound,
} from "lucide-react";
import { api } from "../api/client";
import { Brand } from "../components/Brand";
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
  map_id: "MAP-001",
  enabled: true,
  management_address: "",
  management_username: "",
  management_password: "",
};

export function RobotEditorPage({ mode }: { mode: "create" | "edit" }) {
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const { robotId = "" } = useParams();
  const [quickForm, setQuickForm] = useState(EMPTY_QUICK_FORM);
  const [editForm, setEditForm] = useState(EMPTY_EDIT_FORM);
  const [createdRobotId, setCreatedRobotId] = useState("");
  const [showPassword, setShowPassword] = useState(false);
  const [confirmDelete, setConfirmDelete] = useState(false);
  const [saved, setSaved] = useState(false);
  const [changePassword, setChangePassword] = useState(false);

  const robotQuery = useQuery({
    queryKey: ["robot", robotId],
    queryFn: () => api.robot(robotId),
    enabled: mode === "edit" && Boolean(robotId),
  });

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
    onSuccess: (robot) => {
      setCreatedRobotId(robot.robot_id);
      void queryClient.invalidateQueries({ queryKey: ["robots"] });
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
      setSaved(true);
      setChangePassword(false);
      setEditForm((current) => ({ ...current, management_password: "" }));
      void queryClient.invalidateQueries({ queryKey: ["robots"] });
      void queryClient.invalidateQueries({ queryKey: ["robot", robotId] });
      window.setTimeout(() => setSaved(false), 1800);
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
    <main className="configuration-page robot-editor-page">
      <header className="app-header">
        <Brand compact />
        <div className="app-header__context">
          <span>Quản lý thiết bị</span>
          <strong>{mode === "create" ? "Thêm robot" : editForm.name || robotId}</strong>
        </div>
        <button type="button" className="header-action" onClick={() => navigate("/robots")}>
          <ArrowLeft size={18} /> Danh sách robot
        </button>
      </header>

      <section className="robot-editor-shell">
        <aside className="robot-editor-guide">
          <span className="editor-device-icon"><Bot size={38} /></span>
          <p className="eyebrow">ROBOT CONNECTION</p>
          <h1>{mode === "create" ? "Thêm robot trong vài giây" : "Thông tin quản lý"}</h1>
          <p>
            {mode === "create"
              ? "Nhập đúng thông tin đăng nhập có sẵn trên robot. Center sẽ nhận diện thiết bị khi edge agent hoạt động."
              : "Tên và khu vực thuộc Center; camera, microphone và kết nối phần cứng vẫn thuộc robot."}
          </p>
          <ol>
            <li><span>01</span>Nhập địa chỉ và tài khoản robot</li>
            <li><span>02</span>Center lưu mật khẩu dưới dạng hash</li>
            <li><span>03</span>Robot chạy sẽ tự chuyển online</li>
          </ol>
        </aside>

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
              <p className="eyebrow">{mode === "create" ? "KẾT NỐI NHANH" : "HỒ SƠ ROBOT"}</p>
              <h2>{mode === "create" ? "Thông tin đăng nhập robot" : "Chỉnh sửa robot"}</h2>
            </div>
            {mode === "edit" && (
              <span className={`connection-chip ${isOnline ? "is-online" : ""}`}>
                <i /> {isOnline ? "Đang online" : "Đang offline"}
              </span>
            )}
          </div>

          {mode === "create" && createdRobotId ? (
            <section className="quick-add-success">
              <span><Check size={30} /></span>
              <p className="eyebrow">ĐÃ THÊM ROBOT</p>
              <h3>{createdRobotId}</h3>
              <p>
                Robot đang được theo dõi. Nếu edge agent đang chạy, trạng thái sẽ
                tự chuyển online; nếu đang tắt, robot tiếp tục hiển thị offline.
              </p>
              <button type="button" className="button button--primary" onClick={() => navigate("/robots")}>
                Xem danh sách robot
              </button>
              <button type="button" className="text-action" onClick={() => {
                setCreatedRobotId("");
                setQuickForm(EMPTY_QUICK_FORM);
              }}>
                Thêm robot khác
              </button>
            </section>
          ) : (
            <>
              {mode === "create" ? (
                <div className="editor-fields editor-fields--quick">
                  <label className="config-field config-field--wide">
                    <span><Network size={17} /> IP hoặc hostname robot</span>
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
                    <small>Địa chỉ quản lý mà edge agent báo về Center.</small>
                  </label>
                  <label className="config-field">
                    <span><UserRound size={17} /> Tài khoản robot</span>
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
                    <span><LockKeyhole size={17} /> Mật khẩu robot</span>
                    <span className="password-control">
                      <input
                        type={showPassword ? "text" : "password"}
                        value={quickForm.password}
                        onChange={(event) => setQuickForm({
                          ...quickForm,
                          password: event.target.value,
                        })}
                        placeholder="Ít nhất 6 ký tự"
                        minLength={6}
                        autoComplete="current-password"
                        required
                      />
                      <button
                        type="button"
                        aria-label={showPassword ? "Ẩn mật khẩu" : "Hiện mật khẩu"}
                        onClick={() => setShowPassword((value) => !value)}
                      >
                        {showPassword ? <EyeOff size={17} /> : <Eye size={17} />}
                      </button>
                    </span>
                  </label>
                  <div className="quick-add-note config-field--wide">
                    <LockKeyhole size={18} />
                    <span>
                      <strong>Không lưu mật khẩu rõ</strong>
                      <small>Center chỉ lưu hash PBKDF2. Robot luôn chủ động kết nối outbound.</small>
                    </span>
                  </div>
                </div>
              ) : robotQuery.isLoading ? (
                <div className="configuration-loading">Đang tải hồ sơ robot…</div>
              ) : robotQuery.isError ? (
                <div className="configuration-error" role="alert">
                  <h3>Không tải được robot</h3>
                  <p>{robotQuery.error instanceof Error ? robotQuery.error.message : "Robot không tồn tại"}</p>
                </div>
              ) : (
                <div className="editor-fields">
                  <label className="config-field">
                    <span>Tên hiển thị</span>
                    <input
                      value={editForm.name}
                      onChange={(event) => setEditForm({ ...editForm, name: event.target.value })}
                      required
                    />
                  </label>
                  <label className="config-field">
                    <span>Khu vực hoạt động</span>
                    <input
                      value={editForm.site_id}
                      onChange={(event) => setEditForm({ ...editForm, site_id: event.target.value })}
                      required
                    />
                  </label>
                  <label className="config-field">
                    <span>IP hoặc hostname robot</span>
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
                    <span>Tài khoản robot</span>
                    <input
                      value={editForm.management_username}
                      onChange={(event) => setEditForm({
                        ...editForm,
                        management_username: event.target.value,
                      })}
                    />
                  </label>
                  <label className="config-field">
                    <span>Map mặc định</span>
                    <select
                      value={editForm.map_id}
                      onChange={(event) => setEditForm({ ...editForm, map_id: event.target.value })}
                    >
                      <option value="MAP-001">MAP-001 · Bản đồ bảo tàng</option>
                    </select>
                  </label>
                  {changePassword ? (
                    <label className="config-field">
                      <span>Mật khẩu mới</span>
                      <input
                        type="password"
                        name="rovera-robot-password-change"
                        value={editForm.management_password}
                        onChange={(event) => setEditForm({
                          ...editForm,
                          management_password: event.target.value,
                        })}
                        placeholder="Ít nhất 6 ký tự"
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
                        Không đổi mật khẩu
                      </button>
                    </label>
                  ) : (
                    <div className="password-reset-action">
                      <span><KeyRound size={17} /></span>
                      <div>
                        <strong>Mật khẩu robot</strong>
                        <small>Chỉ thay đổi khi bạn chủ động yêu cầu.</small>
                      </div>
                      <button type="button" onClick={() => setChangePassword(true)}>
                        Đổi mật khẩu
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
                      <strong>Cho phép robot kết nối</strong>
                      <small>{isOnline ? "Có thể sửa IP, tài khoản và mật khẩu mà không ngắt phiên hiện tại." : "Vô hiệu hoá để chặn robot lấy JWT."}</small>
                    </span>
                  </label>
                </div>
              )}

              {error && (
                <p role="alert" className="form-error">
                  {error instanceof Error ? error.message : "Không thể lưu robot"}
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
                    {confirmDelete ? "Nhấn lại để xác nhận xoá" : "Xoá robot"}
                  </button>
                )}
                <span>{saved ? "Đã lưu thay đổi" : ""}</span>
                <button type="button" className="button button--outline" onClick={() => navigate("/robots")}>
                  Huỷ
                </button>
                <button
                  type="submit"
                  className="button button--primary"
                  disabled={create.isPending || update.isPending || robotQuery.isError}
                >
                  <Save size={18} />
                  {create.isPending || update.isPending
                    ? "Đang lưu…"
                    : mode === "create"
                      ? "Thêm robot"
                      : "Lưu thay đổi"}
                </button>
              </div>
            </>
          )}
        </form>
      </section>
    </main>
  );
}
