import { useState } from "react";
import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";
import {
  ArrowLeft, Ban, Check, ChevronLeft, ChevronRight, KeyRound, Plus,
  Search, ShieldCheck, UserCog, UserRound, UsersRound, X,
} from "lucide-react";
import { api } from "../api/client";
import { AccountMenu } from "../components/AccountMenu";
import { Brand } from "../components/Brand";
import { GlobalLanguageSelect } from "../components/GlobalLanguageSelect";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";
import type { AdminUserCreateInput, User } from "../types";

const ROLE_LABELS = {
  admin: "Quản trị viên",
  operator: "Vận hành",
  guest: "Khách",
} as const;

const EMPTY_FORM: AdminUserCreateInput = {
  full_name: "",
  username: "",
  email: "",
  password: "",
  role: "operator",
  must_change_password: true,
};

export function UserManagementPage() {
  const { t } = useI18n();
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const currentUser = useAppStore((state) => state.user);
  const isAdmin = currentUser?.role === "admin";
  const [page, setPage] = useState(1);
  const [search, setSearch] = useState("");
  const [role, setRole] = useState("all");
  const [status, setStatus] = useState("all");
  const [showCreate, setShowCreate] = useState(false);
  const [form, setForm] = useState<AdminUserCreateInput>(() => ({
    ...EMPTY_FORM,
    role: isAdmin ? "operator" : "guest",
  }));
  const [formError, setFormError] = useState("");
  const [resetTarget, setResetTarget] = useState<User | null>(null);
  const [resetPassword, setResetPassword] = useState("");
  const [actionMessage, setActionMessage] = useState("");

  const usersQuery = useQuery({
    queryKey: ["admin-users", page, search, role, status],
    queryFn: () => api.users({ page, pageSize: 10, search, role, status }),
  });

  function refreshUsers() {
    void queryClient.invalidateQueries({ queryKey: ["admin-users"] });
  }

  const createMutation = useMutation({
    mutationFn: api.createUser,
    onSuccess: () => {
      setForm({ ...EMPTY_FORM, role: isAdmin ? "operator" : "guest" });
      setShowCreate(false);
      setFormError("");
      setActionMessage(t(isAdmin ? "Đã tạo tài khoản nhân viên" : "Đã tạo tài khoản khách"));
      refreshUsers();
    },
    onError: (reason) => {
      setFormError(reason instanceof Error ? reason.message : t("Không thể tạo tài khoản"));
    },
  });

  const updateMutation = useMutation({
    mutationFn: ({ userId, input }: {
      userId: string;
      input: { role?: "operator" | "guest"; active?: boolean };
    }) => api.updateUser(userId, input),
    onSuccess: () => {
      setActionMessage(t("Đã cập nhật quyền tài khoản"));
      refreshUsers();
    },
    onError: (reason) => {
      setActionMessage(reason instanceof Error ? reason.message : t("Không thể cập nhật tài khoản"));
    },
  });

  const resetMutation = useMutation({
    mutationFn: ({ userId, password }: { userId: string; password: string }) =>
      api.resetUserPassword(userId, password),
    onSuccess: () => {
      setActionMessage(t("Đã đặt mật khẩu tạm thời"));
      setResetTarget(null);
      setResetPassword("");
      refreshUsers();
    },
    onError: (reason) => {
      setActionMessage(reason instanceof Error ? reason.message : t("Không thể đặt lại mật khẩu"));
    },
  });

  const summary = usersQuery.data?.summary ?? {
    total: 0,
    admin: 0,
    operator: 0,
    guest: 0,
    inactive: 0,
  };
  const users = usersQuery.data?.items ?? [];

  function submitCreate(event: React.FormEvent) {
    event.preventDefault();
    setFormError("");
    createMutation.mutate(form);
  }

  return (
    <main className="account-page user-admin-page">
      <header className="account-header">
        <div>
          <Brand compact />
          <span className="header-divider" />
          <button type="button" onClick={() => navigate("/robots")}><ArrowLeft size={18} /> {t("Quản lý robot")}</button>
        </div>
        <GlobalLanguageSelect />
        <AccountMenu />
      </header>

      <div className="user-admin-page__content">
        <section className="user-admin-heading">
          <div>
            <p className="eyebrow">PEOPLE · ROLES · ACCESS</p>
            <h1>{t("Quản lý tài khoản")}</h1>
            <p>{t(isAdmin
              ? "Tạo tài khoản vận hành, phân quyền và kiểm soát trạng thái truy cập."
              : "Tạo, khoá và đặt lại mật khẩu cho các tài khoản khách.")}</p>
          </div>
          <button type="button" className="button button--primary" onClick={() => setShowCreate((value) => !value)}>
            {showCreate ? <X size={18} /> : <Plus size={18} />}
            {showCreate ? t("Đóng biểu mẫu") : t(isAdmin ? "Tạo tài khoản nhân viên" : "Tạo tài khoản khách")}
          </button>
        </section>

        <section className={`role-summary${isAdmin ? "" : " role-summary--operator"}`} aria-label={t("Phân loại tài khoản")}>
          {(isAdmin ? ([
            ["all", "Tất cả", summary.total, UsersRound],
            ["admin", "Quản trị viên", summary.admin, ShieldCheck],
            ["operator", "Vận hành", summary.operator, UserCog],
            ["guest", "Khách", summary.guest, UserRound],
          ] as const) : ([
            ["all", "Tài khoản khách", summary.total, UsersRound],
            ["guest", "Đang quản lý", summary.guest, UserRound],
          ] as const)).map(([value, label, count, Icon]) => (
            <button
              type="button"
              key={value}
              className={role === value ? "is-active" : ""}
              onClick={() => {
                setRole(value);
                setPage(1);
              }}
            >
              <Icon size={18} />
              <span><small>{t(label)}</small><strong>{count}</strong></span>
            </button>
          ))}
        </section>

        {showCreate && (
          <section className="create-user-panel">
            <div className="create-user-panel__intro">
              <span><UserCog size={24} /></span>
              <div>
                <h2>{t(isAdmin ? "Tài khoản nhân viên mới" : "Tài khoản khách mới")}</h2>
                <p>{t(isAdmin
                  ? "Admin tạo tài khoản vận hành hoặc khách. Nhân viên phải đổi mật khẩu ở lần bàn giao đầu tiên."
                  : "Nhân viên vận hành chỉ tạo và quản lý tài khoản khách; không thể xem hay thay đổi admin.")}</p>
              </div>
            </div>
            <form onSubmit={submitCreate}>
              <label className="form-field"><span>{t("Họ và tên")}</span><input value={form.full_name} onChange={(event) => setForm({ ...form, full_name: event.target.value })} required /></label>
              <label className="form-field"><span>{t("Tên đăng nhập")}</span><input value={form.username} pattern="[a-z0-9][a-z0-9._-]{2,31}" onChange={(event) => setForm({ ...form, username: event.target.value.toLocaleLowerCase() })} required /></label>
              <label className="form-field"><span>Email</span><input type="email" value={form.email} onChange={(event) => setForm({ ...form, email: event.target.value })} required /></label>
              <label className="form-field"><span>{t("Mật khẩu tạm thời")}</span><input type="password" minLength={8} value={form.password} onChange={(event) => setForm({ ...form, password: event.target.value })} required /></label>
              <label className="form-field">
                <span>{t("Loại tài khoản")}</span>
                <select
                  value={form.role}
                  disabled={!isAdmin}
                  onChange={(event) => setForm({ ...form, role: event.target.value as "operator" | "guest" })}
                >
                  <option value="operator">{t("Nhân viên vận hành")}</option>
                  <option value="guest">{t("Tài khoản khách")}</option>
                </select>
              </label>
              <label className="checkbox create-user-panel__checkbox">
                <input type="checkbox" checked={form.must_change_password} onChange={(event) => setForm({ ...form, must_change_password: event.target.checked })} />
                <span>{t("Yêu cầu đổi mật khẩu khi bàn giao")}</span>
              </label>
              {formError && <p role="alert" className="form-error">{formError}</p>}
              <button className="button button--primary" disabled={createMutation.isPending}>{createMutation.isPending ? t("Đang tạo…") : t("Tạo tài khoản")}</button>
            </form>
          </section>
        )}

        <section className="user-directory">
          <div className="user-directory__toolbar">
            <label className="fleet-search">
              <Search size={17} />
              <input
                value={search}
                placeholder={t("Tìm tên, username hoặc email…")}
                onChange={(event) => {
                  setSearch(event.target.value);
                  setPage(1);
                }}
              />
            </label>
            <select value={status} onChange={(event) => { setStatus(event.target.value); setPage(1); }}>
              <option value="all">{t("Tất cả trạng thái")}</option>
              <option value="active">{t("Đang hoạt động")}</option>
              <option value="inactive">{t("Đã vô hiệu hoá")}</option>
            </select>
            <span className="user-directory__inactive"><Ban size={15} /> {summary.inactive} {t("đã khoá")}</span>
          </div>

          {actionMessage && <div className="notice notice--warning user-action-message">{actionMessage}</div>}

          <div className="user-table" role="table" aria-label={t("Danh sách tài khoản")}>
            <div className="user-table__head" role="row">
              <span>{t("Tài khoản")}</span>
              <span>{t("Loại tài khoản")}</span>
              <span>{t("Đăng nhập gần nhất")}</span>
              <span>{t("Trạng thái")}</span>
              <span>{t("Thao tác")}</span>
            </div>
            {usersQuery.isLoading ? (
              Array.from({ length: 5 }, (_, index) => <div className="user-row user-row--loading" key={index} />)
            ) : users.length === 0 ? (
              <div className="user-directory__empty"><UsersRound size={30} /><strong>{t("Không tìm thấy tài khoản")}</strong></div>
            ) : users.map((account) => (
              <div className={`user-row${!account.active ? " is-inactive" : ""}`} role="row" key={account.id}>
                <div className="user-row__identity">
                  <span className="account-avatar">
                    {account.avatar_url ? <img src={account.avatar_url} alt={account.full_name} referrerPolicy="no-referrer" /> : account.full_name.slice(0, 1).toLocaleUpperCase()}
                  </span>
                  <span><strong>{account.full_name}</strong><small>@{account.username} · {account.email}</small></span>
                </div>
                <div>
                  {account.role === "admin" ? (
                    <span className="role-badge role-badge--admin"><ShieldCheck size={14} /> {t(ROLE_LABELS.admin)}</span>
                  ) : !isAdmin ? (
                    <span className="role-badge role-badge--guest"><UserRound size={14} /> {t(ROLE_LABELS.guest)}</span>
                  ) : (
                    <select
                      className={`role-select role-select--${account.role}`}
                      value={account.role}
                      aria-label={t("Vai trò của {name}", { name: account.full_name })}
                      disabled={updateMutation.isPending}
                      onChange={(event) => updateMutation.mutate({
                        userId: account.id,
                        input: { role: event.target.value as "operator" | "guest" },
                      })}
                    >
                      <option value="operator">{t(ROLE_LABELS.operator)}</option>
                      <option value="guest">{t(ROLE_LABELS.guest)}</option>
                    </select>
                  )}
                </div>
                <span className="user-row__time">{account.last_login_at ? new Date(account.last_login_at).toLocaleString("vi-VN", { dateStyle: "short", timeStyle: "short" }) : t("Chưa đăng nhập")}</span>
                <span className={account.active ? "account-status is-active" : "account-status"}>
                  <i /> {account.active ? t("Đang hoạt động") : t("Đã vô hiệu hoá")}
                </span>
                <div className="user-row__actions">
                  {account.role === "admin" ? (
                    <span className="admin-lock"><ShieldCheck size={15} /> {t("Tài khoản gốc")}</span>
                  ) : (
                    <>
                      <button type="button" onClick={() => { setResetTarget(account); setResetPassword(""); }}><KeyRound size={16} /> {t("Đặt mật khẩu")}</button>
                      <button
                        type="button"
                        className={account.active ? "is-danger" : "is-enable"}
                        disabled={updateMutation.isPending}
                        onClick={() => updateMutation.mutate({ userId: account.id, input: { active: !account.active } })}
                      >
                        {account.active ? <Ban size={16} /> : <Check size={16} />}
                        {account.active ? t("Khoá") : t("Mở")}
                      </button>
                    </>
                  )}
                </div>
              </div>
            ))}
          </div>

          {resetTarget && (
            <form
              className="password-reset-strip"
              onSubmit={(event) => {
                event.preventDefault();
                resetMutation.mutate({ userId: resetTarget.id, password: resetPassword });
              }}
            >
              <span><KeyRound size={18} /><strong>{t("Đặt mật khẩu tạm cho {name}", { name: resetTarget.full_name })}</strong></span>
              <input type="password" minLength={8} value={resetPassword} placeholder={t("Ít nhất 8 ký tự")} onChange={(event) => setResetPassword(event.target.value)} required autoFocus />
              <button type="submit" className="button button--primary" disabled={resetMutation.isPending}>{t("Xác nhận")}</button>
              <button type="button" className="button" onClick={() => setResetTarget(null)}>{t("Huỷ")}</button>
            </form>
          )}

          <footer className="user-directory__pagination">
            <span>{t("{total} tài khoản", { total: usersQuery.data?.total ?? 0 })}</span>
            <div>
              <button type="button" disabled={page <= 1} onClick={() => setPage((value) => Math.max(1, value - 1))}><ChevronLeft size={18} /></button>
              <strong>{page} / {usersQuery.data?.total_pages ?? 1}</strong>
              <button type="button" disabled={page >= (usersQuery.data?.total_pages ?? 1)} onClick={() => setPage((value) => value + 1)}><ChevronRight size={18} /></button>
            </div>
          </footer>
        </section>
      </div>
    </main>
  );
}
