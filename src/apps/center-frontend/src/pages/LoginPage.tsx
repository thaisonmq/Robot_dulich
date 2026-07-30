import { useState } from "react";
import { Activity, Eye, EyeOff, LockKeyhole, RadioTower } from "lucide-react";
import { api, authStorage } from "../api/client";
import { Brand } from "../components/Brand";
import { useNavigate } from "../router";
import { useAppStore } from "../state/appStore";

export function LoginPage() {
  const navigate = useNavigate();
  const setUser = useAppStore((state) => state.setUser);
  const [email, setEmail] = useState("demo@rovera.local");
  const [password, setPassword] = useState("demo123");
  const [showPassword, setShowPassword] = useState(false);
  const [remember, setRemember] = useState(true);
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  async function submit(event: React.FormEvent) {
    event.preventDefault();
    setLoading(true);
    setError("");
    try {
      const result = await api.login(email, password);
      authStorage.set(result.access_token);
      setUser(result.user);
      sessionStorage.setItem("rovera_user", JSON.stringify(result.user));
      if (remember) localStorage.setItem("rovera_email", email);
      navigate("/robots");
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : "Đăng nhập thất bại");
    } finally {
      setLoading(false);
    }
  }

  return (
    <main className="login-page">
      <section className="login-image" aria-label="Robot telepresence tại bảo tàng">
        <img src="/assets/login-robot-museum.png" alt="Robot telepresence trong hành lang bảo tàng" />
        <div className="login-image__overlay">
          <Brand />
          <div className="login-image__status">
            <span><Activity size={18} /> Realtime operations</span>
            <strong>Trung tâm vận hành robot</strong>
            <p>Giám sát đội hình, mở phiên video và điều khiển thiết bị trong một không gian thống nhất.</p>
          </div>
          <div className="login-image__signal"><RadioTower size={17} /> Gateway sẵn sàng</div>
        </div>
      </section>
      <section className="login-panel">
        <div className="login-panel__inner">
          <div className="login-security"><LockKeyhole size={18} /><span>Operator access</span></div>
          <div className="login-copy">
            <h1>Đăng nhập</h1>
            <p>Sử dụng tài khoản vận hành của bạn.</p>
          </div>
          <form className="login-form" onSubmit={submit}>
            <label className="form-field">
              <span>Email</span>
              <input
                id="email"
                type="email"
                value={email}
                autoComplete="email"
                onChange={(event) => setEmail(event.target.value)}
                required
              />
            </label>
            <div className="form-field">
              <label htmlFor="password">Mật khẩu</label>
              <span className="password-field">
                <input
                  id="password"
                  type={showPassword ? "text" : "password"}
                  value={password}
                  autoComplete="current-password"
                  onChange={(event) => setPassword(event.target.value)}
                  required
                />
                <button type="button" onClick={() => setShowPassword((value) => !value)} aria-label="Hiện mật khẩu">
                  {showPassword ? <EyeOff size={19} /> : <Eye size={19} />}
                </button>
              </span>
            </div>
            <div className="form-row">
              <label className="checkbox">
                <input type="checkbox" checked={remember} onChange={(event) => setRemember(event.target.checked)} />
                <span>Ghi nhớ đăng nhập</span>
              </label>
              <button type="button" className="text-button">Quên mật khẩu?</button>
            </div>
            {error && <p role="alert" className="form-error">{error}</p>}
            <button className="button button--primary button--large" disabled={loading}>
              {loading ? "Đang kết nối…" : "Đăng nhập"}
            </button>
            <div className="demo-credentials">
              <span>Tài khoản demo</span>
              <code>demo@rovera.local · demo123</code>
            </div>
          </form>
          <div className="system-ready"><i /> Kết nối trung tâm ổn định</div>
        </div>
      </section>
    </main>
  );
}
