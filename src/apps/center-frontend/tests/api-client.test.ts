import { afterEach, describe, expect, it, vi } from "vitest";
import { api, AUTH_EXPIRED_EVENT, authStorage } from "../src/api/client";

describe("API authentication", () => {
  afterEach(() => {
    authStorage.clear();
    sessionStorage.clear();
    vi.unstubAllGlobals();
    vi.restoreAllMocks();
  });

  it("clears an expired session and notifies the application on 401", async () => {
    authStorage.set("expired-token");
    sessionStorage.setItem("rovera_user", JSON.stringify({ id: "USER-1" }));
    const expired = vi.fn();
    window.addEventListener(AUTH_EXPIRED_EVENT, expired, { once: true });
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: false,
      status: 401,
      json: vi.fn().mockResolvedValue({ detail: "Token không hợp lệ" }),
    }));

    await expect(api.robots()).rejects.toThrow("Token không hợp lệ");

    expect(authStorage.get()).toBeNull();
    expect(sessionStorage.getItem("rovera_user")).toBeNull();
    expect(expired).toHaveBeenCalledOnce();
  });

  it("does not expire an existing session when login credentials are rejected", async () => {
    authStorage.set("existing-token");
    const expired = vi.fn();
    window.addEventListener(AUTH_EXPIRED_EVENT, expired, { once: true });
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: false,
      status: 401,
      json: vi.fn().mockResolvedValue({ detail: "Sai tài khoản hoặc mật khẩu" }),
    }));

    await expect(api.login("demo@rovera.local", "wrong")).rejects.toThrow(
      "Sai tài khoản hoặc mật khẩu",
    );

    expect(authStorage.get()).toBe("existing-token");
    expect(expired).not.toHaveBeenCalled();
  });
});

describe("Robot media source scan", () => {
  afterEach(() => {
    authStorage.clear();
    sessionStorage.clear();
    vi.unstubAllGlobals();
    vi.restoreAllMocks();
  });

  it("requests the selected media kind from the robot", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true,
      status: 200,
      json: vi.fn().mockResolvedValue({
        media_kind: "audio",
        video_sources: [],
        audio_sources: [{
          type: "device",
          value: "plughw:CARD=Camera,DEV=0",
          label: "USB Camera · USB Audio",
        }],
      }),
    });
    vi.stubGlobal("fetch", fetchMock);

    const sources = await api.robotMediaSources("ROBOT-229", "audio");

    expect(fetchMock).toHaveBeenCalledWith(
      "/api/robots/ROBOT-229/media-sources?media_kind=audio",
      expect.any(Object),
    );
    expect(sources.audio_sources[0].value).toBe(
      "plughw:CARD=Camera,DEV=0",
    );
  });

  it("requests speaker outputs independently from microphones", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true,
      status: 200,
      json: vi.fn().mockResolvedValue({
        media_kind: "speaker",
        video_sources: [],
        audio_sources: [],
        speaker_sources: [{
          type: "pulse",
          value: "pulse:alsa_output.usb-speaker",
          label: "USB Speaker",
        }],
      }),
    });
    vi.stubGlobal("fetch", fetchMock);

    const sources = await api.robotMediaSources("ROBOT-229", "speaker");

    expect(fetchMock).toHaveBeenCalledWith(
      "/api/robots/ROBOT-229/media-sources?media_kind=speaker",
      expect.any(Object),
    );
    expect(sources.speaker_sources[0].label).toBe("USB Speaker");
  });
});
