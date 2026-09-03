import {
  ControlAlreadyConnectedError,
  WebSocketControlTransport,
} from "../src/transports/ControlTransport";

class FakeWebSocket {
  static readonly CONNECTING = 0;
  static readonly OPEN = 1;
  static readonly CLOSING = 2;
  static readonly CLOSED = 3;
  static instances: FakeWebSocket[] = [];

  readonly url: string;
  readyState = FakeWebSocket.CONNECTING;
  onopen: (() => void) | null = null;
  onerror: (() => void) | null = null;
  onmessage: ((event: MessageEvent<string>) => void) | null = null;
  onclose: ((event: CloseEvent) => void) | null = null;
  sent: string[] = [];

  constructor(url: string | URL) {
    this.url = String(url);
    FakeWebSocket.instances.push(this);
  }

  open(): void {
    this.readyState = FakeWebSocket.OPEN;
    this.onopen?.();
  }

  acceptControl(): void {
    const clientId = new URL(this.url).searchParams.get("client_id");
    this.onmessage?.({
      data: JSON.stringify({
        message_type: "control.ready",
        payload: { client_id: clientId },
      }),
    } as MessageEvent<string>);
  }

  send(data: string): void {
    this.sent.push(data);
  }

  close(code = 1000): void {
    if (this.readyState === FakeWebSocket.CLOSED) return;
    this.readyState = FakeWebSocket.CLOSED;
    this.onclose?.({ code } as CloseEvent);
  }

  loseConnection(): void {
    this.readyState = FakeWebSocket.CLOSED;
    this.onclose?.({ code: 1006 } as CloseEvent);
  }

  rejectDuplicate(): void {
    this.close(4009);
  }
}

describe("WebSocketControlTransport", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    FakeWebSocket.instances = [];
    vi.stubGlobal("WebSocket", FakeWebSocket);
    sessionStorage.setItem("rovera_access_token", "test-token");
  });

  afterEach(() => {
    vi.useRealTimers();
    vi.unstubAllGlobals();
    sessionStorage.clear();
  });

  it("keeps an idle session alive and reconnects after a network drop", async () => {
    const disconnected = vi.fn();
    const reconnected = vi.fn();
    const transport = new WebSocketControlTransport(
      vi.fn(),
      disconnected,
      reconnected,
    );

    const connecting = transport.connect("ROBOT-001", "session-1", "/ws/control");
    FakeWebSocket.instances[0].open();
    FakeWebSocket.instances[0].acceptControl();
    await connecting;
    expect(transport.isSessionController()).toBe(true);

    await vi.advanceTimersByTimeAsync(15_000);
    const heartbeat = JSON.parse(FakeWebSocket.instances[0].sent[0]);
    expect(heartbeat.message_type).toBe("session.heartbeat");

    FakeWebSocket.instances[0].loseConnection();
    expect(disconnected).toHaveBeenCalledOnce();
    await vi.advanceTimersByTimeAsync(1_000);
    expect(FakeWebSocket.instances).toHaveLength(2);

    FakeWebSocket.instances[1].open();
    FakeWebSocket.instances[1].acceptControl();
    await Promise.resolve();
    expect(reconnected).toHaveBeenCalledOnce();

    await transport.disconnect();
    await vi.advanceTimersByTimeAsync(10_000);
    expect(FakeWebSocket.instances).toHaveLength(2);
  });

  it("keeps the first tab connected when a duplicated tab opens the same session", async () => {
    const firstTab = new WebSocketControlTransport(vi.fn(), vi.fn());
    const duplicatedTab = new WebSocketControlTransport(vi.fn(), vi.fn());

    const firstConnection = firstTab.connect("ROBOT-001", "session-1", "/ws/control");
    FakeWebSocket.instances[0].open();
    FakeWebSocket.instances[0].acceptControl();
    await firstConnection;

    const duplicateConnection = duplicatedTab.connect("ROBOT-001", "session-1", "/ws/control");
    FakeWebSocket.instances[1].open();
    FakeWebSocket.instances[1].rejectDuplicate();

    await expect(duplicateConnection).rejects.toBeInstanceOf(ControlAlreadyConnectedError);
    expect(firstTab.isConnected()).toBe(true);
    expect(firstTab.isSessionController()).toBe(true);
    expect(duplicatedTab.isSessionController()).toBe(false);
    expect(
      new URL(FakeWebSocket.instances[0].url).searchParams.get("client_id"),
    ).not.toBe(
      new URL(FakeWebSocket.instances[1].url).searchParams.get("client_id"),
    );

    await vi.advanceTimersByTimeAsync(10_000);
    expect(FakeWebSocket.instances).toHaveLength(2);
  });

  it("sends PTZ motion and stop commands through the active control channel", async () => {
    const transport = new WebSocketControlTransport(vi.fn(), vi.fn());
    const connecting = transport.connect("ROBOT-001", "session-1", "/ws/control");
    FakeWebSocket.instances[0].open();
    FakeWebSocket.instances[0].acceptControl();
    await connecting;

    transport.sendPtz({ operation: "move", pan: -1, tilt: 0, speed: "slow" });
    transport.sendPtz({ operation: "stop" });

    const messages = FakeWebSocket.instances[0].sent.map((raw) => JSON.parse(raw));
    expect(messages.map((message) => message.message_type)).toEqual([
      "camera.ptz",
      "camera.ptz",
    ]);
    expect(messages[0].payload).toEqual({
      operation: "move",
      pan: -1,
      tilt: 0,
      speed: "slow",
    });
    expect(messages[1].payload).toEqual({ operation: "stop" });
  });

  it("sends the selected manual obstacle-avoidance mode with every velocity", async () => {
    const transport = new WebSocketControlTransport(vi.fn(), vi.fn());
    const connecting = transport.connect("ROBOT-001", "session-1", "/ws/control");
    FakeWebSocket.instances[0].open();
    FakeWebSocket.instances[0].acceptControl();
    await connecting;

    transport.sendVelocity({ linear_x: 0.1, angular_z: 0 });
    transport.setObstacleAvoidanceEnabled(false);
    transport.sendVelocity({ linear_x: 0.2, angular_z: -0.1 });

    const messages = FakeWebSocket.instances[0].sent.map((raw) => JSON.parse(raw));
    expect(messages[0].payload.obstacle_avoidance_enabled).toBe(true);
    expect(messages[1].payload.obstacle_avoidance_enabled).toBe(false);
  });

  it("uses a distinct command to reset the software E-stop", async () => {
    const transport = new WebSocketControlTransport(vi.fn(), vi.fn());
    const connecting = transport.connect("ROBOT-001", "session-1", "/ws/control");
    FakeWebSocket.instances[0].open();
    FakeWebSocket.instances[0].acceptControl();
    await connecting;

    transport.resetEstop();

    const message = JSON.parse(FakeWebSocket.instances[0].sent[0]);
    expect(message.message_type).toBe("control.estop.reset");
    expect(message.payload.reason).toBe("operator_estop_reset");
  });
});
