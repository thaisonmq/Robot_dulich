"""
VMS Client - đẩy metadata nhận diện từ DeepStream sang backend VMS (MQ AI Vision).

Backend VMS nhận metadata rồi tự vẽ hộp bao/nhãn lên luồng video và bù độ trễ
giữa metadata với hình ảnh. Xem giao thức tại: GET {vms_url}/api/ai/schema

Nguyên tắc thiết kế:
  * KHÔNG bao giờ chặn luồng GStreamer — probe chỉ bỏ gói vào hàng đợi rồi trả về.
  * KHÔNG ném lỗi ra ngoài — mọi sự cố mạng đều nuốt và tự kết nối lại.
  * KHÔNG thêm thư viện ngoài — chỉ dùng thư viện chuẩn Python.

Hai kênh gửi:
  * metadata  → /api/ai/stream (NDJSON, một kết nối dài) hoặc /api/ai/metadata (POST rời)
  * sự kiện   → /api/events (cảnh báo xâm nhập, biển số đọc được...)
"""

from __future__ import annotations

import http.client
import json
import queue
import re
import threading
import time
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse


def now_ms() -> int:
    """Mốc thời gian hiện tại theo epoch mili-giây."""
    return int(time.time() * 1000)


def frame_timestamp_ms(frame_meta) -> int:
    """
    Mốc thời gian của khung hình để backend đồng bộ độ trễ.

    Ưu tiên ntp_timestamp của DeepStream (khi nguồn RTSP có RTCP sender report),
    nếu giá trị không hợp lệ thì dùng đồng hồ hệ thống lúc probe chạy — vẫn đúng
    mô hình "thời điểm khung hình tới nơi xử lý" mà backend đang dùng.
    """
    try:
        ntp = int(getattr(frame_meta, "ntp_timestamp", 0) or 0)
        if ntp > 0:
            ms = ntp // 1_000_000                      # nano-giây → mili-giây
            # Chỉ nhận nếu nằm trong khoảng hợp lý (sau 2001, trước 2100)
            if 1_000_000_000_000 < ms < 4_100_000_000_000:
                return ms
    except Exception:
        pass
    return now_ms()


class VMSClient:
    """
    Đẩy metadata và sự kiện sang backend VMS.

    Cấu hình (khối `vms:` trong config.yaml của branch):
        vms:
          enable: true
          url: "http://192.168.6.1:8088"
          channel: "main"
          transport: "ndjson"        # ndjson | post
          queue_size: 120
          camera_map:                # camera_id DeepStream -> camera id trong VMS
            cam1: 6
          auto_map: true             # tự dò camera VMS theo IP trong URI nếu không có trong map
          verbose: false
    """

    def __init__(self, config: Dict[str, Any], source_mapper=None, tag: str = "VMS"):
        vms = (config or {}).get("vms", {}) or {}

        self._enabled: bool = bool(vms.get("enable", False))
        self._url: str = str(vms.get("url", "http://127.0.0.1:8088")).rstrip("/")
        self._channel: str = str(vms.get("channel", "main"))
        self._transport: str = str(vms.get("transport", "ndjson")).lower()
        self._verbose: bool = bool(vms.get("verbose", False))
        self._auto_map: bool = bool(vms.get("auto_map", True))
        self._tag = tag
        self._source_mapper = source_mapper
        # Tên nguồn đi kèm mỗi gói: VMS tách track theo (camera, channel, source)
        # nên hai branch cùng đẩy về một camera không đè kết quả của nhau.
        self._source: str = str(vms.get("source", tag))

        parsed = urlparse(self._url)
        self._host: str = parsed.hostname or "127.0.0.1"
        self._port: int = parsed.port or (443 if parsed.scheme == "https" else 80)
        self._https: bool = parsed.scheme == "https"

        # camera_id (DeepStream, chuỗi) -> camera id (VMS, số)
        self._camera_map: Dict[str, int] = {
            str(k): int(v) for k, v in (vms.get("camera_map", {}) or {}).items()
        }
        self._resolved: Dict[str, Optional[int]] = {}      # cache kết quả tra cứu
        self._map_lock = threading.Lock()

        qsize = int(vms.get("queue_size", 120))
        self._meta_q: queue.Queue = queue.Queue(maxsize=qsize)
        self._event_q: queue.Queue = queue.Queue(maxsize=64)

        self._stop = threading.Event()
        self._meta_thread: Optional[threading.Thread] = None
        self._event_thread: Optional[threading.Thread] = None

        self._seq = 0
        self._sent = 0
        self._dropped = 0
        self._errors = 0
        self._last_error = ""

    # ------------------------------------------------------------------ #
    # Vòng đời
    # ------------------------------------------------------------------ #

    @property
    def enabled(self) -> bool:
        return self._enabled

    def start(self) -> None:
        if not self._enabled or self._meta_thread is not None:
            return
        self._stop.clear()
        self._meta_thread = threading.Thread(target=self._meta_loop, name=f"{self._tag}-meta", daemon=True)
        self._event_thread = threading.Thread(target=self._event_loop, name=f"{self._tag}-event", daemon=True)
        self._meta_thread.start()
        self._event_thread.start()
        print(f"[{self._tag}] Bật đẩy metadata sang {self._url} "
              f"(transport={self._transport}, channel={self._channel})")

    def stop(self) -> None:
        if not self._enabled:
            return
        self._stop.set()
        for t in (self._meta_thread, self._event_thread):
            if t is not None:
                t.join(timeout=2.0)
        self._meta_thread = self._event_thread = None
        print(f"[{self._tag}] Dừng đẩy metadata "
              f"(đã gửi={self._sent}, rớt={self._dropped}, lỗi={self._errors})")

    def stats(self) -> Dict[str, Any]:
        return {
            "vms_sent": self._sent,
            "vms_dropped": self._dropped,
            "vms_errors": self._errors,
            "vms_queue": self._meta_q.qsize(),
            "vms_last_error": self._last_error,
        }

    # ------------------------------------------------------------------ #
    # Ánh xạ camera DeepStream -> camera VMS
    # ------------------------------------------------------------------ #

    def resolve_camera(self, camera_id: Optional[str]) -> Optional[int]:
        """
        Tìm id camera bên VMS ứng với camera_id của DeepStream.

        Thứ tự: camera_map trong config → số có trong tên (cam6 → 6)
                → dò theo IP/tên qua API VMS (nếu auto_map bật).
        """
        if not camera_id:
            return None
        with self._map_lock:
            if camera_id in self._camera_map:
                return self._camera_map[camera_id]
            if camera_id in self._resolved:
                return self._resolved[camera_id]

        vms_id = None
        digits = re.findall(r"\d+", camera_id)
        if digits:
            vms_id = int(digits[-1])

        if self._auto_map:
            found = self._lookup_by_uri(camera_id)
            if found is not None:
                vms_id = found

        with self._map_lock:
            self._resolved[camera_id] = vms_id
        if self._verbose:
            print(f"[{self._tag}] Ánh xạ camera {camera_id} -> VMS id {vms_id}")
        return vms_id

    def _lookup_by_uri(self, camera_id: str) -> Optional[int]:
        """Dò camera bên VMS bằng cách so IP trong URI RTSP của camera DeepStream."""
        try:
            ip = None
            mapper = self._source_mapper
            uri = getattr(mapper, "get_uri", None)
            if callable(uri):
                ip_match = re.search(r"@?([\d.]+):\d+", uri(camera_id) or "")
                ip = ip_match.group(1) if ip_match else None
            if not ip:
                return None
            cams = self._request_json("GET", "/api/cameras")
            if not isinstance(cams, list):
                return None
            for cam in cams:
                if str(cam.get("ip", "")) == ip:
                    return int(cam.get("id"))
        except Exception:
            return None
        return None

    # ------------------------------------------------------------------ #
    # Gửi dữ liệu (gọi từ probe — chỉ bỏ vào hàng đợi, không chặn)
    # ------------------------------------------------------------------ #

    def push(self, camera_id: Optional[str], objects: List[Dict[str, Any]],
             ts_ms: int, frame_w: int, frame_h: int, latency_ms: int = 0) -> None:
        """Đẩy một gói metadata của một khung hình. An toàn khi gọi từ probe."""
        if not self._enabled:
            return
        vms_id = self.resolve_camera(camera_id)
        if vms_id is None:
            return

        self._seq += 1
        packet = {
            "camera_id": vms_id,
            "channel": self._channel,
            "source": self._source,
            "ts": ts_ms,
            "seq": self._seq,
            "latency_ms": latency_ms,
            "frame": {"w": frame_w, "h": frame_h},
            "objects": objects,
        }
        self._enqueue(self._meta_q, packet)

    def send_event(self, camera_id: Optional[str], event_type: str, label: str,
                   confidence: float = 0.0, snapshot: str = "") -> None:
        """Ghi một sự kiện (cảnh báo/nhận diện) vào danh sách bên VMS."""
        if not self._enabled:
            return
        vms_id = self.resolve_camera(camera_id)
        if vms_id is None:
            return
        payload = {
            "type": event_type,
            "camera_id": vms_id,
            "label": label,
            "confidence": round(float(confidence), 4),
        }
        if snapshot:
            payload["snapshot"] = snapshot
        self._enqueue(self._event_q, payload)

    def _enqueue(self, q: queue.Queue, item: Any) -> None:
        """Bỏ vào hàng đợi; đầy thì bỏ gói cũ nhất để luôn ưu tiên dữ liệu mới."""
        try:
            q.put_nowait(item)
        except queue.Full:
            self._dropped += 1
            try:
                q.get_nowait()
                q.put_nowait(item)
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    # Luồng nền: metadata
    # ------------------------------------------------------------------ #

    def _new_conn(self) -> http.client.HTTPConnection:
        if self._https:
            return http.client.HTTPSConnection(self._host, self._port, timeout=5)
        return http.client.HTTPConnection(self._host, self._port, timeout=5)

    def _meta_loop(self) -> None:
        backoff = 1.0
        while not self._stop.is_set():
            try:
                if self._transport == "ndjson":
                    self._run_ndjson()
                else:
                    self._run_post()
                backoff = 1.0
            except Exception as exc:                       # mất kết nối → thử lại
                self._errors += 1
                self._last_error = f"{type(exc).__name__}: {exc}"
                if self._verbose:
                    print(f"[{self._tag}] Lỗi gửi metadata: {self._last_error} "
                          f"(thử lại sau {backoff:.0f}s)")
                self._stop.wait(backoff)
                backoff = min(backoff * 2, 15.0)

    def _run_ndjson(self) -> None:
        """
        Mở một kết nối POST dài tới /api/ai/stream và ghi mỗi gói một dòng JSON.
        Dùng chunked encoding để không phải biết trước độ dài nội dung.
        """
        conn = self._new_conn()
        conn.putrequest("POST", "/api/ai/stream", skip_host=False, skip_accept_encoding=True)
        conn.putheader("Content-Type", "application/x-ndjson")
        conn.putheader("Transfer-Encoding", "chunked")
        conn.endheaders()

        sock = conn.sock
        if sock is None:
            raise RuntimeError("Không mở được socket tới VMS")
        sock.settimeout(5)

        try:
            while not self._stop.is_set():
                try:
                    packet = self._meta_q.get(timeout=0.5)
                except queue.Empty:
                    continue
                body = (json.dumps(packet, ensure_ascii=False) + "\n").encode("utf-8")
                sock.sendall(b"%X\r\n" % len(body) + body + b"\r\n")
                self._sent += 1
        finally:
            try:
                sock.sendall(b"0\r\n\r\n")                 # kết thúc chunked
            except Exception:
                pass
            try:
                conn.close()
            except Exception:
                pass

    def _run_post(self) -> None:
        """Gửi từng gói bằng POST /api/ai/metadata, dùng lại kết nối (keep-alive)."""
        conn = self._new_conn()
        try:
            while not self._stop.is_set():
                try:
                    packet = self._meta_q.get(timeout=0.5)
                except queue.Empty:
                    continue
                body = json.dumps(packet, ensure_ascii=False).encode("utf-8")
                conn.request("POST", "/api/ai/metadata", body,
                             {"Content-Type": "application/json", "Connection": "keep-alive"})
                resp = conn.getresponse()
                resp.read()
                if resp.status >= 400:
                    raise RuntimeError(f"VMS trả về HTTP {resp.status}")
                self._sent += 1
        finally:
            try:
                conn.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    # Luồng nền: sự kiện
    # ------------------------------------------------------------------ #

    def _event_loop(self) -> None:
        while not self._stop.is_set():
            try:
                payload = self._event_q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._request_json("POST", "/api/events", payload)
            except Exception as exc:
                self._errors += 1
                self._last_error = f"{type(exc).__name__}: {exc}"

    # ------------------------------------------------------------------ #
    # Gọi API đơn lẻ (dùng cho sự kiện và tra cứu)
    # ------------------------------------------------------------------ #

    def _request_json(self, method: str, path: str, payload: Any = None) -> Any:
        conn = self._new_conn()
        try:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8") if payload is not None else None
            headers = {"Content-Type": "application/json"} if body else {}
            conn.request(method, path, body, headers)
            resp = conn.getresponse()
            data = resp.read()
            if resp.status >= 400:
                raise RuntimeError(f"HTTP {resp.status}: {data[:160].decode('utf-8', 'ignore')}")
            return json.loads(data) if data else None
        finally:
            try:
                conn.close()
            except Exception:
                pass

    def get_json(self, path: str) -> Any:
        """Đọc dữ liệu từ API VMS (dùng cho đồng bộ vùng giám sát)."""
        return self._request_json("GET", path)


# --------------------------------------------------------------------------- #
# Tiện ích dựng đối tượng đúng schema của backend VMS
# --------------------------------------------------------------------------- #

def split_plate_code(plate_text: str) -> str:
    """
    Tách "mã biển" (mã tỉnh + seri chữ) khỏi biển số đầy đủ.

    Biển Việt Nam: 2 chữ số mã tỉnh + 1-2 chữ cái seri, xe máy có thêm 1 chữ số.
        "51H-123.45"  -> "51H"
        "92B1-345.67" -> "92B1"
        "29A67890"    -> "29A"     (5 chữ số phía sau là số thứ tự)
        "92B134567"   -> "92B1"    (6 chữ số ⇒ chữ số đầu thuộc mã seri)
    Không nhận dạng được thì trả về chuỗi rỗng.
    """
    text = (plate_text or "").strip().upper()
    if not text:
        return ""

    parts = re.split(r"[-\s.]", text, maxsplit=1)
    head = parts[0]
    has_separator = len(parts) > 1

    if has_separator:
        # Có dấu phân cách thì phần đầu chính là mã biển
        match = re.match(r"^\d{2}[A-Z]{1,2}\d?$", head)
        return match.group(0) if match else head[:4]

    # Không có dấu phân cách: tách phần chữ rồi xét độ dài số còn lại.
    match = re.match(r"^(\d{2}[A-Z]{1,2})(\d*)$", head)
    if not match:
        return head[:4]
    base, rest = match.group(1), match.group(2)
    # Số thứ tự chuẩn dài 4-5 chữ số; dư ra ⇒ chữ số đầu thuộc mã seri xe máy
    return base + rest[0] if len(rest) > 5 else base


def make_object(obj_type: str, rect, frame_w: int, frame_h: int,
                score: float = 0.0, track_id: Optional[int] = None,
                **extra) -> Dict[str, Any]:
    """
    Dựng một đối tượng metadata từ rect_params của DeepStream.

    Toạ độ được chuẩn hoá về 0..1 theo kích thước muxer nên backend vẽ đúng ở
    mọi độ phân giải hiển thị.
    """
    left, top = float(rect.left), float(rect.top)
    width, height = float(rect.width), float(rect.height)
    fw = float(frame_w) or 1.0
    fh = float(frame_h) or 1.0

    def clamp(v: float) -> float:
        return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)

    item: Dict[str, Any] = {
        "type": obj_type,
        "box": [clamp(left / fw), clamp(top / fh),
                clamp((left + width) / fw), clamp((top + height) / fh)],
        "score": round(float(score), 4),
    }
    if track_id is not None and int(track_id) < (1 << 63) - 1:
        item["track_id"] = int(track_id)
    item.update({k: v for k, v in extra.items() if v not in (None, "")})
    return item
