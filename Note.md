# BÁO CÁO RÀ SOÁT TOÀN BỘ PROJECT ROBOT DU LỊCH

> Ngày rà soát: 29/08/2026<br>
> Commit được rà soát: `4161786`<br>
> Phạm vi: mã nguồn Center, Edge/Pi, ROS 2/Nav2/SLAM, mô phỏng, frontend, giao thức điều khiển, quản lý bản đồ, kiểm thử và tài liệu.<br>
> Nguyên tắc: **chỉ đọc, phân tích và chạy kiểm thử; không sửa mã nguồn/cấu hình**. File này là sản phẩm duy nhất được tạo nội dung theo yêu cầu.

## 1. Kết luận điều hành

Project có nền tảng thuật toán tốt hơn một prototype robot thông thường: quản lý map theo phiên bản bất biến, kiểm hash nhiều lớp, localization có kiểm chứng bằng scan/map thay vì tin mù pose cũ, bộ lập đường xét đúng footprint hình chữ nhật, kiểm tra swept-footprint khi quay, cơ chế đi hành lang hẹp khá chặt, và nhiều lớp dừng/watchdog ở Edge. Bộ test thuật toán cũng khá sâu.

Tuy nhiên, **chưa nên coi hệ thống là sẵn sàng cho robot tự hành không giám sát ở nơi có người**, nhất là hành lang hẹp, khu vực đông người hoặc môi trường có kính/vật thấp/mép cầu thang. Lý do không nằm ở một thuật toán đơn lẻ mà ở các điểm giao giữa Center–Edge–ROS–cảm biến:

1. UI có thể báo “Đã dừng an toàn” sau khi Center mới chỉ chuyển tiếp lệnh, chưa có bằng chứng bánh xe đã dừng.
2. Các tín hiệu bảo vệ như bumper, cliff, range và external stop không có kiểm tra freshness đầy đủ; nguồn cảm biến chết sau trạng thái `false/clear` có thể bị hiểu là vẫn an toàn.
3. Tại thời điểm audit, vùng tự lọc LiDAR `0,40 × 0,36 m` lớn hơn footprint va chạm `0,30 × 0,20 m`. Đợt sửa 2026-08-29 đã thống nhất lại thân xe `0,30 × 0,20 × 0,15 m`, LiDAR giữa mặt trên và self-mask đúng bằng footprint; HIL quanh chu vi vẫn là bước nghiệm thu bắt buộc.
4. “Dừng khẩn cấp” hiện là software stop có thể tự bỏ latch khi nhận lệnh chuyển động mới; chế độ tắt chống vật cản bỏ qua gần như toàn bộ gate LiDAR/range/external obstacle. Đây không tương đương E-stop an toàn phần cứng.
5. Lỗi runtime trong localization resume và đường recovery autosave đã được sửa ở mức phần mềm; autosave nay có generation/hash/map/pose atomic và Edge truyền terminal pose vào bước scan-to-map verify. Còn cần ROS integration/chaos test trên Pi thật.
6. Khi robot reconnect, đồng bộ map đã xóa chạy song song với khôi phục map/mission; map đã xóa có thể được nạp lại và mission cũ có khả năng tự resume.
7. Điều khiển phân tán chủ yếu giữ trạng thái trong RAM, chưa có transactional outbox/durable result reconciliation. Restart, timeout và ACK đến muộn có thể làm Center và robot hiểu hai trạng thái khác nhau.
8. Chưa có điều phối giao thông nhiều robot, đặt chỗ hành lang/giao lộ, quản lý thang máy/cửa, tối ưu pin/sạc hay cân bằng nhiệm vụ như các fleet manager AMR hiện đại.
9. Exception trong callback localization chưa thu hồi ngay quyền phát vận tốc navigation; đồng thời một số worker recovery không mang generation fence tới điểm commit, nên có race để chuyển động cũ tái khởi động sau cancel/E-stop/takeover.
10. `map.set_initial_pose` chưa chặn mission đang chạy; `map.relocalize` chỉ nhìn Nav2 goal handle nên không nhận ra pha TURN do adapter tự phát vận tốc. Pose có thể bị reset giữa chuyển động.

**Phán quyết:** phù hợp để tiếp tục R&D và thử nghiệm có giám sát, sau khi sửa các lỗi P0/P1; chưa đủ bằng chứng để triển khai tự hành an toàn ngoài hiện trường. Không thể suy ra tuân thủ ISO hoặc chứng nhận an toàn chỉ từ source code.

## 2. Phạm vi, phương pháp và giới hạn

Đã rà soát 321 file được Git theo dõi, tập trung sâu vào:

- `demo/robot-simulator/navigation-stack/navigation_core.py` — lập đường, footprint, hành lang, vật cản động, localization/map.
- `demo/robot-simulator/navigation-stack/adapter_node.py` — orchestration ROS/Nav2/SLAM, load/save map, localization recovery, mission recovery.
- `demo/robot-simulator/simulator/client.py`, `map_cache.py` — Edge protocol, cache, tombstone, autosave/recovery.
- `demo/robot-simulator/motion-safety/*` — lọc vận tốc, E-stop, obstacle gate, watchdog.
- `src/apps/center-backend/*` — session, command, map registry/storage, WebSocket.
- `src/apps/center-frontend/*` — HMI điều khiển, stop, mapping, telemetry.
- các file cấu hình URDF, Nav2, SLAM, EKF, sensor normalization và tài liệu kiến trúc.

Phương pháp gồm đọc tĩnh theo luồng dữ liệu, tìm các trạng thái/race/fail-open, chạy test hiện có, build frontend, lint chọn lọc, audit dependency JS và đối chiếu tài liệu chính thức Nav2/ROS/Open-RMF cùng thông tin công khai của một số AMR thương mại.

Giới hạn quan trọng:

- Không có robot vật lý, ROS graph/cảm biến thật, mặt sàn, tải trọng, pin hoặc E-stop phần cứng để đo stopping distance và failure response.
- Không làm penetration test mạng hoặc fault injection phần cứng.
- `pip-audit` chưa hoàn tất vì Docker registry bị TLS handshake timeout; do đó không kết luận dependency Python sạch lỗ hổng.
- Các nhận định về MiR/OTTO/Locus bên dưới là **tính năng do nhà cung cấp công bố**, không phải kết quả kiểm định độc lập.

### Thang mức độ

| Mức | Ý nghĩa trong báo cáo |
|---|---|
| **P0 / Blocker** | Có thể tạo chuyển động không được xác nhận an toàn, hồi sinh trạng thái nguy hiểm, hoặc phá vỡ lớp bảo vệ; phải xử lý và kiểm chứng trước chạy thật không giám sát. |
| **P1 / High** | Lỗi xác định hoặc rủi ro lớn về điều hướng, map, tính nhất quán phân tán hay vận hành. |
| **P2 / Medium** | Làm giảm độ tin cậy, khả năng bảo trì, quan sát hoặc mở rộng; cần lên kế hoạch gần. |
| **P3 / Low** | Drift tài liệu/UX/chất lượng kỹ thuật, ít gây nguy hiểm trực tiếp nhưng nên sửa. |

“Lỗi xác định” nghĩa là có đường thực thi cụ thể trong mã. “Rủi ro/thiếu bằng chứng” nghĩa là phải xác nhận bằng HIL/robot thật; không được hiểu là chắc chắn đã xảy ra tai nạn.

## 3. Kiến trúc và luồng tự hành hiện tại

```text
Web operator
   │ control/mapping WebSocket + REST
   ▼
Center Backend ── registry DB / map archive
   │ outbound robot WebSocket
   ▼
Edge client trên Pi ── local map cache / persisted runtime
   │ Unix datagram + ROS service/action
   ▼
ROS adapter ── Nav2 / SLAM Toolbox / AMCL / custom planner
   │ /cmd_vel_raw
   ▼
Motion Safety ── normalized LiDAR + range + bumper + cliff + external stop
   │ /cmd_vel
   ▼
Motor driver / robot
```

Luồng hợp lý về mặt phân lớp: Center quản lý người dùng/map/nhiệm vụ, Edge là đầu mối outbound từ robot, adapter quản lý lifecycle và thuật toán, còn motion-safety sở hữu đầu ra vận tốc cuối. Điểm yếu là “bằng chứng hoàn tất” chưa quay trở lại UI theo cùng chuỗi có thẩm quyền, và nhiều trạng thái phân tán chỉ nằm trong RAM hoặc được phục hồi song song.

## 4. Danh sách phát hiện ưu tiên

| ID | Mức | Loại | Phát hiện |
|---|---:|---|---|
| SAF-01 | **P0** | Lỗi kiến trúc/HMI | UI xác nhận dừng an toàn trước khi Edge/driver xác nhận bánh xe bằng 0. |
| SAF-02 | **P0 → đã sửa cấu hình** | Còn HIL | LiDAR self-mask đã bằng footprint 0,30 × 0,20 m và extrinsic đã về tâm; còn phải kiểm vật thật quanh chu vi. |
| SAF-03 | **P0** | Fail-open | Bumper/cliff/range/external stop thiếu freshness; external interlock watchdog mặc định tắt. |
| SAF-04 | **P0** | Thiết kế an toàn | Software “E-stop” tự clear khi có lệnh mới; bypass vật cản quá rộng. |
| MAP-01 | **P0 → đã sửa phần mềm** | Còn chaos/HIL | Reconnect barrier, durable tombstone và command/upload/restore gate đã chặn map bị xóa sống lại; còn nghiệm thu mất điện và restart thật. |
| NAV-01 | **P1** | Lỗi xác định | Localization resume có `UnboundLocalError`, kẹt trạng thái recovery/planning. |
| NAV-05 | **P0** | Fail-safe | Exception callback localization không revoke motion owner/cancel/zero navigation ngay. |
| NAV-06 | **P0** | Race xác định | Worker recovery bỏ generation fence ở commit, có thể hồi sinh navigation sau cancel/takeover/E-stop. |
| NAV-07 | **P0** | State invariant | Set initial pose/relocalize có thể bắt đầu trong lúc robot còn đi hoặc đang TURN. |
| LOC-01 | **P2** | Async race | Callback global-localization cũ thiếu attempt/map fence, có thể ghi đè attempt mới. |
| LOC-02 | **P2** | Freshness | AMCL pose/particle bỏ header stamp/frame và được đóng dấu mới theo receipt time. |
| LOC-03 | **P2** | Freshness | Localization TF freshness đo thời điểm poll thành công, không bắt transform nguồn bị đóng băng. |
| MAP-02 | **P1 → đã sửa phần mềm** | Còn ROS/chaos test | Autosave dùng committed generation có map/posegraph/terminal pose/checksum; recovery verify toàn bộ rồi mới nạp. |
| CMD-01 | **P1** | Phân tán | ACK Center chỉ là “đã forward”; pending/result/ownership chưa bền qua restart và timeout. |
| MAP-03 | **P1 → đã sửa phần mềm** | Còn fuzz/chaos | Center/Edge đã ràng buộc semantic `map.yaml`–image–metadata, giới hạn archive và re-verify cache; còn stress/fuzz trên thiết bị đích. |
| MAP-04 | **P1** | Lifecycle | Archive, delete, resync và tombstone có thể gây split-brain hoặc ghi flash lặp vô hạn. |
| NAV-02 | **P1** | Safety case | Controller RPP tắt collision detection độc lập; phụ thuộc chung vào custom Python + safety layer. |
| NAV-03 | **P1** | Cảm biến | Chỉ có 2D LaserScan, không thấy kính/vật thấp/overhang/drop-off; IMU chưa được fusion. |
| MAP-05 | **P1** | Ownership | Mapping API không ràng buộc exclusive control lease và có thể cắt navigation của operator khác. |
| CMD-02 | **P1** | Idempotency | `request_id` không bind payload/state; retry khác nội dung có thể làm Center/Edge lệch map. |
| UI-01 | **P1** | Trạng thái | Mapping UI không nối transport async có thẩm quyền; fault/upload completion có thể không hiện. |
| NAV-04 | **P2** | Hành vi | Stop–turn an toàn và dễ giải thích nhưng giật/chậm, có thể thất bại nơi controller liên tục đi được. |
| COR-01 | **P1 → đã sửa phần mềm** | Còn HIL | Vi phạm hard margin ở bất kỳ phía đã quan sát nào nay luôn chặn; recovery lùi-thẳng/tìm turn bay/replan giữ nguyên đích. |
| NAV-08 | **P2** | Toàn vẹn route | Path và goal không được bind ở pre-motion; robot có thể đi hết path sai endpoint rồi mới replan. |
| FLEET-01 | **P2** | Khoảng trống | Không có traffic reservation, queue/deadlock, task allocator, charging/resource manager. |
| OPS-01 | **P2** | Vận hành | Health/readiness, audit event, metrics/trace và protocol contract còn yếu. |
| DOC-01 | **P3** | Drift | Tài liệu nói NavigateToPose/1 Hz replan/các margin cũ, khác đường thực thi FollowPath hiện tại. |

## 5. Rà soát tạo map, lưu map, nạp map và localization

### 5.1 Điểm mạnh

- Map sau khi finish được coi như version bất biến; whole-archive SHA-256 và hash từng artifact giúp phát hiện truyền lỗi.
- Center và Edge chặn path tuyệt đối, `..`, symlink/hardlink/special tar member; giải nén vào staging rồi atomic rename.
- Nạp map đi qua staging → verify bundle/artifact → map_server → kiểm `/map` → AMCL; adapter có rollback map trước nếu load thất bại.
- Bounds có xét `origin yaw`, không giả định ảnh map luôn song song trục thế giới (`center-backend/app/api/maps.py:125-135`, `navigation.py:242-249`).
- Continue mapping yêu cầu đủ posegraph/data và không sửa graph nguồn. Pose gợi ý chỉ là xấp xỉ: hệ thống làm heading/global scan-match, uniqueness và multi-scan confirmation trước khi nhận scan mới.
- Localization chặt hơn AMCL mặc định: kiểm scan/TF/clock freshness, covariance, stability, uniqueness, scan-to-map/raycast evidence; last pose chỉ là seed và chỉ được persist sau khi có evidence.
- Khi confidence mất trong lúc đi, logic dừng rồi relocalize thay vì tiếp tục dead-reckoning vô hạn.

### 5.2 `MAP-01` — map đã xóa có thể sống lại khi reconnect (**P0 → đã sửa phần mềm**)

Trong `_connected`, Edge khởi động đồng thời tombstone sync và active-map/navigation restore (`simulator/client.py:578-587`). Restore đọc local active cache ngay khi Center chưa có active map (`client.py:1822-1881`), còn tombstone cần hoàn thành HTTP poll (`client.py:1763-1820`). `active_load_payload` chỉ biết tombstone đã có ở local (`map_cache.py:265-299`). Adapter lại có khả năng phục hồi persisted mission với `resume_automatically` (`adapter_node.py:8368-8419`).

Kịch bản cụ thể:

1. Robot offline, Center xóa map đang dùng.
2. Robot reconnect với cache và persisted mission cũ.
3. Restore thắng race, nạp map cũ và đưa mission vào luồng resume.
4. Tombstone đến sau mới xóa cache; robot đã có thể localization/khởi chạy lại nhiệm vụ.

Trạng thái sửa 2026-08-29: Edge nay đóng một reconnect barrier trước restore,
upload retry và mọi lệnh map/mapping/navigation có thể cấp chuyển động. Nó phát
stop, hủy navigation cũ, lấy snapshot có thẩm quyền, ghi tombstone bền vững
trước khi deactivate/xóa runtime, rồi mới ACK và mở `map_registry.ready`.
Download cũng kiểm tra lại tombstone ngay trước atomic install. Center đưa barrier
vào preflight navigation/mapping. Chi tiết triển khai và test nằm ở mục 14.6.

Phần còn lại không còn là đường race phần mềm đã biết mà là bằng chứng nghiệm
thu: chaos test tại mọi điểm mất điện/restart, integration với ROS process thật,
và deletion generation/cursor để scale snapshot mà không làm yếu quy tắc
fail-closed.

### 5.3 `MAP-02` — recovery autosave không hoạt động end-to-end (**P1 → đã sửa phần mềm**)

Edge chuyển `mapping.recover` thành `mapping.start`, truyền `posegraph_path` nhưng đặt `initial_pose=None` (`simulator/client.py:1501-1527`). Adapter thấy posegraph thì bắt buộc `_verify_mapping_continuation_pose`, rồi từ chối pose thiếu x/y/yaw với `MAPPING_POSE_HINT_REQUIRED` (`adapter_node.py:13562-13617`).

Còn một blocker độc lập: autosave chỉ serialize `.posegraph` và `.data` (`adapter_node.py:14015-14029`), trong khi nhánh verify continuation tìm `map.yaml` cạnh posegraph (`adapter_node.py:13623-13630`). UI vẫn hiển thị “Khôi phục & tiếp tục mapping” (`MapManagementPage.tsx:367-376`). Test hiện mock backend nên không bắt được composition thật.

Autosave cũng chạy daemon định kỳ 60 giây mà chưa thấy single-flight/lock hoặc manifest generation. Hai lần serialize chồng nhau, hoặc mất điện giữa hai file, có thể để cặp posegraph/data khác generation. Chỉ kiểm “tồn tại và non-empty” là chưa đủ.

Trạng thái hiện tại: yêu cầu trên đã được triển khai. Adapter tuần tự hóa mọi
SLAM save/pause/resume bằng một lock, chỉ autosave khi measured velocity và
command velocity đều bằng 0, ghi `map.yaml`, `map.pgm`, posegraph/data,
terminal pose và SHA-256 vào một thư mục generation tạm. Sau validate, nó
`fsync`, atomic promote generation rồi mới atomic replace `latest.json`; giữ
hai generation gần nhất. Edge chỉ đọc generation được pointer commit, kiểm
identity/version, đường dẫn, kích thước và checksum mọi artifact, rồi truyền
terminal pose vào continuation verifier. Generation dở dang hoặc artifact bị
sửa sẽ bị từ chối. Recovery vẫn vào `PAUSED` sau khi scan-to-map/SLAM
relocalization thành công để operator chủ động resume.

Phần còn mở là integration test với SLAM Toolbox/TF/service thật và chaos test
cắt nguồn tại từng ranh giới service-write → fsync → rename → pointer; unit test
hiện đã phủ incomplete generation, pointer chưa promote, corruption checksum và
payload Edge có pose/generation đúng.

### 5.4 `MAP-03` — bundle hợp lệ về hash nhưng chưa chắc hợp lệ về ngữ nghĩa (**P1 → đã sửa phần mềm**)

Center kiểm member/path/hash nhưng không parse và đối chiếu đầy đủ `map.yaml ↔ image ↔ metadata`; chưa chặn chắc NaN/Inf, origin sai shape/type, kích thước/resolution bất nhất (`map_storage.py:30-149`, `maps.py:1014-1031`). Loader sau đó dùng trực tiếp trường `image:` trong YAML (`navigation_core.py:8617-8626`, `adapter_node.py:8025-8046`). Path tuyệt đối hoặc `../` trong chính YAML chưa được ràng buộc vào artifact đã hash, nên uploader đã xác thực nhưng bị xâm nhập có thể làm robot đọc một ảnh local khác và điều hướng theo map không đúng.

Ngoài ra, giới hạn chủ yếu áp dụng cho file tar nén (mặc định tới 512 MiB); chưa có cap tổng byte giải nén, member count, kích thước từng member. Đây là bề mặt decompression bomb/đầy đĩa/RAM/CPU. Cache fast path tin marker `.sha256` mà không re-hash artifact (`map_cache.py:361-371`), nên corruption sau cài đặt có thể tồn tại.

Trạng thái hiện tại: Center và Edge chỉ nhận flat regular-file archive, từ chối
canonical duplicate/path lồng/symlink/special member, giới hạn byte nén, byte
giải nén, từng member, số member, tỷ lệ nén và số pixel. Cả hai parse JSON/YAML
theo schema hữu hạn, bắt buộc image là basename hợp lệ, decode ảnh, đối chiếu
dimensions/resolution/origin, artifact set/hash, occupancy checksum và tính đầy
đủ posegraph. Edge giữ archive đã verify và mỗi fast path/boot restore đều
re-hash archive, bind metadata trong archive với metadata đã giải nén rồi kiểm
lại toàn bộ artifact; marker `.sha256` không còn được tin độc lập.

Phần còn lại là fuzz archive/parser và stress CPU/RAM/disk trên phần cứng đích;
không còn đường path escape, semantic mismatch hoặc cache-marker trust đã biết
trong logic hiện tại. Chi tiết triển khai và test nằm ở mục 14.9.

### 5.5 `MAP-04` — lifecycle map có nhiều trạng thái split-brain (**P1**)

- **Archive:** endpoint chỉ đổi record/version thành `ARCHIVED`, không cancel mission, deactivate robot hoặc clear `active_version` (`maps.py:1190-1205`). UI vẫn suy ra “active” khi `active_version != null`, trong khi navigation API từ chối map không ACTIVE. Robot có thể tiếp tục runtime cũ còn Center không nhận lệnh mới.
- **Tombstone:** endpoint chủ ý trả mọi map từng xóa để robot mất registry vẫn không resurrect map. Edge đã tránh ghi/fsync lại tombstone local nếu trạng thái `DELETED` không đổi, nhưng vẫn phải scan/ACK snapshot định kỳ; tải mạng/DB vẫn tăng theo lịch sử map × robot và endpoint chưa có generation/cursor/compaction an toàn.
- **Resync (đã sửa phần mềm):** upload cùng version/checksum nay vẫn nhận và verify lại toàn bộ bundle, atomic replace artifact Center, cập nhật storage/metadata rồi mới đặt `SYNCED`; do đó nút resync chữa được file Center mất/hỏng. Robot thiếu bundle hoặc từ chối lệnh đặt `SYNC_FAILED`, không để UI báo thành công giả. Còn race upload đồng thời và transaction DB–filesystem ở các nhánh lifecycle khác.
- **Upload race:** archive được `os.replace` trước DB insert/semantic validation; request lỗi có thể để orphan, và upload đồng thời cùng version có khả năng làm row DB không khớp file cuối.
- **Duplicate tar name:** normalization bằng set/dict có thể parse metadata từ một member nhưng hash/extract member trùng tên khác.

Khuyến nghị: state machine một nguồn sự thật, archive/delete transaction có runtime quiesce + ACK; tombstone theo robot/generation/cursor và ngừng phát lại sau ACK; resync luôn verify storage; khóa `(map_id, version)`; cleanup rollback; reject tên canonical trùng.

### 5.6 `MAP-05`, `CMD-02`, `UI-01` — ownership, idempotency và UI mapping (**P1**)

- Mapping start/action chỉ kiểm role/online/capability/health, không bind exclusive control session. Operator/API thứ hai có thể chuyển mode; Edge sẽ cancel navigation đang chạy mà Center không thể hiện ownership tương ứng.
- Receipt chỉ bind `robot_id + command_type`, không hash payload/expected state. Cùng `request_id` dùng lại cho map B có thể nhận ACK cũ của map A nhưng handler ghi Center là B, làm Center/Edge lệch nhau.
- `MappingTransport.ts` có listener nhưng không thấy được instantiate; panel giữ local state, fetch lúc mount rồi tự cập nhật sau action. Fault/reset/upload completion bất đồng bộ từ robot có thể không hiện, và UI có thể tiếp tục gửi `expected_state` cũ.
- Phần UI resync đã sửa: ACK `SYNC_PENDING` chỉ hiện “Chờ đồng bộ”, version được poll 2 giây/lần và nút bị khóa trong lúc marker upload đang chờ; chỉ upload checksum đúng mới chuyển `SYNCED`. Vấn đề mapping intent bị xóa khỏi `sessionStorage` trước khi fetch/start chắc chắn thành công vẫn còn.

Khuyến nghị: mọi mutation mapping phải có `control_session_id`, lease epoch và robot mode precondition; idempotency key bind canonical payload hash; Center phát state/version sequence có snapshot-resync; UI chỉ hiển thị trạng thái authoritative và phân biệt pending/completed/failed.

### 5.7 `LOC-01..03` — freshness và callback cũ trong localization (**P2, cần hardening**)

Ba gap không nên bỏ qua vì localization READY là điều kiện cấp quyền chuyển động:

1. Callback hoàn tất của `_start_global_localization()` không capture/kiểm `attempt_id`, generation, map identity hoặc state hiện tại và không lấy `localization_lock` (`adapter_node.py:6376-6387`). Future của attempt A trả lỗi muộn sau operator hint, map deactivate hoặc attempt B vẫn có thể đặt `LOCALIZATION_FAILED`, ghi đè state mới. Đáng chú ý, worker global-scan khác đã có fence đúng, nên có thể áp dụng cùng pattern.
2. `_amcl_pose_callback` và `_particle_cloud_callback` bỏ `message.header.frame_id/stamp`, rồi ghi freshness bằng `time.monotonic()` lúc callback (`adapter_node.py:3003-3100`). Message cũ còn trong executor queue, replay hoặc sai frame có thể được tính là evidence mới của attempt hiện tại. READY dùng chính receipt-age này (`7012,7027-7035`). Các evidence scan/map khác vẫn phải qua gate nên đây không phải một mình nó chắc chắn false-localize, nhưng làm suy yếu giả định độc lập của gate.
3. `_update_pose()` lookup latest transform rồi mỗi poll thành công lại đặt `last_map_tf_monotonic=now`, không đọc source stamp (`adapter_node.py:6050-6077`). Diagnostic có tính `age_ms`, nhưng monitor chỉ fail khi link unavailable, không khi stale (`4996-5076`). TF buffer bị đóng băng có thể tiếp tục được gọi là “fresh” cho localization health. Execution pose có một stamp guard riêng, giúp giảm tác động ở một số đường chạy nhưng không sửa semantics READY chung.

Khuyến nghị: bind mọi localization message/callback với map generation + attempt ID; reject frame sai, source stamp trước attempt/reset và stamp vượt age/future limit; TF gate dùng transform source age. Test delayed/reordered callbacks, DDS queue cũ, publisher restart và simulated clock jump.

## 6. Rà soát lập đường và đi tự động

### 6.1 Điểm mạnh của planner hiện tại

- Planner dùng footprint chữ nhật thực, không rút gọn robot thành một điểm hoặc vòng tròn quá bảo thủ.
- Dùng exact swept-footprint/SAT và Euclidean distance transform để kiểm segment thẳng và vùng quay (`navigation_core.py`, vùng logic khoảng dòng 1199, 1579, 1722, 1901 trở đi).
- State lattice stop–turn xét sweep khi đổi hướng, kiểm toàn bộ segment sau tạo, và chỉ cho reverse ban đầu có giới hạn để tìm “turn bay”; không dùng reverse lặp tùy tiện.
- Goal và route được kiểm đúng map/version, bounds và active state; hardware backend guard không cho simulator navigation điều khiển nhầm robot ROS thật.
- Khi đang đi, hệ thống có live corridor gate, raw LiDAR motion safety, localization monitor, timeout/TTL và khả năng wait/replan/alternative route.
- Logic cụ thể, deterministic và dễ giải thích hơn một controller học máy dạng hộp đen; rất có giá trị cho debug và safety argument.

### 6.2 `NAV-01` — lỗi scope trong localization resume (**P1, lỗi xác định**)

`adapter_node.py:6677-6785` lưu `original_directions` ở scope ngoài. Hàm lồng `resume()` đọc biến này rồi gán lại tại dòng 6715-6716; Python coi nó là local và phát `UnboundLocalError` khi đường giữ lại có ít nhất hai điểm. `except` chỉ bắt `AdapterError`, vì vậy thread thoát mà không chắc reset `localization_resume_in_progress`; robot có thể kẹt ở PLANNING/recovery. Ruff cũng phát hiện `F823` tại đây, nhưng test hiện chưa đi qua nhánh runtime này.

Khuyến nghị: thêm regression test ép nhánh preserved path ≥2 điểm, exception cleanup bằng `finally`, và invariant rằng mọi recovery kết thúc ở một state hữu hạn với zero velocity. Đây là lỗi nhỏ về cú pháp scope nhưng tác động vận hành lớn.

### 6.3 `NAV-05` — lỗi localization chưa dừng navigation ngay (**P0, lỗi fail-safe xác định**)

Decorator `localization_callback` bắt mọi exception từ AMCL pose, particle cloud và scan callback (`adapter_node.py:132-161`, các chỗ dùng tại dòng 3003, 3083, 5448). Nhánh lỗi đặt `localized=False`, đổi state và dừng **localization rotation**, nhưng không:

- thu hồi `motion_owner="NAVIGATION"`;
- tăng `navigation_goal_generation`/`execution_segment_token`;
- cancel `current_goal_handle`;
- phát zero lên `navigation_velocity`.

`_auto_velocity_callback` vẫn forward lệnh khi motion owner còn NAVIGATION (`adapter_node.py:1605-1732`). Ở pha straight, pose-staleness cuối cùng có thể chặn sau timeout, và costmap/motion-safety còn là lớp phòng thủ; nhưng không có invariant “localization fault → zero ngay trong cùng transition”. Một exception giữa đoạn có thể để command cũ/FollowPath tiếp tục trong cửa sổ khi pose đã không còn đáng tin.

Khuyến nghị: gom mọi fatal localization transition vào một hàm idempotent dưới lock: revoke owner, bump generation/token, clear active segment, publish zero trước, cancel action async, persist fault và chỉ sau đó cập nhật UI. Test inject exception ở từng callback trong STRAIGHT/TURN/DISPATCHING và đo `/cmd_vel` cuối.

### 6.4 `NAV-06` — recovery cũ có thể vượt qua cancel (**P0, race xác định**)

Nhiều worker kiểm `expected_generation` ở đầu hoặc ngay trước `_navigate`, nhưng không truyền nó vào `_navigate`: sensor-time resume (`adapter_node.py:6634-6643`), localization resume (`6720-6757`), dynamic replan (`12431-12469`), execution replan (`12756-12829`) và failed-segment recovery (`13196-13207`). Trong `_navigate`, fence tại commit chỉ chạy khi tham số khác `None` (`9559-9568`).

Race cụ thể:

1. Worker recovery kiểm generation hợp lệ rồi bắt đầu plan/validate.
2. Operator cancel/manual takeover hoặc E-stop tăng generation, revoke owner và phát zero.
3. Worker cũ gọi `_navigate(... expected_generation=None)` sau đó.
4. Commit bỏ fence, tăng generation mới, đặt `NAVIGATING` và chuẩn bị phát chuyển động lại.

`_cancel_navigation` còn không clear `localization_resume_context` (`adapter_node.py:13211-13266`), nên context goal cũ có thể tiếp tục được nhặt khi localization trở lại READY. Motion-safety/E-stop vẫn có thể chặn đầu ra khi E-stop còn asserted, nhưng cancel/takeover không nên phụ thuộc lớp cuối để ngăn nhiệm vụ cũ hồi sinh.

Khuyến nghị: mọi async work nhận immutable `{generation, mission_id, map_id, version, boot_epoch}` và bắt buộc CAS tại **mọi** commit/dispatch; cancel thu hồi context của tất cả recovery; stale worker chỉ được cleanup riêng generation của nó, không được đổi state hiện tại. Dùng deterministic concurrency tests với barrier ngay trước `_navigate` commit.

### 6.5 `NAV-07` — pose reset/relocalize trong khi chuyển động (**P0, invariant bị thiếu**)

Dispatcher cho `map.set_initial_pose` đi thẳng vào `_set_initial_pose` mà không kiểm mission/motion (`adapter_node.py:2439-2440,8521-8578`). Lệnh này reset evidence và publish initial pose nhưng không cancel, revoke owner hoặc zero trước.

`map.relocalize` có guard `current_goal_handle is not None` (`adapter_node.py:2441-2447`), nhưng pha TURN được adapter tự thực thi và publish angular velocity (`10255-10272`) mà không cần giữ một FollowPath handle. Vì vậy guard “không có goal handle” không đồng nghĩa robot đứng yên. Reset/relocalize giữa STRAIGHT hoặc TURN có thể thay đổi map→odom/pose khi bộ chấp hành vẫn nhận vận tốc.

Khuyến nghị: tạo invariant duy nhất `motion_quiesced` dựa trên owner, execution phase, action handle, command và measured velocity; mọi initial-pose/relocalize/map transition phải gọi stop-and-confirm, bump generation, cancel, đợi measured-zero rồi mới reset pose. Reject thay vì tự làm nếu command không có quyền mở transition này.

### 6.6 `NAV-08` — path hợp lệ chưa được bind với goal (**P2, lỗi integrity**)

`_navigate` nhận `goal_payload` và `command_payload.points` độc lập (`adapter_node.py:9444-9537`). `_ensure_executable_path(points, goal)` có tham số `goal` nhưng không dùng nó (`4761-4801`). Vì thế một path vẫn có thể vượt toàn bộ swept-footprint validation dù endpoint không tương ứng goal/route/mission. Sau khi robot đi hết, completion logic mới đo khoảng cách, phát hiện chưa tới goal và replan; an toàn hình học của path vẫn được giữ nhưng robot đã di chuyển không đúng ý định.

Khuyến nghị: trước cấp motion authority, bind canonical path hash với map/version/mission/route/goal và kiểm endpoint position/yaw tolerance; receipt/idempotency dùng cùng hash. Không cho lỗi ghép payload bị “sửa” sau khi robot đã đi.

### 6.7 Kiến trúc thực thi và điểm lệch Nav2

Adapter thực tế gửi từng segment qua `FollowPath` (`adapter_node.py:12868-12899`). Cấu hình RPP đặt `use_collision_detection: false` (`config/nav2_params.yaml:447-454`), vì collision được kỳ vọng do exact global validator, live corridor và motion-safety xử lý. Cách này có ba lớp phòng thủ nhưng chúng có common-mode dependency vào map/TF/scan normalization và custom Python; mất một lớp collision check độc lập ngay tại controller.

Runtime BT được tạo trong `speed_profiles.py`, nhưng đường adapter đang dùng trực tiếp `FollowPath`; `behavior_tree_paths` gần như chỉ được gán. Tài liệu lại nói NavigateToPose và replan 1 Hz. Đây là dấu hiệu hai kiến trúc cùng tồn tại nhưng chỉ một kiến trúc thật sự có hiệu lực, dễ làm test/tuning nhầm component.

Khuyến nghị: lập một “executable architecture” duy nhất; hoặc dùng BT Navigator đầy đủ với replanning/recovery có kiểm soát, hoặc ghi rõ custom orchestration là nguồn sự thật. Nếu giữ `use_collision_detection=false`, phải chứng minh độc lập bằng fault-injection rằng các lớp còn lại luôn fail-safe; tốt hơn là đánh giá bật controller collision checking mà không double-stop sai.

### 6.8 Hiệu quả và chất lượng đường

Stop–turn làm quỹ đạo dễ kiểm chứng, phù hợp differential drive và hành lang rất chặt. Nhược điểm là dừng nhiều, jerk cao, tốn thời gian/năng lượng, tạo cảm giác thiếu tự nhiên trước hành khách và có thể thất bại ở hình học mà quỹ đạo cong liên tục vẫn đi được. Planner hiện tối ưu mạnh theo tính khả thi hình học; chưa thấy objective đầy đủ cho comfort, năng lượng, crowd/social cost, hướng tiếp cận, hay ETA toàn fleet.

Nên benchmark song song, không thay ngay:

- baseline hiện tại;
- Nav2 Smac Hybrid/Lattice + smoother;
- RPP collision checking bật;
- MPPI với differential-drive model và constraints tương đương.

So bằng cùng rosbag/scenario: tỷ lệ hoàn thành, min clearance thật, số emergency stop, localization loss, jerk, thời gian, quãng đường, năng lượng và CPU p95/p99. Chỉ chọn phương án mới khi safety envelope không giảm.

### 6.9 Lập quãng đường, xếp hạng route và ETA

Phần này được làm tương đối bài bản. `route_geometry_metadata` cộng chiều dài Euclid của mọi segment, thống kê turn count/turn angle, passage/side/turn clearance và ước lượng thời gian gồm thời gian tịnh tiến + quay + overhead dừng/settle (`navigation_core.py:1980-2162`). Planner không đơn giản chọn đường ngắn nhất: ranking lexicographic ưu tiên loại reverse không được hỗ trợ, ít turn, reverse-to-bay ngắn, ETA, chiều dài, rồi clearance/độ lệch hướng (`navigation_core.py:7063-7110`). Route thay thế còn bị lọc theo overlap để không đưa ba biến thể gần như giống nhau. Đây là ưu điểm vì đường ngắn nhất chưa chắc an toàn, mượt hoặc thực thi được.

Các giới hạn cần ghi đúng cho UI/operator:

- `distance_m` là tổng chiều dài hình học của preview (`adapter_node.py:3520-3527,9431-9439`), không phải odometer thực tế. Nó không tự bao gồm các replan/retreat/wait/manual recovery phát sinh sau khi bắt đầu.
- ETA dùng tốc độ danh định và turn/settle model; chưa thấy mô hình tải, pin, acceleration thực đo, crowd delay, cửa/thang máy hoặc congestion fleet. Do đó chỉ là estimate lập kế hoạch, không phải cam kết thời gian.
- Nhánh auto-start sau timeout dynamic route sắp lại candidate theo `total_length` (`adapter_node.py:12172-12212`), có thể làm tiêu chí “ngắn nhất” lấn át ranking giàu thông tin về turn/ETA/clearance đã dùng khi plan. Cần dùng cùng một canonical cost/ranking xuyên preview, auto-select và execution.
- Path hash hiện chưa bind goal như `NAV-08`; distance đúng của path không chứng minh path đúng của mission.

Nên báo đồng thời `planned_distance`, `executed_odometry_distance`, `remaining_distance`, ETA interval và nguyên nhân thay đổi route; metric phải reset/bind theo mission + boot/odom epoch để không cộng qua pose reset.

## 7. Rà soát tự động tránh vật cản và lớp an toàn chuyển động

### 7.1 Điểm mạnh

- Chỉ motion-safety xuất `/cmd_vel` cuối; nguồn điều khiển khác đi qua `/cmd_vel_raw`. Thiết kế exclusive ownership này giảm nguy cơ hai node cùng lái motor.
- Edge dùng datagram có `boot_id`, sequence, TTL và latest-only; motion bridge có watchdog, còn driver có lớp watchdog riêng. Disconnect chủ động cancel navigation rồi gửi stop.
- Scan normalized có freshness check; khoảng dừng được tính theo vận tốc, latency, braking deceleration và clearance thay vì một ngưỡng cố định hoàn toàn.
- Vật cản động không làm bẩn static map. Local costmap được cluster/spatial-hash, lọc cell thuộc static map, tích lũy observation, TTL và motion confirmation; planner có TTC projection, wait/replan và route thay thế.
- Manual mode có sequence/TTL và mode freshness khoảng 350 ms; mất luồng lệnh thì không tiếp tục giữ vận tốc vô hạn.
- Các thay đổi mode/map đều cố đưa vận tốc về zero, cancel action cũ; đây là nền tảng tốt cho transition an toàn.

### 7.2 `SAF-01` — “Đã dừng an toàn” không phải bằng chứng dừng vật lý (**P0**)

Control WebSocket ở Center trả ACK sau khi `hub.forward_to_robot()` mới chỉ `send_json` tới socket robot (`center-backend/app/api/websockets.py:395-425`, `hub.py:511-523`). Edge xử lý stop ở `simulator/client.py:776-779`, gửi zero tại `_stop_motion` (`2217-2220`) rồi phát ACK riêng trên robot WebSocket. ACK này đi vào luồng telemetry của Center (`websockets.py:270-291`); frontend `ControlTransport` chỉ nghe control WebSocket (`ControlTransport.ts:100-120`).

Trong khi đó `useTeleoperation.ts:17-20,48-52` đặt ngay trạng thái “Đã dừng an toàn”, và không phân biệt `accepted` với `completed`. Lệnh local là ba Unix datagram nonblocking (`motion_driver.py:113-123`), chưa có xác nhận motor/wheel encoder bằng zero.

Điều này tạo một HMI hazard: mạng Pi lỗi, Edge process chết, socket driver đầy hoặc motor controller không phản hồi nhưng operator vẫn thấy thông báo an toàn.

Khuyến nghị trạng thái end-to-end:

```text
REQUESTED → FORWARDED → EDGE_EXECUTING → DRIVER_ACCEPTED
          → MEASURED_ZERO → COMPLETED
          ↘ timeout/disconnect → UNKNOWN (không được hiển thị SAFE)
```

`COMPLETED` cần gắn cùng command ID/boot epoch và bằng chứng vận tốc đo dưới ngưỡng trong N mẫu. Nút software stop không được thay thế E-stop phần cứng safety-rated.

**Cập nhật 2026-08-29:** chuỗi browser → Center → Edge nay chờ ACK cuối từ robot. Edge chỉ trả `completed` sau khi odometry mới xác nhận `|v| ≤ 0,015 m/s` và `|ω| ≤ 0,03 rad/s` liên tục 250 ms; quá 3 giây hoặc feedback stale trả `unknown`, không trả SAFE. UI bỏ timer 140 ms và giữ trạng thái `stopping` nếu chưa có measured-zero. Đây là xác nhận software dựa trên odometry, chưa phải bằng chứng encoder/HIL độc lập và không thay thế E-stop phần cứng.

### 7.3 `SAF-02` — xung đột self-filter và footprint (**P0, cần xác minh hình học ngay**)

URDF, planner, costmap và motion safety đều dùng thân robot `0,30 × 0,20 m` (`robot.urdf:5-6`; `nav2_params.yaml:227-240,491,537`; `motion-safety/config/safety.yaml:9-23`). Sensor normalizer lại xóa mọi endpoint LiDAR trong hình chữ nhật `0,40 × 0,36 m` (`sensor_time.yaml:16-24`; `sensor_normalizer.py:41-47,205-216`) trước khi safety đọc `/scan/normalized` (`safety_node.py:89-94`).

Hai khả năng đều phải xử lý:

- Nếu phần chênh lệch là không khí trống, vật cản bên ngoài chassis nhưng lọt vào rectangle mask sẽ biến mất khỏi scan safety.
- Nếu phần chênh lệch là bumper, khung, cảm biến hoặc phụ kiện thật mà LiDAR nhìn thấy, footprint collision đang nhỏ hơn robot thật.

Mask hình chữ nhật còn over-mask các góc. Comment “accessory envelope” không thay thế số đo. Cần lấy CAD/tape measure, định nghĩa footprint bảo thủ của toàn bộ phần cứng, self-filter theo polygon/range/angle đúng hình học, và replay rosbag với vật cản đặt quanh toàn chu vi. Đây là gate trước thử nghiệm hành lang hẹp.

**Cập nhật 2026-08-29:** người vận hành xác nhận thân xe `0,30 × 0,20 × 0,15 m`, LiDAR ở giữa mặt trên. URDF đã đổi chiều cao thành 0,15 m, `base_link` ở z=0,075 m và `laser_frame` ở z=0,15 m so với mặt đáy. Offset phẳng LiDAR đã về `(0,0)`; self-mask giảm đúng bằng nửa dài/rộng `(0,15; 0,10)` và test bắt buộc mọi điểm ngoài thân không bị mask. Finding cấu hình này đã đóng; thử vật thật quanh toàn chu vi vẫn còn mở.

### 7.4 `SAF-03` — nguồn bảo vệ chết có thể giữ trạng thái “clear” (**P0**)

`safety_node.py:101-136` khởi tạo bumper/cliff/range/external stop ở `false/clear`; callback cập nhật trạng thái (`205-256`). Tick kiểm stale cho scan (`557-560`) nhưng chưa thấy timestamp/heartbeat tương đương cho bumper, cliff, range, external stop/direction (`437-460`). Nếu một driver chết ngay sau lần báo clear, robot có thể tiếp tục.

External obstacle watchdog mặc định `ROS_OBSTACLE_WATCHDOG_MS=0` trong compose/Edge env; `control_protocol.py:70-80` hiểu 0 là vô hiệu hóa expiry. Odom stale lại fallback sang vận tốc yêu cầu (`safety_node.py:566-575`) thay vì dừng; cách này có thể bảo thủ khi tính braking speed nhưng không phát hiện slip/overspeed thực.

Khuyến nghị: mỗi safety source có `source_id`, monotonic timestamp, sequence/epoch, deadline và trạng thái `UNKNOWN`; `UNKNOWN` phải stop hoặc hạ cấp theo hazard analysis. Dùng ROS 2 QoS deadline/liveliness và application watchdog; cấu hình production không được cho phép watchdog 0. Odom stale khi robot đang có lệnh chuyển động nên tạo controlled stop và fault.

**Cập nhật 2026-08-29:** motion-safety đã có deadline helper dùng monotonic time cho E-stop, cliff, bumper, range, external stop và external direction. Software E-stop heartbeat là bắt buộc với timeout 1,2 s; các nguồn phần cứng chưa lắp được khai báo explicit `*_heartbeat_required=false` thay vì ngầm coi chúng đang khỏe. Production external-obstacle watchdog mặc định tăng từ 0 lên 500 ms. Phần freshness đã được cải thiện nhưng finding chưa đóng hoàn toàn: odometry stale vẫn cần policy controlled-stop và các sensor phần cứng phải được bật `heartbeat_required` khi lắp.

### 7.5 `SAF-04` — software E-stop và bypass vật cản (**P0**)

Frontend gọi nút là “Dừng khẩn cấp” (`ControlPad.tsx:85-97`) và gửi reason `emergency_stop`. `control_bridge.py:279-285` latch software flag khi stop, nhưng bất kỳ motion command mới nào cũng tự clear latch (`314-318`); source còn ghi rõ E-stop vật lý là bắt buộc. Đây là stop command có latch mềm, không phải emergency stop độc lập, manual-reset, safety-rated.

Nút “Chống vật cản TẮT” là toggle người dùng bình thường (`ControlPad.tsx:140-162`). Ở nhánh bypass, `safety_node.py:466-497` cho command đi qua mà bỏ LiDAR, range, external stop/directional gates; chỉ còn E-stop/cliff/bumper/watchdog. Trong môi trường du lịch có người, một thao tác UI đơn lẻ không nên vô hiệu nhiều lớp bảo vệ như vậy.

**Cập nhật 2026-08-29:** software E-stop đã thành latch chỉ nhả bằng message `control.estop.reset`; lệnh vận tốc mới không còn tự clear latch. UI khóa cả nút/keyboard di chuyển khi latch hoặc đang xác minh dừng và chỉ hiện thao tác reset riêng. Finding vẫn mở một phần vì đây chỉ là E-stop phần mềm; yêu cầu phần cứng safety-rated và việc khóa bypass bằng service mode vật lý chưa được triển khai.

Khuyến nghị:

- Đổi nhãn software control thành “Dừng điều khiển” nếu chưa có safety chain thật.
- E-stop vật lý phải cắt/disable torque theo risk assessment, latch độc lập và chỉ reset tại robot sau khi kiểm vùng an toàn.
- Bypass chỉ dùng service mode tại chỗ: role riêng, key/physical enable, hold-to-run, tốc độ rất thấp, thời gian giới hạn, audit log và banner liên tục; không cho autonomous navigation.

### 7.6 `NAV-03` — giới hạn của cảm biến 2D và braking model (**P1/validation gap**)

Costmap chủ yếu nhận planar `LaserScan` (`nav2_params.yaml:493-520,566-599`). Tài liệu project tự thừa nhận LiDAR không thấy chắc kính, vật quá thấp, overhang, drop-off và chưa có anti-cliff sensor đầy đủ. IMU không được fusion vì orientation/gyro chưa đáng tin (`config/ekf.yaml:16-30`); localization phụ thuộc wheel odometry + AMCL, nên trượt bánh làm xấu heading/pose.

Safety config dùng các giả định cố định như latency `0,12 s`, braking deceleration `0,60 m/s²`, clearance `0,04 m`, angular deceleration `2,0 rad/s²`. Chưa có bằng chứng các số này bao phủ tải nặng nhất, pin yếu, sàn trơn, dốc, lốp mòn, CPU/network p99 và nhiệt độ vận hành.

Khuyến nghị: thêm sensing theo hazard thực tế (safety lidar/depth/3D, cliff/drop sensor, bumper chain), calibrate extrinsics có version, và đo stopping envelope thật. Tính ngưỡng từ worst-case confidence interval, không chỉ giá trị danh định.

### 7.7 Vật cản động: khá tốt cho một robot 2D, chưa phải dự đoán hành vi người

Trong `navigation_core.py:161-482,7874-7988` và `nav2_params.yaml:150-195`, hệ thống cluster obstacle, associate gần nhất, yêu cầu nhiều observation, xác nhận chuyển động, TTL khoảng 2 giây và chiếu TTC khoảng 3 giây. Đây là thiết kế hợp lý để không biến nhiễu một frame thành vật cản “thật”, đồng thời còn motion-safety xử lý tức thời.

Hạn chế: association heuristic và constant-velocity 2D không hiểu người dừng/đổi hướng, occlusion, nhóm người, xe đẩy, cửa mở hay vật thể cao/thấp; fixed thresholds có thể vừa chậm phản ứng vừa tạo oscillation tùy mật độ. Chưa có shared dynamic obstacle giữa nhiều robot.

Nên giữ lớp phản xạ hình học làm safety envelope, rồi bổ sung prediction/social cost chỉ như lớp tối ưu. Không để semantic AI có quyền bỏ qua raw safety stop.

## 8. Rà soát đi đường hẹp

### 8.1 Ưu điểm nổi bật

Đây là một trong những phần mạnh nhất của project:

- `assess_corridor` tách clearance phía trước, hai bên khi đi thẳng và vùng quay (`navigation_core.py:7649-7795`).
- Dùng paired walls/median để tránh một tia nhiễu làm đổi quyết định; phân biệt hard margin và comfort margin.
- Turn sweep dùng footprint thật; initial reverse có giới hạn chỉ để tìm vùng quay, sau đó không reverse lặp khó đoán.
- Đường toàn cục vẫn là authority; live sensing tạo gate tức thời. Khi không đủ rộng, hệ thống có thể dừng/chờ, yêu cầu người quyết định hoặc chọn alternative route.
- Các tham số hard/comfort và localization uncertainty được tách riêng, thuận lợi cho tuning có bằng chứng.

### 8.2 `COR-01` — một vách quá sát có thể bị ghi đè thành passable (**P1 → đã sửa phần mềm**)

Trong `evaluate_corridor`, nếu không có bin nào đồng thời thấy cả trái và phải, code vẫn tính clearance riêng từ wall return một phía (`navigation_core.py:7706-7747`). Một wall quan sát được nằm gần hơn hard margin làm `hard_side_clear=False`. Tuy nhiên ngay sau đó, điều kiện `not both_sides_observed and front_clear` vô điều kiện đặt lại `physically_passable=True` (`7754-7760`).

Ví dụ: vách trái live chỉ còn 1,5 cm trong khi hard margin là 2 cm, vách phải không có return và trước mặt trống. Evidence đã chứng minh một phía vi phạm hard margin nhưng classification trở thành `NARROW_OR_UNCERTAIN/LIVE_SIDE_INCOMPLETE`, `can_go_straight=True`. Adapter chỉ handoff khi evidence là `PHYSICALLY_BLOCKED` (`adapter_node.py:5558-5567`), nên segment có thể tiếp tục tự động. Motion-safety còn reserve thẳng 1 cm, vì thế khoảng 1–2 cm này không chắc bị lớp cuối dừng.

Trạng thái sửa 2026-08-29: `evaluate_corridor` nay giữ cờ observed riêng cho
trái/phải/cặp hai phía và áp invariant `any observed hard violation => blocked`.
Phía không quan sát vẫn chỉ là uncertainty; nếu phía đã thấy đạt hard margin thì
static route có thể tiếp tục làm authority, không biến mọi missing return thành
false stop.

Adapter phát zero ngay từ mẫu `PHYSICALLY_BLOCKED` đầu tiên; chuỗi 5 mẫu/0,40 s
chỉ dùng để quyết định recovery, không phải thời gian robot được phép đi thêm.
Khi xác nhận, adapter cancel FollowPath và tăng segment token, giữ nguyên goal,
đánh dấu đoạn lỗi tạm thời, rồi dùng turn-bay recovery hiện có:

1. ưu tiên lùi thẳng với heading cố định nếu rear direction còn được safety cho phép;
2. kiểm toàn bộ swept footprint trên Saved Map và dynamic exclusions;
3. chạy đoạn thoát ở profile `SLOW`, có giới hạn bởi
   `turn_bay_max_relocation_distance`;
4. khi tới vùng quay, replan từ pose mới tới **đích ban đầu**;
5. nếu không chứng minh được khoảng thoát, giữ zero nhưng vẫn giữ mission và
   chạy bounded wait/periodic alternative replan, không chuyển sang dừng hẳn
   `NARROW_PATH_DECISION`.

Không có lệnh “lùi mù”: clearance quan sát phải lớn hơn 0, pose/map và atomic
safety snapshot phải sẵn sàng, rear/forward mask phải cho phép, static sweep
phải hợp lệ. Cancel, E-stop hoặc manual takeover vẫn vô hiệu worker bằng
generation fence. Chức năng bật/tắt chống vật cản thủ công trên giao diện được
giữ nguyên, không bị thay đổi bởi bản sửa này.

### 8.3 Rủi ro và trade-off

- Hard margin cỡ 1–2 cm và localization uncertainty 4 cm rất nhạy so với global map resolution 5 cm, sai số extrinsic, méo scan, rung chassis và độ rơ bánh. Adapter có cộng allowance theo resolution, nhưng chưa thay thế error budget thực nghiệm.
- Khi thiếu cả hai vách live nhưng phía trước clear, logic có thể coi hình học là passable và dựa vào static route/downstream safety (`navigation_core.py:7754-7758`). Trong hành lang bị thay đổi, mất side returns hoặc kính, giả định này nguy hiểm.
- Stop–turn làm robot chiếm hành lang lâu và có thể block người/robot khác; chưa có queue, right-of-way, corridor mutex, retreat negotiation hoặc deadlock resolution.
- Comfort margin không phải safety margin. Không được giảm hard margin chỉ vì simulation chạy qua.

### 8.4 Điều kiện chấp nhận cho đường hẹp

Không nên quyết định theo “đi lọt một lần”. Cần xây error budget:

```text
clearance tối thiểu thật
  ≥ footprint thật + phụ kiện/dao động
  + sai số localization p99.9
  + sai số map/scan/extrinsic p99.9
  + tracking overshoot và stopping lateral envelope
  + manufacturing/tải trọng/sàn margin
```

Test theo ma trận chiều rộng hành lang, cua chữ L/U, cửa hẹp, vách kính/mờ, một bên không phản xạ, vật mềm nhô ra, người bước ngang, tải lệch tâm, sàn trơn và mất một sensor source. Đo clearance bằng ground truth độc lập, không dùng chính AMCL làm chuẩn.

## 9. Center–Edge, giao thức điều khiển và độ tin cậy phân tán

### 9.1 Điểm mạnh

- Robot tạo outbound WSS, dùng JWT enrollment ngắn hạn; giảm nhu cầu mở inbound port trên Pi.
- Control session bảo đảm một controller, có reconnect identity; browser command có TTL/sequence.
- Lệnh dài như navigation/map I/O chạy background nên không chặn hoàn toàn đường stop.
- Start navigation có nhiều preflight: lease, robot/map/version, localization/health; backend thật có guard không chạy simulator backend lên phần cứng ROS.

### 9.2 `CMD-01` — command receipt chưa bền và chưa phản ánh kết quả thật (**P1**)

`ConnectionHub` giữ robot socket, pending request, session/lock/lease/route trong dict RAM (`hub.py:95-113`). Redis có cấu hình/dependency nhưng không thấy dùng làm nguồn state realtime. Restart hoặc chạy nhiều worker mất pending/ownership/routes; startup chỉ khôi phục một phần session.

Command receipt được insert `PENDING` và commit trước dispatch (`navigation.py:263-277`; mapping tương tự). Nếu process crash ở cửa sổ này, retry gặp receipt pending nhưng không redeliver. Nếu Center timeout, receipt bị đánh rejected dù Edge có thể hoàn tất sau đó. Edge gặp duplicate command ID lại trả `accepted` chung, không replay original terminal result; cache chỉ giữ tối đa 2048 ID.

Khuyến nghị: transactional outbox cùng DB transaction, durable inbox/result ở Edge, payload hash, monotonic command epoch, terminal result replay và reconciliation khi reconnect. ACK phải tách transport acceptance, execution và effect verification.

### 9.3 Các lỗi consistency/ownership khác

- Robot→Center message thiếu TTL, monotonic sequence và discriminated type schema toàn tuyến; message đến muộn có thể ghi đè state mới. Runtime Pydantic nhận `message_type: str`, payload dict trong khi JSON schema liệt kê enum khác.
- Center preflight đọc health boolean cache nhưng không có per-source timestamp. Một heartbeat bất kỳ làm robot “online”, không chứng minh sensor health còn mới.
- Mission có `control_session_id`, nhưng action chủ yếu kiểm lease hiện tại, chưa bind mission với session đã tạo nó; session mới có thể thao tác mission cũ.
- Retry `start_navigation` sau lần thành công có thể bị chặn vì mission không còn READY trước khi code tra idempotency receipt.
- Legacy `/preview`, `/goal`, `/cancel` còn dùng route in-memory/Manhattan và ràng buộc map yếu hơn; cần deprecate hoặc đưa về cùng state machine.
- Broadcast telemetry gửi tuần tự và được await trong robot receive loop; một browser client chậm có thể tạo head-of-line blocking. Send cần timeout/bounded queue/drop policy.
- Frontend chỉ sequence pose. Edge restart đưa sequence về 0; nếu browser socket sống, pose mới có thể bị bỏ đến khi đếm vượt giá trị cũ. Sequence phải gồm boot epoch.
- Sync SQLAlchemy trong async WebSocket có thể chặn event loop khi tải cao.
- JWT user đặt trong query string WebSocket dễ lọt log/history; nên dùng one-time ticket hoặc secure cookie/subprotocol.
- `/health` luôn trả healthy, chưa phản ánh DB/storage/event-loop/robot gateway readiness.
- Default secret/password có fallback; production startup nên fail nếu còn giá trị mặc định.

## 10. Frontend/HMI và khả năng vận hành

Điểm tốt là UI có control lease, hiển thị map/version, localization status, lựa chọn route thay thế và công cụ manual override. Tuy nhiên, HMI an toàn phải hiển thị **trạng thái thực tế**, không chỉ kết quả HTTP/forward:

- Stop cần trạng thái `UNKNOWN` rõ ràng nếu mất ACK/measurement.
- Mapping “activate”, “resync”, “recover” đang đặt tên mạnh hơn hành động thật; cần phân biệt registry promotion, robot load, validation và async upload.
- Fault/event từ robot cần timeline có sequence, timestamp nguồn và correlation ID; không chỉ toast tạm thời.
- Mọi bypass/service mode phải nổi bật liên tục, có countdown, speed cap và audit operator.
- Khi Center–robot split-brain, UI nên khóa lệnh mới và yêu cầu reconcile, không suy luận active chỉ từ `active_version != null`.

Build frontend thành công nhưng bundle main khoảng 1,097 MB minified và một chunk media khoảng 540 KB; build cảnh báo code-splitting. Đây là vấn đề hiệu năng/maintainability, không phải lỗi navigation trực tiếp, nhưng tablet yếu/mạng chậm có thể làm HMI phản hồi kém đúng lúc cần thao tác.

## 11. Hạ tầng, bảo mật và khả năng bảo trì

- `npm audit --omit=dev` không thấy vulnerability production. Full audit có một high advisory ở dependency dev `nanoid@3.3.16` đi qua Vite/PostCSS; tác động chủ yếu tới custom generator build-time, vẫn nên cập nhật lockfile/toolchain.
- Python dependencies chưa audit CVE hoàn tất vì lỗi mạng nêu ở phần giới hạn.
- Docker cài `ros-humble-slam-toolbox` không pin package version; một rebuild có thể đổi behavior. Cần SBOM và image/package digest bất biến.
- `navigation_core.py` và `adapter_node.py` rất lớn (xấp xỉ 10 nghìn và 14 nghìn dòng), gom nhiều trách nhiệm và state transition; review coverage và formal reasoning khó. Nên tách state machine, geometry, localization, map lifecycle và transport theo interface có typed contract.
- Static analysis chọn lọc phát hiện lỗi thật `F823`; CI nên chạy Ruff/Pyright/Mypy ở mức tăng dần, không chỉ test happy path.
- Không thấy durable append-only operational event log/flight recorder đủ để tái dựng một near-miss xuyên Center–Edge–ROS. Đây là yêu cầu quan trọng khi robot chạy gần người.
- KeepoutZone/SpeedZone có model/read path nhưng adapter trả mảng rỗng và chưa thấy CRUD/enforcement end-to-end. Hiện nên coi chúng là placeholder, không phải vùng hạn chế thật.

## 12. So sánh với robot tự hành hiện đại

### 12.1 Mốc tham chiếu

Đối chiếu được thực hiện với tài liệu chính thức/trang sản phẩm truy cập ngày 29/08/2026:

- [Nav2 Configuration Guide](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/) — kiến trúc server/plugin/lifecycle.
- [Nav2 Collision Monitor](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/) — vùng stop/slow/limit/TTC và nhiều nguồn cảm biến; chính tài liệu cũng nói đây không phải safety-certified hard real-time system.
- [Nav2 MPPI Controller](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/) — predictive sampling, motion models và critic-based optimization.
- [Nav2 Route Server](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/) — graph route, dynamic cost/closure và reroute.
- [Nav2 Behavior Trees](https://docs.nav2.org/behavior_trees/) và [Lifecycle Manager](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/).
- [ROS 2 QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) — deadline, lifespan và liveliness cho phát hiện nguồn dữ liệu lỗi.
- [SLAM Toolbox Humble](https://docs.ros.org/en/humble/p/slam_toolbox/) — mapping/localization và serialized pose graph.
- [Open-RMF core systems](https://osrf.github.io/ros2multirobotbook/rmf-core.html) — traffic schedule, future itinerary, conflict negotiation, fleet adapter và tích hợp tài nguyên tòa nhà.
- [ISO 3691-4:2023](https://www.iso.org/standard/83545.html) — phạm vi yêu cầu an toàn và verification cho driverless industrial trucks/AMR cùng operating zone. Đây là mốc phạm vi, không phải tuyên bố project tuân thủ.
- Trang nhà cung cấp: [MiR Fleet](https://mobile-industrial-robots.com/products/software/mir-fleet), [OTTO Fleet Manager](https://ottomotors.com/fleet-manager/), [LocusONE](https://locusrobotics.com/locusone). Các con số/quy mô trên trang này là tuyên bố của vendor.
- Ví dụ model cụ thể: [MiR250](https://mobile-industrial-robots.com/products/robots/mir250), [OTTO software update cho OTTO 1500 và tight-space traffic](https://ottomotors.com/company/newsroom/press-releases/otto-boosts-material-flow-efficiency-with-advanced-traffic-management-in-latest-software-update/), [Locus Origin 2026 datasheet](https://locusrobotics.com/wp-content/uploads/2026/04/Locus-Origin-Datasheet-2026.pdf).

### 12.2 Bảng so sánh

| Năng lực | Project hiện tại | Nav2/Open-RMF/AMR hiện đại | Đánh giá |
|---|---|---|---|
| Localization | AMCL với gate evidence, uniqueness, covariance, stability, pose persistence bảo thủ | Thường kết hợp lifecycle, sensor-health, multi-sensor/3D và tooling chẩn đoán; tùy nền tảng | **Mạnh về logic xác minh 2D**, yếu ở redundancy và cảm biến thật. |
| Lập đường tĩnh | Custom exact rectangular footprint, stop–turn lattice, deterministic | Plugin planner/smoother/controller; Hybrid/Lattice, RPP, MPPI và BT orchestration | Project dễ giải thích và hợp hành lang; hiện đại linh hoạt, mượt và modular hơn. |
| Điều khiển cục bộ | FollowPath từng segment, RPP collision checking tắt | Controller thường tự collision-check/predict trajectory; MPPI tối ưu nhiều trajectory | Project mất một lớp độc lập và kém mượt; cần benchmark, không thay theo phong trào. |
| Vật cản động | 2D cluster + observation/TTL + constant velocity/TTC, wait/replan | Collision Monitor nhiều source; controller dự đoán; một số nền tảng thêm 3D/semantic/social navigation | Tốt cho single-robot 2D, chưa đủ người đông/occlusion/đồ vật khó thấy. |
| Đường hẹp | Exact sweep, hard/comfort margin, turn bay, human decision | Route lanes/zones, traffic resource/mutex/queue; controller quỹ đạo liên tục | **Thuật toán hình học là ưu điểm**, nhưng thiếu error-budget thực và coordination. |
| Safety | Motion filter + watchdog + software stop/bypass | Safety-rated sensor/controller/E-stop và validation theo hazard/standard tùy sản phẩm; Nav2 tự nói Collision Monitor không phải safety certification | Chưa có safety case/chứng nhận; một số fail-open cần xử lý P0. |
| Map lifecycle | Immutable version, checksums, local cache, posegraph continuation | Production systems thường có deployment/validation/rollback/fleet-wide version consistency | Ý tưởng tốt, nhưng recovery/delete/archive/resync còn race và split-brain. |
| Fleet traffic | Gần như single robot; không reservation/negotiation | Open-RMF traffic schedule/negotiation; vendor công bố traffic/intersection/queue management | **Khoảng cách lớn nhất** nếu cần nhiều robot. |
| Task/pin/sạc | Mission theo robot; chưa có allocator/charging optimization | MiR/OTTO/Locus công bố task assignment, traffic, monitoring, charging/utilization ở fleet scale | Cần một lớp fleet orchestration riêng, không nên nhồi vào planner robot. |
| Độ bền lệnh | Session/TTL/idempotency cơ bản nhưng state RAM và ACK chưa end-to-end | Hệ production cần durable queue/result, reconciliation, HA và observability | Prototype tốt, chưa đạt fault tolerance production. |
| Khả năng quan sát | Có status/debug và test; thiếu event log xuyên tầng/readiness đầy đủ | Fleet manager hiện đại thường có dashboard, API, alert/history/analytics | Cần flight recorder và SLO trước pilot lớn. |

### 12.3 So sánh với một số mẫu cụ thể

Các so sánh dưới đây chỉ dùng thông tin vendor công bố, không suy ra thiết kế safety nội bộ vốn không public:

- **MiR250:** MiR công bố chassis `580 × 800 mm`, tốc độ tối đa 2 m/s, payload 250 kg và khả năng đi không gian hẹp 800 mm; robot dùng MiR Fleet cho task prioritization/traffic management. Project có chassis nhỏ hơn nhiều (`300 × 200 mm`) và exact footprint có thể tạo lợi thế ở lối cực hẹp, nhưng chưa được phép kết luận “đi hẹp tốt hơn”: MiR nêu một operating envelope sản phẩm đã định nghĩa, còn project đang có conflict self-mask/footprint và hard-margin chưa qua đo thật. Project cũng chưa có traffic coordination/cửa/thang máy ở fleet level.
- **OTTO 1500:** OTTO công bố phần mềm có thể điều chỉnh LiDAR safety footprint theo payload overhang và dùng Yield Line/traffic controls để giảm stop–go ở tight spaces; Fleet Manager còn chọn robot theo pin/tải, opportunistic charging và phòng congestion. Project dùng footprint/self-mask gần như cố định, stop–turn theo từng segment và chưa có payload-dependent envelope hoặc quyền sở hữu giao lộ/hành lang. Bù lại, exact swept rectangle của project minh bạch hơn những chi tiết planner vendor không công khai.
- **Locus Origin/Vector:** datasheet 2026 đặt Origin trong LocusONE cùng Vector/Array để phối hợp human-assisted và fully autonomous workflows. Điểm khác biệt chính không phải A* hay AMCL, mà là orchestration nhiều loại robot, workflow người–robot và scale kho. Project hiện tập trung đúng vào navigation một robot; muốn đạt cấp này cần lớp fleet/workflow riêng thay vì chỉ nâng local planner.

Kết luận từ ba ví dụ: project có thuật toán hình học single-robot đáng giá, nhưng AMR hiện đại cạnh tranh ở **toàn hệ thống**—safety envelope theo cấu hình/payload, traffic/resource ownership, task/pin/sạc, observability và quy trình deployment—không chỉ tìm một đường không va chạm.

### 12.4 Ưu điểm cạnh tranh thật sự của project

1. **Exact footprint cho robot nhỏ trong hành lang:** ít bảo thủ hơn circle footprint nhưng chặt hơn point planner.
2. **Localization có evidence:** không tự động tin last pose; đây là lựa chọn đúng cho startup/recovery.
3. **Luồng map bất biến và local-first:** phù hợp Edge mất mạng, có cơ sở để phát triển thành deployment pipeline tốt.
4. **Phân lớp motion safety riêng:** đúng hướng kiến trúc, miễn là freshness/geometry/E-stop được hoàn thiện.
5. **Determinism và giải thích được:** hữu ích cho audit, replay và tìm nguyên nhân near-miss.

### 12.5 Điểm thua rõ so với hệ hiện đại

1. Chưa có fleet-level traffic/resource/task/charging orchestration.
2. Chưa có safety-rated hardware chain và evidence-based verification package.
3. Quỹ đạo stop–turn kém mượt/comfort và hiệu suất hơn predictive continuous controller.
4. Perception 2D đơn nguồn, thiếu redundancy/3D/change detection.
5. Distributed command/state chưa durable/HA, UI chưa thể hiện uncertainty đúng.
6. Kiến trúc custom monolith làm nâng cấp, formal verification và tuyển người bảo trì khó hơn hệ plugin chuẩn.

## 13. Mâu thuẫn tài liệu và mã

Các mâu thuẫn này không chỉ là câu chữ; chúng có thể khiến operator/tester tin sai đường thực thi:

- `docs/MAPPING_AND_NAVIGATION.md` và `docs/navigation-mapping.md` nói navigation dùng `NavigateToPose` và replan 1 Hz; adapter thực tế direct `FollowPath`, cấu hình còn nói không timer replan.
- Tài liệu nêu safety clearance `+0,10 m`, slow `+0,20 m`, hysteresis 400 ms; runtime `safety.yaml` dùng lần lượt khoảng `0,04`, `0,10`, 200 ms.
- `MAP_REGISTRY.md` trước đây nói tombstone endpoint chỉ trả deletion chưa ACK;
  contract nay chủ ý trả snapshot toàn bộ tombstone và startup đã có reconnect
  barrier. Cách phát toàn lịch sử an toàn nhưng còn cần cursor/compaction khi scale.
- Tài liệu mô tả “Activate” sẽ tải/verify/load ở Edge; API activate chủ yếu đổi registry DB, robot load ở bước Control khác.
- Docs còn mô tả occupancy RLE/live scan Web dù implementation hiện xem phần đó obsolete/RViz-only.
- Comment speed profile nói controller collision checking trong khi RPP `use_collision_detection=false`.
- README nhắc Redis hỗ trợ realtime, nhưng `ConnectionHub` hiện vẫn là in-memory demo implementation.
- `docs/MAPPING_AND_NAVIGATION.md:49` nói mất localization sẽ không tự tiếp tục goal cũ; code và contract test lại chủ ý lưu context rồi auto-resume (`adapter_node.py:6677-6787`, `test_localization_contract.py:2054-2067`). Đây phải là quyết định safety/product rõ ràng, không thể để tài liệu và runtime trái nhau.

Khuyến nghị: sinh architecture/state diagrams và parameter tables từ cấu hình/code trong CI; mỗi release ghi rõ “effective runtime path”, xóa/deprecate đường legacy và fail test khi docs contract lệch.

## 14. Kết quả kiểm thử và kiểm tra tự động

### 14.1 Kết quả đã chạy

| Hạng mục | Kết quả | Ghi chú |
|---|---:|---|
| Frontend Vitest toàn bộ | **143 passed / 23 files** | Không có test fail. |
| Frontend production build | **Passed** | Có cảnh báo chunk lớn; main ~1,097 MB minified, media chunk ~540 KB. |
| Backend tests | **72 passed** | Chạy trong Docker/PYTHONPATH phù hợp project. |
| Simulator full test suite | **537 passed, 1 failed** | Failure VAAPI do audit container không có `/dev/dri/renderD*`, không chỉ ra lỗi navigation; có 2 warning tar `extractall`. |
| Focused map/motion/localization contract | **156 passed, 2 warnings** | Bao gồm regression tombstone/reconnect trước khi thêm test loop cuối. |
| `npm audit --omit=dev` | **0 production vulnerability** | Theo lockfile tại thời điểm rà soát. |
| Full `npm audit` | **1 high dev-only** | `nanoid@3.3.16` qua Vite/PostCSS; fix available. |
| Ruff chọn lọc `E9,F,B,ASYNC,PLE` | **Có lỗi** | Quan trọng nhất là `F823` đúng tại localization resume; còn lại gồm unused/false-positive cần triage. |
| Python dependency CVE audit | **Chưa hoàn tất** | Docker registry TLS handshake timeout; không được suy ra là sạch. |

Test pass nhiều chứng minh nền tảng tốt, nhưng không phủ định các lỗi integration/race: autosave test mock adapter và stop test chưa đo wheel-zero end-to-end. Startup tombstone/restore nay có regression test ép restore chạy trước barrier và test deletion đến giữa download; vẫn cần chaos/HIL trên Pi thật.

### 14.2 Test còn thiếu quan trọng

- Edge client thật + adapter thật cho autosave recovery.
- Chaos/HIL startup barrier: cắt điện/restart ở từng bước persist, deactivate,
  purge và ACK; không được load/resume map tombstone ở mọi lịch thực thi.
- Stop command xuyên browser→Center→Edge→driver→encoder, gồm mất gói/restart/socket đầy.
- Safety-source death sau lần `clear`, stale TF/odom/scan, sequence reset và clock jump.
- Inject exception vào AMCL/particle/scan callback ở mọi execution phase; xác nhận cùng transition thu hồi owner và `/cmd_vel` bằng zero.
- Cancel/E-stop/manual takeover đúng tại barrier trước recovery commit; worker cũ không được đổi state hoặc phát motion.
- Set initial pose/force relocalize trong STRAIGHT, TURN, DISPATCHING; bắt buộc reject hoặc stop-and-confirm trước reset.
- AMCL/particle sai frame, stamp trước attempt, callback service đến muộn và TF source stamp đóng băng.
- Same request ID với payload/map/expected-state khác; Center crash ở mọi điểm trước/sau dispatch/ACK.
- Archive/delete khi robot đang navigate; resync khi DB còn nhưng archive mất/hỏng.
- Concurrent upload cùng version, upload-vs-delete, DB rollback và filesystem cleanup.
- NaN/Inf, origin sai, YAML image escape, metadata/image mismatch, duplicate tar member, decompression bomb.
- Mapping hai operator/lease conflict và busy navigation.
- HIL hành lang, kính, vật thấp, cliff, người cắt ngang, tải/pin/sàn xấu.

### 14.3 Trạng thái sửa mã đợt 2026-08-29

Đã triển khai nhóm sửa phần mềm ưu tiên đầu tiên trong `navigation-stack/adapter_node.py`:

- **NAV-01 đã sửa ở mức mã:** worker localization resume không còn gán lại `original_directions` ở scope cục bộ; lỗi `UnboundLocalError`/Ruff `F823` đã hết.
- **NAV-06 đã sửa ở các điểm cấp chuyển động đã nhận diện:** mọi `_navigate(... recovery_attempt=True)` đều truyền `expected_generation`; sensor-time resume cũng mang fence; commit di chuyển tới turn bay kiểm tra generation và cập nhật state dưới cùng một lock.
- Callback localization phát sinh exception nay đi qua một transition fail-closed chung: thu hồi action handle và motion owner, tăng goal generation/segment token, xóa active segment và mọi auto-resume context, phát zero lên cả navigation/localization velocity, cancel Nav2 và chỉ lưu mission ở chế độ không tự resume.
- Cancel, Pause, E-stop và manual takeover đều vô hiệu hóa cả sensor-time resume lẫn localization resume. E-stop/manual takeover cũng phủ các pha planning, dynamic wait/replan, turn-bay recovery và route selection, không chỉ pha `NAVIGATING`.
- **NAV-07 đã sửa ở mức state phần mềm:** `map.set_initial_pose` và `map.relocalize` cùng dùng một guard; command bị reject nếu còn action handle, motion owner, rotation hoặc execution phase/state hoạt động. Khi state đã quiesced, guard vẫn tăng generation/token, xóa recovery context và phát zero trước khi đổi localization.

Kiểm tra sau sửa:

| Hạng mục | Kết quả |
|---|---:|
| Python compile + `git diff --check` | **Passed** |
| Ruff `E9,F821,F823` trên adapter và contract test | **Passed** |
| Localization/navigation contract | **129 passed** |
| Toàn bộ `demo/robot-simulator/tests` | **519 passed, 1 failed** |

Failure duy nhất vẫn là test VAAPI cần `/dev/dri/renderD*` nhưng audit container không có thiết bị GPU; không nằm trên đường navigation. Các test mới là regression/contract ở mức source và state invariant, **chưa thay thế** concurrency test có barrier, ROS integration test đo topic vận tốc cuối, encoder measured-zero hoặc HIL trên robot thật.

Vì vậy các finding lịch sử ở trên vẫn được giữ để truy nguyên. Trạng thái sau nhóm sửa navigation nên hiểu là: lỗi `NAV-01` đã đóng; phần “worker cũ tái cấp chuyển động” của `NAV-06` đã đóng bằng generation fence; `NAV-07` đã đóng ở software-state guard nhưng yêu cầu **measured-zero vật lý** khi đổi pose vẫn mở. Tình trạng hình học/protective-source freshness được cập nhật ở mục 14.4; stop ACK/software latch được cập nhật tiếp ở mục 14.5. Hardware E-stop, startup tombstone và stopping envelope vẫn mở.

### 14.4 Trạng thái sửa hình học và safety freshness đợt 2026-08-29

- Nguồn chuẩn hình học: thân xe `0,30 × 0,20 × 0,15 m`; LiDAR ở tâm mặt trên, tọa độ so với `base_footprint` là `(0, 0, 0,15 m)`.
- Planner, local/global costmap và motion-safety giữ footprint phẳng `0,30 × 0,20 m`; URDF nay dùng đúng chiều cao 0,15 m.
- Sensor normalizer và motion-safety cùng dùng extrinsic phẳng LiDAR `(0,0,0)`; self-mask đổi từ `0,40 × 0,36 m` về đúng `0,30 × 0,20 m`, không còn chủ ý xóa điểm ngoài collision body.
- E-stop software phải có heartbeat mới trong 1,2 s. Mất heartbeat chuyển thành `estop_timeout`, phát zero và giữ fault hysteresis.
- Cliff, bumper, range và external compatibility inputs có deadline riêng và cờ `heartbeat_required`; hiện khai báo `false` vì người dùng chưa xác nhận có các phần cứng này. Khi lắp sensor, bắt buộc đổi cờ tương ứng thành `true`.
- External obstacle stream ở bridge có production watchdog mặc định 500 ms thay vì 0. Nếu hệ thống không phát định kỳ cả trạng thái clear/block, robot sẽ chủ ý giữ khóa zero; chỉ cho phép watchdog 0 trong bench/service mode đã cô lập motion output.

Kết quả sau đợt này: motion-safety core **28 passed**; test hình học/cấu hình tập trung **4 passed**; toàn bộ simulator **520 passed, 1 failed**. Failure duy nhất vẫn là VAAPI do audit container không có `/dev/dri/renderD*`. Ruff `E9,F821,F823`, Python compile và `git diff --check` đều passed.

### 14.5 Trạng thái sửa stop ACK và software E-stop đợt 2026-08-29

- Motion-safety đưa vận tốc odometry đo được, tuổi dữ liệu và cờ freshness vào atomic safety snapshot; navigation adapter xuất các trường này trong `system.status`.
- Edge phát zero trước, sau đó yêu cầu vận tốc tuyến tính và góc nằm trong ngưỡng liên tục 250 ms. Không có odometry mới, vận tốc còn lớn hoặc hết timeout 3 giây đều trả `unknown/MEASURED_ZERO_UNCONFIRMED`.
- Center không còn ACK `accepted` ngay khi chỉ mới ghi lên robot WebSocket đối với `control.stop` và `control.estop.reset`; nó correlate theo request ID và chuyển ACK cuối cùng của Edge về đúng control socket.
- Frontend không còn timer tự chuyển sang “Sẵn sàng”. Trong khi chờ xác nhận, cả nút hướng lẫn keyboard velocity bị chặn; chỉ hiển thị “đã xác nhận robot đứng yên” khi nhận `completed`.
- ROS bridge không còn tự nhả latch khi có velocity mới. Reset dùng datagram `estop_reset` riêng, vẫn ép zero, và Edge chỉ ACK hoàn tất khi vừa thấy measured-zero vừa thấy trạng thái E-stop đã nhả.
- Ngưỡng mặc định có thể cấu hình qua `STOP_CONFIRMATION_TIMEOUT_SECONDS`, `STOP_ZERO_DWELL_SECONDS`, `STOP_LINEAR_THRESHOLD`, `STOP_ANGULAR_THRESHOLD`; đây là ngưỡng xác nhận trạng thái chứ không phải stopping-envelope đã được chứng minh.

Giới hạn còn lại: odometry có thể báo zero trong khi cơ cấu truyền động/encoder lỗi hoặc bánh bị nhấc; chưa có ACK trực tiếp từ motor controller, boot epoch/durable result replay, E-stop phần cứng độc lập hoặc HIL đo khoảng dừng. Vì vậy kết quả này đóng lỗi UI báo SAFE khi mới dispatch và lỗi velocity tự nhả software latch, nhưng **không tạo safety certification**.

Kiểm tra sau sửa: frontend **143 passed/23 files** và production build passed; backend **69 passed**; simulator **524 passed, 1 failed**; motion-safety **28 passed**. Failure duy nhất vẫn là test VAAPI cần `/dev/dri/renderD*` không có trong container audit. Python compile, Compose config, `git diff --check` và Ruff chọn lọc `E9,F63,F7,F82` đều passed.

### 14.6 Trạng thái sửa race map đã xóa/restore đợt 2026-08-29

- **Reconnect fail-closed:** mỗi kết nối Center mới xóa cờ
  `map_registry.ready`, phát stop và hủy navigation còn hoạt động. Restore active
  map không còn chạy song song tự do với tombstone sync.
- **Barrier cấp quyền:** restore, upload retry và các lệnh `map`, `mapping`,
  `navigation` có thể nạp map hoặc cấp motion authority bị khóa cho tới khi một
  snapshot tombstone hoàn chỉnh được reconcile. Các lệnh đưa robot về an toàn
  như stop/cancel/pause/deactivate/manual handoff vẫn được phép.
- **Thứ tự crash-safe:** Edge ghi tombstone vào `registry.json`, xóa active ref
  và last pose, atomic rename rồi `fsync` cả file/directory **trước** khi yêu cầu
  runtime deactivate và trước khi xóa artifact. Nếu mất điện giữa chừng, lần
  boot sau vẫn không tạo được payload restore cho map đó.
- **Tombstone thắng download:** cache kiểm tombstone cả trước khi tải và ngay
  trước atomic install; deletion đến giữa download/verify không thể dựng lại
  thư mục version.
- **Snapshot có thẩm quyền:** Center tiếp tục trả mọi tombstone kể cả đã ACK.
  Đây là chủ ý để robot mất local registry vẫn học lại toàn bộ deletion, thay vì
  xem ACK cũ là bằng chứng robot hiện tại còn giữ state.
- **Preflight xuyên tầng:** Edge phát `health.map_registry` gồm `ready`,
  `syncStatus`, `lastSyncAt`, `error`; Center từ chối navigation/start mapping
  trên robot thật khi barrier chưa mở.

Kiểm tra sau sửa: backend **72 passed**; simulator **530 passed, 1 failed**.
Test mới xác nhận lần sync lỗi đầu giữ cửa đóng và chỉ lần snapshot kế tiếp thành
công mới mở cửa. Failure duy nhất là test
VAAPI cần `/dev/dri/renderD*` không có trong môi trường audit, không nằm trên
đường map registry. Frontend production build passed. Python compile,
`git diff --check` và Ruff chọn lọc `E9,F63,F7,F82` passed.

Giới hạn còn lại của chính phần này:

1. Chưa chaos-test bằng cách kill process/cắt nguồn tại từng ranh giới
   persist→deactivate→purge→ACK và chưa HIL với ROS/Nav2 thật.
2. Snapshot toàn lịch sử ưu tiên an toàn nhưng chi phí tăng theo số tombstone ×
   robot; bước scale tiếp theo là generation/cursor + compaction có watermark
   bền vững, không phải lọc đơn giản theo ACK.
3. Chưa có transaction phân tán tuyệt đối giữa DB, bundle storage và runtime;
   các race archive/resync/upload khác trong `MAP-04` vẫn còn P1.

### 14.7 Trạng thái sửa hành lang thiếu dữ liệu một phía đợt 2026-08-29

- Một phía quan sát được dưới hard margin nay trả
  `PHYSICALLY_BLOCKED/OBSERVED_SIDE_HARD_MARGIN`, kể cả phía đối diện không có
  return hoặc hai phía không ghép được cùng longitudinal bin.
- Một phía đạt hard margin, phía còn lại missing vẫn là
  `NARROW_OR_UNCERTAIN/LIVE_SIDE_INCOMPLETE`; không gây dừng giả vô điều kiện.
- Velocity gate dừng ngay ở mẫu physical-block đầu tiên. Sau confirmation,
  action cũ bị cancel/token-fence và chuyển sang reverse-first turn-bay recovery.
- Relocation luôn dùng profile `SLOW`, giữ heading, có giới hạn quãng đường và
  chỉ chạy sau map/safety/swept-footprint validation. Hoàn tất relocation sẽ
  replan tới goal cũ; không tìm được đường thoát thì mission được giữ trong
  bounded wait để retry/alternative-route thay vì bị xóa.
- Không sửa hành vi toggle chống vật cản thủ công trên UI.

Kiểm tra sau sửa: geometry + localization/state contract **336 passed**; toàn bộ
simulator **537 passed, 1 failed**. Failure duy nhất vẫn là VAAPI do môi trường
audit không có `/dev/dri/renderD*`; không liên quan navigation. Python compile,
Ruff chọn lọc `E9,F63,F7,F82` và `git diff --check` passed.

Giới hạn còn lại: chưa có HIL đo clearance thật khi lùi cạnh tường, ma sát/lệch
bánh, vật kính, người đi vào vùng rear và chuyển tiếp từ lùi sang quay. Trước
khi chạy gần người cần replay rosbag và test robot thật với ground truth độc lập.

### 14.8 Trạng thái sửa autosave recovery và nút Đồng bộ map đợt 2026-08-30

- **Autosave atomic theo generation:** mỗi snapshot có occupancy map,
  posegraph/data, terminal pose, map/version/generation và SHA-256 từng
  artifact. Ghi vào staging, validate, `fsync`, atomic rename generation rồi
  mới atomic replace `latest.json`; mất điện trước bước cuối không làm mất
  generation cũ đã commit.
- **Single-flight SLAM:** save/serialize/pause/resume của autosave và command
  operator dùng chung `mapping_operation_lock`. Autosave đang chạy không chồng
  Save/Finish/Pause/Resume và timer mới sẽ bỏ lượt thay vì tạo thêm writer.
- **Không snapshot khi xe đang chạy:** autosave chỉ bắt đầu khi safety snapshot
  còn fresh, cả vận tốc đo được lẫn vận tốc command tuyến tính/góc nằm trong
  ngưỡng zero. Khi cần đóng băng SLAM, toggle pause/resume nằm trong cùng lock;
  resume lỗi chuyển mapping sang `MAPPING_ERROR` thay vì giả vờ tiếp tục.
- **Recovery fail-closed:** Edge không còn chấp nhận chỉ hai file non-empty.
  Nó kiểm pointer, manifest identity, tên file, symlink, non-empty và checksum
  của đủ map/posegraph; terminal pose hữu hạn được truyền vào scan-to-saved-map
  search và SLAM refinement. Corrupt/half-written/wrong-version bị reject.
- **Nút Đồng bộ map:** UI không còn báo “Đã đồng bộ” khi backend mới ACK queue.
  `SYNC_PENDING` hiển thị “Chờ đồng bộ”, khóa duplicate click và poll detail mỗi
  2 giây. Marker retry trên Edge cũng được ghi temp + `fsync` + atomic replace,
  nên ACK pending luôn đi kèm công việc bền qua restart. Robot thiếu local
  bundle trả lỗi; Center ghi `SYNC_FAILED`. Khi robot upload lại, Center luôn
  verify và atomic replace bundle kể cả row version đã tồn tại, vì vậy file
  Center bị mất/hỏng thực sự được phục hồi; chỉ checksum trùng ACK mới đặt
  `SYNCED`.

Kiểm tra sau sửa: backend **74 passed**; frontend **144 passed/23 files** và
production build passed; simulator **538 passed, 1 failed**. Failure duy nhất
vẫn là VAAPI do container không có `/dev/dri/renderD*`, không liên quan map.
Test tập trung bổ sung các case generation chưa promote, checksum corruption,
Edge recovery mang pose/generation, Edge resync có/thiếu local bundle, Center
repair storage đã mất, ACK/reject sync và UI pending state. Python compile và
`git diff --check` passed.

Giới hạn còn lại: chưa cắt nguồn thật giữa từng syscall trên filesystem của Pi,
chưa chạy service SLAM Toolbox thật để đo thời gian pause/save/resume và chưa
test upload đồng thời với archive/delete. Vì vậy `MAP-02` đóng ở mức logic phần
mềm nhưng vẫn cần ROS integration/chaos; phần transaction phân tán còn lại của
`MAP-04` vẫn là P1.

### 14.9 Trạng thái sửa toàn vẹn semantic map bundle đợt 2026-08-30

- **Schema xuyên artifact:** Center và Edge cùng kiểm identity, frame, timestamp,
  resolution, origin, dimensions, tên/slam mode, terminal pose và mọi số phải
  hữu hạn. `map.yaml` chỉ được trỏ tới `map.pgm`/`map.png` cùng bundle; YAML và
  metadata phải đồng nhất về resolution/origin, còn ảnh decode thật phải đúng
  width/height đã khai báo.
- **Manifest khép kín:** `metadata.files` phải đúng bằng tập artifact thực tế và
  SHA-256 từng file phải khớp. Occupancy checksum phải thuộc đúng ảnh YAML chọn;
  cờ `has_posegraph` phải khớp sự hiện diện đầy đủ của posegraph/data.
- **Archive fail-closed:** chỉ nhận file thường phẳng; từ chối path tuyệt đối,
  `..`, nested path, symlink/hardlink/special member và canonical duplicate như
  `map.yaml` với `./map.yaml`. Có cap archive nén, tổng giải nén, mỗi member, số
  member, tỷ lệ nén, metadata/YAML và tổng pixel ảnh.
- **Cache không tin marker:** Edge giữ `.map-bundle.tar.gz` đã đạt whole-archive
  SHA-256. Mỗi lần dùng fast path hoặc restore active map đều re-hash archive,
  kiểm lại cấu trúc/giới hạn, bind byte metadata trong archive với file đã giải
  nén, sau đó re-hash/decode toàn bộ artifact. Cache cũ thiếu bằng chứng hoặc
  cache bị sửa bị từ chối và đường `map.load` sẽ tải lại từ Center.
- **Atomic install:** download và extract vẫn ở staging; chỉ sau khi kiểm semantic
  hoàn tất mới atomic rename vào cache version. Tombstone vẫn được kiểm lần hai
  ngay trước install nên thay đổi này không làm yếu bản sửa `MAP-01`.
- **Dependency runtime:** Pillow và PyYAML đã được pin trong cả image Center lẫn
  image Edge, không chỉ môi trường test.

Kiểm thử bổ sung bao gồm YAML image escape, dimension mismatch, JSON NaN,
canonical duplicate, giới hạn giải nén/tỷ lệ nén, artifact cache bị sửa và tình
huống kẻ tấn công sửa đồng thời ảnh + metadata nhưng không thể sửa archive đã
verify. Backend **79 passed**; simulator **542 passed, 1 failed**. Failure duy
nhất là test VAAPI yêu cầu `/dev/dri/renderD*` không có trong môi trường audit,
không liên quan map. Ruff cho hai validator và Python compile đều passed.

Giới hạn còn lại: chưa fuzz parser tar/YAML/image dài hạn, chưa benchmark giới
hạn trên Pi khi gần đầy disk và chưa chaos-test cắt nguồn tại từng lần fsync/
rename. Các ngưỡng mặc định là guard tài nguyên, không thay thế quota volume,
monitor disk và hardening container.

## 15. Kế hoạch xử lý ưu tiên

### P0 — trước khi cho robot chạy tự hành gần người

1. **Đã sửa phần mềm; còn HIL/phần cứng:** stop ACK chỉ hoàn tất khi có measured-zero mới và software E-stop có reset riêng; vẫn phải lắp/kiểm E-stop phần cứng độc lập và xác nhận bằng encoder/motor controller.
2. **Đã sửa cấu hình; còn HIL:** thống nhất URDF–planner–costmap–motion safety–LiDAR self-mask theo thân xe 0,30 × 0,20 × 0,15 m.
3. **Đã làm một phần:** thêm deadline/fail-safe và watchdog production; còn bật bắt buộc cho sensor phần cứng khi lắp và xử lý odometry stale.
4. Khóa bypass vào service mode vật lý/role riêng/hold-to-run/speed cap/audit.
5. **Đã sửa phần mềm; còn chaos/HIL:** barrier tombstone/registry trước restore,
   upload retry và lệnh cấp chuyển động; tombstone được persist trước deactivate/purge.
6. Đo stopping envelope và sensor coverage trên robot thật ở worst-case; không dùng thông số giả định làm safety claim.
7. Fatal localization transition phải revoke/cancel/zero theo transaction; generation fence bắt buộc ở mọi recovery commit.
8. Cấm initial-pose/relocalize khi chưa xác nhận motion quiesced; thống nhất chính sách auto-resume sau mất localization.

### P1 — độ tin cậy chức năng

1. Sửa `UnboundLocalError` localization resume và cleanup state bằng invariant/regression test.
2. **Đã sửa phần mềm; còn ROS/chaos:** autosave recovery dùng bundle atomic có generation/hash/pose/map snapshot.
3. Durable outbox/inbox/result reconciliation; idempotency bind payload + epoch + state.
4. State machine thống nhất cho activate/load/archive/delete/resync; transaction giữa DB và object storage.
5. **Đã sửa phần mềm; còn fuzz/chaos:** validate semantic map bundle, path containment, decompression limits và cache re-verification.
6. Bind mapping/navigation mission vào control lease/session ownership.
7. Làm UI mapping/stop theo authoritative event stream có sequence/snapshot/reconnect.
8. Đánh giá bật collision checking độc lập tại controller hoặc chứng minh fault containment tương đương.
9. **Đã sửa phần one-sided corridor; còn HIL:** bind path endpoint/hash với goal/mission trước cấp motion authority.
10. Fence localization callback theo attempt/map generation và kiểm source header stamp/frame/TF age.

### P2 — hiện đại hóa và mở rộng

1. Tách monolith thành typed state machines và plugin boundaries; xóa/deprecate luồng legacy/dead BT config.
2. Benchmark smoother/Smac/MPPI trên cùng scenario thay vì thay thuật toán theo cảm tính.
3. Thêm perception phù hợp hazard: cliff/depth/3D/glass strategy và sensor redundancy.
4. Nếu có nhiều robot, xây traffic schedule/reservation, corridor mutex/queue/deadlock, task allocator và charging/resource manager; cân nhắc tích hợp Open-RMF thay vì tự viết toàn bộ.
5. Thêm flight recorder, structured event correlation, metrics/SLO, readiness, SBOM, signed artifacts và chaos/HIL pipeline.
6. Biến KeepoutZone/SpeedZone từ schema placeholder thành artifact versioned được planner/safety thực thi và test end-to-end.

## 16. Ma trận nghiệm thu tối thiểu đề xuất

| Nhóm | Tình huống | Tiêu chí tối thiểu |
|---|---|---|
| Stop | Mất Center, mất Edge, socket local đầy, driver treo, encoder còn quay | UI không báo SAFE nếu thiếu measured-zero; hardware stop vẫn tác dụng; có fault/audit. |
| Sensor freshness | Rút từng nguồn scan/range/bumper/cliff/external/odom khi đang chạy | Chuyển UNKNOWN trong deadline và controlled stop theo safety case. |
| Self-mask | Đặt vật nhỏ quanh từng cạnh/góc từ sát thân ra ngoài 30 cm | Không có vùng mù ngoài collision footprint; không false-stop do chính robot. |
| Braking | Tải max/min, pin thấp, sàn trơn/dốc, nhiệt độ, tốc độ max | Khoảng dừng p99.9 + uncertainty luôn nằm trong protective field. |
| Hành lang | Nhiều chiều rộng/cua/cửa, một vách kính, side return mất | Không chạm; không tự đi nếu confidence/clearance thiếu; ground truth độc lập. |
| Vật cản người | Người cắt ngang, dừng đột ngột, đi ra từ occlusion | Không vi phạm protective envelope; tránh oscillation nguy hiểm; stop có kiểm soát. |
| Localization | Kidnap, map lặp, scan mù, wheel slip, TF/clock jump | Không nhận pose sai; dừng; recovery hữu hạn hoặc yêu cầu operator. |
| Map recovery | Mất điện tại từng bước autosave/upload/install | Chỉ nhận bundle cùng generation/hash; không load half-written map. |
| Delete/reboot | Xóa map khi robot offline rồi reconnect nhiều lịch khác nhau | Không bao giờ load/resume map đã tombstone; ACK idempotent, không ghi flash lặp. |
| Distributed command | Center/Edge restart trước/sau dispatch và ACK đến muộn | Exactly-once effect hoặc at-least-once + dedupe đúng; state cuối reconcile được. |
| Bundle security | Tar bomb, duplicate name, path escape, YAML image escape, NaN | Reject sớm trong bounded CPU/RAM/disk; không để orphan. |
| Multi-robot | Hai robot đối đầu hành lang/giao lộ, một robot chết giữa đường | Không deadlock/collision; reservation thu hồi có kiểm soát và có retreat plan. |

Mỗi test cần lưu rosbag, command/event timeline, firmware/container digest, map hash, calibration version, tải trọng, điều kiện sàn và ground-truth measurement để tái lập.

## 17. Kết luận cuối

Logic tự hành của project **có nhiều thành phần đáng giữ**: exact rectangular footprint, swept-turn validation, hành lang hard/comfort margin, dynamic obstacle TTL/TTC, localization evidence gate, map immutable/checksum và phân lớp motion safety. Với một robot differential-drive 2D chạy có giám sát, đây là nền tảng kỹ thuật tốt.

Điểm yếu lớn nhất còn lại là tính an toàn và nhất quán **xuyên tầng**: stop ACK và software latch đã được làm fail-closed hơn nhưng chưa có xác nhận motor/HIL hay E-stop phần cứng; protective sources chưa hoàn chỉnh theo phần cứng thực; hình học chưa được chứng minh ngoài hiện trường; bypass chưa đủ chuẩn; command durability và các race archive/resync/upload vẫn còn. Race riêng “map đã xóa được restore trước tombstone” và lỗi semantic/cache trust của map bundle đã đóng ở mức phần mềm, nhưng chưa có chaos/HIL/fuzz làm bằng chứng. Ở cấp sản phẩm hiện đại, hệ thống còn thiếu fleet coordination, durable orchestration, sensor redundancy và bộ evidence/verification ngoài hiện trường.

Do đó, hướng đúng không phải viết lại toàn bộ planner. Nên khóa các P0 trước, giữ và benchmark phần hình học/localization đang tốt, làm state/command/map lifecycle bền vững, rồi mới tối ưu độ mượt và mở rộng fleet. Chỉ sau khi vượt ma trận HIL/robot thật với số liệu worst-case mới có cơ sở nâng trạng thái từ “R&D có giám sát” sang “pilot vận hành”, và việc đáp ứng tiêu chuẩn an toàn phải được đánh giá độc lập theo use case/operating zone cụ thể.
