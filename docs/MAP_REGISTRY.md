# Map Registry và đồng bộ Pi ↔ Center

## Identity và artifact

Map mới dùng UUID làm `map_id`; identity active luôn là `(map_id, version)`, không phải filename. Version là immutable và Continue Mapping tạo số mới.

Bundle tối thiểu có `map.yaml`, ảnh PGM/PNG và `metadata.json`; triển khai hiện tại còn có preview cùng `posegraph.posegraph`/`posegraph.data`. Metadata bắt buộc khai báo map/version, robot, timestamps, resolution, dimensions, origin, frame `map`, `has_posegraph`, `slam_mode`, checksum ảnh occupancy chính và SHA-256 từng artifact trong `files`.

Center giữ catalog/version/tombstone và bundle chuẩn. Pi giữ cache local đã kiểm tra để vẫn navigation khi mất Center. `registry.json` trên Pi được ghi qua file tạm + `fsync` + atomic replace; bundle được download vào staging, verify SHA-256, validate tar và nội dung rồi atomic rename.

## Hợp đồng toàn vẹn bundle

Center và Edge cùng áp dụng một hợp đồng fail-closed trước khi lưu hoặc nạp:

- archive chỉ được chứa file thường ở cấp gốc; từ chối path tuyệt đối, `..`,
  thư mục con, symlink/hardlink/special member và tên canonical trùng nhau;
- `metadata.json` phải là JSON hữu hạn, đúng `map_id/version`, frame `map`,
  dimensions/resolution/origin hợp lệ và khai báo chính xác toàn bộ artifact;
- `map.yaml.image` chỉ được là basename `map.pgm` hoặc `map.png` có trong
  bundle; resolution/origin phải khớp metadata, mode/negate/threshold hợp lệ;
- ảnh occupancy phải decode hoàn chỉnh, kích thước pixel phải khớp metadata;
  SHA-256 của từng artifact và checksum ảnh occupancy chính đều phải khớp;
- `has_posegraph` phải nhất quán với cặp `posegraph.posegraph` và
  `posegraph.data`; terminal pose nếu có phải là ba số hữu hạn.

Giới hạn mặc định là 512 MiB archive nén, 1 GiB tổng giải nén, 768 MiB mỗi
member, 64 member, tỷ lệ nén 2000:1 và 100 triệu pixel. Có thể hạ các ngưỡng
`MAP_BUNDLE_MAX_*` theo dung lượng Pi/site, nhưng không tăng nếu chưa đo ngân
sách RAM/disk/CPU. Metadata và YAML còn có cap nhỏ độc lập để tránh parse input
quá lớn.

Edge giữ lại bản archive đã được Center ký danh tính bằng whole-bundle SHA-256
trong cache. Mỗi fast path và mỗi lần khôi phục active map đều kiểm lại hash
archive, cấu trúc tar, byte `metadata.json` giữa archive và cache, rồi hash/decode
toàn bộ artifact. Marker `.sha256` một mình không còn đủ để cấp quyền load. Cache
cũ thiếu archive tin cậy hoặc cache bị sửa sẽ bị từ chối; `map.load` hoặc luồng
khôi phục active map sau restart sẽ tải lại bundle từ Center bằng checksum đã
lưu, cài atomic rồi mới cấp quyền load cho Nav2.

## State đồng bộ

```text
LOCAL_ONLY -> SYNC_PENDING -> SYNCED
                         \-> SYNC_FAILED
DELETION_PENDING -> DELETED
```

Save thành công được định nghĩa là artifact local hợp lệ. Upload là background
task có marker `.upload-pending.json` ghi bằng temp + `fsync` + atomic replace;
retry sau reconnect và chỉ mark `SYNCED` khi checksum Center trả về trùng
checksum local.

Center expose:

- `GET /api/maps`: registry chưa xóa.
- `GET /api/maps/{map_id}`: metadata và toàn bộ version.
- `POST /api/maps/{map_id}/versions`: upload immutable + checksum.
- `POST /api/maps/{map_id}/versions/{version}/resync`: yêu cầu robot tạo lại
  upload từ bundle local. ACK command chỉ đặt `SYNC_PENDING`; `SYNCED` chỉ xuất
  hiện sau khi Center nhận và kiểm đúng checksum.
- `POST /api/maps/{map_id}/activate`: promote version đã validate trong registry.
- `GET /api/maps/tombstones`: snapshot có thẩm quyền gồm toàn bộ tombstone. ACK
  được dùng để theo dõi cleanup, không làm tombstone biến mất khỏi snapshot;
  nhờ đó robot mất `registry.json` cũng không thể tải lại map đã xóa.
- `POST /api/maps/tombstones/{map_id}/ack`: xác nhận local deletion.
- `GET /api/maps/registry/health`: số cache, pending sync/delete.

## Xóa active/offline

Sau confirm xóa active map, Center cancel mission/mapping, dispatch `map.deactivate`, robot cancel Nav2, gửi zero qua motion-safety, chuyển supervisor sang `IDLE` để dừng localization/map_server/Nav2, xóa active ref và local artifacts. Center soft-delete record/version, xóa bundle và broadcast registry change. Robot offline nhận tombstone khi reconnect; tombstone được lưu ngay nhưng chỉ ACK Center sau khi runtime xác nhận `IDLE`.

Mỗi lần mở lại WebSocket, Edge đóng hàng rào `map_registry.ready`, phát stop và
hủy navigation đang còn hoạt động trước khi lấy snapshot tombstone. Tombstone
được ghi bền vững vào registry **trước** khi deactivate runtime và xóa bundle;
file registry lẫn directory chứa phép atomic rename đều được `fsync`. Chỉ khi
toàn bộ snapshot đã được áp dụng, runtime đã xác nhận map liên quan ở trạng thái
`IDLE`/`NO_ACTIVE_MAP` và ACK thành công thì hàng rào mới mở. Trong thời gian đó:

- không restore active map hoặc mission cũ;
- không retry upload bundle;
- từ chối các lệnh có thể cấp quyền chuyển động hoặc tạo/nạp map;
- vẫn cho phép stop, cancel, pause, manual handoff và deactivate để hệ thống có
  thể đi về trạng thái an toàn.

Edge công bố `health.map_registry.ready`, `syncStatus`, `lastSyncAt` và `error`.
Center cũng đưa `map_registry.ready` vào preflight của navigation và start
mapping đối với robot thật. Nếu Center hoặc bước reconcile lỗi ở lần kết nối
đầu, hệ thống giữ hàng rào đóng và retry có backoff; không fail-open sang cache
cũ.

Tombstone được kiểm tra lúc bắt đầu tải và kiểm tra lại ngay trước atomic install,
nên deletion đến trong lúc download/validate vẫn thắng. Center cũng từ chối
upload vào map đã xóa. Vì vậy marker upload cũ không thể làm map sống lại. Không
có symlink/hardlink trong bundle được chấp nhận và map id dành riêng (`created`,
`registry`, `staging`, `autosave`) không thể là deletion target.

## Khôi phục lỗi

- Checksum download sai: bỏ staging, giữ active directory cũ.
- Mất điện khi download/extract: thư mục active không đổi; staging được dọn ở lần xử lý.
- Center offline khi save: giữ local + marker `SYNC_PENDING`.
- Version đã tồn tại cùng checksum: Center vẫn verify và atomic replace bundle
  để sửa storage bị mất/hỏng, sau đó mới `SYNCED`; checksum khác: `409
  CONFLICT`, không ghi đè.
- Robot không còn bundle local khi resync: version chuyển `SYNC_FAILED`; UI
  không được diễn giải ACK queue là đã đồng bộ.
- Delete offline: tombstone thắng mọi cache/upload cũ.

Snapshot hiện phát lại toàn bộ lịch sử tombstone để ưu tiên tính an toàn. Khi số
map/robot lớn, nên bổ sung deletion generation/cursor và compaction có watermark
bền vững; không được tối ưu bằng cách chỉ ẩn các bản ghi đã ACK nếu robot có thể
mất registry local.

Không sửa tay `registry.json` hoặc đổi tên directory để tạo identity. Dùng API/Control để tránh active reference không nhất quán.
