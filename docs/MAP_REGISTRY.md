# Map Registry và đồng bộ Pi ↔ Center

## Identity và artifact

Map mới dùng UUID làm `map_id`; identity active luôn là `(map_id, version)`, không phải filename. Version là immutable và Continue Mapping tạo số mới.

Bundle tối thiểu có `map.yaml`, ảnh PGM/PNG và `metadata.json`; triển khai hiện tại còn có preview cùng `posegraph.posegraph`/`posegraph.data`. Metadata bắt buộc khai báo map/version, robot, timestamps, resolution, dimensions, origin, frame `map`, `has_posegraph`, `slam_mode`, checksum ảnh occupancy chính và SHA-256 từng artifact trong `files`.

Center giữ catalog/version/tombstone và bundle chuẩn. Pi giữ cache local đã kiểm tra để vẫn navigation khi mất Center. `registry.json` trên Pi được ghi qua file tạm + `fsync` + atomic replace; bundle được download vào staging, verify SHA-256, validate tar path/type rồi atomic rename.

## State đồng bộ

```text
LOCAL_ONLY -> SYNC_PENDING -> SYNCED
                         \-> CONFLICT
DELETION_PENDING -> DELETED
```

Save thành công được định nghĩa là artifact local hợp lệ. Upload là background task có marker `.upload-pending.json`; retry sau reconnect và chỉ mark `SYNCED` khi checksum Center trả về trùng checksum local.

Center expose:

- `GET /api/maps`: registry chưa xóa.
- `GET /api/maps/{map_id}`: metadata và toàn bộ version.
- `POST /api/maps/{map_id}/versions`: upload immutable + checksum.
- `POST /api/maps/{map_id}/activate`: promote version đã validate trong registry.
- `GET /api/maps/tombstones`: deletion chưa được robot xác nhận.
- `POST /api/maps/tombstones/{map_id}/ack`: xác nhận local deletion.
- `GET /api/maps/registry/health`: số cache, pending sync/delete.

## Xóa active/offline

Sau confirm xóa active map, Center cancel mission/mapping, dispatch `map.deactivate`, robot cancel Nav2, gửi zero qua motion-safety, chuyển supervisor sang `IDLE` để dừng localization/map_server/Nav2, xóa active ref và local artifacts. Center soft-delete record/version, xóa bundle và broadcast registry change. Robot offline nhận tombstone khi reconnect; tombstone được lưu ngay nhưng chỉ ACK Center sau khi runtime xác nhận `IDLE`.

Tombstone được kiểm tra trước khi ghi local registry; Center cũng từ chối upload vào map đã xóa. Vì vậy marker upload cũ không thể làm map sống lại. Không có symlink/hardlink trong bundle được chấp nhận và map id dành riêng (`created`, `registry`, `staging`, `autosave`) không thể là deletion target.

## Khôi phục lỗi

- Checksum download sai: bỏ staging, giữ active directory cũ.
- Mất điện khi download/extract: thư mục active không đổi; staging được dọn ở lần xử lý.
- Center offline khi save: giữ local + marker `SYNC_PENDING`.
- Version đã tồn tại cùng checksum: idempotent; checksum khác: `409 CONFLICT`, không ghi đè.
- Delete offline: tombstone thắng mọi cache/upload cũ.

Không sửa tay `registry.json` hoặc đổi tên directory để tạo identity. Dùng API/Control để tránh active reference không nhất quán.
