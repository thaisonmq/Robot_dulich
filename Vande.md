Bạn đang làm việc trên source code hiện tại của dự án **Robot du lịch**.

Nhiệm vụ lần này là sửa hoàn chỉnh các vấn đề Localization / Motion Safety / Auto Navigation hiện tại, đồng thời bổ sung cơ chế xử lý **đường hẹp + chuyển sang Manual + tiếp tục Auto + lựa chọn nhiều tuyến đường giống Google Maps**.

## THÔNG SỐ VẬT LÝ CHÍNH XÁC CỦA ROBOT

Đây là source of truth mới:

```text
Robot length = 0.30 m
Robot width  = 0.20 m

Half length = 0.15 m
Half width  = 0.10 m
```

Tất cả các phần liên quan phải dùng thống nhất kích thước này:

```text
Nav2 footprint
Global Costmap
Local Costmap
Collision checking
Motion Safety
Corridor clearance
Rotation clearance
Path validation
Recovery
```

Không được tiếp tục dùng các kích thước cũ như:

```text
0.40 × 0.36 m
```

nếu không phải một thành phần vật lý khác thực sự cần.

Không được cố tình khai báo robot nhỏ hơn kích thước thật để planner dễ đi.


CÁC THÔNG SỐ NHƯ KÍCH THƯỚC XE HAY CÁC BIẾN CẤU HÌNH CÓ THỂ THAY ĐỔI PHẢI ĐỂ TRONG FILE CONFIG HAY YAML VÀ TẤT CẢ ĐỌC TỪ ĐÓ, KHI NÀO THAY ĐỔI THIẾT BỊ THÌ SẼ SỬA TRONG ĐÓ 

---

# 1. NGUYÊN TẮC CAO NHẤT

Không được tiếp tục sửa theo kiểu:

```text
fix A
→ làm hỏng B
→ thêm workaround cho B
→ làm hỏng C
```

Mỗi thay đổi phải đồng thời thỏa:

```text
Correct physical geometry
+
Localization usable
+
Manual predictable
+
Auto navigation stable
+
Safety preserved
+
No unnecessary stop
+
Recoverable behavior
```

Behavior thực tế trong prompt này là **source of truth**.

Không lấy bất kỳ phiên bản code cũ/mới nào làm chuẩn.

---

# 2. KHÔNG SỬA PHẦN KHÔNG LIÊN QUAN

Chỉ sửa khi cần trong:

```text
navigation-stack
motion-safety
localization / AMCL
TF synchronization
LaserScan processing
global/local costmap
planner/controller
navigation recovery
navigation API/state
frontend map/navigation controls
navigation debug logging
tests liên quan
```

Không sửa:

```text
camera
WebRTC
authentication
database
user management
streaming
unrelated frontend/backend
dependency versions
unrelated ROS nodes
```

Không refactor toàn repo.

---

# 3. CÁC LỖI HIỆN TẠI PHẢI SỬA

Hiện robot thật có các lỗi:

### Localization

```text
Quét vị trí thường không thành công
hoặc khó READY
```

### Manual

```text
Manual bị Motion Safety giảm tốc/dừng từ quá xa
```

và:

```text
ở khu vực hơi hẹp cũng dễ bị chặn
```

### Auto

Đường rộng hoạt động được nhưng phải đảm bảo:

```text
đường rộng
→ đi chuẩn
→ không zig-zag
→ không stop vô lý
→ không replan vô lý
```

### Narrow corridor

Khi gặp đoạn hẹp:

```text
phải tính xem robot thực sự có đi lọt hay không
```

Không chỉ nhìn:

```text
nearest obstacle
```

hoặc:

```text
LiDAR có wall trái/phải
```

mà kết luận BLOCKED.

---

# 4. KHÔNG COI `/rovera/obstacle_stop` LÀ ROOT CAUSE

Trong lần test thực tế gây lỗi:

```text
/rovera/obstacle_stop
/rovera/obstacle_directions
```

không có publisher đang chạy.

Giữ compatibility nếu cần nhưng:

**không xây fix dựa trên giả định legacy obstacle publisher đang chặn robot.**

Trace pipeline thực sự active:

```text
LaserScan
→ motion-safety
→ final cmd_vel
```

---

# 5. TRACE FINAL COMMAND TRƯỚC KHI SỬA

Phải xác định:

```text
Manual input
     ↓
twist_mux
     ↓
velocity smoother
     ↓
motion safety
     ↓
final cmd_vel
```

Log/debug được:

```text
input_v
input_w

after_mux_v
after_mux_w

after_smoother_v
after_smoother_w

safety_output_v
safety_output_w
```

Khi robot không đi:

```text
phải biết node nào đã thay đổi command và tại sao.
```

Không phỏng đoán.

---

# 6. SỬA STOPPING DISTANCE QUÁ BẢO THỦ

Review công thức hiện tại kiểu:

```python
stopping_distance =
    velocity * latency
    + velocity² / (2 * braking_acceleration)
    + clearance
```

Các config trước đang khiến Manual FAST bắt đầu slowdown/hard-stop từ quá xa.

Phải sử dụng deceleration thực tế của chassis/velocity pipeline.

Không để:

```text
velocity smoother:
deceleration A

motion safety:
assume braking acceleration B rất khác A
```

mà không có lý do.

Nên có một source of truth hoặc ít nhất kiểm tra consistency.

---

# 7. STOP DISTANCE PHẢI TÍNH TỪ MÉP ROBOT

LiDAR nằm gần giữa robot.

Robot dài:

```text
0.30m
```

nên khoảng từ tâm/LiDAR tới đầu robot khoảng:

```text
0.15m
```

Phải phân biệt:

```text
LiDAR range
```

với:

```text
clearance từ front bumper tới obstacle.
```

Không sử dụng raw LaserScan range như thể LiDAR nằm ở đầu xe.

---

# 8. SLOWDOWN PHẢI PROGRESSIVE

Behavior:

```text
Obstacle ngoài slowdown zone
→ 100% requested speed

Obstacle vào slowdown zone
→ giảm dần

Obstacle gần required stopping distance
→ giảm mạnh

Obstacle trong hard-stop envelope
→ linear component = 0
```

Không tạo vùng rất dài mà:

```text
speed_scale = 0.05
```

khi obstacle vẫn khá xa.

---

# 9. SAFETY PHẢI COMPONENT-WISE

Không dùng:

```text
obstacle detected
→ Twist() = 0,0
```

cho mọi trường hợp.

Safety phải biết:

```text
FORWARD_BLOCKED
REVERSE_BLOCKED
TURN_LEFT_BLOCKED
TURN_RIGHT_BLOCKED
```

hoặc representation tương đương.

---

# 10. FRONT BLOCKED VẪN PHẢI CHO PHÉP QUAY RA XA

Ví dụ:

```text
Obstacle front-left

ROBOT ↗
```

Input:

```text
v = +0.15
w = -0.20
```

Robot đang muốn quay phải để thoát.

Nếu tiến unsafe:

```text
v → 0
```

nhưng nếu quay phải safe:

```text
w phải được giữ.
```

Expected:

```text
input:
v=0.15
w=-0.20

output:
v=0
w<0
```

Không được:

```text
0,0
```

trừ khi rotation cũng unsafe.

---

# 11. HARD STOP TOÀN CHASSIS CHỈ CHO HAZARD NGHIÊM TRỌNG

Full:

```text
v=0
w=0
```

cho:

```text
E-stop
cliff
bumper
hardware fault
critical watchdog
sensor safety failure nghiêm trọng
motion mà mọi component đều unsafe
```

Không full-stop chỉ vì một side wall.

---

# 12. HYSTERESIS PHẢI THEO DIRECTION

Không dùng một:

```text
global stop hysteresis 400ms
```

để giữ toàn chassis đứng yên sau một geometric obstacle.

Tách:

```text
forward hysteresis
reverse hysteresis
left-turn hysteresis
right-turn hysteresis
```

Hard fault có thể có full-stop hysteresis riêng.

Nếu forward vừa bị block nhưng quay phải safe:

```text
turn-right không được bị global hysteresis khóa.
```

---

# 13. OBSTACLE CHECK PHẢI THEO SWEPT FOOTPRINT

Đừng chỉ kiểm tra:

```text
nearest obstacle distance.
```

Robot command:

```text
v,w
```

tạo ra quỹ đạo.

Safety phải kiểm tra footprint robot trên quỹ đạo đó.

---

# 14. ĐI THẲNG

Robot rộng:

```text
0.20m
```

half width:

```text
0.10m
```

Straight swept corridor:

```text
required straight width =
robot_width
+ 2 × configured_side_margin
+ uncertainty allowance nếu cần
```

Side margin phải config riêng.

Ví dụ nếu side margin:

```text
0.05m
```

thì:

```text
required width = 0.30m
```

Không bắt buộc dùng đúng 5cm.

Phải tune dựa trên robot thật.

---

# 15. ĐƯỜNG RỘNG PHẢI ĐI CHUẨN

Nếu path nằm trong vùng rộng:

```text
AUTO phải đi bình thường.
```

Không được:

```text
zig-zag
đánh lái vô lý
stop/restart liên tục
replan vô điều kiện
```

Nếu:

```text
path straight
front clear
```

ưu tiên giữ đường gần thẳng.

---

# 16. HÀNH LANG ĐỦ CHO ROBOT PHẢI ĐƯỢC ĐI

Ví dụ robot rộng:

```text
20cm
```

đường rộng:

```text
40cm
```

tức gấp đôi width robot.

Nếu side margin thực tế đáp ứng:

```text
CAN_GO_STRAIGHT=true
```

Robot phải đi.

Hai side wall:

```text
KHÔNG được tự động trở thành FRONT obstacle.
```

Áp dụng cho:

```text
Manual
Auto
```

---

# 17. ĐỪNG DÙNG FORWARD BRAKING DISTANCE CHO SIDE WALL

Đi thẳng phải có:

```text
front clearance
→ stopping distance
```

Side wall:

```text
side clearance
→ side margin
```

Không:

```text
side wall cách 15cm
braking distance 25cm
→ STOP
```

vì robot không chạy ngang vào wall.

---

# 18. ARC MOTION

Khi:

```text
v != 0
w != 0
```

không nên chỉ ghép:

```text
forward rectangle
+
full rotation circle
```

nếu cách đó quá bảo thủ.

Có thể dự đoán N future poses trong stopping horizon:

```text
pose 1
pose 2
pose 3
...
```

và kiểm tra physical rectangle tại từng pose.

N nhỏ để CPU thấp, ví dụ khoảng:

```text
4–8 samples
```

nếu phù hợp.

Mục tiêu:

```text
chỉ block quỹ đạo robot thực sự sẽ đi.
```

---

# 19. PHÂN BIỆT `CAN_GO_STRAIGHT` VÀ `CAN_ROTATE`

Robot:

```text
length = 0.30
width  = 0.20
```

Rotation swept radius vật lý xấp xỉ:

```text
sqrt(0.15² + 0.10²)
≈ 0.180m
```

chưa tính rotation margin.

Một corridor có thể:

```text
CAN_GO_STRAIGHT = true
CAN_ROTATE = false
```

Đây là trạng thái hợp lệ.

Nếu path gần thẳng:

```text
robot phải đi thẳng.
```

Không được BLOCKED chỉ vì không đủ không gian quay tại chỗ.

---

# 20. LOCALIZATION PHẢI SỬA DỨT ĐIỂM

Force Rescan hiện đang quá khó READY.

Không được dùng state machine:

```text
phải quay
+
phải đủ bins
+
phải đủ coverage
+
nhiều hard thresholds
```

cho mọi trường hợp.

Rotation phải là công cụ giải ambiguity.

Không phải điều kiện tuyệt đối.

---

# 21. FORCE RESCAN: STATIONARY-FIRST

Flow phải là:

```text
User bấm Quét lại vị trí
        ↓
Global AMCL reset/search
        ↓
robot đứng yên
        ↓
nhiều no-motion updates / fresh scans
        ↓
evaluate candidate
```

Nếu candidate rất mạnh:

```text
READY
```

không bắt buộc quay.

Nếu candidate ambiguous:

```text
mới yêu cầu rotation.
```

---

# 22. STRONG STATIONARY CANDIDATE

Đánh giá từ:

```text
fresh AMCL pose
good covariance
pose stability
scan-map score
median residual
robust p90/trimmed residual
enough residual beams
fresh TF
scan/odom timestamps valid
multiple consistent scans
```

Nếu tất cả đủ mạnh:

```text
READY without rotation.
```

---

# 23. MULTI-HEADING CHỈ KHI CẦN

Nếu stationary evidence chưa đủ:

```text
check rotation clearance
```

Nếu safe:

```text
rotate
collect heading diversity
```

Không bắt heading chính xác lý tưởng kiểu:

```text
0°
45°
90°
...
```

Dùng bins/coverage tolerant với sensor jitter.

Không có threshold sát maximum toán học.

---

# 24. HEADING OBSERVATION VÀ POSE QUALITY PHẢI RIÊNG

Robot đã nhìn heading nào:

```text
observation diversity
```

khác với:

```text
candidate quality.
```

Không chỉ ghi nhận heading nếu candidate đã strict-good.

Nếu không sẽ tạo deadlock.

---

# 25. LOCALIZATION KHÔNG ĐƯỢC CÓ DEAD-END

Không được có:

```text
need more heading
+
robot stopped
+
no way to collect heading
→ wait timeout
```

State machine phải luôn:

```text
READY
hoặc
tiếp tục search
hoặc
tiếp tục bounded rotation
hoặc
fail với reason cụ thể.
```

---

# 26. LOCALIZATION RESIDUAL PHẢI ROBUST

Giữ:

```text
median residual
p90 / percentile residual
```

nhưng không sử dụng threshold quá sát một snapshot robot thật.

Không để vài dynamic beam làm toàn pose fail nếu:

```text
median rất tốt
covariance tốt
pose stable
phần lớn geometry match tốt.
```

Candidate lệch thật:

```text
median residual lớn
```

vẫn phải reject.

---

# 27. SENSOR CLOCK: IMU KHÔNG ĐƯỢC PHÁ LOCALIZATION

Nếu:

```text
scan valid
odom valid
imu invalid
```

và IMU chỉ diagnostic/non-critical:

```text
Localization vẫn phải được phép hoạt động.
```

Tách health:

```text
scan_time_health
odom_time_health
imu_time_health
```

Critical localization:

```text
scan + odom.
```

Không dùng một invalid streak chung khiến IMU lỗi reset tất cả.

---

# 28. TF SYNCHRONIZATION

Ưu tiên TF tại:

```text
LaserScan.header.stamp.
```

Nếu exact TF chưa có:

```text
bounded short wait / deferred
```

sau đó:

```text
nearest transform
```

chỉ nếu age nằm trong tolerance.

Không silently drop hàng loạt scan.

Log TF miss/fallback/age.

---

# 29. ROTATION CLEARANCE PHẢI CÓ MỘT SOURCE OF TRUTH

Adapter và motion-safety không được mâu thuẫn.

Không dùng generic:

```text
safety_direction_mask == 0
```

làm điều kiện tuyệt đối cho localization rotation nếu mask có cả translation direction.

Dùng cùng geometry:

```text
half_length=0.15
half_width=0.10
rotation_margin=config
```

hoặc expose explicit:

```text
rotation_safe=true/false.
```

---

# 30. AUTO NAVIGATION LÀ ƯU TIÊN MẶC ĐỊNH

Đây là yêu cầu UX quan trọng.

Trong mọi trường hợp:

```text
nếu robot tự động có thể đi an toàn
→ cứ tự động đi.
```

Không hiện dialog/hỏi user chỉ vì corridor hơi hẹp.

Chỉ dừng hỏi khi thuật toán xác định:

```text
đường hiện tại không đạt Auto clearance
```

hoặc:

```text
clearance không đủ chắc chắn để Auto tự quyết định.
```

---

# 31. THÊM `CORRIDOR CLEARANCE CLASSIFICATION`

Không chỉ boolean.

Cần ít nhất 3 trạng thái:

```text
CLEAR
NARROW_OR_UNCERTAIN
PHYSICALLY_BLOCKED
```

### CLEAR

```text
available width >= autonomous required width
front clear
localization confidence tốt
```

=> Auto tiếp tục.

### NARROW_OR_UNCERTAIN

Ví dụ:

```text
physical robot có thể lọt
NHƯNG clearance cho Auto quá sát
hoặc sensor/localization uncertainty khiến không chắc.
```

=> dừng an toàn và hỏi user.

### PHYSICALLY_BLOCKED

```text
available width < physical width + hard safety margin
```

=> không được cố đi qua.

Ưu tiên route khác.

---

# 32. AUTO WIDTH VÀ HARD WIDTH PHẢI KHÁC NHAU

Nên có concept:

```text
physical_width = 0.20m

hard_required_width =
physical_width + 2 × hard_side_margin

auto_required_width =
physical_width
+ 2 × auto_side_margin
+ localization/sensor uncertainty
```

Ví dụ:

```text
hard_required_width < auto_required_width
```

Điều này cho phép phân biệt:

```text
"robot vật lý có thể đi qua"
```

với:

```text
"Auto chưa đủ chắc chắn để tự đi qua".
```

Không bắt buộc các margin cụ thể trong prompt.

Configurable.

---

# 33. CHỈ HIỆN THÔNG BÁO KHI AUTO KHÔNG ĐỦ TỰ TIN

Nếu:

```text
CLEAR
```

không hỏi user.

Auto tiếp tục.

Nếu:

```text
NARROW_OR_UNCERTAIN
```

thì:

```text
Pause Auto safely
```

và frontend hiển thị thông báo:

> **Đường đi phía trước có vẻ không đủ rộng để robot tự động đi qua an toàn. Bạn muốn điều khiển robot thủ công qua đoạn này hay tìm tuyến đường khác?**

Hai lựa chọn:

```text
[ Điều khiển thủ công ]

[ Tìm đường khác ]
```

---

# 34. OPTION 1 — ĐIỀU KHIỂN THỦ CÔNG

Khi user chọn:

```text
Điều khiển thủ công
```

phải:

```text
Pause/cancel active Auto controller safely
```

nhưng **KHÔNG xóa destination**.

Lưu:

```text
destination
route context
selected route id nếu có
navigation session
reason for manual handoff
```

Sau đó chuyển UI sang Manual Control.

---

# 35. DESTINATION PHẢI ĐƯỢC GIỮ

Ví dụ goal:

```text
Museum A
(x,y)
```

Robot gặp corridor hẹp.

User chọn Manual.

Goal vẫn phải tồn tại:

```text
selectedDestination = Museum A
```

Không:

```text
setSelectedDestination(null)
```

Không bắt user chọn lại đích.

---

# 36. HIỂN THỊ TRẠNG THÁI MANUAL HANDOFF

Trong Manual Mode cần thông báo rõ, ví dụ:

> **Đang điều khiển thủ công để vượt đoạn đường hẹp. Điểm đến vẫn được giữ. Khi đã vượt qua đoạn hẹp, nhấn “Tiếp tục tự động”.**

Có nút:

```text
[ Tiếp tục tự động ]
```

---

# 37. MANUAL SAFETY VẪN GIỮ

Manual handoff không được disable safety.

Nhưng safety phải theo component-wise geometry đã mô tả.

Người dùng phải có thể:

```text
tiến nếu swept forward safe
quay ra xa obstacle
lùi nếu rear safe
```

Không được dùng Manual để ép robot đi xuyên một corridor thật sự nhỏ hơn hard physical safety width.

---

# 38. `TIẾP TỤC TỰ ĐỘNG`

Khi user bấm:

```text
Tiếp tục tự động
```

phải:

```text
stop manual command
↓
verify robot stationary hoặc controlled transition
↓
verify localization
↓
giữ destination cũ
↓
compute path từ CURRENT pose tới destination
↓
Auto tiếp tục.
```

Không quay lại điểm cũ nơi Auto pause.

---

# 39. KHÔNG FORCE GLOBAL LOCALIZATION KHI RESUME NẾU POSE VẪN TỐT

Resume:

```text
localization valid?
→ dùng luôn.
```

Nếu uncertain:

```text
passive verify.
```

Chỉ global relocalize nếu cần.

Không tạo UX:

```text
Manual qua corridor
→ Continue
→ robot quay global localization vô lý.
```

---

# 40. ROUTE CONTEXT SAU MANUAL

Nếu route cũ từ current pose trở đi vẫn valid:

```text
có thể tiếp tục/rejoin.
```

Nếu không:

```text
compute path mới từ current pose → same destination.
```

Destination không thay đổi.

---

# 41. OPTION 2 — `TÌM ĐƯỜNG KHÁC`

Khi user chọn:

```text
Tìm đường khác
```

không chỉ ComputePath lại một lần rồi trả đúng route cũ.

Phải tìm **nhiều alternative routes thực sự khác nhau** tới cùng destination.

---

# 42. MULTIPLE ROUTE CANDIDATES

Tìm:

```text
2–3 route hợp lệ
```

nếu map có đủ lựa chọn.

Không bắt buộc luôn phải có 3.

Nếu chỉ có:

```text
1 alternative
```

hiển thị 1.

Nếu không có:

```text
không có tuyến thay thế
```

thông báo rõ.

---

# 43. CÁC ROUTE PHẢI THỰC SỰ KHÁC NHAU

Không được trả:

```text
Route A
Route A lệch 1 cell
Route A lệch 2 cell
```

rồi coi là ba route.

Phải kiểm tra route similarity.

Có thể sử dụng:

```text
path overlap ratio
Hausdorff-like distance
shared segment ratio
route corridor identity
```

hoặc metric đơn giản phù hợp.

Các route cần khác đủ để có ý nghĩa cho người dùng.

---

# 44. CÁCH TẠO ALTERNATIVE ROUTES

Không implement planner mới nếu không cần.

Có thể dùng:

```text
temporary keepout / penalty trên corridor hiện tại
→ ComputePath
→ candidate B

thêm penalty/keepout cho corridor B
→ ComputePath
→ candidate C
```

hoặc cơ chế phù hợp Nav2 hiện tại.

Temporary exclusion chỉ dành cho việc tìm alternative.

Không làm permanent map modification.

---

# 45. ROUTE HẸP VỪA FAIL PHẢI ĐƯỢC LOẠI KHỎI ALTERNATIVE

Nếu current route đã gây:

```text
NARROW_OR_UNCERTAIN
```

và user chọn:

```text
Tìm đường khác
```

candidate mới không được lại đi qua cùng narrow segment rồi gọi đó là alternative.

Dùng temporary avoid zone/penalty quanh đoạn problematic.

---

# 46. VALIDATE MỖI ROUTE

Mỗi candidate phải được kiểm tra:

```text
static collision
dynamic obstacle state
unknown space policy
physical footprint
minimum route clearance
narrow segment count
route length
```

Không hiển thị route mà robot chắc chắn không đi được.

---

# 47. ROUTE METADATA

Mỗi route candidate nên có:

```text
route_id
path
total_length
estimated_time nếu có thể
minimum_clearance
narrow_segments
```

Có thể thêm:

```text
recommended
```

cho route tốt nhất.

---

# 48. HIỂN THỊ NHIỀU ROUTE TRÊN MAP GIỐNG GOOGLE MAPS

Frontend Map phải hiển thị cùng lúc các route candidate.

Ví dụ:

```text
Route 1
Route 2
Route 3
```

Các tuyến không được chọn:

```text
thin / muted
```

Tuyến đang chọn:

```text
highlight
```

Giữ phong cách UI hiện tại.

Không redesign toàn app.

---

# 49. ROUTE PHẢI CLICK/SELECT ĐƯỢC

User có thể:

```text
click vào polyline
```

hoặc:

```text
click card/tuyến
```

để chọn.

Khi chọn:

```text
selectedRouteId
```

cập nhật.

Map highlight tuyến đó.

---

# 50. HIỂN THỊ THÔNG TIN TUYẾN

Có thể hiện nhỏ:

```text
Tuyến 1 — 23m
Tuyến 2 — 29m
Tuyến 3 — 34m
```

Nếu có minimum clearance:

```text
Tuyến 1 — 23m — rộng hơn
```

Không cần UI phức tạp.

---

# 51. ĐỀ XUẤT ROUTE MẶC ĐỊNH

Hệ thống có thể đánh dấu:

```text
Đề xuất
```

cho candidate có cost tốt nhất dựa trên:

```text
path length
clearance
turn complexity
dynamic obstacles
```

Không chỉ shortest distance nếu shortest route quá hẹp.

---

# 52. USER PHẢI XÁC NHẬN TUYẾN

Sau khi chọn route:

```text
[ Đi theo tuyến này ]
```

User nhấn.

Sau đó Auto bắt đầu.

Không tự chạy ngay khi route vừa được render.

---

# 53. QUAN TRỌNG: ROBOT PHẢI ĐI THEO ROUTE USER ĐÃ CHỌN

Không được:

```text
preview Route B
user chọn Route B
↓
NavigateToPose(goal)
↓
planner tự compute lại Route A
```

Đây là lỗi UX nghiêm trọng.

Selected path phải trở thành execution constraint.

Ưu tiên:

```text
selected route path
→ FollowPath
```

hoặc cơ chế Nav2 tương đương bảo đảm robot thực sự follow route đó.

---

# 54. KHÔNG REPLAN THÀNH TUYẾN KHÁC ÂM THẦM

Trong lúc đi route user chọn:

```text
nếu route còn valid
→ giữ route.
```

Nếu dynamic obstacle làm route invalid:

```text
pause / replan.
```

Nếu replan cần thay đổi route đáng kể:

```text
không âm thầm đổi sang một tuyến hoàn toàn khác
```

nếu điều đó vi phạm lựa chọn user.

Có thể:

```text
replan cục bộ quanh obstacle
```

nếu vẫn nằm trong cùng route corridor.

Nếu phải đổi route lớn:

```text
thông báo / quay lại route choice
```

khi hợp lý.

---

# 55. ALTERNATIVE ROUTE VÀ LOCAL AVOIDANCE KHÁC NHAU

Dynamic obstacle nhỏ:

```text
controller/local planner tránh nhẹ
```

không cần hỏi user.

Chỉ hỏi lại route khi:

```text
selected route fundamentally invalid
```

hoặc:

```text
corridor clearance không đạt.
```

---

# 56. STATE MACHINE MỚI

Navigation state cần mô hình tương đương:

```text
READY
   ↓
PLANNING
   ↓
NAVIGATING
   ↓
NARROW_PATH_DECISION
```

Từ đó:

### Option Manual

```text
NARROW_PATH_DECISION
↓
MANUAL_BYPASS
↓
CONTINUE_AUTO
↓
PLANNING
↓
NAVIGATING
```

### Option Alternative

```text
NARROW_PATH_DECISION
↓
COMPUTING_ALTERNATIVES
↓
ROUTE_SELECTION
↓
NAVIGATING_SELECTED_ROUTE
```

---

# 57. KHÔNG DÙNG `BLOCKED` CHO NARROW DECISION

Nếu robot phát hiện:

```text
Auto không chắc đi qua
```

không chuyển ngay:

```text
BLOCKED.
```

Dùng state:

```text
NARROW_PATH_DECISION
```

hoặc tên tương đương.

`BLOCKED` chỉ khi:

```text
không thể tiếp tục
+
không alternative
+
không có recovery phù hợp.
```

---

# 58. PHÂN BIỆT `NARROW` VÀ `BLOCKED`

### Narrow

```text
physical passage có thể tồn tại
nhưng Auto confidence/margin không đủ.
```

=> user decision.

### Blocked

```text
không có route an toàn
hoặc obstacle/hazard thật sự ngăn tất cả route.
```

Không gom hai trường hợp.

---

# 59. NARROW DETECTION PHẢI XÉT MỘT ĐOẠN PATH, KHÔNG CHỈ MỘT SCAN

Không quyết định từ một beam.

Đánh giá corridor dọc một lookahead path.

Ví dụ:

```text
current pose
→ next 0.5–1.5m của global path
```

tùy speed/config.

Tính:

```text
minimum available width
left/right clearance
front clearance
localization uncertainty
consistency across multiple scans
```

---

# 60. KHÔNG HIỆN NARROW DIALOG TỪ MỘT FRAME NHIỄU

Cần confirmation:

```text
N consistent samples
```

hoặc:

```text
stable for configured duration.
```

Nhưng không chờ quá lâu đến mức robot sát obstacle.

---

# 61. LOCALIZATION UNCERTAINTY PHẢI ẢNH HƯỞNG AUTO CLEARANCE

Nếu localization confidence thấp:

```text
effective auto margin tăng.
```

Nếu localization tốt:

```text
margin bình thường.
```

Không cần thuật toán quá phức tạp.

Nhưng corridor classification không được giả định pose hoàn hảo khi covariance cao.

---

# 62. KHÔNG ĐỂ LOCALIZATION SAI TẠO FALSE NARROW KEEP-OUT

Nếu localization suspect:

```text
không mark corridor permanently/temporarily blocked.
```

Trước tiên:

```text
verify localization.
```

---

# 63. STATIC/DYNAMIC FILTER

Giữ expected-range/raycast nếu implementation đúng.

Static wall:

```text
expected ~ measured
→ static
```

Object trước wall:

```text
measured << expected
→ dynamic
```

Fail-safe:

```text
không chắc
→ giữ obstacle.
```

---

# 64. PERFORMANCE

Không để scan filtering/raycast làm callback quá nặng.

Hiện target cần:

```text
processing time << LiDAR frame period.
```

Log:

```text
scan_processing_ms
localization_ms
planning_filter_ms
safety_ms
```

Rate-limited.

---

# 65. AUTO RECOVERY KHI ĐƯỜNG THỰC SỰ BLOCKED

Nếu route hiện tại:

```text
CONFIRMED PHYSICALLY BLOCKED
```

có thể chủ động compute alternatives.

Nếu policy UX muốn user chọn:

```text
hiển thị route alternatives.
```

Không retry vô hạn cùng đường.

---

# 66. FAILED SEGMENT MEMORY

Giữ concept temporary failed segment/keepout nếu đúng.

Nhưng chỉ tạo khi:

```text
real insufficient clearance
```

hoặc:

```text
stable obstacle.
```

Không tạo vì false positive Motion Safety.

---

# 67. ALTERNATIVE ROUTE SEARCH PHẢI GIỮ DESTINATION

Mọi route đều:

```text
Current robot pose
→ SAME selected destination.
```

Không thay goal.

---

# 68. KHI MANUAL XONG VẪN GIỮ DESTINATION

Đây là acceptance bắt buộc:

```text
Auto
→ Narrow
→ Manual
→ user lái qua
→ Continue Auto
→ same original destination.
```

---

# 69. NÚT UI CẦN CÓ

Khi narrow:

```text
[ Điều khiển thủ công ]
[ Tìm đường khác ]
```

Trong manual bypass:

```text
[ Tiếp tục tự động ]
```

Trong route selection:

```text
[ Đi theo tuyến này ]
[ Quay lại ]
```

Không cần redesign ngoài flow này.

---

# 70. MESSAGE UI

Narrow:

> **Đường đi phía trước có vẻ không đủ rộng để robot tự động đi qua an toàn. Bạn có thể điều khiển thủ công qua đoạn này hoặc chọn một tuyến đường khác.**

Manual:

> **Điểm đến vẫn được giữ. Khi đã vượt qua đoạn đường hẹp, nhấn “Tiếp tục tự động”.**

No alternative:

> **Không tìm thấy tuyến đường thay thế hợp lệ tới điểm đến. Bạn có thể chuyển sang điều khiển thủ công hoặc dừng điều hướng.**

---

# 71. DEBUG LOG PHẢI GIỮ BẬT

Default:

```env
NAVIGATION_DEBUG_LOG=true
```

Có thể tắt:

```env
NAVIGATION_DEBUG_LOG=false
```

Không bỏ logging hiện tại.

---

# 72. LOG MOTION SAFETY

Log khi command bị modify:

```text
input_v
input_w
output_v
output_w

front_clearance
rear_clearance
left_clearance
right_clearance

forward_blocked
reverse_blocked
turn_left_blocked
turn_right_blocked

hard_stop
slowdown_scale
required_stop_distance
reason
```

---

# 73. LOG LOCALIZATION

Log:

```text
state
AMCL pose
covariance
scan score
median residual
robust p90/percentile
residual beam count
stationary candidate strength
heading bins
heading coverage
TF age
scan time valid
odom time valid
imu diagnostic health
rotation safe
reject reason
```

---

# 74. LOG NARROW DETECTION

Ví dụ:

```text
[NAV][CORRIDOR]

vehicle_width=0.20
vehicle_length=0.30

available_width=...
hard_required_width=...
auto_required_width=...

left_clearance=...
right_clearance=...
front_clearance=...

classification=CLEAR / NARROW_OR_UNCERTAIN / PHYSICALLY_BLOCKED

reason=...
```

---

# 75. LOG USER DECISION

```text
[NAV][NARROW_DECISION]
choice=MANUAL
```

hoặc:

```text
choice=FIND_ALTERNATIVE
```

---

# 76. LOG ALTERNATIVE ROUTES

Ví dụ:

```text
[NAV][ROUTE_CANDIDATES]
count=3
```

và từng route:

```text
route_id
length
minimum_clearance
overlap_with_original
valid
```

---

# 77. LOG ROUTE SELECTION

```text
[NAV][ROUTE_SELECTED]
route_id=...
```

và khi execution:

```text
actual execution path route_id=...
```

để chứng minh robot chạy đúng route user chọn.

---

# 78. TEST — VEHICLE DIMENSIONS

Test/assert tất cả geometry sử dụng:

```text
length=0.30
width=0.20
```

Không còn `0.40 × 0.36` trong active navigation configs nếu đó là footprint robot cũ.

---

# 79. TEST — WIDE PATH

Không vật cản gần.

Expected:

```text
Manual unchanged
Auto unchanged
No unnecessary slowdown
No narrow dialog
```

---

# 80. TEST — CORRIDOR RỘNG 40CM

Robot:

```text
20cm wide
```

Corridor:

```text
40cm wide
```

Nếu configured margins đáp ứng:

```text
classification=CLEAR
```

Auto đi.

Không hiện dialog.

Manual đi.

---

# 81. TEST — ROBOT LỆCH TÂM NHẸ

Corridor 40cm.

Test:

```text
centered
offset 2cm
offset 4cm
```

Nếu vẫn đủ margin:

```text
CLEAR
```

Không false-stop.

---

# 82. TEST — NARROW NHƯNG VẬT LÝ CÓ THỂ QUA

Tạo corridor:

```text
hard_required_width
<
available_width
<
auto_required_width
```

Expected:

```text
NARROW_OR_UNCERTAIN
```

Auto pause.

Frontend hiện:

```text
Manual
Find alternative
```

---

# 83. TEST — MANUAL HANDOFF

Sau narrow decision:

```text
user chooses Manual
```

Expected:

```text
Auto goal paused
destination preserved
manual enabled
```

User di chuyển.

Nhấn:

```text
Continue Auto
```

Expected:

```text
same destination
path from current pose
Auto resumes
```

---

# 84. TEST — MANUAL SAFETY

Trong Manual narrow corridor:

```text
side wall gần nhưng straight swept path safe
```

Expected:

```text
forward allowed.
```

Nếu forward unsafe nhưng turn-away safe:

```text
linear=0
safe angular preserved.
```

---

# 85. TEST — FIND ALTERNATIVE

Original Route A đi qua narrow segment.

User chọn:

```text
Find alternative.
```

System tìm:

```text
Route B
Route C
```

nếu tồn tại.

Không được trả Route A lại như một candidate khác.

---

# 86. TEST — ROUTE DISTINCTNESS

Candidate routes phải vượt minimum distinctness threshold.

Không tính hai polyline overlap >95% là hai route khác nhau nếu chỉ lệch vài cell không đáng kể.

Threshold config/hợp lý.

---

# 87. TEST — MAP ROUTE SELECTION

Frontend nhận 2–3 path.

Expected:

```text
all visible
selected route highlighted
click another route changes selection
```

---

# 88. TEST — EXECUTE SELECTED ROUTE

User chọn:

```text
Route B
```

Expected:

```text
robot follows Route B.
```

Không gọi API cuối cùng chỉ có:

```text
NavigateToPose(destination)
```

nếu điều đó làm planner tự đổi sang Route A.

---

# 89. TEST — ROUTE B BỊ DYNAMIC OBSTACLE

Trong khi follow Route B:

Dynamic obstacle nhỏ:

```text
local avoidance/replan nhỏ
```

nếu vẫn giữ cùng route corridor.

Nếu route B fundamentally unusable:

```text
pause / recompute choices
```

không âm thầm đổi sang route khác lớn.

---

# 90. TEST — PHYSICALLY TOO NARROW

```text
available_width < hard_required_width
```

Không cho Auto đi.

Không được dùng Manual để bypass hard collision safety.

System ưu tiên:

```text
Find alternative.
```

Nếu vẫn cho Manual mode để reposition robot, safety phải ngăn việc cố đi xuyên passage vật lý không đủ.

---

# 91. TEST — LOCALIZATION STATIONARY

Force Rescan ở vị trí đặc trưng.

Strong candidate.

Expected:

```text
READY without rotation.
```

---

# 92. TEST — AMBIGUOUS LOCALIZATION

Stationary candidate không đủ mạnh.

Rotation safe.

Expected:

```text
rotate
collect multi-heading evidence
READY.
```

---

# 93. TEST — ROTATION UNSAFE

Stationary candidate ambiguous.

Rotation unsafe.

Expected:

```text
không quay
reason=ROTATION_CLEARANCE_BLOCKED
```

Không timeout mơ hồ.

---

# 94. TEST — IMU INVALID

```text
scan valid
odom valid
IMU diagnostic timestamp invalid
```

Expected:

```text
localization critical timing remains valid.
```

---

# 95. TEST — STOPPING DISTANCE

Với mỗi Manual profile:

```text
Slow
Normal
Fast
```

test:

```text
outside slowdown → 100%
inside slowdown → progressive scale
inside hard stop → linear 0
```

Report khoảng cách tính từ **front bumper**, không chỉ từ LiDAR.

---

# 96. TEST — NO FALSE FAR STOP

Obstacle rõ ràng ngoài:

```text
required_stop_distance + slowdown_margin
```

Expected:

```text
output speed == requested speed.
```

---

# 97. TEST — EMERGENCY SAFETY

Phải tiếp tục pass:

```text
E-stop
cliff
bumper
watchdog
real front collision
real rear collision
unsafe pure rotation
```

Không được weaken.

---

# 98. KHÔNG TUNE PLANNER ĐỂ CHE MOTION SAFETY

Nếu Manual bị stop:

```text
sửa Motion Safety.
```

Không:

```text
đổi Theta*
giảm inflation
thu nhỏ footprint
```

để che lỗi.

---

# 99. KHÔNG TĂNG TIMEOUT ĐỂ CHE LOCALIZATION DEADLOCK

Nếu localization không READY vì state machine:

```text
sửa state machine.
```

Không chỉ:

```text
45s → 90s.
```

---

# 100. KHÔNG GIẢM FOOTPRINT

Physical footprint:

```text
0.30 × 0.20m
```

là source of truth.

Không giảm.

---

# 101. KHÔNG HIỆN DIALOG QUÁ SỚM

Ưu tiên Auto.

Nếu:

```text
CLEAR
```

Auto đi.

Không hỏi user.

Chỉ hỏi khi:

```text
NARROW_OR_UNCERTAIN.
```

Đây là acceptance quan trọng.

---

# 102. KHÔNG ĐỂ AUTO CHẠY CỐ QUA ĐƯỜNG THỰC SỰ KHÔNG ĐỦ

Ngược lại, không được:

```text
auto_required_width fail
→ vẫn cố đi
```

đến khi collision safety hard stop.

Decision phải được đưa ra trước khi robot sát obstacle.

---

# 103. CHANGE BUDGET

Ưu tiên sửa ít file production nhất.

Nếu bắt đầu sửa > khoảng 8–10 file production:

```text
review lại scope.
```

Không phải hard limit nhưng chống scope creep.

Mỗi production change phải gắn với root cause hoặc feature requirement cụ thể.

---

# 104. TRƯỚC KHI CODE

Output:

```text
ROOT CAUSE — LOCALIZATION:
...

ROOT CAUSE — MANUAL FAR STOP:
...

ROOT CAUSE — CORRIDOR:
...

DESIGN — NARROW DECISION:
...

DESIGN — MANUAL HANDOFF:
...

DESIGN — MULTIPLE ROUTES:
...

FILES TO CHANGE:
...
```

Sau đó mới sửa.

---

# 105. SAU KHI CODE

Chạy:

```text
Python compile
navigation pytest
motion-safety pytest
frontend typecheck/build nếu sửa UI
YAML validation
BT XML validation
docker compose config
git diff --check
```

Không sửa unrelated test failure.

---

# 106. FINAL REPORT

Phải trả về:

## Files changed

Từng file.

## Robot geometry

Xác nhận:

```text
length=0.30m
width=0.20m
```

và nơi dùng.

## Localization

Nêu:

```text
stationary criteria
rotation criteria
TF handling
sensor clock handling
residual validation
```

## Motion Safety

Nêu:

```text
stopping distance model
slowdown model
component clipping
hysteresis
```

## Corridor

Nêu:

```text
hard_required_width
auto_required_width
CLEAR/NARROW/BLOCKED logic
```

## Manual handoff

Nêu cách:

```text
destination preserved
Continue Auto
resume from current pose
```

## Multiple routes

Nêu:

```text
candidate generation
route distinctness
route validation
map rendering
route selection
selected route execution
```

## Tests

Command + result.

## Robot test checklist

Test thực tế cần chạy.

---

# 107. ACCEPTANCE CUỐI CÙNG

Task chưa hoàn thành nếu thiếu bất kỳ behavior nào sau.

### Localization

> Quét lại vị trí phải có khả năng READY khi robot đứng yên nếu evidence đủ mạnh. Chỉ quay khi cần giải ambiguity.

### Manual

> Manual không bị giảm tốc/dừng từ khoảng cách quá xa ngoài braking envelope thực sự cần thiết.

### Escape motion

> Nếu tiến unsafe nhưng quay ra xa obstacle safe, robot vẫn được phép quay ra xa.

### Wide road

> Đường rộng phải đi ổn định và chính xác, không zig-zag/stop vô lý.

### Corridor

> Robot dài 30cm, rộng 20cm phải được đánh giá bằng đúng footprint này. Đường đủ clearance phải được Auto đi qua.

### Auto priority

> Nếu đường đạt Auto clearance thì robot tự đi, không hỏi user.

### Narrow uncertainty

> Chỉ khi đường không đủ Auto margin hoặc uncertainty quá cao mới pause và hỏi user.

### Manual option

> User chọn Manual → giữ nguyên destination → user lái qua → bấm “Tiếp tục tự động” → robot tiếp tục tới đúng destination cũ từ vị trí hiện tại.

### Alternative option

> User chọn “Tìm đường khác” → hệ thống tìm nhiều tuyến hợp lệ thực sự khác nhau → hiển thị cùng lúc trên map → user chọn tuyến → robot đi đúng tuyến đã chọn.

### Route integrity

> Không được preview một tuyến nhưng execution âm thầm tính lại và chạy tuyến khác.

### Real obstacle

> Dynamic obstacle thật vẫn được phát hiện.

### Real narrow passage

> Passage thật sự nhỏ hơn hard physical clearance không được cố đi xuyên.

### Safety

> E-stop, cliff, bumper, watchdog và collision protection phải giữ nguyên.

### No regression

> Một fix chỉ được coi là hoàn thành khi không làm behavior khác tệ hơn.

---

# 108. NGUYÊN TẮC CUỐI CÙNG

Không thêm workaround để che bug.

Không giảm safety để làm robot đi được.

Không thu nhỏ robot để planner tìm path.

Không tăng timeout để che localization lỗi.

Không biến side wall thành front obstacle.

Không dừng Auto chỉ vì đường “có vẻ hẹp” nếu tính toán vẫn xác nhận đủ clearance.

Không bắt user điều khiển thủ công nếu Auto vẫn có thể tự xử lý.

**Ưu tiên tự động hoàn toàn. Chỉ chuyển quyền quyết định cho người dùng khi hệ thống xác định đường phía trước không đủ Auto clearance hoặc không đủ độ tin cậy để tự đi an toàn.**

Khi cần user quyết định:

```text
1. Điều khiển thủ công
2. Tìm đường khác
```

và cả hai flow phải giữ nguyên destination ban đầu.

**Không sửa bất kỳ phần nào không trực tiếp phục vụ các yêu cầu trong prompt này.**
