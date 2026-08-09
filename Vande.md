# Vấn đề hiện tại

Robot `.170` đang chạy hệ thống chính với các thành phần phần cứng, điều khiển web, an toàn chuyển động, SLAM và Nav2 đã được quản lý theo một luồng duy nhất.

Một chương trình khác muốn tạo map theo hướng dẫn Yahboom bằng chuỗi lệnh:

```bash
sh ~/start_agent_rpi5.sh
sh ros2_humble.sh
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
ros2 launch yahboomcar_nav display_launch.py
ros2 launch yahboomcar_nav map_gmapping_launch.py
```

Chuỗi lệnh này không chỉ chạy Gmapping mà khởi động lại gần như toàn bộ ROS 2 hardware stack của Yahboom. Khi chạy song song với hệ thống hiện tại, nó tạo ra các xung đột sau.

---

# 1. Tranh chấp `/dev/ttyUSB0`

`start_agent_rpi5.sh` tạo thêm một micro-ROS Agent:

```bash
serial --dev /dev/ttyUSB0 -b 921600 -v4
```

Trong khi `/dev/ttyUSB0` đã được Agent của hệ thống chính sử dụng. Hai Agent cùng truy cập cổng serial có thể làm:

* MCU mất kết nối hoặc liên tục reconnect.
* Mất `/scan`, `/imu`, `/odom_raw`, `/battery`.
* Robot không nhận `/cmd_vel`.
* Container Agent restart liên tục.
* Cổng USB bị kẹt, đôi khi phải reset MCU hoặc ngắt nguồn phần cứng.

> Đây là xung đột vật lý nên namespace hoặc `ROS_DOMAIN_ID` không giải quyết được.

---

# 2. Khởi động trùng Yahboom runtime

`ros2_humble.sh` và `yahboomcar_bringup_launch.py` cố tạo thêm các node mà hệ thống đang có:

* EKF / `robot_localization`
* IMU complementary filter
* `robot_state_publisher`
* `joint_state_publisher`
* static TF publishers
* các node điều khiển/joystick Yahboom

**Hậu quả:**

* Có nhiều node cùng tên.
* Nhiều publisher cùng phát một TF.
* `odom → base_footprint` bị nhiều nguồn điều khiển.
* Vị trí robot trên map nhảy, xoay hoặc vẽ đường đi hỗn loạn.
* ROS graph không ổn định và điều khiển web có thể mất tác dụng.

---

# 3. Trùng quyền phát `map → odom`

`map_gmapping_launch.py` khởi động Gmapping và một static TF LiDAR mới. Trong khi hệ thống chính đang chạy:

* AMCL ở chế độ Navigation; hoặc
* SLAM Toolbox ở chế độ Mapping.

Nếu Gmapping chạy cùng domain, nhiều node có thể cùng phát `map → odom`.

**Kết quả là:**

* Vị trí robot nhảy liên tục.
* Map bị chồng, méo hoặc xoay.
* Occupancy map thay đổi bất thường dù robot đứng yên.
* Nav2 không thể localization chính xác.
* Map tạo ra không đủ tin cậy để chạy tự động.

> Chỉ đổi namespace không khắc phục được vì các frame `map`, `odom`, `base_link` và `laser_frame` vẫn là tên toàn cục.

---

# 4. Tăng RAM, CPU và số lượng thread

Chuỗi lệnh Yahboom tạo thêm nhiều ROS 2 process và DDS participant. Mỗi node ROS 2 có thể tạo nhiều thread truyền thông và reserve vùng nhớ lớn.

Khi chạy trùng sẽ xuất hiện nhiều bản sao của:

* `micro_ros_agent`
* `yahboom_joy`
* EKF
* `robot_state_publisher`
* `joint_state_publisher`
* Gmapping
* RViz2
* các static TF publisher

RViz2 chạy trực tiếp trên Pi cũng tiêu thụ đáng kể RAM/GPU. Agent chạy `-v4` còn sinh log rất lớn.

**Hậu quả có thể là:**

* RAM gần đầy và swap bị sử dụng hết.
* CPU tăng cao, load average tăng.
* DDS tạo hàng trăm thread.
* Điều khiển web bị delay, giật hoặc watchdog phát lệnh dừng.
* Process bị OOM Killer kết thúc.
* Pi treo hoặc phải tắt nguồn và bật lại.

Do đó hiện tượng đầy RAM không nhất thiết do riêng Gmapping; nguyên nhân chính là chạy thêm toàn bộ Yahboom stack bên cạnh stack đang hoạt động.

---

# Ảnh hưởng đến hệ thống chính

Khi chương trình Yahboom trên được chạy, có thể xảy ra đồng thời:

* Mất topic cảm biến.
* Mất điều khiển từ web.
* Robot nhận lệnh từ sai publisher.
* Safety và `/cmd_vel` không còn một nguồn có thẩm quyền.
* Map sai và vị trí robot nhảy.
* Nav2 không thể chọn hoặc activate map.
* Container restart, tăng RAM và làm Pi mất ổn định.
* Phải reset MCU hoặc nguồn để phục hồi serial.

---

# Nguyên tắc xử lý

Trên robot chỉ được có:

* Một tiến trình sở hữu `/dev/ttyUSB0`.
* Một Yahboom hardware/base runtime.
* Một nguồn `odom → base_footprint`.
* Một nguồn `map → odom`.
* Một publisher cuối cùng vào `/cmd_vel`.

Người cần tạo map phải dùng chức năng **Maps → Tạo map SLAM** của hệ thống. Bộ chuyển chế độ sẽ tạm dừng Nav2, chạy một SLAM stack và giữ nguyên Agent, cảm biến, camera, điều khiển web cùng lớp an toàn.

Nếu bắt buộc sử dụng Gmapping Yahboom, phải triển khai nó ở ROS domain riêng, chỉ nhận dữ liệu cảm biến qua bridge một chiều. Không được chạy lại micro-ROS Agent, Yahboom bringup hoặc ghi ngược `/cmd_vel`, `/map`, `/tf` vào graph chính.
