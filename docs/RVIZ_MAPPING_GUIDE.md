# RViz2 từ máy Ubuntu kỹ thuật

Project dùng ROS 2 **Humble**. Robot giữ graph vận hành kín ở `ROS_DOMAIN_ID=20`; RViz dùng domain quan sát một chiều `ROS_DOMAIN_ID=21`. RViz chỉ chạy trên máy Ubuntu Admin cùng LAN; Raspberry Pi không cài/chạy GUI.

## Cài đặt

Trên Ubuntu 22.04 đã cấu hình ROS apt repository:

```bash
sudo apt update
sudo apt install \
  ros-humble-rviz-assimp-vendor \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins \
  ros-humble-rviz-ogre-vendor \
  ros-humble-rviz-rendering \
  ros-humble-rviz2 \
  ros-humble-ros2cli \
  ros-humble-ros2topic
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=21
```

Nếu máy đã có `ros-humble-desktop` thì RViz đã được cài. Không tự khởi động robot bringup, SLAM, Nav2, micro-ROS agent hoặc Yahboom stack từ máy Admin.

Để nút **Xem map** trên Web mở RViz bằng một lần nhấp, cài URL launcher một lần cho từng tài khoản Linux Admin:

```bash
./scripts/install_rviz_url_handler.sh
```

Trình duyệt có thể hỏi xác nhận mở `Rovera RViz Viewer` trong lần đầu. Launcher chạy hoàn toàn trên máy Admin và không nhận lệnh shell từ URL.

## Mở preset

Từ root repository:

```bash
./scripts/open_mapping_rviz.sh
ROS_DOMAIN_ID=20 ./scripts/open_navigation_rviz.sh
```

`mapping.rviz` dùng fixed frame `map`, `/map`, `/scan_mapping`, `/tf`, `/tf_static` và `/odometry/filtered` trên domain 21. Bridge không chuyển topic điều khiển, cảm biến thô, service hoặc action. `/scan_mapping` là bản scan đã được adapter chuẩn hóa timestamp và cũng là dữ liệu SLAM thực sự nhận. `navigation.rviz` vẫn là công cụ kỹ thuật trực tiếp trên domain 20 và không được mở trên máy không được quản trị.

Preset mapping mặc định bật Grid, Live SLAM Map, LiDAR, `Đường đi đã quét` và `Vị trí xe hiện tại`. Vị trí xe là mũi tên xanh lá lấy từ `/odometry/filtered`, hướng mũi tên là hướng đầu xe và chỉ giữ một pose mới nhất nên không để lại vệt. Đường đi màu xanh dương lấy trực tiếp từ pose graph `/slam_toolbox/graph_visualization`, vì vậy có toàn bộ quỹ đạo đã quét từ đầu phiên và được hiệu chỉnh theo SLAM thay vì tích lũy sai số odometry. Preset chỉ hiện các cạnh quỹ đạo, ẩn các chấm pose để bản đồ dễ nhìn. TF/RobotModel vẫn tắt; có thể bật lại từng lớp trong bảng Displays khi cần debug.

## Kiểm tra DDS graph

```bash
ros2 topic list
ros2 topic hz /scan_mapping
ros2 topic hz /map
ros2 topic echo /odometry/filtered --once
ros2 topic info /tf --verbose
```

Mapping trên máy Admin phải thấy `/map`, `/scan_mapping`, `/tf`, `/tf_static` và `/odometry/filtered`. Những topic vận hành domain 20 không xuất hiện ở domain 21 là đúng thiết kế.

## Troubleshooting LAN/DDS

1. Máy Admin dùng domain 21; graph vận hành trên Pi dùng domain 20.
2. So sánh `echo $RMW_IMPLEMENTATION`. Fast DDS và CycloneDDS có thể liên lạc theo chuẩn DDS, nhưng khi debug nên dùng cùng RMW/version.
3. Đảm bảo hai máy cùng LAN/VLAN có multicast UDP. Wi-Fi AP isolation, guest network và routed VLAN thường chặn discovery.
4. Tạm kiểm tra firewall Ubuntu (`sudo ufw status`). Chỉ mở DDS trên LAN tin cậy; không expose DDS ra Internet.
5. Nếu laptop có Ethernet, Wi-Fi, VPN và Docker cùng lúc, tắt interface/VPN không dùng hoặc cấu hình DDS interface whitelist.
6. Kiểm tra IP/subnet và `ping` hai chiều. DDS discovery không đi qua NAT/router mặc định.
7. Kiểm tra QoS: `/scan_mapping` dùng best-effort/volatile; `/map` reliable/transient-local. Preset và bridge đã đặt tương ứng.

Nếu viewport bị phủ màu hồng và log có `GLSL link result` hoặc `active samplers with a different type`, kiểm tra không để `rviz2` và các package `rviz-*` khác phiên bản. Hai script mở RViz sẽ phát hiện trường hợp này và in đúng lệnh đồng bộ package. Sau đó script tự tạo một media overlay không cần `sudo` trong runtime directory và dùng compatibility shader cho Map/Costmap. Overlay không thay đổi file ROS trong `/opt` và được tạo lại tự động khi package hoặc shader thay đổi.

Nếu `ros2 topic list` không thấy graph, lỗi nằm ở domain/discovery/network chứ không phải RViz display. Nếu thấy topic nhưng không render, kiểm tra Fixed Frame `map`, TF `map -> odom -> base_footprint -> base_link -> laser_frame` và timestamp.
