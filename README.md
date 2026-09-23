# UR3 vẽ chữ M và quỹ đạo tròn 360° — MoveIt 2

Package ROS 2 điều khiển mô phỏng **UR3** bằng **MoveIt 2** và `joint_trajectory_controller`.

- Quỹ đạo 1: đầu công tác vẽ chữ **M** (chữ cái đầu của Minh), kích thước mặc định **12 × 10 cm**.
- Quỹ đạo 2: đầu công tác vẽ vòng tròn Cartesian **360°**, đường kính mặc định **8 cm**.
- Mô phỏng chạy trên Gazebo/GZ và quan sát bằng RViz.

## Cấu trúc package

```text
ur3_write_m/
├── config/
│   ├── write_m.yaml              # Tham số chữ M
│   ├── circle.yaml               # Bán kính/số điểm vòng tròn
│   └── moveit_controllers_gz.yaml
├── launch/
│   ├── write_m.launch.py
│   └── draw_circle.launch.py
└── src/
    ├── write_m_node.cpp
    └── draw_circle_node.cpp
```

## Build

```bash
cd /home/minh/AI_Homework
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_write_m --symlink-install
source install/setup.bash
```

## Chạy chữ M

Trước mỗi lần chạy, dọn các tiến trình ROS/Gazebo cũ:

```bash
pkill -f "ros2 launch|gz sim|move_group|rviz2|parameter_bridge|robot_state_publisher|write_m_node"
ps -ef | grep -E "ros2 launch|gz sim|move_group|rviz2"
```

Sau đó chạy:

```bash
cd /home/minh/AI_Homework
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=56 ros2 launch ur3_write_m write_m.launch.py execute:=true
```

Node bắt đầu sau khoảng **60 giây** để Gazebo và controller khởi tạo hoàn tất. Robot vào `test_configuration` với khuỷu gập thay vì pose `up` có cánh tay gần duỗi thẳng. Trước khi vẽ, node kiểm tra IK liên tục dọc nét, yêu cầu khuỷu và cổ tay cách vùng singularity ít nhất khoảng 17°, rồi thử tối đa sáu vị trí lân cận theo bước 5 cm nếu cần. MoveIt dùng kế hoạch tránh va chạm khi tiếp cận và chuyển nét; nét vẽ nội suy Cartesian có collision check. Nếu IK chỉ trả về một phần đường, node tiếp tục từ trạng thái mới trên cùng nét; nếu không thể nối liên tục thì dừng an toàn, không dùng đường vòng để nối qua chữ. Hai node giữ chạy sau khi vẽ để marker RViz còn hiển thị; nhấn Ctrl+C để kết thúc.

Để hiện chữ M đỏ trong RViz:

```text
Add → By topic → /write_m_path → Marker
```

## Chạy vòng tròn 360°

Trước mỗi lần chạy, dọn các tiến trình ROS/Gazebo cũ:

```bash
pkill -f "ros2 launch|gz sim|move_group|rviz2|parameter_bridge|robot_state_publisher|write_m_node"
ps -ef | grep -E "ros2 launch|gz sim|move_group|rviz2"
```

Sau đó chạy:

```bash
cd /home/minh/AI_Homework
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=56 ros2 launch ur3_write_m draw_circle.launch.py execute:=true
```

Để hiện vòng tròn xanh trong RViz:

```text
Add → By topic → /circle_path → Marker
```

Node vòng tròn giữ chạy sau khi hoàn thành để marker không biến mất. Nhấn `Ctrl+C` để kết thúc.

## Thiết lập Marker trong RViz

Thực hiện sau khi RViz đã mở và node quỹ đạo đang chạy:

1. Ở khung **Displays** bên trái, nhấn **Add** ở góc dưới.
2. Chọn tab **By topic**.
3. Chọn một trong hai topic:

   ```text
   /write_m_path  → visualization_msgs/Marker   (chữ M đỏ)
   /circle_path   → visualization_msgs/Marker   (vòng tròn xanh)
   ```

4. Nhấn **OK**. RViz tự thêm một display kiểu `Marker`.
5. Kiểm tra ô chọn cạnh `Marker` đang bật và `Status: Ok`.

Nếu không thấy hình:

- Trong `Global Options`, đặt `Fixed Frame` là `base_link` hoặc `world`.
- Dùng chuột giữa/scroll để di chuyển và zoom camera tới vị trí robot.
- Với vòng tròn, chỉ thêm `/circle_path` sau khi node đã bắt đầu; node được giữ chạy để marker còn tồn tại.
- Kiểm tra topic ở terminal:

  ```bash
  source /opt/ros/jazzy/setup.bash
  ROS_DOMAIN_ID=56 ros2 topic list | grep -E 'write_m_path|circle_path'
  ```

## Lưu ý

- Chỉ chạy **một launch tại một thời điểm**.
- Không chạy thêm `ros2 run ...` khi một launch đang chạy.
- Trước khi chạy lại, nhấn `Ctrl+C` và chờ Gazebo đóng hoàn toàn.
- Kiểm tra controller khi cần:

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=56 ros2 control list_controllers -c /controller_manager
```

Kết quả đúng:

```text
joint_state_broadcaster     active
joint_trajectory_controller active
```

## Chỉnh tham số

- Chữ M: `config/write_m.yaml` (`letter_width`, `letter_height`, `top_clearance`, `pen_lift`, `eef_step`, `workspace_search_step`). Kích thước mặc định là 12 × 10 cm; chữ được vẽ bằng một đường Cartesian liên tục qua 5 waypoint theo thứ tự chân phải → đỉnh phải → điểm lõm giữa → đỉnh trái → chân trái; node dùng pose khởi đầu khuỷu gập và sàng lọc waypoint để tránh singularity; TCP chỉ nâng khi tiếp cận/rời điểm đầu/cuối, không nhấc giữa chữ. Bước nội suy mặc định là 3 mm; node dò quanh vị trí chuẩn theo bước 5 cm nếu điểm chuẩn khó với tới.
- Vòng tròn: `config/circle.yaml` (`circle_radius`, `circle_points`, `top_clearance`, `eef_step`, `workspace_search_step`). Đường tròn mặc định bán kính 4 cm, cách pose up 16 cm ở mép trên, với 96 điểm; node bắt đầu từ pose khuỷu gập, vào vòng tại đỉnh rồi chạy theo chiều kim đồng hồ; nó kiểm tra IK liên tục cùng khoảng cách tới singularity trên toàn vòng trước khi chọn vị trí lân cận. MoveIt thực thi cả vòng hoặc chia thành các cung Cartesian ngắn nếu IK không theo được một trajectory dài.

Các đoạn Cartesian bật kiểm tra va chạm trong MoveIt. Planner được giới hạn 6 giây và 3 lần thử cho mỗi kế hoạch để tránh chờ lâu; nếu IK chỉ tính được một phần nét, node thực thi phần hợp lệ rồi tính tiếp từ trạng thái mới. Vị trí mới được chọn trước khi vẽ, nên marker RViz bám theo đúng vị trí robot thực hiện.

Sau khi sửa, build lại:

```bash
cd /home/minh/AI_Homework
colcon build --packages-select ur3_write_m --symlink-install
source install/setup.bash
```
