# Bao Cao Phase 1 - mini_slam

## 1. Muc Tieu

Phase 1 cua `mini_slam` xay dung mot SLAM frontend co ban cho 2D LiDAR.
Node `mini_slam_node` nhan du lieu tu:

- `/scan`: `sensor_msgs/msg/LaserScan`
- `/odom`: `nav_msgs/msg/Odometry`

Sau do node:

1. Chuyen LaserScan thanh cac diem 2D trong he toa do local cua LiDAR.
2. Lay pose robot gan nhat tu odometry.
3. Ghep scan voi odom pose de tao `LocalizedScan`.
4. Luu history cac `LocalizedScan`.
5. Publish trajectory debug ra topic `/mini_slam/trajectory` dang `nav_msgs/msg/Path`.

Phase nay chua lam mapping, chua scan matching, chua loop closure. `estimated_pose`
hien tai duoc gan bang `odom_pose`.

## 2. Cau Truc Package

Package nam tai:

```text
source_slam/mini_slam/mini_slam
```

Cac file chinh:

```text
include/mini_slam/types.hpp
include/mini_slam/scan_processor.hpp
src/scan_processor.cpp
src/mini_slam_node.cpp
CMakeLists.txt
package.xml
```

## 3. File `types.hpp`

File:

```text
mini_slam/mini_slam/include/mini_slam/types.hpp
```

File nay dinh nghia cac kieu du lieu loi cua frontend.

### `Point2D`

```cpp
struct Point2D
{
  double x{0.0};
  double y{0.0};
};
```

`Point2D` dai dien cho mot diem 2D sau khi convert tu mot beam cua LaserScan.
Diem nay dang nam trong he toa do local cua LiDAR.

### `Pose2D`

```cpp
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};
```

`Pose2D` la pose phang cua robot:

- `x`: vi tri robot theo truc x.
- `y`: vi tri robot theo truc y.
- `theta`: yaw cua robot, don vi radian.

### `LocalizedScan`

```cpp
struct LocalizedScan
{
  vector<Point2D> local_points;
  Pose2D odom_pose;
  Pose2D estimated_pose;
  builtin_interfaces::msg::Time stamp;
  string frame_id;
};
```

`LocalizedScan` la cau truc tuong tu y tuong `LocalizedRangeScan` trong Karto.
No gom:

- `local_points`: cac diem 2D local convert tu LaserScan.
- `odom_pose`: pose lay tu `/odom`.
- `estimated_pose`: pose uoc luong cua SLAM. Phase 1 tam thoi bang `odom_pose`.
- `stamp`: timestamp cua LaserScan.
- `odom_stamp`: timestamp cua odometry duoc ghep voi LaserScan.
- `frame_id`: frame cua LaserScan.

## 4. File `scan_processor.hpp`

File:

```text
mini_slam/mini_slam/include/mini_slam/scan_processor.hpp
```

File nay khai bao class `ScanProcessor`, chua cac ham xu ly logic doc lap voi ROS node.

### `convertScanToLocalPoints`

```cpp
vector<Point2D> convertScanToLocalPoints(
  const sensor_msgs::msg::LaserScan & scan) const;
```

Ham nay nhan mot `LaserScan` va tra ve danh sach `Point2D` hop le.

### `poseFromOdometry`

```cpp
Pose2D poseFromOdometry(const nav_msgs::msg::Odometry & odom) const;
```

Ham nay lay `x`, `y` va yaw `theta` tu message `/odom`.

### `makeLocalizedScan`

```cpp
LocalizedScan makeLocalizedScan(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & odom_pose,
  const builtin_interfaces::msg::Time & odom_stamp) const;
```

Ham nay ghep LaserScan voi odom pose gan nhat de tao `LocalizedScan`.

## 5. File `scan_processor.cpp`

File:

```text
mini_slam/mini_slam/src/scan_processor.cpp
```

### LaserScan sang 2D points

Ham:

```cpp
vector<Point2D> ScanProcessor::convertScanToLocalPoints(
  const sensor_msgs::msg::LaserScan & scan) const
```

Moi phan tu trong `scan.ranges` la mot khoang cach `range` tai mot goc `angle`.
Goc beam dau tien la:

```cpp
double angle = scan.angle_min;
```

Sau moi beam:

```cpp
angle += scan.angle_increment;
```

Dieu kien range hop le:

```cpp
const bool valid_range =
  isfinite(range) &&
  range >= scan.range_min &&
  range <= scan.range_max;
```

Code bo qua:

- `NaN`
- `inf`
- range nho hon `range_min`
- range lon hon `range_max`

Cong thuc convert tu polar sang Cartesian:

```text
x = r * cos(theta)
y = r * sin(theta)
```

Trong code:

```cpp
points.push_back(Point2D{
  static_cast<double>(range) * cos(angle),
  static_cast<double>(range) * sin(angle)});
```

Ket qua tra ve la `vector<Point2D> points`.

### Odometry sang Pose2D

Ham:

```cpp
Pose2D ScanProcessor::poseFromOdometry(const nav_msgs::msg::Odometry & odom) const
```

Lay position:

```cpp
const auto & position = odom.pose.pose.position;
```

Lay quaternion:

```cpp
const auto & q = odom.pose.pose.orientation;
```

Cong thuc quaternion sang yaw:

```cpp
const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
theta = atan2(siny_cosp, cosy_cosp);
```

Tra ve:

```cpp
return Pose2D{
  position.x,
  position.y,
  atan2(siny_cosp, cosy_cosp)};
```

### Tao LocalizedScan

Ham:

```cpp
LocalizedScan ScanProcessor::makeLocalizedScan(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & odom_pose) const
```

No tao `LocalizedScan` nhu sau:

```cpp
localized_scan.local_points = convertScanToLocalPoints(scan);
localized_scan.odom_pose = odom_pose;
localized_scan.estimated_pose = odom_pose;
localized_scan.stamp = scan.header.stamp;
localized_scan.odom_stamp = odom_stamp;
localized_scan.frame_id = scan.header.frame_id;
```

Trong Phase 1:

```cpp
estimated_pose = odom_pose;
```

Vi chua co scan matching de refine pose.

## 6. File `mini_slam_node.cpp`

File:

```text
mini_slam/mini_slam/src/mini_slam_node.cpp
```

### Class `MiniSlamNode`

```cpp
class MiniSlamNode : public rclcpp::Node
```

Node ROS2 nay phu trach subscribe `/scan`, subscribe `/odom`, tao
`LocalizedScan`, luu history va publish trajectory.

### Constructor

```cpp
MiniSlamNode()
: Node("mini_slam_node")
```

Tao node ten `mini_slam_node`.

Subscribe `/scan`:

```cpp
scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
  "/scan",
  rclcpp::SensorDataQoS(),
  bind(&MiniSlamNode::onScan, this, placeholders::_1));
```

Subscribe `/odom`:

```cpp
odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
  "/odom",
  rclcpp::QoS(20),
  bind(&MiniSlamNode::onOdom, this, placeholders::_1));
```

Publisher trajectory:

```cpp
trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
  "/mini_slam/trajectory",
  rclcpp::QoS(10));
```

### Dong bo timestamp giua `/scan` va `/odom`

Node khong con lay odom moi nhat mot cach truc tiep cho moi scan. Thay vao do,
`onOdom()` luu cac pose odometry vao buffer:

```cpp
std::deque<StampedOdomPose> odom_history_;
```

Moi phan tu co:

```cpp
struct StampedOdomPose
{
  Pose2D pose;
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
};
```

Khi `/scan` den, node goi:

```cpp
findNearestOdomPose(msg->header.stamp)
```

Ham nay tim odom co `stamp` gan nhat voi `scan.header.stamp`. Sau do node tinh
do lech thoi gian:

```cpp
scan_odom_dt =
  abs(timeToSeconds(scan_stamp) - timeToSeconds(odom_stamp));
```

Neu `scan_odom_dt` lon hon tham so:

```cpp
max_scan_odom_time_delta
```

thi scan bi bo qua. Gia tri mac dinh la `0.1` giay.

So odom trong buffer duoc gioi han boi:

```cpp
max_odom_history_size
```

Gia tri mac dinh la `200` message.

### Callback `onOdom`

```cpp
void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
```

Moi khi nhan `/odom`, node them pose vao history:

```cpp
odom_history_.push_back(StampedOdomPose{
  scan_processor_.poseFromOdometry(*msg),
  msg->header.stamp,
  msg->header.frame_id.empty() ? "odom" : msg->header.frame_id});
```

Neu history vuot qua `max_odom_history_size_`, node xoa odom cu nhat:

```cpp
odom_history_.pop_front();
```

### Callback `onScan`

```cpp
void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
```

Khi nhan `/scan`, node tim odom gan timestamp scan nhat:

```cpp
const auto synced_odom = findNearestOdomPose(msg->header.stamp);
```

Neu khong co odom trong history:

```cpp
if (!synced_odom) {
  RCLCPP_WARN_THROTTLE(...);
  return;
}
```

Neu odom gan nhat lech thoi gian qua lon:

```cpp
if (scan_odom_dt > max_scan_odom_time_delta_) {
  RCLCPP_WARN(...);
  return;
}
```

Khi odom hop le ve thoi gian, tao `LocalizedScan`:

```cpp
LocalizedScan localized_scan =
  scan_processor_.makeLocalizedScan(*msg, synced_odom->pose, synced_odom->stamp);
```

Luu history:

```cpp
localized_scans_.push_back(localized_scan);
```

Them pose vao trajectory:

```cpp
appendTrajectoryPose(localized_scan);
```

Publish trajectory:

```cpp
trajectory_pub_->publish(trajectory_);
```

Log debug:

```cpp
RCLCPP_INFO(
  get_logger(),
  "scan beams=%zu valid_points=%zu odom_pose=(x=%.3f, y=%.3f, theta=%.3f) localized_scans=%zu",
  msg->ranges.size(),
  localized_scan.local_points.size(),
  pose.x,
  pose.y,
  pose.theta,
  localized_scans_.size());
```

Log hien thi:

- So beam goc cua scan: `msg->ranges.size()`
- So point hop le: `localized_scan.local_points.size()`
- Odom pose hien tai: `pose.x`, `pose.y`, `pose.theta`
- So `LocalizedScan` da luu: `localized_scans_.size()`

### Ham `appendTrajectoryPose`

```cpp
void appendTrajectoryPose(const LocalizedScan & localized_scan)
```

Cap nhat header cua path:

```cpp
trajectory_.header.stamp = localized_scan.stamp;
trajectory_.header.frame_id = odom_frame_id_;
```

Tao `PoseStamped`:

```cpp
geometry_msgs::msg::PoseStamped pose_stamped;
pose_stamped.header = trajectory_.header;
```

Gan position tu `estimated_pose`:

```cpp
pose_stamped.pose.position.x = localized_scan.estimated_pose.x;
pose_stamped.pose.position.y = localized_scan.estimated_pose.y;
pose_stamped.pose.position.z = 0.0;
```

Chuyen yaw sang quaternion:

```cpp
tf2::Quaternion orientation;
orientation.setRPY(0.0, 0.0, localized_scan.estimated_pose.theta);
pose_stamped.pose.orientation = tf2::toMsg(orientation);
```

Them pose vao `nav_msgs::msg::Path`:

```cpp
trajectory_.poses.push_back(pose_stamped);
```

Sau do `onScan()` publish:

```cpp
trajectory_pub_->publish(trajectory_);
```

## 7. Flow Du Lieu Tong The

```text
/odom
  -> MiniSlamNode::onOdom()
  -> ScanProcessor::poseFromOdometry()
  -> latest_odom_pose_
  -> odom_frame_id_

/scan
  -> MiniSlamNode::onScan()
      -> tim odom gan scan.header.stamp nhat trong odom_history_
      -> kiem tra scan_odom_dt <= max_scan_odom_time_delta_
      -> ScanProcessor::makeLocalizedScan()
       -> ScanProcessor::convertScanToLocalPoints()
       -> gan odom_pose
       -> gan estimated_pose = odom_pose
       -> gan stamp, odom_stamp va frame_id
  -> localized_scans_.push_back(localized_scan)
  -> appendTrajectoryPose(localized_scan)
  -> trajectory_pub_->publish(trajectory_)
  -> /mini_slam/trajectory
```

## 8. Build

Build bang symlink install:

```bash
cd /home/lenovo/VNX/SLAM/source_slam/mini_slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths mini_slam --packages-select mini_slam
```

Source workspace:

```bash
source install/setup.bash
```

## 9. Run

Chay node:

```bash
cd /home/lenovo/VNX/SLAM/source_slam/mini_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run mini_slam mini_slam_node
```

## 10. Test Khong Can Robot That

Terminal publish `/odom`:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /odom nav_msgs/msg/Odometry "
header:
  frame_id: odom
pose:
  pose:
    position:
      x: 1.0
      y: 2.0
      z: 0.0
    orientation:
      w: 1.0
"
```

Terminal publish `/scan`:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /scan sensor_msgs/msg/LaserScan "
header:
  frame_id: laser
angle_min: -1.57
angle_max: 1.57
angle_increment: 0.785
range_min: 0.1
range_max: 10.0
ranges: [1.0, 2.0, .nan, .inf, 0.05, 3.0]
"
```

Neu node chay dung, log se co dang:

```text
scan beams=6 valid_points=3 odom_pose=(x=1.000, y=2.000, theta=0.000) localized_scans=...
```

Kiem tra trajectory:

```bash
ros2 topic echo /mini_slam/trajectory
```

## 11. Debug Bang RViz

Chay:

```bash
rviz2
```

Trong RViz:

- Fixed Frame: `odom`
- Add display: `Path`
- Topic: `/mini_slam/trajectory`

Trajectory hien thi la duong di dua tren odometry, vi Phase 1 chua toi uu pose.

## 12. Han Che Cua Phase 1

Phase 1 hien tai co cac gioi han:

- Chua co scan matching.
- Chua co occupancy grid map.
- Chua co pose graph.
- Chua co loop closure.
- Chua transform point tu laser frame sang base/odom/map.
- `estimated_pose` dang bang `odom_pose`.
- `localized_scans_` tang mai theo thoi gian, chua gioi han bo nho.
- Viec gan `/scan` voi `/odom` chi dung odom gan nhat da nhan, chua noi suy theo timestamp.

## 13. Huong Phase 2

Phase 2 nen tap trung vao scan matching va tao map co ban:

1. Transform `local_points` sang global frame dua tren pose robot.
2. Them scan matching de refine `estimated_pose`.
3. Co the bat dau voi ICP hoac correlative scan matching don gian.
4. Tao occupancy grid map va publish `/mini_slam/map`.
5. Them keyframe de khong luu moi scan.
6. Dong bo `/scan` va `/odom` tot hon bang timestamp hoac `message_filters`.
7. Sau do moi tien toi pose graph va loop closure.
