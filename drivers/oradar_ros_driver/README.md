# oradar_ros_driver

这个包提供 Oradar MS500 网口激光雷达驱动和 pluginlib runtime。驱动作为
device-manager 进程内 `IDeviceRuntime` 运行，不提供独立 ROS 可执行文件。

## Device Manager 集成

在中央 Device Manager 的设备注册中使用 runtime：

```yaml
instances:
  - device_id: front_lidar
    device_type: lidar
    runtime: oradar_ros_driver/OradarLidarRuntime
```

设备参数由配置的参数服务提供。常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device.interface.ip_address` | `192.168.1.100` | MS500 雷达 IP |
| `device.interface.udp_port` | `2007` | MS500 UDP 端口 |
| `lidar.topic` | 按设备 ID 推导 | 前雷达默认 `scan`，后雷达默认 `scan_rear` |
| `lidar.frame_id` | 按设备 ID 推导 | 前雷达 `right_front_laser_link`，后雷达 `left_behind_laser_link` |
| `lidar.scan_frequency` | `15` | MS500 旋转频率 |
| `lidar.scan_angle_min` | `-135` | LaserScan 角度下限（度） |
| `lidar.scan_angle_max` | `135` | LaserScan 角度上限（度） |
| `lidar.range_min` | `0.05` | LaserScan 最小距离（米） |
| `lidar.range_max` | `30.0` | LaserScan 最大距离（米） |

## 事件

- `oradar_lidar.connect_failed`：配置阶段无法打开或连接雷达，目标状态为 `UNCONFIGURED`。
- `oradar_lidar.disconnected`：读帧或重连失败，runtime 保持 `ACTIVE` 并继续重连。
- `oradar_lidar.online`：成功读取到扫描帧。

事件 source 均为 `oradar_lidar_driver`。

## 构建测试

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select device_manager_core oradar_ros_driver
colcon test --packages-select oradar_ros_driver
colcon test-result --verbose
```
