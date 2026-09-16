# SMIT 串口雷达 ROS2 驱动

这个包提供 SMIT 串口激光雷达驱动和 pluginlib runtime。

串口读线程与重连状态机在 `SmitLidarDriver`（ROS-free）中，0xA5 0x5A 帧切分 / CRC8 / 配置包与点云包解码在 `SmitProtocol`（ROS-free）中，点云到整圈扫描装配在 `SmitScanAssembler`（ROS-free）中。生命周期和 LaserScan 发布由 `SmitLidarRuntime` 持有并交给中央 Device Manager 编排。该包不提供独立可执行文件。

## 1. 功能说明

- 从串口读取 SMIT 雷达字节流，按 `0xA5 0x5A` 帧（大端长度域 + CRC8）切分。
- 解码配置包（msg_id 0x02：SN / 版本 / 转速 / 点频 / 温度 / 电压）与点云包（msg_id 0x01，每包最多 64 点，delta_angle ∉ [0.172, 0.188] 丢包）。
- 按角度窗口把点云拼成整圈 `sensor_msgs/msg/LaserScan` 发布（满圈按跨边界切圈，部分窗口按进出窗口切圈，时间戳回退半圈）。
- 串口断线自愈：运行期读错误或数据停滞超时时，关闭串口并按退避间隔重开，恢复后继续发布，进程不退出。
- 设备管理观测和操作统一由中央 Device Manager 暴露。

## 2. Topics

### 发布

| Topic | 消息类型 | QoS |
| --- | --- | --- |
| 由 `lidar.topic` 参数决定（默认按设备 ID 区分前后雷达） | `sensor_msgs/msg/LaserScan` | `rclcpp::SensorDataQoS()` |

## 3. 参数

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `device.interface.serial_port` | `""` | 串口设备路径 |
| `device.interface.serial_baudrate` | `921600` | 波特率（9600 / 115200 / 921600 / 2000000） |
| `lidar.topic` | 按设备 ID 推导 | LaserScan 发布话题名 |
| `lidar.frame_id` | 按设备 ID 推导 | 发布消息的 `header.frame_id` |
| `lidar.scan_angle_min` | `-180` | 扫描窗口下限（度） |
| `lidar.scan_angle_max` | `180` | 扫描窗口上限（度） |
| `read_chunk_size` | `1024` | 单次 read 缓冲区大小 |
| `data_timeout_ms` | `2000` | 超过该时长未解出任何帧判定为断线 |
| `reconnect_backoff_ms` | `1000` | 断线后重开串口的退避间隔 |
| `device.enable` | `true` | 为 `false` 时设备保持 `FINALIZED` |

## 4. 生命周期与自愈

Lidar runtime 初始为 `FINALIZED`。启用时固定 hook 会按 `FINALIZED -> UNCONFIGURED -> INACTIVE -> ACTIVE` 推进；`configure` 打不开串口时返回错误并上报 `smit_lidar.connect_failed` 事件（目标状态 `UNCONFIGURED`），由 hook 周期性重试。

`ACTIVE` 状态下读线程持续解码；出现读错误或超过 `data_timeout_ms` 无帧时，驱动核关闭串口、按 `reconnect_backoff_ms` 退避重开，**生命周期保持 `ACTIVE` 不变**，只上报事件：

- `smit_lidar.disconnected`（kError）：判定断线时上报一次；
- `smit_lidar.online`（kOk）：重开成功、连接恢复时上报。

事件 source 均为 `smit_lidar_driver`。

与旧驱动的差异：旧驱动 `get_config_data` 用信号量永久阻塞等配置包；新实现改为非阻塞状态位（`config_received`），等配置期间读线程照常运行，`stop()` 与 `data_timeout_ms` 都能正常打断。设备配置不随重连失效，断线恢复后无需等待新的配置包即可继续出 scan。

## 5. 编译

在 device-manager 仓根目录执行：

```bash
colcon build --packages-select device_manager_core device_manager_msgs device_manager_ros smit_ros_driver
source install/setup.bash
```

## 6. 加载

在中央 Device Manager 的设备注册中使用 runtime
`smit_ros_driver/SmitLidarRuntime`。设备参数由配置的参数服务提供。
