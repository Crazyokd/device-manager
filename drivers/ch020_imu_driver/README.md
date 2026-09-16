# CH020 IMU ROS2 驱动

这个包提供 HiPNUC CH0X0 系列 IMU 的串口驱动和 pluginlib runtime。

串口读线程与重连状态机在 `Ch020ImuDriver` 中，生命周期和 topic 输出由 `Ch020ImuRuntime` 持有并交给中央 Device Manager 队列/Hook 编排。该包不提供独立可执行文件；中央 Device Manager 按 `ch020_imu_driver/Ch020ImuRuntime` 加载插件。

## 1. 功能说明

- 从串口读取 HiPNUC CH0X0 字节流，按 `0x91 IMUSOL` 帧（手册 §6）解码。
- 发布 `/imu/data`（`sensor_msgs/msg/Imu`）、`/imu/mag`（`sensor_msgs/msg/MagneticField`，仅磁场非零时）、`/imu/pressure`（`sensor_msgs/msg/FluidPressure`，仅气压非零时）。
- 串口断线自愈：运行期读错误（USB 串口拔出）或数据停滞超时（板载 UART 拔线只表现为无数据）时，关闭串口并按退避间隔重开，恢复后继续发布，进程不退出。
- 设备管理观测和操作统一由中央 Device Manager 暴露。

## 2. Topics

### 发布

| Topic | 消息类型 | QoS |
| --- | --- | --- |
| `/imu/data` | `sensor_msgs/msg/Imu` | 10 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 10 |
| `/imu/pressure` | `sensor_msgs/msg/FluidPressure` | 10 |

## 3. 参数

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `device.interface.serial_port` | `/dev/ttyS7` | 串口设备路径 |
| `device.interface.serial_baudrate` | `115200` | 波特率（9600 / 115200 / 460800 / 921600） |
| `imu.frame_id` | `imu_link` | 发布消息的 `header.frame_id` |
| `read_chunk_size` | `256` | 单次 read 缓冲区大小 |
| `imu.use_hardware_time` | `false` | 为 `true` 时用硬件时间戳 + 最小偏移量算法换算 stamp |
| `data_timeout_ms` | `2000` | 超过该时长未解出任何帧判定为断线 |
| `reconnect_backoff_ms` | `1000` | 断线后重开串口的退避间隔 |
| `device.enable` | `true` | 为 `false` 时设备保持 `FINALIZED` |

## 4. 生命周期与自愈

IMU runtime 初始为 `FINALIZED`。启用时固定 hook 会按 `FINALIZED -> UNCONFIGURED -> INACTIVE -> ACTIVE` 推进；`configure` 打不开串口时返回错误并上报 `ch020_imu.connect_failed` 事件（目标状态 `UNCONFIGURED`），由 hook 周期性重试。

`ACTIVE` 状态下读线程持续解码；出现读错误或超过 `data_timeout_ms` 无帧时，驱动核关闭串口、按 `reconnect_backoff_ms` 退避重开，**生命周期保持 `ACTIVE` 不变**，只上报事件：

- `ch020_imu.disconnected`（kError）：判定断线时上报一次；
- `ch020_imu.online`（kOk）：重开成功、连接恢复时上报。

事件 source 均为 `ch020_imu_driver`。

## 5. 编译

在 device-manager 仓根目录执行：

```bash
colcon build --packages-select device_manager_core device_manager_msgs device_manager_ros ch020_imu_driver
source install/setup.bash
```

## 6. 加载

在中央 Device Manager 的设备注册中使用 runtime
`ch020_imu_driver/Ch020ImuRuntime`。设备参数由配置的参数服务提供。
