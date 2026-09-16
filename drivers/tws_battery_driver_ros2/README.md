# TWS 电池 ROS2 驱动

这个包提供一个基于 `ROS2 + SocketCAN` 的 TWS 电池驱动，默认使用 `can1` 接口与 BMS 通信，并按 TWS Modbus 寄存器定义发布电池信息。

协议轮询在 `TwsBatteryDriver` 中，生命周期和 topic 发布由 `TwsBatteryRuntime` 持有并交给中央 Device Manager 队列/Hook 编排。该包不提供独立可执行文件；中央 Device Manager 按 `tws_battery_driver_ros2/TwsBatteryRuntime` 加载插件。

## 1. 功能说明

- 通过 `can1` 发送和接收 `Modbus RTU over CAN` 数据。
- 发布标准电池状态 topic：`/battery/state`
- 发布详细电池状态 topic：`/battery/status`
- 设备管理观测和操作统一由中央 Device Manager 暴露。

## 2. 协议依据

本驱动使用了以下协议信息：

- CAN 默认扩展帧 ID：`0x00000041`
- BMS 默认地址：`0x01`
- 功能码：
  - `0x03`：读保持寄存器
- CRC：标准 `Modbus CRC16`，发送顺序为 `CRC 低字节 -> CRC 高字节`
- 关键寄存器：
  - `0x9000`：BMS 工作状态
  - `0x9002~0x9003`：总电压，单位 `mV`
  - `0x9004~0x9005`：总电流，单位 `mA`
  - `0x9006`：最高单体电压，单位 `mV`
  - `0x9007`：最低单体电压，单位 `mV`
  - `0x9008`：最高单体温度，手册示例为 `66 -> 26°C`，因此按 `raw - 40` 转换
  - `0x9009`：最低单体温度
  - `0x900A`：最高板温
  - `0x900B`：最低板温
  - `0x900C~0x900D`：保护状态 bitmask
  - `0x9014~0x9015`：IO 状态
  - `0x9016~0x9025`：SN，ASCII
  - `0x9026`：软件版本
  - `0x9027`：硬件版本
  - `0x9028`：SOC
  - `0x9029`：SOH
  - `0x902A~0x902B`：剩余容量，单位 `mAh`
  - `0x902C~0x902D`：循环次数

## 3. Topics

### 3.1 发布

#### `/battery/state`

消息类型：`sensor_msgs/msg/BatteryState`

主要字段：

- `voltage`
- `current`
- `percentage`
- `temperature`
- `power_supply_status`

#### `/battery/status`

消息类型：`tws_battery_driver_ros2/msg/BmsStatus`

包含更完整的信息，例如：

- 工作状态与状态文本
- 总压、总流
- 单体最高/最低电压
- 电芯/板温
- `SOC / SOH / 剩余容量 / 循环次数`
- `protect_status`
- `active_protections`
- `io_status`
- `charge_mos_control / charge_mos_state`，仅表示当前 IO 状态
- `serial_number / software_version / hardware_version`

## 4. 参数

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `device.interface.can_name` | `can1` | SocketCAN 接口名 |
| `battery.can_id` | `65` | BMS CAN ID，即 `0x00000041` |
| `battery.use_extended_frame` | `true` | 是否使用 CAN 扩展帧 |
| `battery.device_address` | `1` | Modbus 从站地址 |
| `request_timeout_ms` | `200` | 单次请求超时 |
| `poll_interval_ms` | `1000` | 轮询周期 |
| `identity_poll_divider` | `30` | 每隔多少个周期读取一次 SN/版本 |
| `device.enable` | `true` | 为 `false` 时设备保持 `FINALIZED` |

## 5. 生命周期

电池 runtime 初始为 `FINALIZED`。启用时固定 hook 会按 `FINALIZED -> UNCONFIGURED -> INACTIVE -> ACTIVE` 推进。`ACTIVE` 状态下周期轮询 BMS；轮询失败会上报 `tws_battery.poll_failed` 事件，目标状态为 `UNCONFIGURED`，随后由 device-manager 队列执行 `ACTIVE -> INACTIVE -> UNCONFIGURED`，再按正常启动路径重试。

## 6. 编译

在工作空间根目录执行：

```bash
colcon build --packages-select tws_battery_driver_ros2
source install/setup.bash
```

## 7. 运行

### 7.1 启动 CAN

请先根据你的现场波特率配置 `can1`，例如 500K：

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 500000 dbitrate 500000 fd on
sudo ip link set can1 up
```

本项目当前已验证通过的现场配置为：

- 波特率：`500 kbit/s`
- CAN 帧类型：`扩展帧`
- BMS CAN ID：`0x00000041`
- BMS 地址：`0x01`

### 7.2 加载 runtime

在中央 Device Manager 的设备注册中使用 runtime
`tws_battery_driver_ros2/TwsBatteryRuntime`。设备参数由配置的参数服务提供。

## 8. 如何测试获取电池信息

下面给出一套推荐的自测流程，适合常规 ROS 2 工作空间环境。

### 8.1 加载工作空间环境

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 8.2 确认 `can1` 状态

先确认 CAN 接口已经起来：

```bash
ip -details link show can1
```

你至少要看到下面这些关键信息：

- `state UP`
- 波特率接近 `500000`
- 接口名是 `can1`

如果接口没有起来，先在宿主机或容器里完成 CAN 配置，再继续下面的步骤。

### 8.3 启动 Device Manager

通过整车 bringup 启动中央 Device Manager。启用设备后，runtime 会周期性读取电池寄存器。

### 8.4 查看是否成功获取电池信息

打开第二个终端，加载同一个工作空间环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

先确认 topic 已经出现：

```bash
ros2 topic list | grep battery
```

正常情况下应该能看到类似：

```bash
/battery/state
/battery/status
```

查看详细电池信息：

```bash
ros2 topic echo /battery/status
```

查看标准电池状态：

```bash
ros2 topic echo /battery/state
```

如果只想看一帧结果：

```bash
ros2 topic echo /battery/status --once
```

如果想看发布频率：

```bash
ros2 topic hz /battery/status
```

### 8.5 如何判断通信正常

如果通信正常，`/battery/status` 里通常会看到这些特征：

- `connected: true`
- `last_error: ''`
- `pack_voltage_v` 是大于 0 的实际电压
- `soc`、`soh`、`cycle_count` 等字段有合理数值

例如本项目在现场测试时读到过类似结果：

```yaml
connected: true
pack_voltage_v: 52.688
pack_current_a: -0.01
soc: 41.0
soh: 100.0
remain_capacity_mah: 6132.0
cycle_count: 3
```

### 8.6 如果通信失败怎么看

如果通信失败，最常见的现象是：

- `connected: false`
- `last_error: 等待 BMS 响应超时`

这通常说明节点已经在发请求，但 BMS 没有回包。优先检查：

- `can1` 是否真的接在电池总线上
- 波特率是否是 `500 kbit/s`
- 是否使用了扩展帧
- CAN ID 是否为 `0x00000041`
- 从站地址是否为 `0x01`
- 电池是否已经上电、唤醒

### 8.7 需要进一步排查时

可以直接抓总线报文：

```bash
candump can1
```

如果节点运行时能看到总线有报文变化，但 `/battery/status` 仍然是超时，就要重点检查：

- 收发 ID 是否一致
- 是否用了扩展帧而不是标准帧
- 现场协议是否和手册配置一致

## 9. 说明与假设

- 手册给出了 `CAN ID` 与 `Modbus` 帧格式，但没有单独描述更复杂的 CAN 分包规则。
- 为保证经典 CAN 8 字节帧可用，本驱动读取 32 位字段时采用“逐个 16 位寄存器读取后重组”的方式。
- 如果你的设备固件对寄存器值字节序或寄存器定义有定制差异，需要按现场协议调整。
