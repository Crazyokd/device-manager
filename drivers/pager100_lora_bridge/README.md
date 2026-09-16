# PG100 LoRa ROS2 驱动

这个包提供 PG100 呼叫器车端 LoRa 模组的串口桥和 pluginlib runtime。

串口读线程与重连状态机在 `Pager100LoraDriver`（ROS-free）中，`0x7E` SOF + 转义 + CRC16 帧编解码在 `SerialFrameDecoder` / `encode_frame`（ROS-free）中。生命周期和 `/pager100/lora/rx`、`/pager100/lora/tx` 由 `Pager100LoraRuntime` 持有并交给中央 Device Manager 编排。该包不提供独立可执行文件；中央 Device Manager 按 `pager100_lora_bridge/Pager100LoraRuntime` 加载插件。

## 1. 功能说明

- 从串口读 PG100 字节流，按 `0x7E` SOF 帧切分（转义字节 `0x7D ^ 0x20`，CRC16 Modbus 低字节先行），仅透传数据帧（frame_type `0x01`），payload 上限 64 字节。
- 上行：串口数据帧 → `/pager100/lora/rx`；下行：`/pager100/lora/tx` → 编码后写串口（sequence 逐帧递增）。
- 串口断线自愈：运行期读/写错误时关闭串口并按退避间隔重开，恢复后继续透传，进程不退出。
- 设备管理观测和操作统一由中央 Device Manager 暴露。

## 2. Topics

| Topic | 方向 | 消息类型 | QoS |
| --- | --- | --- | --- |
| `/pager100/lora/rx` | 发布 | `std_msgs/msg/UInt8MultiArray` | 10 |
| `/pager100/lora/tx` | 订阅 | `std_msgs/msg/UInt8MultiArray` | 10 |

## 3. 参数

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `device.interface.serial_port` | `/dev/ttyS6` | 串口设备路径 |
| `device.interface.serial_baudrate` | `115200` | 波特率（9600 / 115200 / 460800 / 921600） |
| `read_chunk_size` | `128` | 单次 read 缓冲区大小 |
| `reconnect_backoff_ms` | `1000` | 断线后重开串口的退避间隔 |
| `device.enable` | `true` | 为 `false` 时设备保持 `FINALIZED` |

## 4. 生命周期与自愈

Lora runtime 初始为 `FINALIZED`。启用时固定 hook 会按 `FINALIZED -> UNCONFIGURED -> INACTIVE -> ACTIVE` 推进；`configure` 打不开串口时返回错误并上报 `pager100.connect_failed` 事件（目标状态 `UNCONFIGURED`），由 hook 周期性重试。

运行期读/写错误判定为断线：关闭串口、上报 `pager100.disconnected`（kError，目标状态 `ACTIVE`，不触发拆栈），读线程按 `reconnect_backoff_ms` 退避重开，成功后上报 `pager100.online`。事件来源为 `pager100_lora_bridge`。

注意：LoRa 链路空闲是常态（无人呼叫时无流量），连接状态只依据串口读写健康判断，不做数据停滞超时判定——无法区分“模组故障”和“无人呼叫”。

## 5. 事件契约

| 事件 | level | target_state | 含义 |
| --- | --- | --- | --- |
| `pager100.connect_failed` | kError | `UNCONFIGURED` | configure 打不开串口，hook 周期重试 |
| `pager100.disconnected` | kError | `ACTIVE` | 运行期读/写错误，仅更新观测，驱动内部重连 |
| `pager100.online` | kOk | - | 串口重开成功 |
