# Device Manager

Generic lifecycle management for hardware drivers. The repository keeps device policy independent of ROS while providing ROS 2 contracts, a ROS-facing API adapter, and ROS runtime adapters.

## Packages

| Package | Responsibility |
| --- | --- |
| `device_manager_core` | Lifecycle types, runtime contract, per-device serial queue, hooks, events, and in-memory parameters |
| `device_manager_msgs` | ROS 2 messages and services for observation and operations |
| `device_manager_ros` | ROS executable wiring, ROS API adapter, standard ROS Lifecycle adapter, and process-hosted ROS-device runtime |
| `drivers/tws_battery_driver_ros2` | TWS battery protocol, topics, and in-process `IDeviceRuntime` plugin |
| `drivers/ch020_imu_driver` | HiPNUC CH0X0 IMU protocol, topics, and in-process `IDeviceRuntime` plugin with serial disconnect self-healing |
| `drivers/smit_ros_driver` | SMIT serial lidar protocol, topics, and in-process `IDeviceRuntime` plugin with serial disconnect self-healing |
| `drivers/oradar_ros_driver` | Oradar MS500 Ethernet lidar SDK, topics, and in-process `IDeviceRuntime` plugin with network reconnect self-healing |
| `drivers/pager100_lora_bridge` | PG100 pager LoRa serial bridge and in-process `IDeviceRuntime` plugin with serial disconnect self-healing |
| `drivers/gmsl_v4l2_camera_driver` | GMSL V4L2 camera capture and image topics as an in-process `IDeviceRuntime` plugin |

The detailed state, queue, event, and parameter semantics are documented in [docs/design.md](docs/design.md).

## Build And Test

ROS 2 Jazzy, `colcon`, and the package dependencies declared in each `package.xml` are required.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -y
colcon build --packages-select device_manager_core device_manager_msgs device_manager_ros tws_battery_driver_ros2 ch020_imu_driver smit_ros_driver oradar_ros_driver pager100_lora_bridge gmsl_v4l2_camera_driver
source install/setup.bash
colcon test --packages-select device_manager_core device_manager_msgs device_manager_ros tws_battery_driver_ros2 ch020_imu_driver smit_ros_driver oradar_ros_driver pager100_lora_bridge gmsl_v4l2_camera_driver
colcon test-result --verbose
```

## Quick Start

Build the core packages and one in-process demo runtime:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -y
colcon build --packages-select device_manager_core device_manager_msgs device_manager_ros ch020_imu_driver
source install/setup.bash
```

Start the example parameter API in one terminal:

```bash
python3 examples/parameter_api.py
```

If port 3000 is already in use, run it with `PORT=3001` and use the matching
`parameter_api_url`.

Start Device Manager in another terminal:

```bash
ros2 run device_manager_ros device_manager --ros-args \
  -p config_file:="$PWD/examples/device-config.yaml" \
  -p parameter_api_url:=http://127.0.0.1:3000/parameters
```

Configure and activate it from a third terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 lifecycle set /device_manager configure
ros2 lifecycle set /device_manager activate
ros2 service call /device_manager/get_devices device_manager_msgs/srv/GetDevices "{}"
```

The example keeps `demo_imu` disabled, so it does not open `/dev/ttyUSB0`.
Set `device.enable` to `true` in [examples/parameter_api.py](examples/parameter_api.py) after
connecting a compatible IMU and adjusting the serial port.

For a stricter local quality gate, build with warnings as errors:

```bash
export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
colcon build --packages-select device_manager_core device_manager_msgs device_manager_ros tws_battery_driver_ros2 ch020_imu_driver smit_ros_driver oradar_ros_driver pager100_lora_bridge gmsl_v4l2_camera_driver \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_CXX_FLAGS=-Werror
colcon test --packages-select device_manager_core device_manager_msgs device_manager_ros tws_battery_driver_ros2 ch020_imu_driver smit_ros_driver oradar_ros_driver pager100_lora_bridge gmsl_v4l2_camera_driver
colcon test-result --verbose
```

## Configuration

The ROS lifecycle node reads runtime metadata from `config_file` and loads each
device's authoritative parameters from the configured parameter API during
`configure`:

```yaml
instances:
  - device_id: top_camera
    device_type: camera
    node_name: top_camera_driver
```

`node_name` defaults to `device_id`. Device parameters, including
`device.enable`, must not be duplicated in this file.

`parameter_api_url` can point to any HTTP endpoint that returns device-scoped
parameters. Device Manager calls it with `scope=device&device_id=<id>` and
expects this shape:

```json
{
  "data": {
    "items": {
      "device.enable": {"type": "bool", "value": true},
      "camera.device": {"type": "string", "value": "/dev/video0"},
      "gain": {"type": "float64", "value": 1.0}
    }
  }
}
```

Supported parameter types are `bool`, `int32`, `int64`, `float32`, `float64`,
`string`, and `enum`. Entries without `value` are ignored, so category records
can be returned by the same endpoint.

## Lifecycle Semantics

Device Manager keeps the ROS 2 lifecycle shape but extends the graph with
`FINALIZED <-> UNCONFIGURED` so it can own the full runtime chain:

```text
FINALIZED -> UNCONFIGURED -> INACTIVE -> ACTIVE
ACTIVE -> INACTIVE -> UNCONFIGURED -> FINALIZED
```

`FINALIZED` means the permanent `IDeviceRuntime` adapter exists, but the
concrete ROS process or in-process driver instance is not materialized.
`FINALIZED -> UNCONFIGURED` starts the ROS process or constructs the driver
instance. `UNCONFIGURED -> FINALIZED` stops/reaps that process or destroys the
driver instance. No extra wrapper layer is introduced: each process-hosted or
in-process implementation is still just a concrete `IDeviceRuntime`.

`device.enable` gates only materialization from `FINALIZED`. When it is `true`,
the automatic hook may materialize and then advance toward `ACTIVE`; when it is
`false`, the device remains dematerialized. A running device is disabled by an
explicit reverse path to `FINALIZED`.

`runtime` is optional and defaults to `ros2_lifecycle`.

Use `ros2_process` for an ordinary ROS driver process that does not implement
ROS Lifecycle. The child driver owns device-specific health detection and
publishes `device_manager_msgs/msg/DeviceEvent`; Device Manager does not infer
health from the driver's business topics:

```yaml
instances:
  - device_id: external_camera
    device_type: camera
    runtime: ros2_process
    process:
      package: vendor_camera_driver
      executable: camera_node
      node_name: external_camera
      namespace: /perception
      parameter_mappings:
        device: camera.device
```

`ros2_process` owns `FINALIZED -> UNCONFIGURED` by starting the configured ROS
process and owns `UNCONFIGURED -> FINALIZED` by stopping and reaping the process
group. The driver reports device health through `<node_name>/device_event`;
the runtime forwards those events without inferring health from driver topics.
`process.namespace` is optional and launches the child process under the given
ROS namespace (`__ns` remap); it defaults to the empty namespace.
Use `process.parameter_mappings` to select and optionally rename parameters
loaded from the configured parameter API. Only explicitly mapped parameters are passed to the
child process.

Use a pluginlib lookup name as `runtime` to load a concrete `IDeviceRuntime`
implementation from a driver package without changing Device Manager:

```yaml
instances:
  - device_id: imu
    device_type: imu
    runtime: ch020_imu_driver/Ch020ImuRuntime
```

The plugin class derives directly from `IDeviceRuntime`, is default
constructible, and receives the complete `DeviceDefinition` through
`initialize()`. There is no plugin-specific runtime wrapper. The runtime owns
its execution model: it may host an in-process driver instance or manage a
separate ROS 2 driver process and node.

Start the platform with:

```bash
ros2 run device_manager_ros device_manager --ros-args \
  -p config_file:=/path/to/device-config.yaml \
  -p parameter_api_url:=http://127.0.0.1:3000/parameters \
  -p update_period_ms:=1000 \
  -p service_timeout_ms:=1000
```

The process starts `UNCONFIGURED`. Its lifecycle `configure` callback loads
parameters and creates runtimes; `activate` starts the manager hooks and ROS API;
`deactivate` stops them; `cleanup` releases all device runtimes.

The concrete drivers under `drivers/` are plugin libraries, not standalone
executables. The central `device_manager` owns their lifecycle and exposes the
only device-management ROS API. External drivers remain supported through
`ros2_process` and publish events on their own `<node_name>/device_event` topic.

## ROS API

With the default node name, the platform exposes:

| Name | Type | Meaning |
| --- | --- | --- |
| `/device_manager/devices` | `device_manager_msgs/msg/DeviceStateArray` | Reliable, transient-local device observations |
| `/device_manager/get_devices` | `device_manager_msgs/srv/GetDevices` | Read the current observations |
| `/device_manager/change_device_state` | `device_manager_msgs/srv/ChangeDeviceState` | Atomically enqueue one or more lifecycle transitions |
| `/device_manager/patch_parameters` | `device_manager_msgs/srv/PatchDeviceParameters` | Merge an in-memory patch and enqueue runtime-selected handling |

An `accepted=true` response means the submission entered the device queue. It does not mean that the driver completed the transition. Completion and failures are observed through device state, the latest event, and the last transition record.

Parameter patches update the manager's desired in-memory parameter set. A runtime may select the required lifecycle transitions; a runtime without a parameter planner performs no lifecycle transition. A successful queue submission does not prove the runtime applied the values; check the later transition result and observed state. A separate authoritative parameter service must persist changes before calling this API when persistence is required.

## Driver Integration

A driver is integrated by implementing the single [`IDeviceRuntime`](device_manager_core/include/device_manager_core/device_manager.hpp) contract:

- `initialize()` receives the complete device definition after dynamic construction.
- `state()` returns the authoritative lifecycle state.
- `request_transition()` performs one bounded transition and returns `SUCCESS`, `FAILURE`, or `ERROR`.
- `parameter_transitions()` receives the complete desired parameter map and the
  accumulated unhandled patch, then selects any required lifecycle transitions.
  Returning an empty sequence explicitly means no lifecycle reaction is needed.
- `materialized()`, `materialize()`, and `dematerialize()` describe concrete
  resource ownership. The framework still drives them through lifecycle
  transitions, not as a second control path.
- `set_event_handler()` reports generic diagnostic events without changing lifecycle state directly.
- `register_hook()` installs one fixed periodic policy hook during initialization.

A runtime can explicitly reuse `standard_parameter_transitions()` for full
lifecycle reconfiguration. Core does not interpret ordinary parameter names or
choose a reconfiguration policy.

Concrete runtimes that follow the standard graph can inherit `DeviceRuntimeBase`
and implement the actual edge methods instead of writing their own
`request_transition()` dispatcher.

A custom driver exports its concrete runtime with pluginlib:

```cpp
PLUGINLIB_EXPORT_CLASS(ch020_imu_driver::Ch020ImuRuntime, device_manager::IDeviceRuntime)
```

Its package calls
`pluginlib_export_plugin_description_file(device_manager_core plugins.xml)`.
Adding another custom runtime changes only that driver package and device
configuration; it does not add a driver-specific branch or dependency to the
Device Manager packages, regardless of whether it manages an in-process object
or a separate driver process.

The `IDeviceRuntime` object is the permanent management adapter for one logical
device. It must remain present even when its concrete process or driver object
does not. Process disappearance is first reported as an event; the framework
then reaches `FINALIZED` through the normal transition queue, and
`FINALIZED -> UNCONFIGURED` materializes the concrete instance again.

`Ros2LifecycleDriverAdapter` maps this contract to standard Lifecycle `get_state`, `change_state`, transition events, and `set_parameters`. Its target driver node may additionally publish `device_manager_msgs/msg/DeviceEvent` on `<node_name>/device_event`.

A process-hosted ROS runtime maps the same contract to an owned child process.
It is the default direction for newly onboarded ROS drivers that should be fully
started, stopped, enabled, and disabled by Device Manager.

For an in-process driver, implement `IDeviceRuntime` directly as the concrete
runtimes under `drivers/` do. Automatic recovery remains a framework behavior:
each periodic tick evaluates the hook again and may submit another attempt.

`drivers/tws_battery_driver_ros2` exports
`tws_battery_driver_ros2/TwsBatteryRuntime` and publishes `/battery/state` and
`/battery/status` while hosted by the central Device Manager.

`drivers/ch020_imu_driver` is the second concrete in-process runtime. Its
plugin `ch020_imu_driver/Ch020ImuRuntime` publishes `/imu/data`, `/imu/mag`, and
`/imu/pressure` while hosted by the central Device Manager. Its ROS-free driver core
owns the serial read thread and reconnects on read errors or data stalls, so a
runtime disconnect keeps the lifecycle `ACTIVE` and only reports
`ch020_imu.disconnected` / `ch020_imu.online` events. See the package
[README](drivers/ch020_imu_driver/README.md) for parameters and event
semantics.

`drivers/smit_ros_driver` is the third concrete in-process runtime. Its
plugin `smit_ros_driver/SmitLidarRuntime` publishes
`sensor_msgs/msg/LaserScan` while hosted by the central Device Manager. Its ROS-free driver core
owns the serial read thread and reconnects on read errors or data stalls, so a
runtime disconnect keeps the lifecycle `ACTIVE` and only reports
`smit_lidar.disconnected` / `smit_lidar.online` events. See the package
[README](drivers/smit_ros_driver/README.md) for parameters and event
semantics.

`drivers/oradar_ros_driver` is the Ethernet MS500 lidar runtime. Its plugin
`oradar_ros_driver/OradarLidarRuntime` publishes `sensor_msgs/msg/LaserScan`
from Oradar SDK frames while hosted by the central Device Manager. It reads
`device.interface.ip_address` / `device.interface.udp_port` and emits
`oradar_lidar.disconnected` / `oradar_lidar.online` events while keeping the
runtime lifecycle `ACTIVE` during network reconnect attempts.

`drivers/pager100_lora_bridge` is the fourth concrete in-process runtime. Its
plugin `pager100_lora_bridge/Pager100LoraRuntime` bridges
`/pager100/lora/rx` and `/pager100/lora/tx` while hosted by the central Device
Manager. Unlike the IMU and lidar runtimes it has no data-stall detection — an
idle LoRa link is normal — so connection state only reflects serial read/write
health. See the package [README](drivers/pager100_lora_bridge/README.md) for
parameters and event semantics.

`drivers/gmsl_v4l2_camera_driver` exports
`gmsl_v4l2_camera_driver/GmslV4l2CameraRuntime` and publishes
`/<camera_name>/image_raw`, `/<camera_name>/camera_info`, and
`/<camera_name>/<camera_name>/device_event` while hosted by the central Device
Manager. It opens `device.interface.gmsl_device`, captures V4L2 frames, converts
supported formats to `bgr8`, and reports capture failure through
`gmsl_v4l2.capture_failed` with `target_state=UNCONFIGURED`. See the package
[README](drivers/gmsl_v4l2_camera_driver/README.md) for parameters and event
semantics.
