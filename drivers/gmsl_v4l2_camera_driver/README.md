# GMSL V4L2 Camera Driver

`gmsl_v4l2_camera_driver/GmslV4l2CameraRuntime` is an in-process
`device_manager::IDeviceRuntime` plugin for V4L2 GMSL cameras.

The runtime reads device parameters from Device Manager, opens the configured
V4L2 device, publishes RGB images as `sensor_msgs/msg/Image` with `bgr8`
encoding, publishes matching `sensor_msgs/msg/CameraInfo`, and reports health
through Device Manager events.

## Parameters

| Key | Default | Meaning |
| --- | --- | --- |
| `camera.basic.camera_name` | `device_id` | ROS namespace and node name |
| `camera.basic.frame_id` | `<camera_name>_link` | image and camera info frame |
| `device.interface.gmsl_device` | `/dev/video0` | V4L2 device path |
| `camera.driver.image_topic` | `image_raw` | image topic under camera namespace |
| `camera.driver.undistort_image` | `true` | publish undistorted images on `image_topic` when calibration is available |
| `camera.driver.undistort_focal_scale` | `0.85` | virtual focal-length scale used for undistorted output |
| `camera.driver.calibration_crop_y` | `-1` | source-row offset when adapting taller calibration data to the capture frame; `-1` keeps centered-crop inference |
| `camera.driver.publish_distorted_image` | `false` | also publish the original distorted image for debugging |
| `camera.driver.distorted_image_topic` | `image_raw_distorted` | original distorted image topic under camera namespace |
| `camera.driver.camera_info_topic` | `camera_info` | camera info topic under camera namespace |
| `camera.driver.serial_number_topic` | `serial_number` | serial number topic under camera namespace |
| `camera.driver.intrinsic_params` | `""` | JSON-like camera calibration string for `CameraInfo` |
| `camera.driver.read_otp_on_start` | `true` | read OX01F10 OTP on configure and prefer it over configured intrinsics |
| `camera.driver.otp_i2c_bus` | `-1` | I2C bus to read OTP from; must be set to a non-negative bus number to access OTP |
| `camera.driver.otp_i2c_address` | `0x36` | OX01F10 OTP I2C address |
| `camera.driver.otp_read_offset` | `0x10000` | OTP flash offset for the exported data block |
| `camera.driver.otp_read_size` | `8192` | OTP bytes to read |
| `camera.driver.image_resolution_width` | `1280` | capture width |
| `camera.driver.image_resolution_height` | `720` | capture height |
| `camera.driver.color_format` | `NV12` | V4L2 pixel format |
| `camera.driver.color_fps` | `10` | capture frame rate |
| `camera.driver.read_timeout_ms` | `1000` | frame read timeout |

Supported formats are `NV12`, `NV21`, `YUYV`, `UYVY`, `RGB8`, `BGR8`, and
`MONO8`.

## Runtime

```yaml
instances:
  - device_id: gmsl_cam
    device_type: camera
    runtime: gmsl_v4l2_camera_driver/GmslV4l2CameraRuntime
```

With `camera.basic.camera_name=gmsl_cam`, the runtime publishes:

- `/gmsl_cam/image_raw` as the undistorted image when calibration is available
- `/gmsl_cam/image_raw_distorted` when `camera.driver.publish_distorted_image=true`
- `/gmsl_cam/camera_info`
- `/gmsl_cam/serial_number`
- `/gmsl_cam/gmsl_cam/device_event`

`sensor_msgs/msg/CameraInfo` does not have a serial-number field. The runtime
therefore keeps `CameraInfo.header.frame_id` as the optical frame id, publishes
the serial number on `serial_number`, and also includes it in online
`DeviceEvent` values when configured.

When `camera.driver.undistort_image=true`, the runtime undistorts `image_topic`
before publishing. The published `camera_info` keeps the calibrated projection
matrix but uses zero distortion coefficients so downstream consumers do not
apply the same correction again. Set `camera.driver.publish_distorted_image` to
`true` only when the original distorted image is needed for field comparison.
`camera.driver.undistort_focal_scale` is not a factory calibration value; lower
values keep more field of view with stronger edge stretching, while higher
values reduce stretching with a narrower output view.

When calibration height is greater than capture height, set
`camera.driver.calibration_crop_y` to the first source row retained by the
capture path. For example, adapting 1280x960 calibration to a 1280x720 top crop
uses `0`. The default `-1` preserves the legacy centered-crop inference.

`camera.driver.intrinsic_params` accepts either explicit `k/r/p/d` arrays or
`fx/fy/cx/cy` plus distortion coefficients, for example:

```json
{
  "width": 1280,
  "height": 720,
  "distortion_model": "equidistant",
  "fx": 316.4589527674,
  "fy": 316.4021688498,
  "cx": 641.2050576828,
  "cy": 363.1482381048,
  "d": [0.0975673817, -0.0076456599, -0.0015543389, 0.000051554]
}
```

When `camera.driver.read_otp_on_start` is enabled, the runtime reads the Senyun
OX01F10 OTP block every time the device is configured, but only when
`camera.driver.otp_i2c_bus` is explicitly configured. A valid OTP block must
pass CRC32 for the Cam0 intrinsic block and the SN block. If OTP read or decode
fails, the driver keeps using `camera.driver.intrinsic_params` as a fallback so
the image topic can still start.
