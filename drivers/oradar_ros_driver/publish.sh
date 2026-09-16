#!/bin/sh
# Emit the deb publish manifest for this driver as JSON on stdout.
set -eu

cd "$(dirname "$0")"

package_name="oradar-ros-driver"
archs="amd64 arm64"
depends="device-manager-msgs device-manager ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rclcpp ros-jazzy-sensor-msgs"

version="$(
  sed -n 's:.*<version>\(.*\)</version>.*:\1:p' package.xml | sed -n '1p'
)"
version="$(printf '%s' "$version" | tr -c '[:alnum:].+~:-' '-')"
[ -n "$version" ] || { echo "missing <version> in package.xml" >&2; exit 1; }
case "$version" in
  [0-9]*) ;;
  *) version="0~${version}" ;;
esac

python3 - "$package_name" "$version" "$archs" "$depends" <<'PY'
import json, sys
pkg, version, archs, depends = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
print(json.dumps({
    "package_name": pkg,
    "version": version,
    "archs": archs.split(),
    "depends": depends.split(),
    "description": "Oradar MS500 Ethernet lidar driver for ROS 2 Jazzy",
    "stage_dir": "install-oradar-ros-driver",
    "ros_packages": ["oradar_ros_driver"],
}, ensure_ascii=False))
PY
