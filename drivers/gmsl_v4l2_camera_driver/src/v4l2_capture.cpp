#include "gmsl_v4l2_camera_driver/gmsl_v4l2_camera_runtime.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace gmsl_v4l2_camera_driver
{
namespace
{

int xioctl(int fd, unsigned long request, void * arg)
{
  int result = 0;
  do {
    result = ::ioctl(fd, request, arg);
  } while (result == -1 && errno == EINTR);
  return result;
}

std::uint32_t fourcc_from_format(const std::string & format)
{
  if (format == "NV12") {
    return V4L2_PIX_FMT_NV12;
  }
  if (format == "NV21") {
    return V4L2_PIX_FMT_NV21;
  }
  if (format == "YUYV" || format == "YUY2") {
    return V4L2_PIX_FMT_YUYV;
  }
  if (format == "UYVY") {
    return V4L2_PIX_FMT_UYVY;
  }
  if (format == "RGB8") {
    return V4L2_PIX_FMT_RGB24;
  }
  if (format == "BGR8") {
    return V4L2_PIX_FMT_BGR24;
  }
  if (format == "MONO8" || format == "GRAY8" || format == "GREY") {
    return V4L2_PIX_FMT_GREY;
  }
  return 0;
}

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

std::size_t effective_bytesused(std::uint32_t bytesused, std::size_t length)
{
  if (bytesused == 0U || bytesused > length) {
    return length;
  }
  return bytesused;
}

}  // namespace

namespace detail
{

v4l2_buf_type select_capture_buffer_type(
  std::uint32_t capabilities,
  std::uint32_t device_caps)
{
  const auto effective_caps =
    (capabilities & V4L2_CAP_DEVICE_CAPS) != 0U ? device_caps : capabilities;
  if ((effective_caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0U) {
    return V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  }
  if ((effective_caps & V4L2_CAP_VIDEO_CAPTURE) != 0U) {
    return V4L2_BUF_TYPE_VIDEO_CAPTURE;
  }
  return static_cast<v4l2_buf_type>(0);
}

}  // namespace detail

V4l2CaptureBackend::~V4l2CaptureBackend()
{
  close();
}

bool V4l2CaptureBackend::configure(
  const GmslV4l2CameraConfig & config,
  std::string & error_message)
{
  close();
  const auto fourcc = fourcc_from_format(config.pixel_format);
  if (fourcc == 0U) {
    error_message = "unsupported V4L2 pixel format: " + config.pixel_format;
    return false;
  }
  fd_ = ::open(config.video_device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error_message = errno_message("open " + config.video_device);
    return false;
  }

  v4l2_capability capability {};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &capability) < 0) {
    error_message = errno_message("VIDIOC_QUERYCAP");
    close();
    return false;
  }
  buffer_type_ =
    detail::select_capture_buffer_type(capability.capabilities, capability.device_caps);
  if (buffer_type_ == static_cast<v4l2_buf_type>(0)) {
    error_message = "V4L2 device does not support capture";
    close();
    return false;
  }

  v4l2_format format {};
  format.type = buffer_type_;
  if (buffer_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
    format.fmt.pix_mp.width = static_cast<std::uint32_t>(config.width);
    format.fmt.pix_mp.height = static_cast<std::uint32_t>(config.height);
    format.fmt.pix_mp.pixelformat = fourcc;
    format.fmt.pix_mp.field = V4L2_FIELD_NONE;
  } else {
    format.fmt.pix.width = static_cast<std::uint32_t>(config.width);
    format.fmt.pix.height = static_cast<std::uint32_t>(config.height);
    format.fmt.pix.pixelformat = fourcc;
    format.fmt.pix.field = V4L2_FIELD_NONE;
  }
  if (xioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
    error_message = errno_message("VIDIOC_S_FMT");
    close();
    return false;
  }

  v4l2_streamparm streamparm {};
  streamparm.type = buffer_type_;
  streamparm.parm.capture.timeperframe.numerator = 1U;
  streamparm.parm.capture.timeperframe.denominator = static_cast<std::uint32_t>(config.fps);
  static_cast<void>(xioctl(fd_, VIDIOC_S_PARM, &streamparm));

  v4l2_requestbuffers request {};
  request.count = 4U;
  request.type = buffer_type_;
  request.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &request) < 0 || request.count < 2U) {
    error_message = errno_message("VIDIOC_REQBUFS");
    close();
    return false;
  }

  buffers_.resize(request.count);
  for (std::uint32_t index = 0; index < request.count; ++index) {
    v4l2_buffer buffer {};
    v4l2_plane planes[VIDEO_MAX_PLANES] {};
    buffer.type = buffer_type_;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    if (buffer_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      buffer.m.planes = planes;
      buffer.length = VIDEO_MAX_PLANES;
    }
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
      error_message = errno_message("VIDIOC_QUERYBUF");
      close();
      return false;
    }

    const auto plane_count =
      buffer_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ? buffer.length : 1U;
    buffers_[index].planes.reserve(plane_count);
    for (std::uint32_t plane_index = 0; plane_index < plane_count; ++plane_index) {
      const auto length =
        buffer_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ?
        planes[plane_index].length : buffer.length;
      const auto offset =
        buffer_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ?
        planes[plane_index].m.mem_offset : buffer.m.offset;
      void * start = ::mmap(
        nullptr,
        length,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd_,
        static_cast<off_t>(offset));
      if (start == MAP_FAILED) {
        error_message = errno_message("mmap");
        close();
        return false;
      }
      buffers_[index].planes.push_back({start, length});
    }
  }

  for (std::uint32_t index = 0; index < buffers_.size(); ++index) {
    v4l2_buffer buffer {};
    v4l2_plane planes[VIDEO_MAX_PLANES] {};
    buffer.type = buffer_type_;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    if (buffer_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      buffer.m.planes = planes;
      buffer.length = static_cast<std::uint32_t>(buffers_[index].planes.size());
    }
    if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
      error_message = errno_message("VIDIOC_QBUF");
      close();
      return false;
    }
  }

  v4l2_buf_type type = buffer_type_;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    error_message = errno_message("VIDIOC_STREAMON");
    close();
    return false;
  }
  pixel_format_ = config.pixel_format;
  width_ = config.width;
  height_ = config.height;
  error_message.clear();
  return true;
}

std::optional<CapturedFrame> V4l2CaptureBackend::read_frame(
  std::chrono::milliseconds timeout,
  std::string & error_message)
{
  if (fd_ < 0) {
    error_message = "V4L2 device is not open";
    return std::nullopt;
  }

  pollfd descriptor {fd_, POLLIN, 0};
  const int ready = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
  if (ready == 0) {
    error_message = "capture timeout";
    return std::nullopt;
  }
  if (ready < 0) {
    error_message = errno_message("poll");
    return std::nullopt;
  }

  v4l2_buffer buffer {};
  v4l2_plane planes[VIDEO_MAX_PLANES] {};
  buffer.type = buffer_type_;
  buffer.memory = V4L2_MEMORY_MMAP;
  if (buffer_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
    buffer.m.planes = planes;
    buffer.length = VIDEO_MAX_PLANES;
  }
  if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
    error_message = errno_message("VIDIOC_DQBUF");
    return std::nullopt;
  }
  if (buffer.index >= buffers_.size()) {
    error_message = "V4L2 returned an out-of-range buffer index";
    return std::nullopt;
  }

  CapturedFrame frame;
  frame.width = width_;
  frame.height = height_;
  frame.pixel_format = pixel_format_;
  if (buffer_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
    const auto plane_count = std::min<std::uint32_t>(
      buffer.length, static_cast<std::uint32_t>(buffers_[buffer.index].planes.size()));
    for (std::uint32_t plane_index = 0; plane_index < plane_count; ++plane_index) {
      const auto & plane = buffers_[buffer.index].planes[plane_index];
      const auto * begin = static_cast<const std::uint8_t *>(plane.start);
      const auto bytesused = effective_bytesused(planes[plane_index].bytesused, plane.length);
      frame.data.insert(frame.data.end(), begin, begin + bytesused);
    }
  } else {
    const auto & plane = buffers_[buffer.index].planes.front();
    const auto * begin = static_cast<const std::uint8_t *>(plane.start);
    const auto bytesused = effective_bytesused(buffer.bytesused, plane.length);
    frame.data.assign(begin, begin + bytesused);
  }

  if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
    error_message = errno_message("VIDIOC_QBUF");
    return std::nullopt;
  }
  error_message.clear();
  return frame;
}

void V4l2CaptureBackend::close()
{
  if (fd_ >= 0) {
    v4l2_buf_type type = buffer_type_;
    static_cast<void>(xioctl(fd_, VIDIOC_STREAMOFF, &type));
  }
  for (auto & buffer : buffers_) {
    for (auto & plane : buffer.planes) {
      if (plane.start != nullptr && plane.start != MAP_FAILED) {
        ::munmap(plane.start, plane.length);
      }
    }
  }
  buffers_.clear();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace gmsl_v4l2_camera_driver
