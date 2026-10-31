// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <stdexcept>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

int main() {
  std::promise<gz::msgs::Image> image_promise;
  std::promise<gz::msgs::CameraInfo> info_promise;
  auto image = image_promise.get_future();
  auto info = info_promise.get_future();
  std::atomic<bool> got_image{false}, got_info{false};
  gz::transport::Node node;
  if (!node.Subscribe("/image", std::function<void(const gz::msgs::Image &)>(
          [&](const gz::msgs::Image &value) {
            if (!got_image.exchange(true)) image_promise.set_value(value);
          })) ||
      !node.Subscribe("/info", std::function<void(const gz::msgs::CameraInfo &)>(
          [&](const gz::msgs::CameraInfo &value) {
            if (!got_info.exchange(true)) info_promise.set_value(value);
          }))) throw std::runtime_error("subscribe failed");
  std::cout << "ready" << std::endl;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  if (image.wait_until(deadline) != std::future_status::ready ||
      info.wait_until(deadline) != std::future_status::ready)
    throw std::runtime_error("receive timeout");
  const auto frame = image.get();
  const auto calibration = info.get();
  if (frame.width() != 64 || frame.height() != 64 || frame.step() != 192 ||
      frame.data().size() != 64 * 64 * 3 ||
      frame.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8 ||
      frame.header().stamp().sec() != calibration.header().stamp().sec() ||
      frame.header().stamp().nsec() != calibration.header().stamp().nsec() ||
      frame.header().data_size() != 2 || frame.header().data(0).key() != "frame_id" ||
      frame.header().data(0).value(0) != "camera_frame" ||
      frame.header().data(1).key() != "seq" || frame.header().data(1).value(0) != "0" ||
      calibration.width() != 64 || calibration.height() != 64 ||
      calibration.header().data_size() != 1 ||
      calibration.header().data(0).value(0) != "camera_frame")
    throw std::runtime_error("camera message mismatch");
  const auto offset = (32 * 64 + 32) * 3;
  if (static_cast<unsigned char>(frame.data()[offset]) != 255 ||
      frame.data()[offset + 1] != 0 || frame.data()[offset + 2] != 0)
    throw std::runtime_error("camera pixel mismatch");
  std::cout << frame.header().stamp().sec() << ' ' << frame.header().stamp().nsec() << std::endl;
}
