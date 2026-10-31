// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Calls pinned CameraSensor::PopulateInfo and PublishInfo without an Ogre scene.
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <gz/msgs/camera_info.pb.h>
#include <gz/sensors/CameraSensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Camera.hh>

class CameraInfoProbe : public gz::sensors::CameraSensor {
 public:
  using CameraSensor::AdvertiseInfo;
  using CameraSensor::PopulateInfo;
  using CameraSensor::PublishInfo;
};

int main(int argc, char **argv) {
  if (argc != 2) throw std::runtime_error("expected unique topic");
  sdf::Camera camera;
  camera.SetImageWidth(64);
  camera.SetImageHeight(64);
  camera.SetLensIntrinsicsFx(30);
  camera.SetLensIntrinsicsFy(31);
  camera.SetLensIntrinsicsCx(20);
  camera.SetLensIntrinsicsCy(21);
  camera.SetLensProjectionFx(30);
  camera.SetLensProjectionFy(31);
  camera.SetLensProjectionCx(20);
  camera.SetLensProjectionCy(21);
  camera.SetLensProjectionTx(0);
  camera.SetLensProjectionTy(0);
  CameraInfoProbe sensor;
  sensor.SetFrameId("frame");
  sensor.PopulateInfo(&camera);
  if (!sensor.AdvertiseInfo(argv[1])) throw std::runtime_error("advertise failed");
  std::promise<gz::msgs::CameraInfo> promise;
  auto received = promise.get_future();
  std::atomic<bool> done{false};
  gz::transport::Node node;
  if (!node.Subscribe(argv[1], std::function<void(const gz::msgs::CameraInfo &)>(
      [&](const gz::msgs::CameraInfo &message) {
        if (!done.exchange(true)) promise.set_value(message);
      }))) throw std::runtime_error("subscribe failed");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!sensor.HasInfoConnections()) {
    if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("discovery timeout");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  sensor.PublishInfo(std::chrono::seconds(4) + std::chrono::nanoseconds(5));
  if (received.wait_until(deadline) != std::future_status::ready) throw std::runtime_error("receive timeout");
  const auto message = received.get();
  if (!message.SerializeToOstream(&std::cout)) throw std::runtime_error("serialize failed");
}
