// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <stdexcept>
#include <optional>
#include <thread>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

int pair_mode(int argc, char **argv);
int cylinder_mode(int argc, char **argv);

int main(int argc, char **argv) {
  if (argc != 1) {
    if (argc == 2 && std::string(argv[1]) == "cylinder") return cylinder_mode(argc, argv);
    return pair_mode(argc, argv);
  }
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

// The optional pair mode consumes actual typed packets from two Rust camera
// owners. It does not run native Sensors, a Manager or a rendering backend.
namespace {
void pair_require(bool condition, const char *why) {
  if (!condition) throw std::runtime_error(why);
}
std::string pair_hex(const std::string &bytes) {
  constexpr char digits[] = "0123456789abcdef";
  std::string out; out.reserve(bytes.size() * 2);
  for (unsigned char byte : bytes) {
    out += digits[byte >> 4]; out += digits[byte & 15];
  }
  return out;
}
void pair_header(const gz::msgs::Header &header, unsigned owner, bool image) {
  pair_require(header.has_stamp(), "pair timestamp presence");
  pair_require(header.stamp().sec() == 0 && header.stamp().nsec() == 1000000,
               "actual first pair acquisition timestamp");
  pair_require(header.data_size() == (image ? 2 : 1), "pair header shape");
  const std::string frame = owner == 0 ? "rig_a/body/camera" : "rig_b/body/camera";
  pair_require(header.data(0).key() == "frame_id" && header.data(0).value_size() == 1
      && header.data(0).value(0) == frame, "distinct actual loaded pair frame");
  if (image) pair_require(header.data(1).key() == "seq"
      && header.data(1).value_size() == 1 && header.data(1).value(0) == "0",
      "fresh independent first image sequence");
}
void pair_image(const gz::msgs::Image &image, unsigned owner) {
  pair_require(image.has_header(), "pair image header");
  pair_header(image.header(), owner, true);
  pair_require(image.width() == 64 && image.height() == 64 && image.step() == 192
      && image.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8
      && image.data().size() == 64 * 64 * 3, "pair RGB dimensions/type/stride");
  const std::size_t pixel = (32 * 64 + 32) * 3;
  const auto r = static_cast<unsigned char>(image.data()[pixel]);
  const auto g = static_cast<unsigned char>(image.data()[pixel + 1]);
  const auto b = static_cast<unsigned char>(image.data()[pixel + 2]);
  pair_require((owner == 0 && r == 255 && g == 0 && b == 0)
      || (owner == 1 && r == 0 && g == 0 && b == 255), "distinct actual red/blue camera views");
}
void pair_info(const gz::msgs::CameraInfo &info, unsigned owner) {
  pair_require(info.has_header(), "pair info header"); pair_header(info.header(), owner, false);
  pair_require(info.width() == 64 && info.height() == 64 && info.has_intrinsics()
      && info.intrinsics().k_size() == 9 && info.has_projection()
      && info.projection().p_size() == 12 && info.has_distortion()
      && info.distortion().k_size() == 5 && info.rectification_matrix_size() == 9,
      "complete pair calibration shape");
  constexpr double k[] = {32,0,32,0,32,32,0,0,1};
  constexpr double p[] = {32,0,32,0,0,32,32,0,0,0,1,0};
  constexpr double rect[] = {1,0,0,0,1,0,0,0,1};
  for (int i=0;i<9;++i) pair_require(std::abs(info.intrinsics().k(i)-k[i]) < 1e-12
      && info.rectification_matrix(i) == rect[i], "pair intrinsic/rectification values");
  for (int i=0;i<12;++i) pair_require(std::abs(info.projection().p(i)-p[i]) < 1e-12,
      "pair projection values");
  for (double value : info.distortion().k()) pair_require(value == 0, "pair zero distortion");
}
struct PairReceived {
  std::mutex mutex;
  std::condition_variable changed;
  std::array<gz::msgs::Image,2> images;
  std::array<gz::msgs::CameraInfo,2> infos;
  std::array<bool,4> have{};
  bool accepting=true, failed=false;
};
int pair_cameras() {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  {
    auto state = std::make_shared<PairReceived>();
    gz::transport::Node node;
    for (unsigned owner=0;owner<2;++owner) {
      const std::string topic = owner == 0 ? "/m63/a/" : "/m63/b/";
      std::function<void(const gz::msgs::Image &)> image = [state,owner](const auto &value) noexcept {
        std::lock_guard<std::mutex> guard(state->mutex);
        if (!state->accepting || state->failed || state->have[2*owner]) return;
        try {
          pair_require(value.ByteSizeLong() <= 16384, "pair image copy cap");
          pair_image(value,owner); state->images[owner]=value; state->have[2*owner]=true;
        } catch (...) { state->failed=true; }
        state->changed.notify_all();
      };
      std::function<void(const gz::msgs::CameraInfo &)> info = [state,owner](const auto &value) noexcept {
        std::lock_guard<std::mutex> guard(state->mutex);
        if (!state->accepting || state->failed || state->have[2*owner+1]) return;
        try {
          pair_require(value.ByteSizeLong() <= 16384, "pair info copy cap");
          pair_info(value,owner); state->infos[owner]=value; state->have[2*owner+1]=true;
        } catch (...) { state->failed=true; }
        state->changed.notify_all();
      };
      pair_require(node.Subscribe(topic+"image",image) && node.Subscribe(topic+"camera_info",info),
          "four actual typed pair subscriptions");
    }
    std::cout << "ready pair" << std::endl;
    std::array<gz::msgs::Image,2> images;
    std::array<gz::msgs::CameraInfo,2> infos;
    {
      std::unique_lock<std::mutex> guard(state->mutex);
      pair_require(state->changed.wait_until(guard,deadline,[&]{return state->failed
          || (state->have[0]&&state->have[1]&&state->have[2]&&state->have[3]);}),
          "four actual pair receipt deadline");
      pair_require(!state->failed,"invalid actual pair packet");
      state->accepting=false; images=state->images; infos=state->infos;
    }
    for (unsigned owner=0;owner<2;++owner) {
      const std::string topic = owner == 0 ? "/m63/a/" : "/m63/b/";
      pair_require(node.Unsubscribe(topic+"image") && node.Unsubscribe(topic+"camera_info"),
          "pair subscriber retirement");
      std::cout << "IMAGE " << owner << ' ' << pair_hex(images[owner].SerializeAsString()) << '\n';
      std::cout << "INFO " << owner << ' ' << pair_hex(infos[owner].SerializeAsString()) << '\n';
    }
    std::cout << "PASS canonical pair cameras" << std::endl;
    pair_require(std::cout.good(), "actual pair stdout completion");
  }
  pair_require(std::chrono::steady_clock::now() < deadline, "pair completion/owned cleanup deadline");
  return 0;
}
}
int pair_mode(int argc, char **argv) {
  try {
    pair_require(argc == 2 && std::string(argv[1]) == "pair", "optional exact pair mode");
    return pair_cameras();
  } catch (const std::exception &error) {
    std::cerr << "native pair camera consumer: " << error.what() << '\n';
    return 1;
  }
}

// Additional canonical consumer, not a native rendering oracle. Keep the
// no-argument M5 consumer above unchanged inside the actual main function.
namespace {
struct CylinderMessages {
  std::mutex mutex;
  std::optional<gz::msgs::Image> image;
  std::optional<gz::msgs::CameraInfo> info;
  bool failed{false};
};
std::string hex_bytes(const std::string &value) {
  static constexpr char digits[]="0123456789abcdef";
  std::string result; result.reserve(value.size()*2);
  for (unsigned char byte:value) { result+=digits[byte>>4]; result+=digits[byte&15]; }
  return result;
}
void cylinder_header(const gz::msgs::Header &header, bool sequence) {
  if (!header.has_stamp() || header.stamp().sec()!=0 || header.stamp().nsec()!=1000000 ||
      header.data_size()!=(sequence?2:1) || header.data(0).key()!="frame_id" ||
      header.data(0).value_size()!=1 || header.data(0).value(0)!="rig/body/camera" ||
      (sequence && (header.data(1).key()!="seq" || header.data(1).value_size()!=1 || header.data(1).value(0)!="0")))
    throw std::runtime_error("cylinder header mismatch");
}
}

int cylinder_mode(int argc, char **argv) {
  try {
    if (argc!=2 || std::string(argv[1])!="cylinder") throw std::runtime_error("expected cylinder mode");
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(20);
    std::string frame_wire, info_wire;
    {
      auto messages=std::make_shared<CylinderMessages>();
      {
        gz::transport::Node node;
        if (!node.Subscribe("/m64/rgb/image",std::function<void(const gz::msgs::Image &)>([messages](const gz::msgs::Image &value) noexcept {
          try { std::lock_guard<std::mutex> lock(messages->mutex); if (value.ByteSizeLong()>16384) messages->failed=true; else if (!messages->image) messages->image=value; }
          catch (...) { std::lock_guard<std::mutex> lock(messages->mutex); messages->failed=true; }
        })) || !node.Subscribe("/m64/rgb/camera_info",std::function<void(const gz::msgs::CameraInfo &)>([messages](const gz::msgs::CameraInfo &value) noexcept {
          try { std::lock_guard<std::mutex> lock(messages->mutex); if (value.ByteSizeLong()>16384) messages->failed=true; else if (!messages->info) messages->info=value; }
          catch (...) { std::lock_guard<std::mutex> lock(messages->mutex); messages->failed=true; }
        }))) throw std::runtime_error("cylinder subscribe failed");
        std::cout<<"ready"<<std::endl;if (!std::cout) throw std::runtime_error("ready write failed");
        for (;;) {
          if (std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("cylinder receive timeout");
          {
            std::lock_guard<std::mutex> lock(messages->mutex);
            if (messages->failed) throw std::runtime_error("cylinder callback failure");
            if (messages->image && messages->info) {
              const auto &frame=*messages->image;const auto &info=*messages->info;
              if (!frame.has_header() || !info.has_header() || frame.width()!=64 || frame.height()!=64 || frame.step()!=192 ||
                  frame.data().size()!=12288 || frame.pixel_format_type()!=gz::msgs::PixelFormatType::RGB_INT8 || info.width()!=64 || info.height()!=64)
                throw std::runtime_error("cylinder dimensions mismatch");
              cylinder_header(frame.header(),true);cylinder_header(info.header(),false);
              const auto pixel=[&frame](int x,int y,int r,int g,int b) { const auto offset=(y*64+x)*3;return static_cast<unsigned char>(frame.data()[offset])==r && static_cast<unsigned char>(frame.data()[offset+1])==g && static_cast<unsigned char>(frame.data()[offset+2])==b; };
              if (!pixel(32,32,0,255,0) || !pixel(32,24,255,0,0) || !pixel(40,32,255,0,0) || !pixel(23,23,0,0,255) || !pixel(40,40,0,0,255))
                throw std::runtime_error("cylinder silhouette/occlusion mismatch");
              if (!frame.SerializeToString(&frame_wire) || !info.SerializeToString(&info_wire)) throw std::runtime_error("cylinder serialization failed");
              break;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      } // Subscription ownership is gone before accepting success.
    }
    std::cout<<"IMAGE "<<hex_bytes(frame_wire)<<'\n'<<"INFO "<<hex_bytes(info_wire)<<"\nPASS\n"<<std::flush;
    if (!std::cout || std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("cylinder completion failed");
    return 0;
  } catch (const std::exception &error) { std::cerr<<error.what()<<std::endl;return 1; }
}
