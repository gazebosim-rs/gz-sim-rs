// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <array>
#include <utility>
#include <cmath>
#include <cstring>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <limits>
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
int sphere_mode(int argc, char **argv);
int ellipsoid_mode(int argc, char **argv);
int depth_primitives_mode(int argc, char **argv);
int depth_pair_mode(int argc, char **argv);
int mixed_pair_mode(int argc, char **argv);
int explicit_intrinsics_mode(int argc, char **argv);

int main(int argc, char **argv) {
  if (argc==3 && std::string(argv[1])=="explicit-intrinsics") return explicit_intrinsics_mode(argc, argv);
  if (argc==3 && std::string(argv[1])=="depth-pair") return depth_pair_mode(argc, argv);
  if (argc==3 && std::string(argv[1])=="mixed-pair") return mixed_pair_mode(argc, argv);
  if (argc != 1) {
    if (argc == 2 && std::string(argv[1]) == "depth-primitives") return depth_primitives_mode(argc, argv);
    if (argc == 2 && std::string(argv[1]) == "ellipsoid") return ellipsoid_mode(argc, argv);
    if (argc == 2 && std::string(argv[1]) == "sphere") return sphere_mode(argc, argv);
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


namespace {
struct SphereMessages {
  std::mutex mutex;
  std::optional<gz::msgs::Image> image;
  std::optional<gz::msgs::CameraInfo> info;
  bool failed{false};
};
std::string sphere_hex_bytes(const std::string &value) {
  static constexpr char digits[]="0123456789abcdef";
  std::string result; result.reserve(value.size()*2);
  for (unsigned char byte:value) { result+=digits[byte>>4]; result+=digits[byte&15]; }
  return result;
}
void sphere_header(const gz::msgs::Header &header, bool sequence) {
  if (!header.has_stamp() || header.stamp().sec()!=0 || header.stamp().nsec()!=1000000 ||
      header.data_size()!=(sequence?2:1) || header.data(0).key()!="frame_id" ||
      header.data(0).value_size()!=1 || header.data(0).value(0)!="rig/body/camera" ||
      (sequence && (header.data(1).key()!="seq" || header.data(1).value_size()!=1 || header.data(1).value(0)!="0")))
    throw std::runtime_error("sphere header mismatch");
}
}

int sphere_mode(int argc, char **argv) {
  try {
    if (argc!=2 || std::string(argv[1])!="sphere") throw std::runtime_error("expected sphere mode");
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(20);
    std::string frame_wire, info_wire;
    {
      auto messages=std::make_shared<SphereMessages>();
      {
        gz::transport::Node node;
        if (!node.Subscribe("/m66/rgb/image",std::function<void(const gz::msgs::Image &)>([messages](const gz::msgs::Image &value) noexcept {
          try { std::lock_guard<std::mutex> lock(messages->mutex); if (value.ByteSizeLong()>16384) messages->failed=true; else if (!messages->image) messages->image=value; }
          catch (...) { std::lock_guard<std::mutex> lock(messages->mutex); messages->failed=true; }
        })) || !node.Subscribe("/m66/rgb/camera_info",std::function<void(const gz::msgs::CameraInfo &)>([messages](const gz::msgs::CameraInfo &value) noexcept {
          try { std::lock_guard<std::mutex> lock(messages->mutex); if (value.ByteSizeLong()>16384) messages->failed=true; else if (!messages->info) messages->info=value; }
          catch (...) { std::lock_guard<std::mutex> lock(messages->mutex); messages->failed=true; }
        }))) throw std::runtime_error("sphere subscribe failed");
        std::cout<<"ready"<<std::endl;if (!std::cout) throw std::runtime_error("ready write failed");
        for (;;) {
          if (std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("sphere receive timeout");
          {
            std::lock_guard<std::mutex> lock(messages->mutex);
            if (messages->failed) throw std::runtime_error("sphere callback failure");
            if (messages->image && messages->info) {
              const auto &frame=*messages->image;const auto &info=*messages->info;
              if (!frame.has_header() || !info.has_header() || frame.width()!=64 || frame.height()!=64 || frame.step()!=192 ||
                  frame.data().size()!=12288 || frame.pixel_format_type()!=gz::msgs::PixelFormatType::RGB_INT8 || info.width()!=64 || info.height()!=64)
                throw std::runtime_error("sphere dimensions mismatch");
              sphere_header(frame.header(),true);sphere_header(info.header(),false);
              const auto pixel=[&frame](int x,int y,int r,int g,int b) { const auto offset=(y*64+x)*3;return static_cast<unsigned char>(frame.data()[offset])==r && static_cast<unsigned char>(frame.data()[offset+1])==g && static_cast<unsigned char>(frame.data()[offset+2])==b; };
              if (!pixel(32,32,0,255,0) || !pixel(32,26,255,0,0) || !pixel(38,32,255,0,0) || !pixel(24,24,0,0,255) || !pixel(39,39,0,0,255))
                throw std::runtime_error("sphere silhouette/occlusion mismatch");
              if (!frame.SerializeToString(&frame_wire) || !info.SerializeToString(&info_wire)) throw std::runtime_error("sphere serialization failed");
              break;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      } // Subscription ownership is gone before accepting success.
    }
    std::cout<<"IMAGE "<<sphere_hex_bytes(frame_wire)<<'\n'<<"INFO "<<sphere_hex_bytes(info_wire)<<"\nPASS\n"<<std::flush;
    if (!std::cout || std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("sphere completion failed");
    return 0;
  } catch (const std::exception &error) { std::cerr<<error.what()<<std::endl;return 1; }
}


namespace {
struct EllipsoidMessages {
  std::mutex mutex;
  std::optional<gz::msgs::Image> image;
  std::optional<gz::msgs::CameraInfo> info;
  bool failed{false};
};
std::string ellipsoid_hex_bytes(const std::string &value) {
  static constexpr char digits[]="0123456789abcdef";
  std::string result; result.reserve(value.size()*2);
  for (unsigned char byte:value) { result+=digits[byte>>4]; result+=digits[byte&15]; }
  return result;
}
void ellipsoid_header(const gz::msgs::Header &header, bool sequence) {
  if (!header.has_stamp() || header.stamp().sec()!=0 || header.stamp().nsec()!=1000000 ||
      header.data_size()!=(sequence?2:1) || header.data(0).key()!="frame_id" ||
      header.data(0).value_size()!=1 || header.data(0).value(0)!="rig/body/camera" ||
      (sequence && (header.data(1).key()!="seq" || header.data(1).value_size()!=1 || header.data(1).value(0)!="0")))
    throw std::runtime_error("ellipsoid header mismatch");
}
}

int ellipsoid_mode(int argc, char **argv) {
  try {
    if (argc!=2 || std::string(argv[1])!="ellipsoid") throw std::runtime_error("expected ellipsoid mode");
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(20);
    std::string frame_wire, info_wire;
    {
      auto messages=std::make_shared<EllipsoidMessages>();
      {
        gz::transport::Node node;
        if (!node.Subscribe("/m67/rgb/image",std::function<void(const gz::msgs::Image &)>([messages](const gz::msgs::Image &value) noexcept {
          try { std::lock_guard<std::mutex> lock(messages->mutex); if (value.ByteSizeLong()>16384) messages->failed=true; else if (!messages->image) messages->image=value; }
          catch (...) { std::lock_guard<std::mutex> lock(messages->mutex); messages->failed=true; }
        })) || !node.Subscribe("/m67/rgb/camera_info",std::function<void(const gz::msgs::CameraInfo &)>([messages](const gz::msgs::CameraInfo &value) noexcept {
          try { std::lock_guard<std::mutex> lock(messages->mutex); if (value.ByteSizeLong()>16384) messages->failed=true; else if (!messages->info) messages->info=value; }
          catch (...) { std::lock_guard<std::mutex> lock(messages->mutex); messages->failed=true; }
        }))) throw std::runtime_error("ellipsoid subscribe failed");
        std::cout<<"ready"<<std::endl;if (!std::cout) throw std::runtime_error("ready write failed");
        for (;;) {
          if (std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("ellipsoid receive timeout");
          {
            std::lock_guard<std::mutex> lock(messages->mutex);
            if (messages->failed) throw std::runtime_error("ellipsoid callback failure");
            if (messages->image && messages->info) {
              const auto &frame=*messages->image;const auto &info=*messages->info;
              if (!frame.has_header() || !info.has_header() || frame.width()!=64 || frame.height()!=64 || frame.step()!=192 ||
                  frame.data().size()!=12288 || frame.pixel_format_type()!=gz::msgs::PixelFormatType::RGB_INT8 || info.width()!=64 || info.height()!=64)
                throw std::runtime_error("ellipsoid dimensions mismatch");
              ellipsoid_header(frame.header(),true);ellipsoid_header(info.header(),false);
              const auto pixel=[&frame](int x,int y,int r,int g,int b) { const auto offset=(y*64+x)*3;return static_cast<unsigned char>(frame.data()[offset])==r && static_cast<unsigned char>(frame.data()[offset+1])==g && static_cast<unsigned char>(frame.data()[offset+2])==b; };
              if (!pixel(32,32,0,255,0) || !pixel(42,32,255,0,0) || !pixel(32,27,255,0,0) || !pixel(43,37,0,0,255) || !pixel(32,41,0,0,255))
                throw std::runtime_error("ellipsoid silhouette/occlusion mismatch");
              if (!frame.SerializeToString(&frame_wire) || !info.SerializeToString(&info_wire)) throw std::runtime_error("ellipsoid serialization failed");
              break;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      } // Subscription ownership is gone before accepting success.
    }
    std::cout<<"IMAGE "<<ellipsoid_hex_bytes(frame_wire)<<'\n'<<"INFO "<<ellipsoid_hex_bytes(info_wire)<<"\nPASS\n"<<std::flush;
    if (!std::cout || std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("ellipsoid completion failed");
    return 0;
  } catch (const std::exception &error) { std::cerr<<error.what()<<std::endl;return 1; }
}


namespace {
struct DepthPrimitiveMessages {
  std::mutex mutex;
  std::optional<gz::msgs::Image> image;
  std::optional<gz::msgs::CameraInfo> info;
  bool failed{false};
};
std::string depth_primitives_hex_bytes(const std::string &value) {
  static constexpr char digits[]="0123456789abcdef";
  std::string result; result.reserve(value.size()*2);
  for (unsigned char byte:value) { result+=digits[byte>>4]; result+=digits[byte&15]; }
  return result;
}
void depth_primitives_header(const gz::msgs::Header &header, bool sequence) {
  if (!header.has_stamp() || header.stamp().sec()!=0 || header.stamp().nsec()!=1000000 ||
      header.data_size()!=(sequence?2:1) || header.data(0).key()!="frame_id" ||
      header.data(0).value_size()!=1 || header.data(0).value(0)!="rig/body/camera" ||
      (sequence && (header.data(1).key()!="seq" || header.data(1).value_size()!=1 || header.data(1).value(0)!="0")))
    throw std::runtime_error("depth-primitives header mismatch");
}
}

int depth_primitives_mode(int argc, char **argv) {
  try {
    if (argc!=2 || std::string(argv[1])!="depth-primitives") throw std::runtime_error("expected depth-primitives mode");
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(20);
    std::string frame_wire, info_wire;
    {
      auto messages=std::make_shared<DepthPrimitiveMessages>();
      {
        gz::transport::Node node;
        if (!node.Subscribe("/m68/depth/image",std::function<void(const gz::msgs::Image &)>([messages](const gz::msgs::Image &value) noexcept {
          try { std::lock_guard<std::mutex> lock(messages->mutex); if (value.ByteSizeLong()>32768) messages->failed=true; else if (!messages->image) messages->image=value; }
          catch (...) { std::lock_guard<std::mutex> lock(messages->mutex); messages->failed=true; }
        })) || !node.Subscribe("/m68/depth/camera_info",std::function<void(const gz::msgs::CameraInfo &)>([messages](const gz::msgs::CameraInfo &value) noexcept {
          try { std::lock_guard<std::mutex> lock(messages->mutex); if (value.ByteSizeLong()>16384) messages->failed=true; else if (!messages->info) messages->info=value; }
          catch (...) { std::lock_guard<std::mutex> lock(messages->mutex); messages->failed=true; }
        }))) throw std::runtime_error("depth-primitives subscribe failed");
        std::cout<<"ready"<<std::endl;if (!std::cout) throw std::runtime_error("ready write failed");
        for (;;) {
          if (std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("depth-primitives receive timeout");
          {
            std::lock_guard<std::mutex> lock(messages->mutex);
            if (messages->failed) throw std::runtime_error("depth-primitives callback failure");
            if (messages->image && messages->info) {
              const auto &frame=*messages->image;const auto &info=*messages->info;
              if (!frame.has_header() || !info.has_header() || frame.width()!=64 || frame.height()!=64 || frame.step()!=256 ||
                  frame.data().size()!=16384 || frame.pixel_format_type()!=gz::msgs::PixelFormatType::R_FLOAT32 || info.width()!=64 || info.height()!=64)
                throw std::runtime_error("depth-primitives dimensions mismatch");
              depth_primitives_header(frame.header(),true);depth_primitives_header(info.header(),false);
              static_assert(sizeof(float)==4, "canonical depth requires binary32");
              const auto depth=[&frame](int x,int y) {
                const auto offset=(y*64+x)*4;
                const auto byte=[&](int n) { return static_cast<std::uint32_t>(static_cast<unsigned char>(frame.data()[offset+n])); };
                const std::uint32_t bits=byte(0)|(byte(1)<<8)|(byte(2)<<16)|(byte(3)<<24);
                float value; std::memcpy(&value,&bits,sizeof(value)); return value;
              };
              for (int y=0;y<64;++y) for (int x=0;x<64;++x)
                if (!(depth(x,y)>0.0f) || std::isnan(depth(x,y))) throw std::runtime_error("invalid canonical depth word");
              if (std::abs(depth(32,32)-1.8f)>=1e-4f || std::abs(depth(44,32)-3.5f)>=1e-4f ||
                  std::abs(depth(40,35)-6.9f)>=1e-4f || std::abs(depth(16,34)-6.9f)>=1e-4f ||
                  !(depth(37,32)>3.4f && depth(37,32)<3.9f) || !(depth(20,32)>3.51f && depth(20,32)<4.0f) ||
                  !std::isinf(depth(0,0)) || !std::isinf(depth(63,0)) || !std::isinf(depth(0,63)) || !std::isinf(depth(63,63)))
                throw std::runtime_error("depth primitive mesh/occlusion mismatch");
              if (!frame.SerializeToString(&frame_wire) || !info.SerializeToString(&info_wire)) throw std::runtime_error("depth-primitives serialization failed");
              break;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      } // Subscription ownership is gone before accepting success.
    }
    const std::string output="IMAGE "+depth_primitives_hex_bytes(frame_wire)+"\nINFO "+depth_primitives_hex_bytes(info_wire)+"\nPASS\n";
    if (output.size()+std::string("ready\n").size()>65536) throw std::runtime_error("canonical stdout aggregate cap");
    std::cout<<output<<std::flush;
    if (!std::cout || std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("depth-primitives completion failed");
    return 0;
  } catch (const std::exception &error) { std::cerr<<error.what()<<std::endl;return 1; }
}


namespace {
struct DepthPairReceived {
  std::mutex mutex;
  std::condition_variable changed;
  std::optional<gz::msgs::Image> image;
  std::optional<gz::msgs::CameraInfo> info;
  bool accepting=true, failed=false;
};
void depth_pair_image(const gz::msgs::Image &image, unsigned owner) {
  pair_require(image.has_header(), "depth-pair image header");
  pair_header(image.header(), owner, true);
  pair_require(image.width()==64 && image.height()==64 && image.step()==256 &&
      image.pixel_format_type()==gz::msgs::PixelFormatType::R_FLOAT32 &&
      image.data().size()==16384, "depth-pair type/dimensions/stride");
  static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559,
      "canonical depth uses IEEE binary32");
  // Independently declared planar fixture. Camera-forward X=1 makes this
  // optical-axis depth, not Euclidean range. No received value derives its
  // own oracle: the two box-front distances and projection are literals.
  const double front=owner==0 ? 1.8 : 2.8;
  for (int y=0;y<64;++y) for (int x=0;x<64;++x) {
    const auto offset=(y*64+x)*4;
    const auto byte=[&](int n) {
      return static_cast<std::uint32_t>(static_cast<unsigned char>(image.data()[offset+n]));
    };
    const std::uint32_t bits=byte(0)|(byte(1)<<8)|(byte(2)<<16)|(byte(3)<<24);
    float actual; std::memcpy(&actual,&bits,sizeof(actual));
    const bool occluded=std::abs((32.0-(x+0.5))/32.0*front)<=0.2 &&
        std::abs((32.0-(y+0.5))/32.0*front)<=0.2;
    const double expected=occluded ? front : 5.9;
    pair_require(std::isfinite(actual) && actual>0 && std::abs(actual-expected)<1e-4,
        "every canonical LE depth word needs its independent fixture value");
  }
}
}
int depth_pair_mode(int argc, char **argv) {
  try {
    pair_require(argc==3 && std::string(argv[1])=="depth-pair" &&
        (std::string(argv[2])=="0" || std::string(argv[2])=="1"),
        "expected closed depth-pair owner0 or owner1");
    const unsigned owner=std::string(argv[2])=="0" ? 0u : 1u;
    const std::string prefix=owner==0 ? "/m69/a/" : "/m69/b/";
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(20);
    std::string image_wire,info_wire;
    {
      auto state=std::make_shared<DepthPairReceived>();
      gz::transport::Node node;
      const auto image=[state](const gz::msgs::Image &value) noexcept {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->accepting || state->failed || state->image) return;
        try {
          pair_require(value.ByteSizeLong()<=32768,"depth-pair image cap32768");
          state->image=value;
        } catch (...) { state->failed=true; }
        state->changed.notify_all();
      };
      const auto info=[state](const gz::msgs::CameraInfo &value) noexcept {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->accepting || state->failed || state->info) return;
        try {
          pair_require(value.ByteSizeLong()<=16384,"depth-pair info cap16384");
          state->info=value;
        } catch (...) { state->failed=true; }
        state->changed.notify_all();
      };
      pair_require(node.Subscribe(prefix+"image",std::function<void(const gz::msgs::Image &)>(image)) &&
          node.Subscribe(prefix+"camera_info",std::function<void(const gz::msgs::CameraInfo &)>(info)),
          "actual depth-pair typed subscriptions");
      std::cout<<"ready depth pair "<<owner<<std::endl;
      pair_require(std::cout.good(),"depth-pair readiness write");
      {
        std::unique_lock<std::mutex> lock(state->mutex);
        pair_require(state->changed.wait_until(lock,deadline,[&]{return state->failed || (state->image && state->info);}),
            "depth-pair actual receive deadline");
        pair_require(!state->failed,"depth-pair callback failure");
        state->accepting=false;
        depth_pair_image(*state->image,owner);
        pair_info(*state->info,owner);
        pair_require(static_cast<int>(state->info->distortion().model())==0,"canonical PLUMB_BOB distortion model");
        pair_require(state->image->SerializeToString(&image_wire) && state->info->SerializeToString(&info_wire),
            "full canonical depth-pair serialization");
      }
      pair_require(node.Unsubscribe(prefix+"image") && node.Unsubscribe(prefix+"camera_info"),
          "both owned depth subscriptions retired");
    }
    const std::string ready="ready depth pair "+std::to_string(owner)+"\n";
    const std::string output="IMAGE "+pair_hex(image_wire)+"\nINFO "+pair_hex(info_wire)+
        "\nPASS canonical depth pair "+std::to_string(owner)+"\n";
    pair_require(ready.size()+output.size()<=65536,"per-consumer full stdout64KiB cap");
    std::cout<<output<<std::flush;
    pair_require(std::cout.good() && std::chrono::steady_clock::now()<deadline,
        "canonical depth-pair completion and owned cleanup deadline");
    return 0;
  } catch (const std::exception &error) { std::cerr<<error.what()<<std::endl; return 1; }
}


namespace {
void mixed_rgb_image(const gz::msgs::Image &image) {
  pair_require(image.has_header(),"mixed RGB header");
  pair_header(image.header(),0,true);
  pair_require(image.width()==64 && image.height()==64 && image.step()==192 &&
      image.pixel_format_type()==gz::msgs::PixelFormatType::RGB_INT8 && image.data().size()==12288,
      "mixed RGB canonical type/dimensions/stride");
  const auto pixel=[&](int x,int y,int red,int green,int blue) {
    const auto offset=(y*64+x)*3;
    return static_cast<unsigned char>(image.data()[offset])==red &&
        static_cast<unsigned char>(image.data()[offset+1])==green &&
        static_cast<unsigned char>(image.data()[offset+2])==blue;
  };
  // Declared red box front1.8m, blue planar backdrop5.9m. These selected
  // interiors avoid antialiasing boundaries; full canonical bytes follow.
  for (int y=30;y<=33;++y) for (int x=30;x<=33;++x)
    pair_require(pixel(x,y,255,0,0),"mixed RGB red foreground interior");
  for (const auto &[x,y]:std::array<std::pair<int,int>,4>{{{0,0},{24,24},{40,40},{63,63}}})
    pair_require(pixel(x,y,0,0,255),"mixed RGB blue background");
}
}
int mixed_pair_mode(int argc, char **argv) {
  try {
    pair_require(argc==3 && std::string(argv[1])=="mixed-pair" &&
        (std::string(argv[2])=="0" || std::string(argv[2])=="1"),
        "expected mixed-pair RGB0 or depth1");
    const unsigned owner=std::string(argv[2])=="0" ? 0u : 1u;
    const std::string prefix=owner==0 ? "/m71/rgb/" : "/m71/depth/";
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(20);
    std::string image_wire,info_wire;
    {
      auto state=std::make_shared<DepthPairReceived>();
      gz::transport::Node node;
      const auto image=[state,owner](const gz::msgs::Image &value) noexcept {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->accepting || state->failed || state->image) return;
        try { pair_require(value.ByteSizeLong()<=(owner==0 ? 16384u : 32768u),"mixed per-format Image cap");state->image=value; }
        catch (...) { state->failed=true; }
        state->changed.notify_all();
      };
      const auto info=[state](const gz::msgs::CameraInfo &value) noexcept {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->accepting || state->failed || state->info) return;
        try { pair_require(value.ByteSizeLong()<=16384,"mixed CameraInfo cap16384");state->info=value; }
        catch (...) { state->failed=true; }
        state->changed.notify_all();
      };
      pair_require(node.Subscribe(prefix+"image",std::function<void(const gz::msgs::Image &)>(image)) &&
          node.Subscribe(prefix+"camera_info",std::function<void(const gz::msgs::CameraInfo &)>(info)),
          "actual independent mixed typed subscriptions");
      std::cout<<"ready mixed pair "<<owner<<std::endl;
      pair_require(std::cout.good(),"mixed readiness write");
      {
        std::unique_lock<std::mutex> lock(state->mutex);
        pair_require(state->changed.wait_until(lock,deadline,[&]{return state->failed || (state->image && state->info);}),
            "mixed actual receive deadline");
        pair_require(!state->failed,"mixed callback failure");state->accepting=false;
        if (owner==0) mixed_rgb_image(*state->image);else depth_pair_image(*state->image,owner);
        pair_info(*state->info,owner);
        pair_require(static_cast<int>(state->info->distortion().model())==0,"canonical PLUMB_BOB distortion");
        pair_require(state->image->SerializeToString(&image_wire) && state->info->SerializeToString(&info_wire),
            "full canonical mixed Image/Info serialization");
      }
      pair_require(node.Unsubscribe(prefix+"image") && node.Unsubscribe(prefix+"camera_info"),
          "both owned mixed subscriptions retired");
    }
    const std::string ready="ready mixed pair "+std::to_string(owner)+"\n";
    const std::string output="IMAGE "+pair_hex(image_wire)+"\nINFO "+pair_hex(info_wire)+
        "\nPASS canonical mixed pair "+std::to_string(owner)+"\n";
    pair_require(ready.size()+output.size()<=65536,"mixed full stdout64KiB cap");
    std::cout<<output<<std::flush;
    pair_require(std::cout.good() && std::chrono::steady_clock::now()<deadline,"mixed completion deadline");
    return 0;
  } catch (const std::exception &error) { std::cerr<<error.what()<<std::endl;return 1; }
}

// M72-only canonical consumer. All earlier modes and their assertions stay exact.
namespace {
std::array<double,4> explicit_k(unsigned owner) {
  return owner==0 ? std::array<double,4>{40,48,20,25} : std::array<double,4>{20,24,42,20};
}
struct ExplicitHit { double depth; unsigned box; bool interior; };
ExplicitHit explicit_hit(unsigned owner,int x,int y,bool fallback=false) {
  const auto k=fallback ? std::array<double,4>{32,32,32,32} : explicit_k(owner);
  const std::array<double,3> origin{0,owner==0 ? -2.0 : 2.0,1};
  const std::array<double,3> ray{1,(k[2]-x-0.5)/k[0],(k[3]-y-0.5)/k[1]};
  constexpr std::array<std::array<double,3>,3> centers{{{6,0,1},{2,-2,1},{3,2,1}}};
  constexpr std::array<std::array<double,3>,3> halves{{{0.1,8,8},{0.2,0.2,0.2},{0.2,0.2,0.2}}};
  ExplicitHit best{std::numeric_limits<double>::infinity(),0,false};
  for(unsigned box=0;box<3;++box) {
    double enter=0.1,leave=10;
    for(unsigned axis=0;axis<3;++axis) {
      const double lo=centers[box][axis]-halves[box][axis]-origin[axis];
      const double hi=centers[box][axis]+halves[box][axis]-origin[axis];
      if(ray[axis]==0) { if(lo>0 || hi<0) leave=-1; }
      else { const double a=lo/ray[axis],b=hi/ray[axis];enter=std::fmax(enter,std::fmin(a,b));leave=std::fmin(leave,std::fmax(a,b)); }
    }
    if(enter<=leave && enter<best.depth) {
      const double py=origin[1]+enter*ray[1],pz=origin[2]+enter*ray[2];
      const bool front=std::abs(enter-(centers[box][0]-halves[box][0]))<1e-10;
      best={enter,box,front && std::abs(py-centers[box][1])<halves[box][1]-0.03 &&
          std::abs(pz-centers[box][2])<halves[box][2]-0.03};
    }
  }
  return best;
}
// Bounds cover the complete projected box, not just a center-ray hit.
std::array<double,4> explicit_projected_bounds(unsigned owner,unsigned box,bool front_only) {
  const auto k=explicit_k(owner);
  constexpr std::array<std::array<double,3>,3> centers{{{6,0,1},{2,-2,1},{3,2,1}}};
  constexpr std::array<std::array<double,3>,3> halves{{{0.1,8,8},{0.2,0.2,0.2},{0.2,0.2,0.2}}};
  const auto center=centers[box],half=halves[box];
  const double front=center[0]-half[0],origin_y=owner==0 ? -2.0:2.0;
  const std::array<double,2> xs{front,front_only ? front:center[0]+half[0]};
  std::array<double,4> bounds{std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};
  for(double px:xs) for(double py:std::array<double,2>{center[1]-half[1],center[1]+half[1]})
    for(double pz:std::array<double,2>{center[2]-half[2],center[2]+half[2]}) {
      const double u=k[2]-k[0]*(py-origin_y)/px,v=k[3]-k[1]*(pz-1)/px;
      bounds[0]=std::fmin(bounds[0],u);bounds[1]=std::fmax(bounds[1],u);
      bounds[2]=std::fmin(bounds[2],v);bounds[3]=std::fmax(bounds[3],v);
    }
  return bounds;
}
bool explicit_homogeneous_footprint(unsigned owner,int x,int y,const ExplicitHit &hit) {
  const double x0=x,y0=y,x1=x+1,y1=y+1;
  const auto selected=explicit_projected_bounds(owner,hit.box,true);
  if(!(x0>selected[0] && x1<selected[1] && y0>selected[2] && y1<selected[3]))return false;
  constexpr std::array<double,3> fronts{5.9,1.8,2.8};
  for(unsigned other=0;other<3;++other) if(other!=hit.box && fronts[other]<hit.depth) {
    // Conservatively exclude the whole footprint even if it only touches a
    // nearer eight-corner silhouette bound; no sample-pattern assumption.
    const auto bounds=explicit_projected_bounds(owner,other,false);
    if(x1>=bounds[0] && x0<=bounds[1] && y1>=bounds[2] && y0<=bounds[3])return false;
  }
  return true;
}
void explicit_info(const gz::msgs::CameraInfo &info,unsigned owner) {
  pair_require(info.has_header(),"explicit-K info header");pair_header(info.header(),owner,false);
  pair_require(info.width()==64 && info.height()==64 && info.has_intrinsics() && info.intrinsics().k_size()==9 &&
      info.has_projection() && info.projection().p_size()==12 && info.has_distortion() &&
      info.distortion().k_size()==5 && info.rectification_matrix_size()==9,"explicit-K complete calibration shape");
  const auto v=explicit_k(owner);
  const std::array<double,9> k{v[0],0,v[2],0,v[1],v[3],0,0,1},rect{1,0,0,0,1,0,0,0,1};
  const std::array<double,12> p{v[0],0,v[2],0,0,v[1],v[3],0,0,0,1,0};
  for(int i=0;i<9;++i) pair_require(info.intrinsics().k(i)==k[i] && info.rectification_matrix(i)==rect[i],"explicit-K/identity rectification");
  for(int i=0;i<12;++i) pair_require(info.projection().p(i)==p[i],"explicit P is selected K|0");
  pair_require(static_cast<int>(info.distortion().model())==0,"explicit-K PLUMB_BOB");
  for(double d:info.distortion().k()) pair_require(d==0,"explicit-K zero distortion");
}
void explicit_image(const gz::msgs::Image &image,unsigned owner) {
  pair_require(image.has_header(),"explicit-K image header");pair_header(image.header(),owner,true);
  pair_require(image.width()==64 && image.height()==64 && image.step()==(owner==0 ? 192u:256u) &&
      image.pixel_format_type()==(owner==0 ? gz::msgs::PixelFormatType::RGB_INT8:gz::msgs::PixelFormatType::R_FLOAT32) &&
      image.data().size()==(owner==0 ? 12288u:16384u),"explicit-K image type/stride/length");
  unsigned red=0,blue=0,infinite=0,fallback_disagreements=0,checked=0,excluded=0;
  for(int y=0;y<64;++y) for(int x=0;x<64;++x) {
    const auto hit=explicit_hit(owner,x,y),fallback=explicit_hit(owner,x,y,true);
    if(owner==0) {
      if(!hit.interior || !explicit_homogeneous_footprint(owner,x,y,hit)){++excluded;continue;}
      ++checked;
      const std::array<unsigned char,3> expected=hit.box==0 ? std::array<unsigned char,3>{0,0,255}:
          (hit.box==1 ? std::array<unsigned char,3>{255,0,0}:std::array<unsigned char,3>{0,255,0});
      const std::array<unsigned char,3> actual{static_cast<unsigned char>(image.data()[(y*64+x)*3]),
          static_cast<unsigned char>(image.data()[(y*64+x)*3+1]),static_cast<unsigned char>(image.data()[(y*64+x)*3+2])};
      if(actual!=expected) throw std::runtime_error("RGB owner="+std::to_string(owner)+" x="+std::to_string(x)+" y="+std::to_string(y)+
          " expected=["+std::to_string(expected[0])+","+std::to_string(expected[1])+","+std::to_string(expected[2])+"] actual=["+
          std::to_string(actual[0])+","+std::to_string(actual[1])+","+std::to_string(actual[2])+"]; homogeneous footprint");
      red+=hit.box==1;blue+=hit.box==0;
      fallback_disagreements+=hit.box==1 && (!std::isfinite(fallback.depth) || fallback.box!=1);
    } else if(owner==1) {
      ++checked;
      const auto offset=(y*64+x)*4;
      const auto byte=[&](int i){return static_cast<std::uint32_t>(static_cast<unsigned char>(image.data()[offset+i]));};
      const std::uint32_t bits=byte(0)|(byte(1)<<8)|(byte(2)<<16)|(byte(3)<<24);
      float actual;std::memcpy(&actual,&bits,sizeof(actual));
      const bool valid=std::isinf(hit.depth) ? std::isinf(actual) && actual>0:
          std::isfinite(actual) && actual>0 && std::abs(actual-hit.depth)<1e-4;
      if(!valid) throw std::runtime_error("depth owner="+std::to_string(owner)+" x="+std::to_string(x)+" y="+std::to_string(y)+
          " expected="+std::to_string(hit.depth)+" actual="+std::to_string(actual));
      if(std::isinf(hit.depth))++infinite;
      fallback_disagreements+=std::isinf(hit.depth)!=std::isinf(fallback.depth) ||
          (std::isfinite(hit.depth) && std::isfinite(fallback.depth) && std::abs(hit.depth-fallback.depth)>0.1);
    }
  }
  if(!(owner==0 ? checked+excluded==4096 && excluded>0 : checked==4096 && excluded==0))
    throw std::runtime_error("explicit pixel accounting owner="+std::to_string(owner)+" checked="+std::to_string(checked)+" excluded="+std::to_string(excluded));
  if(!(owner==0 ? red>20 && blue>100 && fallback_disagreements>20 : infinite>100 && fallback_disagreements>100))
    throw std::runtime_error("selected-K coverage owner="+std::to_string(owner)+" checked="+std::to_string(checked)+" excluded="+std::to_string(excluded)+
      " red="+std::to_string(red)+" blue="+std::to_string(blue)+" +INF="+std::to_string(infinite)+" HFOV_disagreements="+std::to_string(fallback_disagreements));
}
}
int explicit_intrinsics_mode(int argc,char **argv) {
  try {
    pair_require(argc==3 && std::string(argv[1])=="explicit-intrinsics" &&
        (std::string(argv[2])=="0" || std::string(argv[2])=="1"),"explicit-intrinsics RGB0/depth1");
    const unsigned owner=std::string(argv[2])=="0" ? 0u:1u;
    const std::string prefix=owner==0 ? "/m72/rgb/":"/m72/depth/";
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(20);
    std::string image_wire,info_wire;
    {
      auto state=std::make_shared<DepthPairReceived>();gz::transport::Node node;
      const auto image=[state,owner](const gz::msgs::Image &v) noexcept {
        std::lock_guard<std::mutex> lock(state->mutex);
        if(!state->accepting || state->failed || state->image)return;
        try {pair_require(v.ByteSizeLong()<=(owner==0 ? 16384u:32768u),"explicit Image cap");state->image=v;}catch(...){state->failed=true;}
        state->changed.notify_all();
      };
      const auto info=[state](const gz::msgs::CameraInfo &v) noexcept {
        std::lock_guard<std::mutex> lock(state->mutex);
        if(!state->accepting || state->failed || state->info)return;
        try {pair_require(v.ByteSizeLong()<=16384,"explicit CameraInfo cap");state->info=v;}catch(...){state->failed=true;}
        state->changed.notify_all();
      };
      pair_require(node.Subscribe(prefix+"image",std::function<void(const gz::msgs::Image &)>(image)) &&
          node.Subscribe(prefix+"camera_info",std::function<void(const gz::msgs::CameraInfo &)>(info)),"actual explicit-K typed subscriptions");
      std::cout<<"ready explicit intrinsics "<<owner<<std::endl;
      pair_require(std::cout.good(),"explicit-K ready write");
      {
        std::unique_lock<std::mutex> lock(state->mutex);
        pair_require(state->changed.wait_until(lock,deadline,[&]{return state->failed || (state->image && state->info);}),"explicit-K receive deadline");
        pair_require(!state->failed,"explicit-K callback failure");state->accepting=false;
        explicit_image(*state->image,owner);explicit_info(*state->info,owner);
        pair_require(state->image->SerializeToString(&image_wire) && state->info->SerializeToString(&info_wire),"explicit-K complete canonical bytes");
      }
      pair_require(node.Unsubscribe(prefix+"image") && node.Unsubscribe(prefix+"camera_info"),"explicit-K owned subscriptions retired");
    }
    const std::string ready="ready explicit intrinsics "+std::to_string(owner)+"\n";
    const std::string output="IMAGE "+pair_hex(image_wire)+"\nINFO "+pair_hex(info_wire)+"\nPASS canonical explicit intrinsics "+std::to_string(owner)+"\n";
    pair_require(ready.size()+output.size()<=65536,"explicit-K stdout64KiB cap");
    std::cout<<output<<std::flush;
    pair_require(std::cout.good() && std::chrono::steady_clock::now()<deadline,"explicit-K completion deadline");return 0;
  }catch(const std::exception &error){std::cerr<<error.what()<<std::endl;return 1;}
}
