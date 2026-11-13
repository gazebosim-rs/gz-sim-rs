// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Fresh CpuLidarSensor COMPONENT ray-input conversion and scan serialization.
// No System/ECM/Physics, demand/rate/reset/removal or global-RNG equivalence.
// Pair beams [3,5]; eight beams 128 each; three direct calls per owner.
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/sensors/CpuLidarSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

using namespace std::chrono_literals;
namespace {
using Scan = gz::msgs::LaserScan;
using Pose = gz::math::Pose3d;
using Vector = gz::math::Vector3d;
using Sensor = gz::sensors::CpuLidarSensor;
constexpr unsigned Rows = 3;
constexpr std::size_t MessageCap = 65536;

void require(bool value, const std::string &message)
{
  if (!value) throw std::runtime_error(message);
}

std::uint64_t rawBits(double value)
{
  std::uint64_t result;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

std::string bits(double value, bool range = false)
{
  require(std::isfinite(value) ||
      (range && value == std::numeric_limits<double>::infinity()),
      "only range/fraction may record positive infinity");
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << rawBits(value);
  return out.str();
}

std::string hex(const std::string &value)
{
  constexpr char digits[] = "0123456789abcdef";
  std::string result;
  result.reserve(value.size() * 2);
  for (unsigned char byte : value)
  {
    result += digits[byte >> 4];
    result += digits[byte & 15];
  }
  return result;
}

void vectorBits(const Vector &value)
{
  std::cout << "[\"" << bits(value.X()) << "\",\"" << bits(value.Y())
      << "\",\"" << bits(value.Z()) << "\"]";
}

std::array<double, 7> poseValues(const Pose &value)
{
  return {value.Pos().X(), value.Pos().Y(), value.Pos().Z(),
      value.Rot().X(), value.Rot().Y(), value.Rot().Z(), value.Rot().W()};
}

std::array<double, 7> poseValues(const gz::msgs::Pose &value)
{
  require(value.has_position() && value.has_orientation(), "complete captured pose");
  return {value.position().x(), value.position().y(), value.position().z(),
      value.orientation().x(), value.orientation().y(), value.orientation().z(),
      value.orientation().w()};
}

template<class PoseType> void poseBits(const PoseType &value)
{
  const auto components = poseValues(value);
  double norm = 0;
  for (unsigned i = 3; i < 7; ++i) norm += components[i] * components[i];
  require(std::isfinite(norm) && std::abs(norm - 1.0) <= 1e-12,
      "existing finite unit-pose domain");
  std::cout << '[';
  for (unsigned i = 0; i < 7; ++i)
  {
    if (i) std::cout << ',';
    std::cout << '"' << bits(components[i]) << '"';
  }
  std::cout << ']';
}

void rangeBits(const std::vector<double> &values)
{
  std::cout << '[';
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i) std::cout << ',';
    std::cout << '"' << bits(values[i], true) << '"';
  }
  std::cout << ']';
}

void maps(const std::string &path)
{
  std::ifstream input("/proc/self/maps");
  require(bool(input), "maps open");
  std::string text, line;
  while (std::getline(input, line))
  {
    require(text.size() + line.size() + 1 <= 4 * 1024 * 1024, "maps cap");
    text += line + '\n';
  }
  require(input.eof(), "complete maps read");
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
  require(fd >= 0, "exclusive maps output");
  std::size_t offset = 0;
  while (offset < text.size())
  {
    const auto count = ::write(fd, text.data() + offset, text.size() - offset);
    if (count <= 0)
    {
      ::close(fd);
      throw std::runtime_error("maps write");
    }
    offset += static_cast<std::size_t>(count);
  }
  const bool flushed = ::fsync(fd) == 0;
  const bool closed = ::close(fd) == 0;
  require(flushed && closed, "maps durable completion");
}

struct Received
{
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<Scan> messages;
  std::shared_ptr<std::atomic<bool>> active =
      std::make_shared<std::atomic<bool>>(true);
  bool bad = false;
  std::string failure;
};

void subscribe(gz::transport::Node &node, const std::string &topic,
    const std::shared_ptr<Received> &state)
{
  auto active = state->active;
  std::function<void(const Scan &)> callback = [state, active](const Scan &message)
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (!active->load(std::memory_order_acquire) || state->bad) return;
    try
    {
      require(message.ByteSizeLong() <= MessageCap && state->messages.size() < Rows,
          "scan callback cap before copy");
      state->messages.push_back(message);
    }
    catch (const std::exception &error)
    {
      state->bad = true;
      state->failure = error.what();
    }
    state->changed.notify_all();
  };
  require(node.Subscribe(topic, callback), "actual typed scan subscription");
}

void ready(gz::transport::Node &node, const std::string &topic, Sensor &sensor)
{
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  for (;;)
  {
    std::vector<gz::transport::MessagePublisher> publishers, subscribers;
    std::vector<gz::transport::MessagePublisher> pointPublishers, pointSubscribers;
    node.TopicInfo(topic, publishers, subscribers);
    node.TopicInfo(topic + "/points", pointPublishers, pointSubscribers);
    if (publishers.size() == 1 && subscribers.size() == 1 &&
        publishers[0].MsgTypeName() == "gz.msgs.LaserScan" &&
        subscribers[0].MsgTypeName() == "gz.msgs.LaserScan" &&
        pointPublishers.size() == 1 &&
        pointPublishers[0].MsgTypeName() == "gz.msgs.PointCloudPacked" &&
        pointSubscribers.empty() && sensor.HasConnections()) return;
    require(std::chrono::steady_clock::now() < deadline,
        "owned scan readiness and unconnected points deadline");
    std::this_thread::yield();
  }
}

Scan receive(const std::shared_ptr<Received> &state, unsigned row)
{
  std::unique_lock<std::mutex> lock(state->mutex);
  require(state->changed.wait_for(lock, 5s, [&]
      { return state->bad || state->messages.size() > row; }),
      "actual scan callback deadline");
  require(!state->bad && state->messages.size() == row + 1,
      "exact callback inventory: " + state->failure);
  return state->messages[row];
}

struct Owner
{
  unsigned index;
  std::string topic;
  std::string frame;
  std::unique_ptr<Sensor> sensor;
  std::shared_ptr<Received> received;
  std::vector<std::pair<Vector, Vector>> rays;
};

void header(const gz::msgs::Header &value, long long ns, unsigned row,
    const std::string &frame)
{
  require(value.has_stamp() && value.stamp().sec() == 0 &&
      value.stamp().nsec() == ns, "actual acquisition stamp");
  require(value.data_size() == 2 && value.data(0).key() == "frame_id" &&
      value.data(0).value_size() == 1 && value.data(0).value(0) == frame &&
      value.data(1).key() == "seq" && value.data(1).value_size() == 1 &&
      value.data(1).value(0) == std::to_string(row), "per-owner frame/sequence");
}

Pose inputPose(unsigned owner, unsigned row)
{
  // Nonzero controlled components avoid conflating absent proto3 scalar fields
  // with the separately retained signed-zero and pure-pose proof domains.
  return Pose(0.25 * (row + 1), 2.0 * (owner + 1), 0.5 * (row + 1),
      0.03 * (row + 1), -0.02 * (row + 1),
      0.05 * (owner + 1) + 0.04 * (row + 1));
}

std::string configuration(unsigned owner, unsigned beams,
    const std::string &topic, const std::string &frame)
{
  std::ostringstream xml;
  xml << std::setprecision(17)
      << "<sdf version='1.12'><model name='rig'><link name='body'>"
      << "<sensor name='laser" << owner << "' type='lidar'>"
      << "<pose>0.5 0 0 0 0 " << 0.05 * owner << "</pose>"
      << "<update_rate>" << (owner % 2 == 0 ? 100 : 50) << "</update_rate>"
      << "<topic>" << topic << "</topic><frame_id>" << frame << "</frame_id>"
      << "<ray><scan><horizontal><samples>" << beams << "</samples>"
      << "<resolution>1</resolution><min_angle>-0.7853981633974483</min_angle>"
      << "<max_angle>0.7853981633974483</max_angle></horizontal><vertical>"
      << "<samples>1</samples><resolution>1</resolution><min_angle>0</min_angle>"
      << "<max_angle>0</max_angle></vertical></scan><range><min>0</min><max>10</max>"
      << "<resolution>0.01</resolution></range></ray></sensor></link></model></sdf>";
  require(xml.str().size() <= 65536, "configuration XML cap");
  return xml.str();
}

void run(const std::string &mode, const std::filesystem::path &mapPath)
{
  require(mode == "pair" || mode == "eight", "closed component modes");
  require(mapPath.is_absolute(), "owned absolute maps path");
  const auto tag = mapPath.parent_path().filename().string();
  require(!tag.empty() && tag.size() <= 128 &&
      tag.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_") ==
          std::string::npos, "owned topic namespace from exclusive packet");
  require(gz::common::Console::Init(mapPath.parent_path().string(),
      mapPath.filename().string() + ".console.log"), "owned Console Init");
  require(gz::common::Console::Directory() == mapPath.parent_path().string(),
      "owned Console directory");
  gz::common::Console::SetVerbosity(2);
  require(gz::common::Console::Verbosity() == 2, "actual Console verbosity");
  const unsigned count = mode == "pair" ? 2 : 8;
  gz::sensors::SensorFactory factory;
  gz::transport::Node node;
  std::vector<Owner> owners;
  std::set<std::uint64_t> identities;
  bool attemptedMaps = false;
  auto capture = [&]
  {
    attemptedMaps = true;
    maps(mapPath.string());
  };
  try
  {
    for (unsigned i = 0; i < count; ++i)
    {
      const unsigned beams = mode == "pair" ? (i == 0 ? 3 : 5) : 128;
      const auto topic = "/m54/native/" + tag + '/' + mode + "/laser" + std::to_string(i);
      const auto frame = "m54/native/laser" + std::to_string(i);
      const auto xml = configuration(i, beams, topic, frame);
      sdf::SDFPtr parsed(new sdf::SDF);
      sdf::init(parsed);
      require(sdf::readString(xml, parsed), "native SDF configuration");
      auto element = parsed->Root()->GetElement("model")->GetElement("link")
          ->GetElement("sensor");
      auto sensor = factory.CreateSensor<Sensor>(element);
      require(bool(sensor), "ElementPtr CpuLidarSensor factory");
      require(identities.insert(sensor->Id()).second &&
          sensor->Name() == "laser" + std::to_string(i) && sensor->Topic() == topic &&
          sensor->FrameId() == frame && sensor->UpdateRate() == (i % 2 == 0 ? 100 : 50),
          "actual independent identity/configuration getters");
      require(sensor->RayCount() == beams && sensor->VerticalRayCount() == 1 &&
          sensor->RangeMin() == 0.0 && sensor->RangeMax() == 10.0 &&
          sensor->AngleMin().Radian() == -0.7853981633974483 &&
          sensor->AngleMax().Radian() == 0.7853981633974483 &&
          sensor->VerticalAngleMin().Radian() == 0.0 &&
          sensor->VerticalAngleMax().Radian() == 0.0, "actual planar range getters");
      auto rays = sensor->GenerateRays();
      require(rays.size() == beams, "actual generated ray count");
      for (const auto &ray : rays)
      {
        require(ray.first.Length() <= 1e-12 &&
            std::abs(ray.second.Length() - 10.0) <= 1e-12,
            "retained M24 local ray endpoint bounds");
      }
      auto received = std::make_shared<Received>();
      subscribe(node, topic, received);
      std::cout << "{\"kind\":\"configuration\",\"mode\":" << std::quoted(mode)
          << ",\"sensor\":" << i << ",\"native_id\":" << sensor->Id()
          << ",\"name\":" << std::quoted(sensor->Name())
          << ",\"topic\":" << std::quoted(sensor->Topic())
          << ",\"frame_id\":" << std::quoted(sensor->FrameId())
          << ",\"rate_bits\":\"" << bits(sensor->UpdateRate())
          << "\",\"horizontal_count\":" << sensor->RayCount()
          << ",\"vertical_count\":" << sensor->VerticalRayCount()
          << ",\"angle_min_bits\":\"" << bits(sensor->AngleMin().Radian())
          << "\",\"angle_max_bits\":\"" << bits(sensor->AngleMax().Radian())
          << "\",\"vertical_angle_min_bits\":\"" << bits(sensor->VerticalAngleMin().Radian())
          << "\",\"vertical_angle_max_bits\":\"" << bits(sensor->VerticalAngleMax().Radian())
          << "\",\"range_min_bits\":\"" << bits(sensor->RangeMin())
          << "\",\"range_max_bits\":\"" << bits(sensor->RangeMax())
          << "\",\"initial_pose_bits\":";
      poseBits(sensor->Pose());
      std::cout << ",\"rays\":[";
      for (unsigned beam = 0; beam < beams; ++beam)
      {
        if (beam) std::cout << ',';
        std::cout << "{\"beam\":" << beam << ",\"start_bits\":";
        vectorBits(rays[beam].first);
        std::cout << ",\"end_bits\":";
        vectorBits(rays[beam].second);
        std::cout << '}';
      }
      std::cout << "],\"console_verbosity\":" << gz::common::Console::Verbosity()
          << ",\"sdf_xml_hex\":" << std::quoted(hex(xml)) << "}\n";
      owners.push_back({i, topic, frame, std::move(sensor), received, std::move(rays)});
    }
    for (auto &owner : owners) ready(node, owner.topic, *owner.sensor);
    unsigned inputs = 0, scans = 0, elements = 0, generated = 0;
    unsigned finiteRanges = 0, infiniteRanges = 0;
    for (const auto &owner : owners) generated += owner.rays.size();
    for (unsigned row = 0; row < Rows; ++row)
    {
      const long long ns = 1000000LL + 10000000LL * row;
      for (auto &owner : owners)
      {
        const auto pose = inputPose(owner.index, row);
        owner.sensor->SetPose(pose);
        const unsigned beams = owner.rays.size();
        std::vector<Sensor::RayResult> results(beams);
        std::vector<double> physical(beams);
        const std::array<double, 5> finite{0.0, 2.5, 5.0, 7.5, 10.0};
        for (unsigned beam = 0; beam < beams; ++beam)
        {
          const bool miss = row == 1 || (row == 0 && (beam + owner.index) % 3 == 1);
          physical[beam] = miss ? std::numeric_limits<double>::infinity() :
              finite[(beam + owner.index + row) % finite.size()];
          auto &result = results[beam];
          result.fraction = miss ? std::numeric_limits<double>::infinity() : physical[beam] / 10.0;
          result.point = miss ? Vector::Zero : owner.rays[beam].first +
              result.fraction * (owner.rays[beam].second - owner.rays[beam].first);
          result.normal = Vector(0, 0, 1);
          result.intensity = 0.0;
        }
        owner.sensor->SetRaycastResults(results);
        std::vector<double> before;
        owner.sensor->Ranges(before);
        require(before.size() == beams, "complete actual pre-Update range getter");
        for (unsigned beam = 0; beam < beams; ++beam)
          require(rawBits(before[beam]) == rawBits(physical[beam]),
              "retained exact dyadic/no-noise conversion and positive-infinite miss");
        std::cout << "{\"kind\":\"input\",\"mode\":" << std::quoted(mode)
            << ",\"sensor\":" << owner.index << ",\"row\":" << row
            << ",\"time_ns\":" << ns << ",\"pose_input_bits\":";
        poseBits(pose);
        std::cout << ",\"getter_pose_bits\":";
        poseBits(owner.sensor->Pose());
        std::cout << ",\"ray_results\":[";
        for (unsigned beam = 0; beam < beams; ++beam)
        {
          if (beam) std::cout << ',';
          const auto &result = results[beam];
          std::cout << "{\"beam\":" << beam << ",\"fraction_bits\":\""
              << bits(result.fraction, true) << "\",\"point_bits\":";
          vectorBits(result.point);
          std::cout << ",\"normal_bits\":";
          vectorBits(result.normal);
          std::cout << ",\"intensity_bits\":\"" << bits(result.intensity)
              << "\",\"raw_range_bits\":\"" << bits(physical[beam], true) << "\"}";
        }
        std::cout << "],\"ranges_before_update_bits\":";
        rangeBits(before);
        std::cout << "}\n";
        ++inputs;
        elements += beams;
        ready(node, owner.topic, *owner.sensor);
        require(owner.sensor->Update(std::chrono::nanoseconds(ns)),
            "explicit derived one-argument Update");
        const auto message = receive(owner.received, row);
        require(message.has_header() && message.has_world_pose(), "actual full scan fields");
        header(message.header(), ns, row, owner.frame);
        require(message.frame() == owner.frame && message.count() == beams &&
            message.vertical_count() == 1 && message.ranges_size() == static_cast<int>(beams) &&
            message.intensities_size() == static_cast<int>(beams), "actual scan cardinality/frame");
        require(message.angle_min() == owner.sensor->AngleMin().Radian() &&
            message.angle_max() == owner.sensor->AngleMax().Radian() &&
            message.angle_step() == (owner.sensor->AngleMax().Radian() -
                owner.sensor->AngleMin().Radian()) / (beams - 1) &&
            message.range_min() == owner.sensor->RangeMin() &&
            message.range_max() == owner.sensor->RangeMax() &&
            message.vertical_angle_min() == 0.0 && message.vertical_angle_max() == 0.0 &&
            message.vertical_angle_step() == 0.0, "actual scan metadata/getter consistency");
        const auto getterPose = poseValues(owner.sensor->Pose());
        const auto messagePose = poseValues(message.world_pose());
        for (unsigned i = 0; i < 7; ++i)
          require(rawBits(getterPose[i]) == rawBits(messagePose[i]),
              "exact recorded getter-to-message pose handoff, no pose estimator");
        std::vector<double> after;
        owner.sensor->Ranges(after);
        require(after.size() == beams, "complete actual post-Update range getter");
        for (unsigned beam = 0; beam < beams; ++beam)
        {
          require(rawBits(after[beam]) == rawBits(before[beam]) &&
              rawBits(message.ranges(beam)) == rawBits(after[beam]) &&
              message.intensities(beam) == 0.0, "actual range getter/wire and zero intensity");
          if (std::isfinite(message.ranges(beam))) ++finiteRanges;
          else
          {
            require(message.ranges(beam) == std::numeric_limits<double>::infinity(),
                "only positive-infinite observed misses");
            ++infiniteRanges;
          }
        }
        std::cout << "{\"kind\":\"observation\",\"mode\":" << std::quoted(mode)
            << ",\"sensor\":" << owner.index << ",\"row\":" << row
            << ",\"time_ns\":" << ns << ",\"sequence\":" << row
            << ",\"world_pose_bits\":";
        poseBits(message.world_pose());
        std::cout << ",\"getter_pose_after_update_bits\":";
        poseBits(owner.sensor->Pose());
        std::cout << ",\"ranges_after_update_bits\":";
        rangeBits(after);
        std::cout << ",\"range_bits\":[";
        for (unsigned beam = 0; beam < beams; ++beam)
        {
          if (beam) std::cout << ',';
          std::cout << '"' << bits(message.ranges(beam), true) << '"';
        }
        std::cout << "],\"intensity_bits\":[";
        for (unsigned beam = 0; beam < beams; ++beam)
        {
          if (beam) std::cout << ',';
          std::cout << '"' << bits(message.intensities(beam)) << '"';
        }
        std::cout << "],\"header_wire\":" << std::quoted(hex(message.header().SerializeAsString()))
            << ",\"wire\":" << std::quoted(hex(message.SerializeAsString())) << "}\n";
        ++scans;
      }
    }
    // Every mode's two/eight component owners remain alive at this snapshot.
    capture();
    for (auto &owner : owners)
    {
      {
        std::lock_guard<std::mutex> lock(owner.received->mutex);
        require(!owner.received->bad && owner.received->messages.size() == Rows,
            "complete bounded callback inventory");
        owner.received->active->store(false, std::memory_order_release);
      }
      require(node.Unsubscribe(owner.topic), "owned scan subscriber cleanup");
    }
    require(inputs == Rows * count && scans == inputs &&
        generated == (mode == "pair" ? 8u : 1024u) &&
        elements == Rows * generated && finiteRanges + infiniteRanges == elements &&
        infiniteRanges == (mode == "pair" ? 11u : 1366u),
        "closed component input/output counts");
    std::cout << "{\"kind\":\"summary\",\"mode\":" << std::quoted(mode)
        << ",\"configuration_count\":" << count << ",\"generated_ray_count\":" << generated
        << ",\"direct_update_calls\":" << inputs << ",\"input_rows\":" << inputs
        << ",\"observed_scans\":" << scans << ",\"canonical_packets\":" << scans
        << ",\"ray_result_elements\":" << elements << ",\"range_elements\":" << elements
        << ",\"finite_range_elements\":" << finiteRanges
        << ",\"positive_infinite_range_elements\":" << infiniteRanges
        << ",\"original_system_executed\":false,\"native_physics_executed\":false"
        << ",\"native_schedule_or_demand_equivalence_claimed\":false"
        << ",\"native_lifecycle_equivalence_claimed\":false,\"global_rng_equivalence_claimed\":false"
        << ",\"pointcloud_subscribers_created\":false,\"old_native_modes_replayed\":false}\n";
  }
  catch (...)
  {
    // Preserve maps before owner destruction, without replacing the original error.
    const auto original = std::current_exception();
    if (!attemptedMaps)
    {
      try { capture(); }
      catch (const std::exception &error)
      {
        std::cerr << "partial owned maps: " << error.what() << '\n';
      }
    }
    for (auto &owner : owners)
    {
      owner.received->active->store(false, std::memory_order_release);
      node.Unsubscribe(owner.topic);
    }
    std::rethrow_exception(original);
  }
}
}  // namespace

int main(int argc, char **argv)
{
  try
  {
    require(argc == 3, "expected MODE MAPS_PATH");
    std::cout << std::setprecision(17);
    run(argv[1], std::filesystem::path(argv[2]));
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "multiple CPU-lidar component: " << error.what() << '\n';
    return 1;
  }
}
