// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Canonical transport only, following native_multiple_joint_controllers_peer.cc,
// native_robot_sensors.cc and native_multiple_odometry_publishers_peer.cc.
// Odometry is the Rust encoder sample, not an original System/trajectory oracle.
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
namespace m59 {
using Clock = std::chrono::steady_clock;
constexpr std::size_t kQueueCap = 128;
constexpr std::size_t kWireCap = 512;
constexpr std::size_t kOutputCap = 1024 * 1024;

void require(bool value, const std::string &why) {
  if (!value) throw std::runtime_error(why);
}
void before(Clock::time_point deadline, const std::string &why) {
  require(Clock::now() < deadline, why);
}
std::uint64_t decimal(const std::string &text, std::uint64_t maximum) {
  require(!text.empty() && text.size() <= 20
      && text.find_first_not_of("0123456789") == std::string::npos
      && (text == "0" || text.front() != '0'), "canonical unsigned decimal");
  std::size_t used = 0;
  const auto value = std::stoull(text, &used);
  require(used == text.size() && value <= maximum, "unsigned decimal bound");
  return static_cast<std::uint64_t>(value);
}
std::string hex(const std::string &bytes) {
  require(!bytes.empty() && bytes.size() <= kWireCap, "bounded actual wire");
  constexpr char digits[] = "0123456789abcdef";
  std::string result;
  result.reserve(2 * bytes.size());
  for (unsigned char byte : bytes) {
    result += digits[byte >> 4];
    result += digits[byte & 15];
  }
  return result;
}
double scalar(const std::string &word) {
  require(word.size() == 16
      && word.find_first_not_of("0123456789abcdef") == std::string::npos,
      "exact lowercase binary64 word");
  const std::uint64_t bits = std::stoull(word, nullptr, 16);
  double value = 0;
  static_assert(sizeof(value) == sizeof(bits), "binary64 width");
  std::memcpy(&value, &bits, sizeof(value));
  require(std::isfinite(value), "finite canonical command");
  return value;
}
std::string side(std::size_t owner) { return owner == 0 ? "left" : "right"; }
std::string command_topic(std::size_t owner) {
  return "/m59/" + side(owner) + "/cmd_vel";
}
std::string odometry_topic(std::size_t owner) {
  return "/m59/" + side(owner) + "/odom";
}
using Stamp = std::pair<std::int64_t, std::int32_t>;
struct Observation {
  Stamp stamp;
  std::uint64_t sequence;
};
bool finite_vector(const gz::msgs::Vector3d &value) {
  return !value.has_header() && std::isfinite(value.x()) && std::isfinite(value.y())
      && std::isfinite(value.z());
}
Observation checked(const gz::msgs::Odometry &message, std::size_t owner) {
  require(message.has_header() && message.has_pose() && message.has_twist(),
      "complete canonical Odometry");
  const auto &header = message.header();
  require(header.has_stamp() && header.data_size() == 3,
      "stamp and exactly three ordered header entries");
  const auto &time = header.stamp();
  require(time.sec() >= 0 && time.nsec() >= 0 && time.nsec() < 1000000000,
      "normalized nonnegative acquisition stamp");
  require(header.data(0).key() == "frame_id" && header.data(0).value_size() == 1
      && header.data(0).value(0) == "m59_" + side(owner) + "_odom"
      && header.data(1).key() == "child_frame_id" && header.data(1).value_size() == 1
      && header.data(1).value(0) == "m59_" + side(owner) + "_base"
      && header.data(2).key() == "seq" && header.data(2).value_size() == 1,
      "exact owner frame pair and sequence order");
  const auto sequence = m59::decimal(header.data(2).value(0),
      std::numeric_limits<std::uint64_t>::max());
  const auto &pose = message.pose();
  require(pose.has_position() && pose.has_orientation() && !pose.has_header()
      && pose.name().empty() && pose.id() == 0, "default nested pose metadata");
  const auto &position = pose.position();
  const auto &rotation = pose.orientation();
  require(m59::finite_vector(position) && position.z() == 0.0
      && !rotation.has_header() && std::isfinite(rotation.x()) && std::isfinite(rotation.y())
      && std::isfinite(rotation.z()) && std::isfinite(rotation.w())
      && rotation.x() == 0.0 && rotation.y() == 0.0
      && std::abs(rotation.z() * rotation.z() + rotation.w() * rotation.w() - 1.0)
          <= 1e-12, "finite planar pose and unit yaw quaternion");
  const auto &twist = message.twist();
  require(!twist.has_header() && twist.has_linear() && twist.has_angular()
      && m59::finite_vector(twist.linear()) && m59::finite_vector(twist.angular())
      && twist.linear().y() == 0.0 && twist.linear().z() == 0.0
      && twist.angular().x() == 0.0 && twist.angular().y() == 0.0,
      "finite planar twist and zero unused axes");
  return {{time.sec(), time.nsec()}, sequence};
}
struct Watch {
  std::mutex mutex;
  std::condition_variable changed;
  std::deque<std::string> rows;
  bool retiring = false;
  bool have_previous = false;
  Observation previous{{0, 0}, 0};
};

struct Peer {
  // Callbacks capture only their own state and this shared terminal flag.
  gz::transport::Node node;
  std::vector<gz::transport::Node::Publisher> publishers;
  std::vector<std::shared_ptr<Watch>> watches;
  std::shared_ptr<std::atomic<bool>> callback_failed =
      std::make_shared<std::atomic<bool>>(false);
  std::mutex output_mutex;
  std::size_t output_bytes = 0;

  explicit Peer(std::size_t owners) : watches(owners) {
    for (std::size_t owner = 0; owner < owners; ++owner) {
      auto publisher = node.Advertise<gz::msgs::Twist>(m59::command_topic(owner));
      require(static_cast<bool>(publisher), "actual canonical Twist advertisement");
      publishers.push_back(publisher);
    }
  }
  void healthy() const {
    require(!callback_failed->load(std::memory_order_acquire),
        "recorded Odometry callback validation, queue or retirement failure");
  }
  void emit(const std::string &line, Clock::time_point deadline) {
    healthy();
    std::lock_guard<std::mutex> lock(output_mutex);
    m59::before(deadline, "phase deadline before actual output");
    require(line.size() + 1 <= kOutputCap - output_bytes,
        "one MiB stdout cap before write");
    output_bytes += line.size() + 1;
    std::cout << line << std::endl;
    require(static_cast<bool>(std::cout), "actual protocol stdout write/flush");
    m59::before(deadline, "same phase deadline after actual output/flush");
  }
  void discovery(std::size_t owner, bool connected, Clock::time_point deadline) {
    for (;;) {
      healthy();
      std::vector<gz::transport::MessagePublisher> pubs, subs;
      require(node.TopicInfo(m59::command_topic(owner), pubs, subs),
          "actual command TopicInfo");
      const bool own_publisher = pubs.size() == 1
          && pubs[0].MsgTypeName() == "gz.msgs.Twist";
      bool complete = own_publisher;
      if (connected) {
        complete = complete && publishers[owner].HasConnections()
            && subs.size() == 1 && subs[0].MsgTypeName() == "gz.msgs.Twist";
      } else {
        complete = complete && !publishers[owner].HasConnections() && subs.empty();
        std::vector<gz::transport::MessagePublisher> odom_pubs, odom_subs;
        const bool found = node.TopicInfo(m59::odometry_topic(owner), odom_pubs, odom_subs);
        // An entirely absent topic is allowed only when this peer has no watch.
        complete = complete && odom_pubs.empty()
            && ((!watches[owner] && odom_subs.empty())
                || (watches[owner] && found && odom_subs.size() == 1
                    && odom_subs[0].MsgTypeName() == "gz.msgs.Odometry"));
      }
      m59::before(deadline, "actual typed discovery/retirement five-second deadline");
      if (complete) return;
      std::this_thread::yield();
    }
  }
  void send(std::size_t owner, const std::string &linear, const std::string &angular,
      Clock::time_point deadline) {
    healthy();
    gz::msgs::Twist message;
    message.mutable_linear()->set_x(m59::scalar(linear));
    message.mutable_linear()->set_y(0.0);
    message.mutable_linear()->set_z(0.0);
    message.mutable_angular()->set_x(0.0);
    message.mutable_angular()->set_y(0.0);
    message.mutable_angular()->set_z(m59::scalar(angular));
    const auto wire = message.SerializeAsString();
    require(!wire.empty() && wire.size() <= kWireCap, "actual Twist wire bound");
    require(publishers[owner].Publish(message), "actual typed Twist Publish");
    emit("SENT " + std::to_string(owner) + " " + linear + " " + angular + " " + m59::hex(wire), deadline);
  }
  void watch(std::size_t owner, Clock::time_point deadline) {
    healthy();
    require(!watches[owner], "watch requires an absent subscription");
    auto state = std::make_shared<Watch>();
    watches[owner] = state;
    const auto failed = callback_failed;
    std::function<void(const gz::msgs::Odometry &)> callback =
        [state, failed, owner](const gz::msgs::Odometry &message) noexcept {
      try {
        require(message.ByteSizeLong() <= kWireCap, "received wire cap before serialization");
        const auto wire = message.SerializeAsString();
        std::lock_guard<std::mutex> lock(state->mutex);
        require(!state->retiring, "callback after retirement began");
        require(!wire.empty() && wire.size() <= kWireCap
            && state->rows.size() < kQueueCap, "actual bounded Odometry FIFO");
        // Retain the actual bounded row before validating it; any fault is terminal.
        state->rows.push_back(wire);
        const auto row = m59::checked(message, owner);
        require(!state->have_previous || (row.stamp > state->previous.stamp
            && row.sequence == state->previous.sequence + std::uint64_t{1}),
            "strict segment stamp order and wrapping contiguous sequence");
        state->previous = row;
        state->have_previous = true;
      } catch (...) {
        failed->store(true, std::memory_order_release);
      }
      state->changed.notify_all();
    };
    require(node.Subscribe(m59::odometry_topic(owner), callback), "actual Odometry Subscribe");
    m59::before(deadline, "subscription five-second deadline after work");
    healthy();
  }
  void take(std::size_t owner, std::size_t count, Clock::time_point deadline) {
    healthy();
    const auto state = watches[owner];
    require(static_cast<bool>(state), "take requires a present watch");
    std::vector<std::string> actual;
    {
      std::unique_lock<std::mutex> lock(state->mutex);
      require(state->changed.wait_until(lock, deadline, [&] {
        return callback_failed->load(std::memory_order_acquire) || state->rows.size() >= count;
      }), "actual Odometry row deadline");
      healthy();
      m59::before(deadline, "Odometry deadline after successful wait");
      for (std::size_t row = 0; row < count; ++row) {
        actual.push_back(std::move(state->rows.front()));
        state->rows.pop_front();
      }
    }
    for (const auto &wire : actual) {
      emit("ODOM " + std::to_string(owner) + " " + m59::hex(wire), deadline);
      m59::before(deadline, "Odometry deadline after actual row output");
    }
    emit("TAKEN " + std::to_string(owner) + " " + std::to_string(count), deadline);
    m59::before(deadline, "Odometry deadline after completion output");
  }
  void unwatch(std::size_t owner, Clock::time_point deadline) {
    healthy();
    const auto state = watches[owner];
    require(static_cast<bool>(state), "unwatch requires a present watch");
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      require(state->rows.empty(), "drain every actual row before unsubscribe");
      state->retiring = true;
    }
    require(node.Unsubscribe(m59::odometry_topic(owner)), "actual public Unsubscribe");
    // Only the owner's slot and this local reference may remain. Any dispatched
    // callback owns a separate reference and must finish before the fresh segment.
    while (state.use_count() != 2) {
      healthy();
      m59::before(deadline, "callback-state retirement deadline");
      std::this_thread::yield();
    }
    healthy();
    m59::before(deadline, "callback-state retirement deadline after work");
    watches[owner].reset();
  }
  void finish(Clock::time_point deadline) {
    healthy();
    for (std::size_t owner = 0; owner < watches.size(); ++owner) {
      if (watches[owner]) unwatch(owner, deadline);
    }
    healthy();
    emit("PASS canonical multiple diff drives", deadline);
    healthy();
  }
};
}  // namespace m59

int main(int argc, char **argv) {
  try {
    m59::require(argc == 2 && (std::string(argv[1]) == "1" || std::string(argv[1]) == "2"),
        "one exact owner-count argument: 1 or 2");
    const auto owners = static_cast<std::size_t>(m59::decimal(argv[1], 2));
    const auto startup_deadline = m59::Clock::now() + 5s;
    m59::Peer peer(owners);
    for (std::size_t owner = 0; owner < owners; ++owner) {
      peer.discovery(owner, true, startup_deadline);
    }
    peer.emit("READY multiple-diff-drives " + std::to_string(owners), startup_deadline);
    for (unsigned phase = 0; phase < 4096; ++phase) {
      peer.healthy();
      char buffer[129]{};
      m59::require(static_cast<bool>(std::cin.getline(buffer, sizeof(buffer)))
          && !std::cin.eof(), "bounded newline-terminated phase or premature EOF");
      const auto deadline = m59::Clock::now() + 5s;
      const auto length = static_cast<std::size_t>(std::cin.gcount() - 1);
      const std::string line(buffer, length);
      m59::require(std::all_of(line.begin(), line.end(), [](unsigned char value) {
        return value >= 0x20 && value <= 0x7e;
      }), "printable ASCII phase without embedded NUL");
      std::istringstream input(line);
      std::string command, owner_text, linear, angular, count_text, extra;
      m59::require(static_cast<bool>(input >> command), "nonempty protocol phase");
      if (command == "finish") {
        m59::require(!(input >> extra), "exact finish arity");
        peer.finish(deadline);
        return 0;
      }
      m59::require(static_cast<bool>(input >> owner_text), "required owner");
      const auto owner = static_cast<std::size_t>(m59::decimal(owner_text, owners - 1));
      if (command == "send") {
        m59::require(static_cast<bool>(input >> linear >> angular) && !(input >> extra),
            "exact send arity");
        peer.send(owner, linear, angular, deadline);
      } else if (command == "take") {
        m59::require(static_cast<bool>(input >> count_text) && !(input >> extra),
            "exact take arity");
        const auto count = static_cast<std::size_t>(m59::decimal(count_text, m59::kQueueCap));
        m59::require(count != 0, "positive row count");
        peer.take(owner, count, deadline);
      } else {
        m59::require(!(input >> extra), "exact owner-phase arity");
        if (command == "watch") {
          peer.watch(owner, deadline);
          peer.emit("WATCHING " + std::to_string(owner), deadline);
        } else if (command == "unwatch") {
          peer.unwatch(owner, deadline);
          peer.emit("UNWATCHED " + std::to_string(owner), deadline);
        } else if (command == "matched" || command == "retired") {
          peer.discovery(owner, command == "matched", deadline);
          peer.emit((command == "matched" ? "MATCHED " : "RETIRED ")
              + std::to_string(owner), deadline);
        } else {
          throw std::runtime_error("unknown finite protocol phase");
        }
      }
      peer.healthy();
    }
    throw std::runtime_error("4096-phase cap");
  } catch (const std::exception &error) {
    // One bounded diagnostic; no callback writes to either protocol channel.
    std::cerr << "native multiple diff drives peer: ";
    std::size_t count = 0;
    while (count < 1000 && error.what()[count] != '\0') ++count;
    std::cerr.write(error.what(), static_cast<std::streamsize>(count));
    std::cerr << '\n';
    return 1;
  } catch (...) {
    std::cerr << "native multiple diff drives peer: unknown terminal failure\n";
    return 1;
  }
}
