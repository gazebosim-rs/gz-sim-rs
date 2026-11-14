// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Canonical Double transport only; no original System, actuation, or trajectory oracle.
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace {
void require(bool value, const std::string &why) {
  if (!value) throw std::runtime_error(why);
}
std::size_t natural(const std::string &value, std::size_t maximum) {
  require(!value.empty() && value.find_first_not_of("0123456789") == std::string::npos,
          "bounded natural number");
  std::size_t consumed = 0;
  const auto result = std::stoull(value, &consumed);
  require(consumed == value.size() && result <= maximum, "natural number range");
  return static_cast<std::size_t>(result);
}
std::string hex(const std::string &bytes) {
  if (bytes.empty()) return "-";
  constexpr char digits[] = "0123456789abcdef";
  std::string result;
  for (unsigned char byte : bytes) {
    result += digits[byte >> 4]; result += digits[byte & 15];
  }
  return result;
}
double scalar(const std::string &word) {
  require(word.size() == 16 && word.find_first_not_of("0123456789abcdef") == std::string::npos,
          "exact lowercase IEEE word");
  const std::uint64_t bits = std::stoull(word, nullptr, 16);
  double result = 0;
  static_assert(sizeof(result) == sizeof(bits), "binary64 width");
  std::memcpy(&result, &bits, sizeof(result));
  require(std::isfinite(result), "finite canonical Double");
  return result;
}
struct Peer {
  gz::transport::Node node;
  std::vector<std::string> topics;
  std::vector<gz::transport::Node::Publisher> publishers;
  explicit Peer(std::size_t owners) {
    for (std::size_t owner = 0; owner < owners; ++owner) {
      const std::string model = owner == 0 ? "arm_left" : "arm_right";
      topics.push_back("/model/" + model + "/joint/base_hinge/0/cmd_pos");
      topics.push_back("/model/" + model + "/joint/tip_hinge/cmd_vel");
    }
    for (const auto &topic : topics) {
      auto publisher = node.Advertise<gz::msgs::Double>(topic);
      require(static_cast<bool>(publisher), "canonical Double advertisement");
      publishers.push_back(publisher);
    }
  }
  void matched(std::size_t owner, bool expected) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    for (;;) {
      bool complete = true;
      for (std::size_t slot = 2 * owner; slot < 2 * owner + 2; ++slot) {
        std::vector<gz::transport::MessagePublisher> pubs, subs;
        require(node.TopicInfo(topics[slot], pubs, subs), "actual topic discovery");
        if (expected) {
          complete = complete && publishers[slot].HasConnections() && pubs.size() == 1
              && subs.size() == 1 && pubs[0].MsgTypeName() == "gz.msgs.Double"
              && subs[0].MsgTypeName() == "gz.msgs.Double";
        } else {
          complete = complete && !publishers[slot].HasConnections() && subs.empty();
        }
      }
      if (complete) return;
      require(std::chrono::steady_clock::now() < deadline, "typed discovery/retirement deadline");
      std::this_thread::yield();
    }
  }
};
}
int main(int argc, char **argv) {
  try {
    require(argc == 2, "one owner-count argument");
    require(std::string(argv[1]) == "1" || std::string(argv[1]) == "2", "one or two owners");
    const auto owners = natural(argv[1], 2);
    Peer peer(owners);
    for (std::size_t owner = 0; owner < owners; ++owner) peer.matched(owner, true);
    std::cout << "READY multiple-joint-controllers " << owners << std::endl;
    for (unsigned phase = 0; phase < 4096; ++phase) {
      char buffer[129]{};
      require(static_cast<bool>(std::cin.getline(buffer, sizeof(buffer))), "bounded phase/EOF");
      std::istringstream input(buffer);
      std::string command, argument, word, extra;
      input >> command;
      if (command == "finish") {
        require(!(input >> extra), "exact finish");
        std::cout << "PASS canonical multiple joint controllers" << std::endl;
        return 0;
      }
      require(static_cast<bool>(input >> argument), "phase argument");
      if (command == "send") {
        require(static_cast<bool>(input >> word) && !(input >> extra), "exact send");
        const auto slot = natural(argument, peer.publishers.size() - 1);
        gz::msgs::Double message; message.set_data(scalar(word));
        const auto bytes = message.SerializeAsString();
        require(bytes.size() <= 32, "canonical Double wire cap");
        require(peer.publishers[slot].Publish(message), "actual typed publish");
        std::cout << "SENT " << slot << ' ' << word << ' ' << hex(bytes) << std::endl;
      } else if (command == "matched" || command == "retired") {
        require(!(input >> extra), "exact owner phase");
        const auto owner = natural(argument, owners - 1);
        peer.matched(owner, command == "matched");
        std::cout << (command == "matched" ? "MATCHED " : "RETIRED ") << owner << std::endl;
      } else throw std::runtime_error("unknown finite phase");
    }
    throw std::runtime_error("phase-count cap");
  } catch (const std::exception &error) {
    std::cerr << "native multiple joint controllers peer: " << error.what() << '\n';
    return 1;
  }
}
