// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// New TU linked to ONE unchanged original object. No SystemLoader or Physics.
#include "OdometryPublisher.hh"
#include <gz/common/Console.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
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
namespace c = gz::sim::components;
using Entity = gz::sim::Entity;
static_assert(sizeof(double)==8 && sizeof(float)==4, "fixed IEEE scalar widths");
void require(bool ok, const std::string &why) {
  if (!ok) throw std::runtime_error(why);
}
std::string hex(const std::string &s) {
  constexpr char digits[] = "0123456789abcdef";
  std::string out;
  out.reserve(s.size() * 2);
  for (unsigned char ch : s) { out += digits[ch >> 4]; out += digits[ch & 15]; }
  return out;
}
std::string bits(double x) {
  require(std::isfinite(x), "finite f64");
  std::uint64_t n;
  std::memcpy(&n, &x, sizeof(n));
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << n;
  return out.str();
}
std::string floatBits(float x) {
  require(std::isfinite(x), "finite f32 covariance");
  std::uint32_t n;
  std::memcpy(&n, &x, sizeof(n));
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(8) << n;
  return out.str();
}
void poseBits(std::ostream &out, const gz::math::Pose3d &p) {
  const double v[] = {p.Pos().X(), p.Pos().Y(), p.Pos().Z(), p.Rot().X(),
                      p.Rot().Y(), p.Rot().Z(), p.Rot().W()};
  require(std::abs(v[3]*v[3]+v[4]*v[4]+v[5]*v[5]+v[6]*v[6]-1) <= 1e-12, "unit input pose");
  out << '[';
  for (unsigned i = 0; i < 7; ++i) { if (i) out << ','; out << std::quoted(bits(v[i])); }
  out << ']';
}
long long stamp(const gz::msgs::Header &h) {
  require(h.has_stamp(), "present actual stamp");
  const auto &t = h.stamp();
  require(t.sec() >= 0 && t.nsec() >= 0 && t.nsec() < 1000000000 &&
          t.sec() <= (LLONG_MAX - t.nsec()) / 1000000000LL, "stamp bounds");
  return t.sec() * 1000000000LL + t.nsec();
}
void header(std::ostream &out, bool present, const gz::msgs::Header &h) {
  out << "{\"present\":" << (present ? "true" : "false");
  if (present) {
    require(h.ByteSizeLong() <= 4096 && h.data_size() <= 16, "bounded header");
    out << ",\"stamp_present\":" << (h.has_stamp() ? "true" : "false");
    if (h.has_stamp()) out << ",\"sec\":" << h.stamp().sec() << ",\"nsec\":" << h.stamp().nsec();
    out << ",\"data\":[";
    for (int i = 0; i < h.data_size(); ++i) {
      if (i) out << ',';
      require(h.data(i).key().size() <= 256 && h.data(i).value_size() <= 8, "header item cap");
      out << "{\"key_hex\":" << std::quoted(hex(h.data(i).key())) << ",\"values_hex\":[";
      for (int j = 0; j < h.data(i).value_size(); ++j) {
        require(h.data(i).value(j).size() <= 1024, "header value cap");
        if (j) out << ',';
        out << std::quoted(hex(h.data(i).value(j)));
      }
      out << "]}";
    }
    out << "],\"wire\":" << std::quoted(hex(h.SerializeAsString()));
  }
  out << '}';
}
void frame(const gz::msgs::Header &h, long long ns,
           const std::string &parent, const std::string &child) {
  require(stamp(h) == ns && h.data_size() == 2 &&
          h.data(0).key() == "frame_id" && h.data(0).value_size() == 1 &&
          h.data(0).value(0) == parent && h.data(1).key() == "child_frame_id" &&
          h.data(1).value_size() == 1 && h.data(1).value(0) == child,
          "actual ordered owner frames/stamp");
}
void vector(std::ostream &out, const gz::msgs::Vector3d &v) {
  out << "{\"header\":"; header(out, v.has_header(), v.header());
  out << ",\"bits\":[" << std::quoted(bits(v.x())) << ',' << std::quoted(bits(v.y()))
      << ',' << std::quoted(bits(v.z())) << "],\"wire\":" << std::quoted(hex(v.SerializeAsString())) << '}';
}
void pose(std::ostream &out, const gz::msgs::Pose &p) {
  require(p.name().size() <= 256 && p.has_position() && p.has_orientation(), "complete bounded captured pose");
  const auto &q = p.orientation();
  require(std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1) <= 1e-12,
          "unit captured orientation");
  out << "{\"header\":"; header(out, p.has_header(), p.header());
  out << ",\"name_hex\":" << std::quoted(hex(p.name())) << ",\"id\":" << p.id()
      << ",\"position\":"; vector(out, p.position());
  out << ",\"orientation\":{\"header\":"; header(out, q.has_header(), q.header());
  out << ",\"bits\":[" << std::quoted(bits(q.x())) << ',' << std::quoted(bits(q.y()))
      << ',' << std::quoted(bits(q.z())) << ',' << std::quoted(bits(q.w()))
      << "],\"wire\":" << std::quoted(hex(q.SerializeAsString())) << "},\"wire\":"
      << std::quoted(hex(p.SerializeAsString())) << '}';
}
void twist(std::ostream &out, const gz::msgs::Twist &t) {
  require(t.has_linear() && t.has_angular(), "complete actual twist");
  out << "{\"header\":"; header(out, t.has_header(), t.header());
  out << ",\"linear\":"; vector(out, t.linear());
  out << ",\"angular\":"; vector(out, t.angular());
  out << ",\"wire\":" << std::quoted(hex(t.SerializeAsString())) << '}';
}
void covariance(std::ostream &out, const gz::msgs::Float_V &v) {
  require(v.data_size() == 36, "all actual covariance words");
  out << "{\"header\":"; header(out, v.has_header(), v.header());
  out << ",\"f32_bits\":[";
  for (int i = 0; i < v.data_size(); ++i) { if (i) out << ','; out << std::quoted(floatBits(v.data(i))); }
  out << "],\"wire\":" << std::quoted(hex(v.SerializeAsString())) << '}';
}
std::string payload(const gz::msgs::Odometry &m, long long ns,
                    const std::string &parent, const std::string &child) {
  require(m.has_header() && m.has_pose() && m.has_twist(), "complete Odometry");
  frame(m.header(), ns, parent, child);
  std::ostringstream out;
  out << "{\"header\":"; header(out, true, m.header());
  out << ",\"pose\":"; pose(out, m.pose());
  out << ",\"twist\":"; twist(out, m.twist());
  out << ",\"wire\":" << std::quoted(hex(m.SerializeAsString())) << '}';
  return out.str();
}
std::string payload(const gz::msgs::OdometryWithCovariance &m, long long ns,
                    const std::string &parent, const std::string &child) {
  require(m.has_header() && m.has_pose_with_covariance() && m.has_twist_with_covariance(), "complete covariance message");
  frame(m.header(), ns, parent, child);
  const auto &p = m.pose_with_covariance(); const auto &t = m.twist_with_covariance();
  require(p.has_pose() && p.has_covariance() && t.has_twist() && t.has_covariance(), "present nested covariance fields");
  std::ostringstream out;
  out << "{\"header\":"; header(out, true, m.header());
  out << ",\"pose\":"; pose(out, p.pose());
  out << ",\"pose_covariance\":"; covariance(out, p.covariance());
  out << ",\"twist\":"; twist(out, t.twist());
  out << ",\"twist_covariance\":"; covariance(out, t.covariance());
  out << ",\"pose_with_covariance_wire\":" << std::quoted(hex(p.SerializeAsString()))
      << ",\"twist_with_covariance_wire\":" << std::quoted(hex(t.SerializeAsString()))
      << ",\"wire\":" << std::quoted(hex(m.SerializeAsString())) << '}';
  return out.str();
}
std::string payload(const gz::msgs::Pose_V &m, long long ns,
                    const std::string &parent, const std::string &child) {
  require(!m.has_header() && m.pose_size() == 1, "original one-pose vector/absent outer header");
  require(m.pose(0).has_header(), "actual per-Pose header");
  frame(m.pose(0).header(), ns, parent, child);
  std::ostringstream out;
  out << "{\"header\":"; header(out, false, m.header());
  out << ",\"poses\":["; pose(out, m.pose(0));
  out << "],\"wire\":" << std::quoted(hex(m.SerializeAsString())) << '}';
  return out.str();
}
struct Packet { unsigned call, channel, ordinal; long long ns; std::string data; };
struct Received {
  std::mutex mutex;
  std::condition_variable changed;
  std::atomic<bool> active{true};
  std::vector<Packet> packets;
  unsigned call=0, total=0;
  long long ns=0;
  bool armed=false, bad=false;
  std::string failure;
};
const char *channelName(unsigned channel) {
  require(channel < 3, "channel index");
  return channel == 0 ? "odometry" : channel == 1 ? "odometry_with_covariance" : "pose_vector";
}
template<class Message>
void subscribe(gz::transport::Node &node, const std::string &topic, unsigned channel,
               const std::string &parent, const std::string &child,
               const std::shared_ptr<Received> &received) {
  std::function<void(const Message &)> callback = [received, channel, parent, child](const Message &m) {
    std::lock_guard<std::mutex> lock(received->mutex);
    if (!received->active.load(std::memory_order_acquire) || received->bad) return;
    try {
      require(received->armed && received->packets.size() < 3 && received->total < 36,
              "armed bounded channel callback before copy");
      require(m.ByteSizeLong() <= 65536, "message cap before copy/serialization");
      for (const auto &p : received->packets) require(p.channel != channel, "no duplicate call/channel");
      const auto data = payload(m, received->ns, parent, child);
      require(data.size() <= 262144, "expanded packet cap");
      received->packets.push_back({received->call, channel, received->total++, received->ns, data});
    } catch (const std::exception &e) { received->bad=true; received->failure=e.what(); }
    received->changed.notify_all();
  };
  require(node.Subscribe(topic, callback), "typed channel subscription");
}
void ready(gz::transport::Node &node, const std::string &topic, const std::string &type) {
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  for (;;) {
    std::vector<gz::transport::MessagePublisher> pubs, subs;
    node.TopicInfo(topic, pubs, subs);
    if (pubs.size()==1 && subs.size()==1 && pubs[0].MsgTypeName()==type && subs[0].MsgTypeName()==type) return;
    require(std::chrono::steady_clock::now() < deadline, "typed complete channel discovery");
    std::this_thread::yield();
  }
}
void maps(const std::string &path) {
  std::ifstream input("/proc/self/maps"); require(bool(input), "maps open");
  std::string text, line;
  while (std::getline(input,line)) {
    require(line.find("odometry-publisher-system") == std::string::npos, "original rejected System DSO mapped");
    require(text.size()+line.size()+1 <= 4*1024*1024, "maps cap"); text += line+'\n';
  }
  require(input.eof(), "complete maps read");
  const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);
  require(fd>=0, "exclusive maps output");
  std::size_t offset=0;
  while (offset<text.size()) {
    const auto n=::write(fd,text.data()+offset,text.size()-offset);
    if (n<=0) { ::close(fd); throw std::runtime_error("maps write"); }
    offset+=static_cast<std::size_t>(n);
  }
  const bool flushed=::fsync(fd)==0; const bool closed=::close(fd)==0;
  require(flushed && closed, "durable maps completion");
}
struct Owner {
  unsigned index, dimensions;
  Entity model;
  std::string name, topic, odomFrame, baseFrame;
  double frequency;
  gz::math::Pose3d offset, globalOffset;
  std::unique_ptr<gz::sim::System> system;
  gz::sim::ISystemPreUpdate *pre;
  gz::sim::ISystemPostUpdate *post;
  std::shared_ptr<Received> received;
};
gz::math::Pose3d controlled(unsigned owner, unsigned call) {
  // Explicit controlled inputs only; no expected output/trajectory reconstruction.
  return gz::math::Pose3d(1+owner+.125*call, 2+.25*owner-.0625*call,
                        3+.03125*call, .015625*call, -.03125*call, .125*owner+.0625*call);
}
void entityInput(std::ostream &out, const gz::sim::EntityComponentManager &ecm,
                 Entity model, Entity world, const gz::math::Pose3d &requested) {
  const auto *p=ecm.Component<c::Pose>(model);
  const auto *parent=ecm.Component<c::ParentEntity>(model);
  const auto *wp=ecm.Component<c::Pose>(world);
  require(p && parent && wp && parent->Data()==world && ecm.ParentEntity(model)==world,
          "actual complete LOCAL/component+graph parent inputs");
  require(p->Data()==requested, "requested LOCAL pose equals actual getback");
  out << "\"native_entity\":" << model << ",\"parent_component\":" << parent->Data()
      << ",\"parent_graph\":" << ecm.ParentEntity(model) << ",\"world_entity\":" << world
      << ",\"input_local_pose_bits\":"; poseBits(out, requested);
  out << ",\"getter_local_pose_bits\":"; poseBits(out, p->Data());
  out << ",\"world_local_pose_bits\":"; poseBits(out, wp->Data());
  out << ",\"getter_world_pose_bits\":"; poseBits(out, gz::sim::worldPose(model,ecm));
}
}
int main(int argc, char **argv) {
  bool captured=false;
  try {
    require(argc==3, "expected MODE MAPS_PATH");
    const std::string mode=argv[1]; require(mode=="pair" || mode=="eight", "closed modes");
    const unsigned count=mode=="pair" ? 2 : 8;
    const std::filesystem::path mapPath(argv[2]); require(mapPath.is_absolute(), "owned absolute maps path");
    require(gz::common::Console::Init(mapPath.parent_path().string(),mapPath.filename().string()+".console.log"), "owned Console Init");
    require(gz::common::Console::Directory()==mapPath.parent_path().string(), "owned Console directory");
    gz::common::Console::SetVerbosity(2); require(gz::common::Console::Verbosity()==2, "actual warning-visible Console2");
    gz::sim::EntityComponentManager ecm; gz::sim::EventManager events;
    const auto world=ecm.CreateEntity(); ecm.CreateComponent(world,c::World());
    ecm.CreateComponent(world,c::Name("reference_world"));
    ecm.CreateComponent(world,c::Pose(gz::math::Pose3d(.5,-.25,.125,0,0,.25)));
    gz::transport::Node node;
    std::vector<Owner> owners;
    std::set<std::string> keys;
    for (unsigned i=0;i<count;++i) {
      const unsigned dimensions=i%2==0 ? 2 : 3; const double rate=i%2==0 ? 100 : 50;
      const std::string name="rig"+std::to_string(i);
      const std::string topic="/m56/native/"+mapPath.parent_path().filename().string()+"/"+mode+"/owner"+std::to_string(i);
      const std::string odomFrame=name+"/odom", baseFrame=name+"/base_footprint";
      for (const auto &key : {topic+"/odom",topic+"/covariance",topic+"/pose"})
        require(keys.insert(key).second, "complete unique channel keys");
      const auto model=ecm.CreateEntity(); ecm.CreateComponent(model,c::Model());
      ecm.CreateComponent(model,c::Name(name)); ecm.CreateComponent(model,c::Pose(controlled(i,0)));
      require(ecm.SetParentEntity(model,world), "model graph parent");
      const gz::math::Pose3d offset(.125,-.25,.0625,.0625,-.03125,.125);
      const gz::math::Pose3d global(.25,.125,-.125,-.03125,.0625,-.125);
      std::ostringstream content;
      content << std::setprecision(17) << "<dimensions>" << dimensions << "</dimensions><odom_publish_frequency>" << rate
              << "</odom_publish_frequency><gaussian_noise>0</gaussian_noise><odom_frame>" << odomFrame
              << "</odom_frame><robot_base_frame>" << baseFrame << "</robot_base_frame><odom_topic>" << topic
              << "/odom</odom_topic><odom_covariance_topic>" << topic << "/covariance</odom_covariance_topic><tf_topic>" << topic
              << "/pose</tf_topic><xyz_offset>0.125 -0.25 0.0625</xyz_offset><rpy_offset>0.0625 -0.03125 0.125</rpy_offset>"
              << "<xyz_global_offset>0.25 0.125 -0.125</xyz_global_offset><rpy_global_offset>-0.03125 0.0625 -0.125</rpy_global_offset>";
      sdf::Plugin plugin("direct-original-object-not-loader-input","gz::sim::systems::OdometryPublisher",content.str());
      const auto parsed=plugin.ToElement();
      require(bool(parsed) && parsed->Get<int>("dimensions")==static_cast<int>(dimensions) &&
              parsed->Get<double>("odom_publish_frequency")==rate && parsed->Get<double>("gaussian_noise")==0 &&
              parsed->Get<std::string>("odom_frame")==odomFrame &&
              parsed->Get<std::string>("robot_base_frame")==baseFrame &&
              parsed->Get<std::string>("odom_topic")==topic+"/odom" &&
              parsed->Get<std::string>("odom_covariance_topic")==topic+"/covariance" &&
              parsed->Get<std::string>("tf_topic")==topic+"/pose", "genuine nonnull parsed configuration");
      std::unique_ptr<gz::sim::System> system(new gz::sim::systems::OdometryPublisher());
      auto *configure=dynamic_cast<gz::sim::ISystemConfigure*>(system.get());
      auto *pre=dynamic_cast<gz::sim::ISystemPreUpdate*>(system.get());
      auto *post=dynamic_cast<gz::sim::ISystemPostUpdate*>(system.get());
      require(configure && pre && post, "checked original public System interfaces");
      configure->Configure(model,parsed,ecm,events);
      auto received=std::make_shared<Received>();
      subscribe<gz::msgs::Odometry>(node,topic+"/odom",0,odomFrame,baseFrame,received);
      subscribe<gz::msgs::OdometryWithCovariance>(node,topic+"/covariance",1,odomFrame,baseFrame,received);
      subscribe<gz::msgs::Pose_V>(node,topic+"/pose",2,odomFrame,baseFrame,received);
      owners.push_back({i,dimensions,model,name,topic,odomFrame,baseFrame,rate,offset,global,std::move(system),pre,post,received});
      std::cout << "{\"kind\":\"configuration\",\"schema_version\":1,\"mode\":" << std::quoted(mode)
                << ",\"owner\":" << i << ",\"world_name\":\"reference_world\",\"name\":" << std::quoted(name)
                << ",\"dimensions\":" << dimensions << ",\"frequency_bits\":" << std::quoted(bits(rate))
                << ",\"gaussian_noise_bits\":" << std::quoted(bits(0)) << ",\"odom_frame\":" << std::quoted(odomFrame)
                << ",\"robot_base_frame\":" << std::quoted(baseFrame) << ",\"topics\":[" << std::quoted(topic+"/odom") << ','
                << std::quoted(topic+"/covariance") << ',' << std::quoted(topic+"/pose") << "],\"offset_pose_bits\":";
      poseBits(std::cout,offset); std::cout << ",\"global_offset_pose_bits\":"; poseBits(std::cout,global);
      std::cout << ",\"console_verbosity\":" << gz::common::Console::Verbosity()
                << ",\"sdf_xml_hex\":" << std::quoted(hex(parsed->ToString(""))) << ',';
      entityInput(std::cout,ecm,model,world,controlled(i,0)); std::cout << "}\n";
    }
    for (const auto &o:owners) {
      ready(node,o.topic+"/odom","gz.msgs.Odometry");
      ready(node,o.topic+"/covariance","gz.msgs.OdometryWithCovariance");
      ready(node,o.topic+"/pose","gz.msgs.Pose_V");
    }
    unsigned inputs=0, preCalls=0, postCalls=0, observations=0, poseRecords=0;
    for (unsigned call=0;call<13;++call) {
      const long long ns=static_cast<long long>(call)*10000000LL;
      // All owner inputs become visible before ANY owner callback for this call.
      for (auto &o:owners) {
        const auto value=controlled(o.index,call);
        (void)ecm.SetComponentData<c::Pose>(o.model,value);
        std::cout << "{\"kind\":\"input\",\"mode\":" << std::quoted(mode) << ",\"owner\":" << o.index
                  << ",\"call_index\":" << call << ",\"time_ns\":" << ns << ",\"real_time_ns\":" << ns
                  << ",\"dt_ns\":" << (call?10000000:0) << ",\"iterations\":" << call << ",\"paused\":false,";
        entityInput(std::cout,ecm,o.model,world,value); std::cout << "}\n"; ++inputs;
      }
      for (auto &o:owners) {
        {
          std::lock_guard<std::mutex> lock(o.received->mutex);
          require(!o.received->bad && o.received->packets.empty() && !o.received->armed, "previous callback fence");
          o.received->call=call; o.received->ns=ns; o.received->armed=true;
        }
        gz::sim::UpdateInfo info;
        info.simTime=std::chrono::nanoseconds(ns); info.realTime=std::chrono::nanoseconds(ns);
        info.dt=std::chrono::nanoseconds(call?10000000:0); info.iterations=call; info.paused=false;
        o.pre->PreUpdate(info,ecm); ++preCalls;
        o.post->PostUpdate(info,ecm); ++postCalls;
        const unsigned wanted=(call>0 && (o.index%2==0 || call%2==0)) ? 3 : 0;
        std::unique_lock<std::mutex> lock(o.received->mutex);
        if (wanted) {
          require(o.received->changed.wait_for(lock,5s,[&]{return o.received->bad || o.received->packets.size()>=wanted;}), "three-channel callback deadline");
        } else {
          (void)o.received->changed.wait_for(lock,25ms,[&]{return o.received->bad || !o.received->packets.empty();});
        }
        require(!o.received->bad && o.received->packets.size()==wanted, "exact per-call channel fence: "+o.received->failure);
        std::set<unsigned> actualChannels;
        for (const auto &p:o.received->packets) {
          require(p.call==call && p.ns==ns && actualChannels.insert(p.channel).second, "unique actual owner/call/channel");
          std::cout << "{\"kind\":\"observation\",\"mode\":" << std::quoted(mode) << ",\"owner\":" << o.index
                    << ",\"call_index\":" << p.call << ",\"channel\":" << std::quoted(channelName(p.channel))
                    << ",\"arrival_ordinal\":" << p.ordinal << ",\"time_ns\":" << p.ns << ",\"message\":" << p.data << "}\n";
          ++observations; if (p.channel==2) ++poseRecords;
        }
        require(wanted==0 || actualChannels==std::set<unsigned>{0,1,2}, "complete actual channel set");
        std::cout << "{\"kind\":\"fence\",\"mode\":" << std::quoted(mode) << ",\"owner\":" << o.index
                  << ",\"call_index\":" << call << ",\"time_ns\":" << ns << ",\"callback_count\":" << o.received->packets.size()
                  << ",\"cumulative_callbacks\":" << o.received->total << "}\n";
        o.received->packets.clear(); o.received->armed=false;
      }
      if (!captured && call==1) { maps(argv[2]); captured=true; }
      ecm.ClearNewlyCreatedEntities();
    }
    for (auto &o:owners) {
      { std::lock_guard<std::mutex> lock(o.received->mutex);
        require(!o.received->bad && o.received->packets.empty() && !o.received->armed, "final bounded callback ledger");
        o.received->active.store(false,std::memory_order_release);
      }
      require(node.Unsubscribe(o.topic+"/odom") && node.Unsubscribe(o.topic+"/covariance") && node.Unsubscribe(o.topic+"/pose"), "owned subscriptions retired");
    }
    owners.clear();  // Invoke original virtual destruction while ECM/events/node live.
    const unsigned published=count/2*18;
    require(captured && inputs==13*count && preCalls==inputs && postCalls==inputs && observations==published*3 && poseRecords==published,
            "complete Source-derived mode ledger");
    std::cout << "{\"kind\":\"summary\",\"mode\":" << std::quoted(mode) << ",\"configuration_count\":" << count
              << ",\"global_call_groups\":13,\"input_rows\":" << inputs << ",\"direct_pre_update_calls\":" << preCalls << ",\"direct_post_update_calls\":" << postCalls
              << ",\"public_interface_invocations\":" << preCalls+postCalls << ",\"fence_rows\":" << inputs << ",\"observed_channel_triples\":" << published << ",\"canonical_packets\":" << observations
              << ",\"pose_vector_records\":" << poseRecords << ",\"captured_pose_bodies\":" << published*3 << ",\"covariance_f32_words\":" << published*72
              << ",\"original_object_linked_System_scope\":true,\"original_Server_or_plugin_loader_executed\":false,\"native_Physics_executed\":false"
              << ",\"native_demand_reset_removal_or_lifecycle_equivalence_claimed\":false,\"global_RNG_equivalence_claimed\":false,\"old_native_modes_replayed\":false}\n";
    std::cout.flush(); require(bool(std::cout), "complete stdout"); return 0;
  } catch (const std::exception &e) {
    if (!captured && argc==3) { try { maps(argv[2]); } catch (...) {} }
    std::cerr << "original direct-object multiple OdometryPublishers: " << e.what() << '\n';
    return 1;
  }
}
