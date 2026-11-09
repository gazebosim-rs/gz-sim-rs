// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// A real canonical Zenoh consumer/command peer, not a native robot/RNG oracle.
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
namespace {
struct Observed {
  std::mutex mutex;
  std::condition_variable changed;
  bool invalid=false, forward=false, turn=false;
  std::int64_t clock=-1, imu=-1, scan=-1, odom=-1;
  std::uint64_t imuSequence=0, scanSequence=0;
  bool haveImu=false, haveScan=false;
};
std::int64_t ns(const gz::msgs::Time &time) {
  if(time.sec()<0 || time.nsec()<0 || time.nsec()>=1000000000)
    throw std::runtime_error("invalid canonical stamp");
  return time.sec()*1000000000LL+time.nsec();
}
bool header(const gz::msgs::Header &h,const std::string &frame,
    std::int64_t &previous,std::uint64_t &sequence,bool &have,std::int64_t period) {
  if(!h.has_stamp() || h.data_size()!=2 || h.data(0).key()!="frame_id" ||
      h.data(0).value_size()!=1 || h.data(0).value(0)!=frame ||
      h.data(1).key()!="seq" || h.data(1).value_size()!=1) return false;
  const auto time=ns(h.stamp());
  std::size_t used=0;
  const auto next=std::stoull(h.data(1).value(0),&used);
  // First demand can arrive between grid ticks; TopicInfo is metadata,
  // not a fence for the Rust matching listener. Later acquisitions catch up
  // to the existing period grid and keep exact consecutive wire sequences.
  if(used!=h.data(1).value(0).size() || time<=0 ||
      (have && (time%period!=0 || time<=previous || next!=sequence+1))) return false;
  previous=time; sequence=next; have=true; return true;
}
bool finiteVector(const gz::msgs::Vector3d &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}
bool covariance(const gz::msgs::Float_V &c,float x) {
  if(c.data_size()!=9) return false;
  for(int i=0;i<9;++i) if(c.data(i)!=(i==0?x:0.0f)) return false;
  return true;
}
// Async Request preserves callback lifetime through transport response cleanup.
bool control(gz::transport::Node &node,unsigned int steps) {
  using Outcome=std::pair<gz::msgs::Boolean,bool>;
  auto completion=std::make_shared<std::promise<Outcome>>();
  auto reply=completion->get_future();
  std::function<void(const gz::msgs::Boolean &,bool)> callback=
    [completion](const auto &message,bool result){completion->set_value({message,result});};
  gz::msgs::WorldControl request; request.set_pause(true); request.set_multi_step(steps);
  if(!node.Request("/world/default/control",request,callback) || reply.wait_for(7s)!=std::future_status::ready)
    return false;
  const auto result=reply.get(); return result.second && result.first.data();
}
}
int main() {
  try {
    auto seen=std::make_shared<Observed>();
    gz::transport::Node node;
    std::function<void(const gz::msgs::Clock &)> clock=[seen](const auto &m){
      std::lock_guard<std::mutex> lock(seen->mutex);
      seen->clock=ns(m.sim()); seen->changed.notify_all();
    };
    std::function<void(const gz::msgs::IMU &)> imu=[seen](const auto &m){
      std::lock_guard<std::mutex> lock(seen->mutex);
      bool valid=m.entity_name()=="robot::base_link::imu" && m.has_header() &&
        header(m.header(),"robot/base_link/imu",seen->imu,seen->imuSequence,seen->haveImu,10000000) &&
        m.has_orientation() && m.has_angular_velocity() && m.has_linear_acceleration() &&
        finiteVector(m.angular_velocity()) && finiteVector(m.linear_acceleration()) &&
        covariance(m.orientation_covariance(),0.0f) && covariance(m.angular_velocity_covariance(),0.0f) &&
        covariance(m.linear_acceleration_covariance(),static_cast<float>(0.01*0.01));
      const auto &q=m.orientation();
      valid=valid && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) && std::isfinite(q.w()) &&
        std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1.0)<=1e-12;
      // Accepted quantization is checked without a native RNG/trajectory claim.
      valid=valid && m.angular_velocity().z()==std::round(m.angular_velocity().z()/0.01)*0.01;
      seen->invalid=seen->invalid || !valid; seen->changed.notify_all();
    };
    std::function<void(const gz::msgs::LaserScan &)> scan=[seen](const auto &m){
      std::lock_guard<std::mutex> lock(seen->mutex);
      bool valid=m.has_header() && header(m.header(),"robot/base_link/lidar",seen->scan,seen->scanSequence,seen->haveScan,50000000) &&
        m.frame()=="robot/base_link/lidar" && m.count()==3 && m.vertical_count()==1 &&
        m.angle_min()==-0.7853981633974483 && m.angle_max()==0.7853981633974483 && m.angle_step()==0.7853981633974483 &&
        m.range_min()==0.0 && m.range_max()==10.0 && m.ranges_size()==3 && m.intensities_size()==3 &&
        m.vertical_angle_min()==0.0 && m.vertical_angle_max()==0.0 && m.vertical_angle_step()==0.0 &&
        m.has_world_pose() && m.world_pose().has_position() && finiteVector(m.world_pose().position()) &&
        !m.world_pose().has_header() && m.world_pose().name().empty() && m.world_pose().id()==0;
      for(int i=0;i<m.ranges_size();++i) {
        const double r=m.ranges(i);
        valid=valid && ((std::isinf(r) && r>0.0) || (std::isfinite(r) && r>=0.0 && r<=10.0));
      }
      for(int i=0;i<m.intensities_size();++i) valid=valid && m.intensities(i)==0.0;
      const auto &q=m.world_pose().orientation();
      valid=valid && m.world_pose().has_orientation() && std::isfinite(q.x()) && std::isfinite(q.y()) &&
        std::isfinite(q.z()) && std::isfinite(q.w()) &&
        std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1.0)<=1e-12;
      seen->invalid=seen->invalid || !valid; seen->changed.notify_all();
    };
    std::function<void(const gz::msgs::Odometry &)> odom=[seen](const auto &m){
      std::lock_guard<std::mutex> lock(seen->mutex);
      const bool valid=m.has_header() && m.header().has_stamp() && m.has_twist() &&
        m.twist().has_linear() && m.twist().has_angular() && finiteVector(m.twist().linear()) && finiteVector(m.twist().angular());
      seen->invalid=seen->invalid || !valid;
      if(valid) {
        seen->odom=ns(m.header().stamp());
        seen->forward=seen->forward || m.twist().linear().x()>0.0;
        seen->turn=seen->turn || (m.twist().linear().x()>0.0 && m.twist().angular().z()>0.0);
      }
      seen->changed.notify_all();
    };
    if(!node.Subscribe("/world/default/clock",clock) || !node.Subscribe("/m33/imu",imu) ||
        !node.Subscribe("/m33/scan",scan) || !node.Subscribe("/model/robot/odometry",odom))
      throw std::runtime_error("canonical subscriptions");
    auto command=node.Advertise<gz::msgs::Twist>("/model/robot/cmd_vel");
    if(!command) throw std::runtime_error("canonical Twist publisher");
    const auto ready=std::chrono::steady_clock::now()+8s;
    for(;;) {
      bool discovered=command.HasConnections();
      for(const auto &entry:std::vector<std::pair<std::string,std::string>>{{"/m33/imu","gz.msgs.IMU"},{"/m33/scan","gz.msgs.LaserScan"}}) {
        std::vector<gz::transport::MessagePublisher> publishers,subscribers;
        node.TopicInfo(entry.first,publishers,subscribers);
        discovered=discovered && publishers.size()==1 && subscribers.size()==1 &&
          publishers[0].MsgTypeName()==entry.second && subscribers[0].MsgTypeName()==entry.second;
      }
      {std::lock_guard<std::mutex> lock(seen->mutex); discovered=discovered && seen->clock==0;}
      if(discovered) break;
      if(std::chrono::steady_clock::now()>=ready) throw std::runtime_error("actual dual-stream/Twist discovery");
      std::this_thread::yield();
    }
    const auto wait=[&](std::int64_t target,bool turning){
      std::unique_lock<std::mutex> lock(seen->mutex);
      if(!seen->changed.wait_for(lock,8s,[&]{return seen->invalid ||
          (seen->clock>=target && seen->imu>=target && seen->scan>=target && seen->odom>=target &&
          (turning?seen->turn:seen->forward));}) || seen->invalid)
        throw std::runtime_error("canonical actual acquisition/odometry completion");
    };
    gz::msgs::Twist forward;forward.mutable_linear()->set_x(0.4);forward.mutable_angular()->set_z(0.0);
    if(!command.Publish(forward) || !control(node,200)) throw std::runtime_error("forward command/control");
    wait(200000000,false);
    gz::msgs::Twist turn;turn.mutable_linear()->set_x(0.2);turn.mutable_angular()->set_z(0.8);
    if(!command.Publish(turn) || !control(node,200)) throw std::runtime_error("turn command/control");
    wait(400000000,true);
    // Each bounded multi-step leaves the real server paused; no synthetic samples.
    if(!control(node,0)) throw std::runtime_error("final pause");
    std::cout<<"PASS native robot_sensors IMU LaserScan forward turn\n";
    return 0;
  } catch(const std::exception &error) {
    std::cerr<<"native robot_sensors: "<<error.what()<<'\n'; return 1;
  }
}
