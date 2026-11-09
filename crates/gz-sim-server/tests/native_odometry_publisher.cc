// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Default: genuine transport-only Rust-server peer. The explicit macro selects
// an independent original-System oracle; never link Sim into the default peer.
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace {
void require(bool condition,const std::string &why) {if(!condition)throw std::runtime_error(why);}
long long stamp(const gz::msgs::Header &h) {
  require(h.has_stamp()&&h.stamp().sec()>=0&&h.stamp().nsec()>=0&&h.stamp().nsec()<1000000000,"stamp");
  return h.stamp().sec()*1000000000LL+h.stamp().nsec();
}
void header(const gz::msgs::Header &h) {
  require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)=="m34/odom"
    &&h.data(1).key()=="child_frame_id"&&h.data(1).value_size()==1&&h.data(1).value(0)=="m34/base","frame headers / no seq");
  (void)stamp(h);
}
void vectorFinite(const gz::msgs::Vector3d &v) {require(std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z()),"finite vector");}
void poseFinite(const gz::msgs::Pose &p) {
  require(p.has_position()&&p.has_orientation(),"pose fields");vectorFinite(p.position());
  const auto &q=p.orientation();require(std::isfinite(q.x())&&std::isfinite(q.y())&&std::isfinite(q.z())&&std::isfinite(q.w())
    &&std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1.0)<=1e-12,"finite unit quaternion");
}
struct Triple {gz::msgs::Odometry odom;gz::msgs::OdometryWithCovariance cov;gz::msgs::Pose_V tf;bool o=false,c=false,t=false;};
struct Observed {std::mutex mutex;std::condition_variable changed;std::map<long long,Triple> rows;bool bad=false;};
struct Subscribers {
  gz::transport::Node node;
  std::shared_ptr<Observed> seen=std::make_shared<Observed>();
  void start() {
    std::function<void(const gz::msgs::Odometry &)> odom=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(m.has_header(),"odom header");const auto n=stamp(m.header());auto &r=s->rows[n];require(!r.o,"duplicate odom");r.odom=m;r.o=true;
      require(s->rows.size()<=256,"callback cap");}catch(...){s->bad=true;}s->changed.notify_all();};
    std::function<void(const gz::msgs::OdometryWithCovariance &)> cov=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(m.has_header(),"cov header");const auto n=stamp(m.header());auto &r=s->rows[n];require(!r.c,"duplicate covariance");r.cov=m;r.c=true;
      require(s->rows.size()<=256,"callback cap");}catch(...){s->bad=true;}s->changed.notify_all();};
    std::function<void(const gz::msgs::Pose_V &)> tf=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(!m.has_header()&&m.pose_size()==1&&m.pose(0).has_header(),"transform envelope");const auto n=stamp(m.pose(0).header());auto &r=s->rows[n];require(!r.t,"duplicate transform");r.tf=m;r.t=true;
      require(s->rows.size()<=256,"callback cap");}catch(...){s->bad=true;}s->changed.notify_all();};
    require(node.Subscribe("/m34/odometry",odom)&&node.Subscribe("/m34/odometry_covariance",cov)&&node.Subscribe("/m34/transform",tf),"subscribe three canonical types");
  }
  void ready() {
    const auto deadline=std::chrono::steady_clock::now()+5s;
    for(;;) {
      bool all=true;const std::vector<std::pair<std::string,std::string>> topics={{"/m34/odometry","gz.msgs.Odometry"},{"/m34/odometry_covariance","gz.msgs.OdometryWithCovariance"},{"/m34/transform","gz.msgs.Pose_V"}};
      for(const auto &entry:topics) {std::vector<gz::transport::MessagePublisher> pubs,subs;node.TopicInfo(entry.first,pubs,subs);
        all=all&&pubs.size()==1&&subs.size()==1&&pubs[0].MsgTypeName()==entry.second&&subs[0].MsgTypeName()==entry.second;}
      if (all) {
        return;
      }
      require(std::chrono::steady_clock::now()<deadline,"three typed provider/consumer readiness deadline");
      std::this_thread::yield();
    }
  }
  void drain(std::size_t count) {
    std::unique_lock<std::mutex> lock(seen->mutex);
    require(seen->changed.wait_for(lock,5s,[&]{if(seen->bad)return true;if(seen->rows.size()!=count)return false;for(const auto &r:seen->rows)if(!r.second.o||!r.second.c||!r.second.t)return false;return true;}),"three-channel callback completion deadline");
    require(!seen->bad&&seen->rows.size()==count,"exact callback count");
  }
};
void validate(const Triple &r) {
  require(r.o&&r.c&&r.t,"all channel receipts");header(r.odom.header());poseFinite(r.odom.pose());
  require(r.odom.pose().position().z()==0.0&&r.odom.has_twist(),"2D pose/twist");
  vectorFinite(r.odom.twist().linear());vectorFinite(r.odom.twist().angular());
  require(r.odom.twist().linear().z()==0.0&&r.odom.twist().angular().x()==0.0&&r.odom.twist().angular().y()==0.0,"2D twist");
  require(r.cov.header().SerializeAsString()==r.odom.header().SerializeAsString()&&r.tf.pose(0).header().SerializeAsString()==r.odom.header().SerializeAsString(),"equal acquisition headers");
  require(r.cov.has_pose_with_covariance()&&r.cov.has_twist_with_covariance(),"covariance wrappers");
  require(r.cov.pose_with_covariance().pose().SerializeAsString()==r.odom.pose().SerializeAsString()
    &&r.cov.twist_with_covariance().twist().SerializeAsString()==r.odom.twist().SerializeAsString(),"equal three-channel values");
  const auto &a=r.cov.pose_with_covariance().covariance();const auto &b=r.cov.twist_with_covariance().covariance();
  require(a.data_size()==36&&b.data_size()==36,"covariance dimensions");for(int i=0;i<36;++i)require(a.data(i)==0.0f&&b.data(i)==0.0f,"noiseless covariance");
  auto tf=r.tf.pose(0);tf.clear_header();require(tf.SerializeAsString()==r.odom.pose().SerializeAsString(),"transform pose / default name-id");
}
#ifndef GZ_SIM_NATIVE_ODOMETRY_ORACLE
bool control(gz::transport::Node &node,unsigned steps) {
  using Reply=std::pair<gz::msgs::Boolean,bool>;auto completion=std::make_shared<std::promise<Reply>>();auto future=completion->get_future();
  std::function<void(const gz::msgs::Boolean &,bool)> callback=[completion](const auto &m,bool ok){completion->set_value({m,ok});};
  gz::msgs::WorldControl request;request.set_pause(true);request.set_multi_step(steps);
  if(!node.Request("/world/default/control",request,callback)||future.wait_for(7s)!=std::future_status::ready)return false;
  const auto reply=future.get();return reply.second&&reply.first.data();
}
#endif
}
#ifndef GZ_SIM_NATIVE_ODOMETRY_ORACLE
int main() {
  try {
    Subscribers subscriptions;subscriptions.start();subscriptions.ready();require(control(subscriptions.node,200),"actual WorldControl acknowledged");subscriptions.drain(10);
    std::lock_guard<std::mutex> lock(subscriptions.seen->mutex);
    long long previous=0;for(const auto &entry:subscriptions.seen->rows) {require(entry.first==previous+20000000,"50Hz chronology");previous=entry.first;validate(entry.second);}
    const auto &last=subscriptions.seen->rows.rbegin()->second.odom;
    require(last.pose().position().x()>2.15&&last.pose().position().y()>-1.0,"actual translating model");
    require(last.twist().angular().z()>0.39&&last.twist().angular().z()<0.41,"actual body yaw rate");
    require(last.pose().orientation().z()>std::sin(0.3),"actual rotating model");
    std::cout<<"ground-truth odometry conformance passed\n";return 0;
  }catch(const std::exception &e){std::cerr<<"ground-truth odometry peer: "<<e.what()<<'\n';return 1;}
}
#else
#include <gz/sim/System.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Plugin.hh>
namespace {
std::string hex(const std::string &bytes) {
  const char *digits="0123456789abcdef";std::string result;result.reserve(bytes.size()*2);
  for(unsigned char byte:bytes){result.push_back(digits[byte>>4]);result.push_back(digits[byte&15]);}return result;
}
void poseJson(const gz::math::Pose3d &p) {
  std::cout<<'['<<p.Pos().X()<<','<<p.Pos().Y()<<','<<p.Pos().Z()<<','<<p.Rot().X()<<','<<p.Rot().Y()<<','<<p.Rot().Z()<<','<<p.Rot().W()<<']';
}
void maps(const std::string &path) {
  std::ifstream input("/proc/self/maps");std::ofstream output(path,std::ios::out|std::ios::trunc);
  require(bool(input)&&bool(output),"map capture open");output<<input.rdbuf();output.flush();require(bool(output),"map capture write");
}
}
int main(int argc,char **argv) {
  bool captured=false;
  try {
    require(argc==3,"oracle MODE MAPS_PATH");const std::string mode=argv[1];
    // Match chrono's native division, not ns*1e-9: that multiplication can
    // round exact 1000 ns above the inclusive math::equal microsecond bound.
    const auto elapsedSeconds=[](long long ns){return std::chrono::duration<double>(std::chrono::nanoseconds(ns)).count();};
    require(elapsedSeconds(1000)<=1e-6&&elapsedSeconds(1500)>1e-6,"inclusive native microsecond boundary");
    const std::vector<std::string> modes={"identity","initial","motion","rate","micro","paused","delayed","tilt","sign"};
    require(std::find(modes.begin(),modes.end(),mode)!=modes.end(),"declared oracle mode");
    const bool throttled=mode=="rate";const auto step=mode=="micro"?500LL:1000000LL;
    const unsigned ticks=mode=="micro"?34u:32u;
    sdf::Plugin config("gz-sim-odometry-publisher-system","gz::sim::systems::OdometryPublisher");
    require(config.InsertContent("<dimensions>2</dimensions><odom_publish_frequency>"+std::string(throttled?"50":"0")+"</odom_publish_frequency><odom_topic>/m34/odometry</odom_topic><odom_covariance_topic>/m34/odometry_covariance</odom_covariance_topic><tf_topic>/m34/transform</tf_topic><odom_frame>m34/odom</odom_frame><robot_base_frame>m34/base</robot_base_frame>"),"plugin config");
    gz::sim::SystemLoader loader;auto loaded=loader.LoadPlugin(config);require(bool(loaded),"original system plugin load");
    auto *configure=(*loaded)->QueryInterface<gz::sim::ISystemConfigure>();auto *pre=(*loaded)->QueryInterface<gz::sim::ISystemPreUpdate>();auto *post=(*loaded)->QueryInterface<gz::sim::ISystemPostUpdate>();
    require(configure&&pre&&post,"original system interfaces");
    gz::sim::EntityComponentManager ecm;gz::sim::EventManager events;
    const auto world=ecm.CreateEntity();ecm.CreateComponent(world,gz::sim::components::World());ecm.CreateComponent(world,gz::sim::components::Name("default"));ecm.CreateComponent(world,gz::sim::components::Pose(gz::math::Pose3d::Zero));
    const auto model=ecm.CreateEntity();ecm.CreateComponent(model,gz::sim::components::Model());ecm.CreateComponent(model,gz::sim::components::Name("rig"));ecm.CreateComponent(model,gz::sim::components::ParentEntity(world));ecm.CreateComponent(model,gz::sim::components::Pose(gz::math::Pose3d::Zero));
    configure->Configure(model,config.ToElement(),ecm,events);
    Subscribers subscriptions;if(mode!="delayed"){subscriptions.start();subscriptions.ready();}
    long long history=-1,lastPub=0;std::size_t expected=0;std::map<long long,gz::math::Pose3d> inputs;
    std::cout<<std::setprecision(17);
    if(mode=="paused") {
      gz::sim::UpdateInfo paused;paused.paused=true;
      pre->PreUpdate(paused,ecm);post->PostUpdate(paused,ecm);
      std::cout<<"{\"kind\":\"paused_input\",\"mode\":\"paused\",\"time_ns\":0,\"dt_ns\":0,\"pose\":[0,0,0,0,0,0,1]}\n";
    }
    for(unsigned tick=0;tick<=ticks;++tick) {
      const auto ns=static_cast<long long>(tick)*step;
      const double seconds=ns*1e-9;gz::math::Pose3d pose;
      if(mode=="initial")pose=gz::math::Pose3d(2,-1,3,0,0,.6);
      else if(mode=="motion"||mode=="rate"||mode=="delayed")pose=gz::math::Pose3d(seconds+seconds*seconds,.5*seconds,3,0,0,.6+.4*seconds);
      else if(mode=="tilt")pose=gz::math::Pose3d(seconds,.5*seconds,3,.2,.3,.6+.4*seconds);
      else if(mode=="sign"){pose=gz::math::Pose3d(0,0,3,0,0,3.13+.8*seconds);if(tick%2)pose.Rot()=-pose.Rot();}
      else pose=gz::math::Pose3d(0,0,3,0,0,0);
      ecm.SetComponentData<gz::sim::components::Pose>(model,pose);
      gz::sim::UpdateInfo info;info.simTime=std::chrono::nanoseconds(ns);info.dt=std::chrono::nanoseconds(tick?step:0);info.iterations=tick;info.paused=false;
      if(mode=="paused"&&tick==10){auto paused=info;paused.paused=true;paused.dt=0ns;pre->PreUpdate(paused,ecm);post->PostUpdate(paused,ecm);
        std::cout<<"{\"kind\":\"paused_input\",\"mode\":\"paused\",\"time_ns\":"<<ns<<",\"dt_ns\":0,\"pose\":";poseJson(pose);std::cout<<"}\n";}
      if(mode=="delayed"&&tick==16){subscriptions.start();subscriptions.ready();}
      pre->PreUpdate(info,ecm);post->PostUpdate(info,ecm);
      std::cout<<"{\"kind\":\"input\",\"mode\":"<<std::quoted(mode)<<",\"tick\":"<<tick<<",\"time_ns\":"<<ns<<",\"dt_ns\":"<<(tick?step:0)<<",\"pose\":";poseJson(pose);std::cout<<"}\n";
      if(history<0){history=ns;continue;}
      if(elapsedSeconds(ns-history)<=1e-6)continue;
      history=ns;const bool due=!throttled||ns-lastPub>=20000000;
      if(due){lastPub=ns;if(mode!="delayed"||tick>=16){++expected;inputs.emplace(ns,pose);}}
      if(!captured){captured=true;maps(argv[2]);}
    }
    subscriptions.drain(expected);
    std::lock_guard<std::mutex> lock(subscriptions.seen->mutex);
    for(const auto &entry:subscriptions.seen->rows) {
      validate(entry.second);const auto input=inputs.find(entry.first);require(input!=inputs.end(),"unexpected acquisition stamp");
      const auto &r=entry.second;const auto &p=r.odom.pose();require(std::abs(p.position().x()-input->second.Pos().X())<=1e-12&&std::abs(p.position().y()-input->second.Pos().Y())<=1e-12,"actual input pose");
      if(mode=="identity"||mode=="micro")require(r.odom.twist().linear().x()==0&&r.odom.twist().angular().z()==0,"stationary zero velocity");
      if(mode=="initial"&&entry.first>=11000000)require(r.odom.twist().linear().x()==0&&r.odom.twist().linear().y()==0&&r.odom.twist().angular().z()==0,"initial transient evicted at history11");
      if(mode=="sign"&&entry.first>=11000000)require(std::abs(r.odom.twist().angular().z()-.8)<1e-8,"quaternion sign and yaw continuity");
      std::cout<<"{\"kind\":\"acquisition\",\"mode\":"<<std::quoted(mode)<<",\"time_ns\":"<<entry.first<<",\"input\":";poseJson(input->second);
      const auto &q=p.orientation();const auto &linear=r.odom.twist().linear();const auto &angular=r.odom.twist().angular();
      std::cout<<",\"pose\":["<<p.position().x()<<','<<p.position().y()<<','<<p.position().z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()
        <<"],\"linear\":["<<linear.x()<<','<<linear.y()<<','<<linear.z()<<"],\"angular\":["<<angular.x()<<','<<angular.y()<<','<<angular.z()<<']';
      std::cout<<",\"odometry_hex\":"<<std::quoted(hex(r.odom.SerializeAsString()))<<",\"covariance_hex\":"<<std::quoted(hex(r.cov.SerializeAsString()))<<",\"transform_hex\":"<<std::quoted(hex(r.tf.SerializeAsString()))<<"}\n";
    }
    require(captured,"actual alive provider maps captured");return 0;
  }catch(const std::exception &e){if(argc==3&&!captured){try{maps(argv[2]);}catch(...){}}std::cerr<<"native ground-truth odometry: "<<e.what()<<'\n';return 1;}
}
#endif
