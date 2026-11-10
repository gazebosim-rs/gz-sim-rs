// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Canonical combined-system peer only; no Sim or Physics library is linked.
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <algorithm>
#include <chrono>
#include <climits>
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
  require(h.stamp().sec()<=(LLONG_MAX-h.stamp().nsec())/1000000000LL,"stamp nanosecond overflow");
  return h.stamp().sec()*1000000000LL+h.stamp().nsec();
}
void header(const gz::msgs::Header &h) {
  require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)=="m36/odom"
    &&h.data(1).key()=="child_frame_id"&&h.data(1).value_size()==1&&h.data(1).value(0)=="m36/base","frame headers / no seq");
  (void)stamp(h);
}
void vectorFinite(const gz::msgs::Vector3d &v) {require(std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z()),"finite vector");}
void poseFinite(const gz::msgs::Pose &p) {
  require(p.has_position()&&p.has_orientation(),"pose fields");vectorFinite(p.position());
  const auto &q=p.orientation();require(std::isfinite(q.x())&&std::isfinite(q.y())&&std::isfinite(q.z())&&std::isfinite(q.w())
    &&std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1.0)<=1e-12,"finite unit quaternion");
}
struct Triple {gz::msgs::Odometry odom;gz::msgs::OdometryWithCovariance cov;gz::msgs::Pose_V tf;bool o=false,c=false,t=false;};
struct Observed {std::mutex mutex;std::condition_variable changed;std::map<long long,Triple> rows;bool bad=false;bool haveClock=false;long long clockNs=0;};
struct Subscribers {
  gz::transport::Node node;
  std::shared_ptr<Observed> seen=std::make_shared<Observed>();
  // This subscription survives GT disconnects on the same owned Node/session.
  void startClock() {
    std::function<void(const gz::msgs::Clock &)> callback=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(!s->bad,"clock callback after retained failure");
      require(m.has_sim()&&m.sim().sec()>=0&&m.sim().nsec()>=0&&m.sim().nsec()<1000000000,"clock simulation stamp");
      require(m.sim().sec()<=(LLONG_MAX-m.sim().nsec())/1000000000LL,"clock nanosecond overflow");
      s->clockNs=m.sim().sec()*1000000000LL+m.sim().nsec();s->haveClock=true;
      }catch(...){s->bad=true;}s->changed.notify_all();};
    require(node.Subscribe("/world/default/clock",callback),"canonical Clock subscription");
  }
  void start() {
    std::function<void(const gz::msgs::Odometry &)> odom=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(!s->bad,"callback after retained failure");
      require(m.has_header(),"odom header");const auto n=stamp(m.header());require(s->rows.size()<=256&&(s->rows.find(n)!=s->rows.end()||s->rows.size()<256),"callback cap before insertion");auto &r=s->rows[n];require(!r.o,"duplicate odom");r.odom=m;r.o=true;
      }catch(...){s->bad=true;}s->changed.notify_all();};
    std::function<void(const gz::msgs::OdometryWithCovariance &)> cov=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(!s->bad,"callback after retained failure");
      require(m.has_header(),"cov header");const auto n=stamp(m.header());require(s->rows.size()<=256&&(s->rows.find(n)!=s->rows.end()||s->rows.size()<256),"callback cap before insertion");auto &r=s->rows[n];require(!r.c,"duplicate covariance");r.cov=m;r.c=true;
      }catch(...){s->bad=true;}s->changed.notify_all();};
    std::function<void(const gz::msgs::Pose_V &)> tf=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(!s->bad,"callback after retained failure");
      require(!m.has_header()&&m.pose_size()==1&&m.pose(0).has_header(),"transform envelope");const auto n=stamp(m.pose(0).header());require(s->rows.size()<=256&&(s->rows.find(n)!=s->rows.end()||s->rows.size()<256),"callback cap before insertion");auto &r=s->rows[n];require(!r.t,"duplicate transform");r.tf=m;r.t=true;
      }catch(...){s->bad=true;}s->changed.notify_all();};
    require(node.Subscribe("/m36/odometry",odom)&&node.Subscribe("/m36/odometry_covariance",cov)&&node.Subscribe("/m36/transform",tf),"subscribe three canonical types");
  }
  void ready() {
    const auto deadline=std::chrono::steady_clock::now()+5s;
    for(;;) {
      bool all=true;const std::vector<std::pair<std::string,std::string>> topics={{"/world/default/clock","gz.msgs.Clock"},{"/m36/odometry","gz.msgs.Odometry"},{"/m36/odometry_covariance","gz.msgs.OdometryWithCovariance"},{"/m36/transform","gz.msgs.Pose_V"}};
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
gz::msgs::Entity target(std::uint64_t id) {
  gz::msgs::Entity value;value.set_id(id);value.set_type(gz::msgs::Entity::LINK);return value;
}
gz::msgs::EntityWrench wrench(std::uint64_t id,double force,double torque) {
  gz::msgs::EntityWrench value;*value.mutable_entity()=target(id);
  value.mutable_wrench()->mutable_force()->set_x(force);
  value.mutable_wrench()->mutable_torque()->set_z(torque);return value;
}
void inputReady(const std::vector<gz::transport::Node::Publisher *> &publishers) {
  const auto deadline=std::chrono::steady_clock::now()+5s;
  for(;;) {
    bool all=true;for(const auto *p:publishers)all=all&&p->HasConnections();
    if(all)return;
    require(std::chrono::steady_clock::now()<deadline,"three input connections deadline");
    std::this_thread::yield();
  }
}
template<class T> void packet(const char *channel,const T &message) {
  const auto bytes=message.SerializeAsString();require(bytes.size()<=8192,"canonical packet cap");
  constexpr char digits[]="0123456789abcdef";std::string hex;hex.reserve(bytes.size()*2);
  for(unsigned char b:bytes){hex+=digits[b>>4];hex+=digits[b&15];}
  std::cout<<"PACKET "<<channel<<' '<<hex<<'\n';
}
}
int main(int argc,char **argv) {
  try {
    require(argc==2,"expected actual public link ID");
    const std::string argument=argv[1];std::size_t used=0;const auto id=std::stoull(argument,&used);
    require(id!=0&&used==argument.size()&&argument.find_first_not_of("0123456789")==std::string::npos,"invalid public link ID");
    Subscribers subscriptions;subscriptions.startClock();subscriptions.start();subscriptions.ready();bool watching=true;
    gz::transport::Node inputs;
    auto once=inputs.Advertise<gz::msgs::EntityWrench>("/world/default/wrench");
    auto persistent=inputs.Advertise<gz::msgs::EntityWrench>("/world/default/wrench/persistent");
    auto clear=inputs.Advertise<gz::msgs::Entity>("/world/default/wrench/clear");
    require(once&&persistent&&clear,"canonical input publishers");inputReady({&once,&persistent,&clear});
    std::cout<<"READY combined"<<std::endl;
    for(unsigned phase=0;phase<128;++phase) {
      char line[65]{};require(static_cast<bool>(std::cin.getline(line,sizeof(line))),"bounded phase input/EOF");
      const std::string command=line;
      if(command=="once-force"||command=="once-torque") {
        require(once.Publish(wrench(id,command=="once-force"?4:0,command=="once-torque"?2:0)),"one-shot publish");
      } else if(command=="persistent-force"||command=="persistent-torque") {
        const bool force=command=="persistent-force";
        require(persistent.Publish(wrench(id,force?1:0,force?0:2))&&persistent.Publish(wrench(id,force?3:0,force?0:4)),"two additive publishes");
      } else if(command=="clear") {
        require(clear.Publish(target(id)),"clear publish");
      } else if(command.rfind("drain ",0)==0) {
        const auto number=command.substr(6);std::size_t consumed=0;const auto count=std::stoul(number,&consumed);
        require(watching&&consumed==number.size()&&count<=4,"finite drain count");
        subscriptions.drain(count);
        std::lock_guard<std::mutex> lock(subscriptions.seen->mutex);
        require(!subscriptions.seen->bad&&subscriptions.seen->rows.size()==count,"completed callback set");
        for(const auto &entry:subscriptions.seen->rows) {
          validate(entry.second);
          packet("odometry",entry.second.odom);packet("covariance",entry.second.cov);packet("transform",entry.second.tf);
        }
        subscriptions.seen->rows.clear();
        std::cout<<"DRAINED "<<count<<std::endl;continue;
      } else if(command=="drop") {
        require(watching,"already disconnected");
        {std::lock_guard<std::mutex> lock(subscriptions.seen->mutex);require(!subscriptions.seen->bad&&subscriptions.seen->rows.empty(),"drop before complete receipt drain");}
        require(subscriptions.node.Unsubscribe("/m36/odometry")&&subscriptions.node.Unsubscribe("/m36/odometry_covariance")&&subscriptions.node.Unsubscribe("/m36/transform"),"drop all three subscriptions");
        watching=false;std::cout<<"DROPPED"<<std::endl;continue;
      } else if(command=="clock-101") {
        require(!watching,"Clock fence only while GT is disconnected");
        std::unique_lock<std::mutex> lock(subscriptions.seen->mutex);
        require(subscriptions.seen->changed.wait_for(lock,5s,[&]{return subscriptions.seen->bad||(subscriptions.seen->haveClock&&subscriptions.seen->clockNs==101000000LL);}),"exact 101 ms Clock deadline");
        require(!subscriptions.seen->bad&&subscriptions.seen->haveClock&&subscriptions.seen->clockNs==101000000LL,"actual canonical Clock fence");
        std::cout<<"CLOCK 101000000"<<std::endl;continue;
      } else if(command=="watch") {
        require(!watching,"duplicate subscriptions");subscriptions.start();subscriptions.ready();inputReady({&once,&persistent,&clear});
        watching=true;std::cout<<"WATCHING"<<std::endl;continue;
      } else if(command=="retired") {
        require(!watching,"retirement requires drained/dropped subscriptions");
        const auto deadline=std::chrono::steady_clock::now()+5s;
        for(;;) {
          bool gone=!once.HasConnections()&&!persistent.HasConnections()&&!clear.HasConnections();
          for(const auto *topic:{"/m36/odometry","/m36/odometry_covariance","/m36/transform"}) {
            std::vector<gz::transport::MessagePublisher> pubs,subs;subscriptions.node.TopicInfo(topic,pubs,subs);gone=gone&&pubs.empty();
          }
          if(gone)break;
          require(std::chrono::steady_clock::now()<deadline,"owned endpoint retirement deadline");std::this_thread::yield();
        }
        std::cout<<"RETIRED"<<std::endl;continue;
      } else if(command=="finish") {
        require(!watching,"drop before finish");
        std::lock_guard<std::mutex> lock(subscriptions.seen->mutex);require(!subscriptions.seen->bad&&subscriptions.seen->rows.empty(),"unconsumed callbacks at finish");
        std::cout<<"PASS canonical wrench odometry"<<std::endl;return 0;
      } else {throw std::runtime_error("unknown finite phase command");}
      std::cout<<"SENT "<<command<<std::endl;
    }
    throw std::runtime_error("finite phase count cap");
  }catch(const std::exception &error){std::cerr<<"native combined peer: "<<error.what()<<'\n';return 1;}
}
