// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Transport-only canonical peer; no Sim / Physics implementation or oracle.
#include <gz/msgs/double.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/transport/Node.hh>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace {
constexpr const char *stateTopic="/m42/joint_state";
constexpr const char *forceTopic="/model/rotor/joint/hinge/cmd_force";
void require(bool condition,const std::string &why) {if(!condition)throw std::runtime_error(why);}
long long stamp(const gz::msgs::Model &m) {
  require(m.has_header()&&m.header().has_stamp()&&m.header().data_size()==0,"Model stamp / no wire epoch or sequence");
  const auto &t=m.header().stamp();
  require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000,"stamp range");
  require(t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"stamp nanosecond overflow");
  return t.sec()*1000000000LL+t.nsec();
}
void pose(const gz::msgs::Pose &p) {
  require(!p.has_header()&&p.name().empty()&&p.id()==0&&p.has_position()&&p.has_orientation(),"complete default pose fields");
  const auto &v=p.position();const auto &q=p.orientation();
  require(!v.has_header()&&!q.has_header(),"pose nested headers absent");
  require(std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z())&&std::isfinite(q.x())&&std::isfinite(q.y())&&std::isfinite(q.z())&&std::isfinite(q.w()),"finite pose");
  require(std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1.0)<=1e-12,"unit pose quaternion");
}
void validate(const gz::msgs::Model &m) {
  (void)stamp(m);
  require(m.name()=="rotor"&&m.id()!=0&&m.has_pose()&&m.joint_size()==1,"model identity and joint count");
  require(!m.is_static()&&!m.deleted()&&!m.self_collide()&&m.link_size()==0&&m.visual_size()==0&&m.model_size()==0&&!m.has_scale()&&!m.has_bounding_box(),"unused Model fields");
  pose(m.pose());
  std::uint32_t previous=0;
  for(const auto &j:m.joint()) {
    require(j.id()>previous&&j.name()=="hinge"&&j.parent()=="world"&&j.child()=="body","sorted joint identity");previous=j.id();
    require(!j.has_header()&&j.parent_id()==0&&j.child_id()==0&&j.type()==gz::msgs::Joint::REVOLUTE&&j.has_pose()&&j.has_axis1()&&!j.has_axis2(),"joint presence/default fields");
    require(j.cfm()==0&&j.bounce()==0&&j.fudge_factor()==0&&j.limit_cfm()==0&&j.limit_erp()==0&&j.suspension_cfm()==0&&j.suspension_erp()==0&&!j.has_gearbox()&&!j.has_screw()&&j.sensor_size()==0,"remaining joint defaults");
    pose(j.pose());const auto &a=j.axis1();
    require(!a.has_header()&&a.has_xyz()&&!a.xyz().has_header()&&a.xyz().x()==0&&a.xyz().y()==1&&a.xyz().z()==0,"axis Y and default headers");
    require(std::isinf(a.limit_lower())&&a.limit_lower()<0&&std::isinf(a.limit_upper())&&a.limit_upper()>0,"valid infinite joint limits");
    require(a.limit_effort()==0&&a.limit_velocity()==0&&a.damping()==0&&a.friction()==0&&a.force()==0&&a.acceleration()==0&&a.xyz_expressed_in().empty(),"unmeasured force and axis defaults");
    require(std::isfinite(a.position())&&std::isfinite(a.velocity()),"actual finite joint measurements");
  }
}
void packet(const gz::msgs::Model &m) {
  const auto bytes=m.SerializeAsString();require(bytes.size()<=8192,"canonical Model packet cap");
  constexpr char digits[]="0123456789abcdef";std::string hex;hex.reserve(bytes.size()*2);
  for(unsigned char b:bytes){hex+=digits[b>>4];hex+=digits[b&15];}
  std::cout<<"PACKET "<<hex<<'\n';
}
enum class Mode {Frozen,Prime,Record};
struct Observed {
  std::mutex mutex;std::condition_variable changed;
  std::deque<gz::msgs::Model> fifo;
  gz::msgs::Model prime;
  Mode mode=Mode::Frozen;long long target=0;bool havePrime=false,bad=false;
  std::string failure;
};
struct Peer {
  gz::transport::Node node;
  std::shared_ptr<Observed> seen=std::make_shared<Observed>();
  std::shared_ptr<std::atomic<bool>> active;
  bool watching=false;
  void watch() {
    require(!watching,"duplicate subscriber");
    auto lease=std::make_shared<std::atomic<bool>>(true);active=lease;
    std::function<void(const gz::msgs::Model &)> callback=[s=seen,lease](const auto &m) {
      std::lock_guard<std::mutex> lock(s->mutex);
      if(!lease->load(std::memory_order_acquire)||s->bad)return;
      try {
        require(m.ByteSizeLong()<=8192,"canonical Model cap before copy");validate(m);
        if(s->mode==Mode::Prime&&stamp(m)==s->target) {
          require(!s->havePrime,"duplicate priming completion");s->prime=m;s->havePrime=true;s->mode=Mode::Frozen;
        } else if(s->mode==Mode::Record) {
          require(s->fifo.size()<256,"FIFO cap before mutation");s->fifo.push_back(m);
        }
      }catch(const std::exception &e){s->bad=true;s->failure=e.what();}
      s->changed.notify_all();
    };
    require(node.Subscribe(stateTopic,callback),"canonical Model subscriber");watching=true;
  }
  void drop() {
    require(watching,"subscriber already absent");
    {std::lock_guard<std::mutex> lock(seen->mutex);require(!seen->bad&&seen->fifo.empty(),"drain before subscriber retirement");
      active->store(false,std::memory_order_release);seen->mode=Mode::Frozen;}
    require(node.Unsubscribe(stateTopic),"actual subscriber retirement");watching=false;
  }
  void ready(gz::transport::Node::Publisher &force) {
    const auto deadline=std::chrono::steady_clock::now()+5s;
    for(;;) {
      std::vector<gz::transport::MessagePublisher> pubs,subs;node.TopicInfo(stateTopic,pubs,subs);
      if(force.HasConnections()&&pubs.size()==1&&subs.size()==1&&pubs[0].MsgTypeName()=="gz.msgs.Model"&&subs[0].MsgTypeName()=="gz.msgs.Model")return;
      require(std::chrono::steady_clock::now()<deadline,"typed command/Model discovery deadline");std::this_thread::yield();
    }
  }
};
long long natural(const std::string &value,long long limit) {
  require(!value.empty()&&value.find_first_not_of("0123456789")==std::string::npos,"finite phase integer");
  std::size_t used=0;const auto n=std::stoll(value,&used);require(used==value.size()&&n<=limit,"phase integer bound");return n;
}
}
int main(int argc,char **) {
  try {
    require(argc==1,"no peer arguments");Peer peer;peer.watch();
    auto force=peer.node.Advertise<gz::msgs::Double>(forceTopic);require(static_cast<bool>(force),"canonical Double publisher");
    peer.ready(force);std::cout<<"READY joint-force-feedback"<<std::endl;
    for(unsigned phase=0;phase<4096;++phase) {
      char line[65]{};require(static_cast<bool>(std::cin.getline(line,sizeof(line))),"bounded phase input/EOF");const std::string command=line;
      if(command=="positive"||command=="negative"||command=="minus-three"||command=="two"||command=="zero") {
        gz::msgs::Double m;
        if(command=="positive")m.set_data(1.5);else if(command=="negative")m.set_data(-1.5);else if(command=="minus-three")m.set_data(-3);else if(command=="two")m.set_data(2);
        require(force.Publish(m),"actual canonical Double publish");std::cout<<"SENT "<<command<<std::endl;
      } else if(command.rfind("prime ",0)==0) {
        const auto target=natural(command.substr(6),1000000000);
        {std::lock_guard<std::mutex> lock(peer.seen->mutex);require(peer.watching&&!peer.seen->bad&&peer.seen->mode==Mode::Frozen&&peer.seen->fifo.empty(),"prime only outside strict recording");
          peer.seen->target=target;peer.seen->havePrime=false;peer.seen->mode=Mode::Prime;}
        std::cout<<"PRIMING "<<target<<std::endl;
        std::unique_lock<std::mutex> lock(peer.seen->mutex);
        require(peer.seen->changed.wait_for(lock,5s,[&]{return peer.seen->bad||peer.seen->havePrime;}),"actual same-Model priming receipt deadline");
        require(!peer.seen->bad&&peer.seen->havePrime,"priming callback: "+peer.seen->failure);packet(peer.seen->prime);std::cout<<"PRIMED "<<target<<std::endl;
      } else if(command=="arm") {
        std::lock_guard<std::mutex> lock(peer.seen->mutex);require(peer.watching&&!peer.seen->bad&&peer.seen->mode==Mode::Frozen&&peer.seen->havePrime&&peer.seen->target>0&&peer.seen->fifo.empty(),"arm after positive-stamp receipt");
        // Do not clear or silently discard any callback once strict recording starts.
        peer.seen->mode=Mode::Record;std::cout<<"ARMED"<<std::endl;
      } else if(command.rfind("drain ",0)==0) {
        const auto count=natural(command.substr(6),1);std::unique_lock<std::mutex> lock(peer.seen->mutex);
        require(peer.watching&&peer.seen->mode==Mode::Record,"drain requires armed subscriber");
        require(peer.seen->changed.wait_for(lock,5s,[&]{return peer.seen->bad||peer.seen->fifo.size()>=static_cast<std::size_t>(count);}),"exact FIFO callback deadline");
        require(!peer.seen->bad&&peer.seen->fifo.size()==static_cast<std::size_t>(count),"exact FIFO count: "+peer.seen->failure);
        for(const auto &m:peer.seen->fifo){validate(m);packet(m);}peer.seen->fifo.clear();std::cout<<"DRAINED "<<count<<std::endl;
      } else if(command=="drop") {
        peer.drop();std::cout<<"DROPPED"<<std::endl;
      } else if(command=="watch") {
        peer.watch();peer.ready(force);std::cout<<"WATCHING"<<std::endl;
      } else if(command=="retired") {
        require(!peer.watching,"drop before endpoint retirement");const auto deadline=std::chrono::steady_clock::now()+5s;
        for(;;){std::vector<gz::transport::MessagePublisher> pubs,subs;peer.node.TopicInfo(stateTopic,pubs,subs);if(!force.HasConnections()&&pubs.empty())break;
          require(std::chrono::steady_clock::now()<deadline,"owned endpoint retirement deadline");std::this_thread::yield();}
        std::cout<<"RETIRED"<<std::endl;
      } else if(command=="finish") {
        require(!peer.watching,"drop before final owned exit");std::lock_guard<std::mutex> lock(peer.seen->mutex);require(!peer.seen->bad&&peer.seen->fifo.empty(),"all FIFO records consumed");
        std::cout<<"PASS canonical joint force feedback"<<std::endl;return 0;
      } else throw std::runtime_error("unknown finite phase command");
    }
    throw std::runtime_error("finite phase count cap");
  }catch(const std::exception &error){std::cerr<<"native joint force feedback peer: "<<error.what()<<'\n';return 1;}
}
