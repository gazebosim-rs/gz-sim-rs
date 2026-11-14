// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Typed canonical Rust Model peer; native reference ID/order evidence remains separate.
#include <gz/msgs/model.pb.h>
#include <set>
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
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace {
using Message=gz::msgs::Model;
void require(bool value,const std::string &why){if(!value)throw std::runtime_error(why);}
long long number(const std::string &s,long long cap){require(!s.empty()&&s.find_first_not_of("0123456789")==std::string::npos,"decimal phase field");std::size_t used=0;auto n=std::stoll(s,&used);require(used==s.size()&&n<=cap,"phase field bound");return n;}
struct Packet{std::string wire;long long stamp;};
Packet validate(const Message &m,unsigned slot){
  require(m.ByteSizeLong()<=65536&&m.has_header()&&m.header().has_stamp()&&m.header().data_size()==0,"bounded Model with stamp-only Header");
  require(m.id()!=0&&m.name()=="rig"+std::to_string(slot)&&m.has_pose(),"independent Rust owner identity and LOCAL model pose");
  const auto &t=m.header().stamp();
  require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000&&t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"bounded stamp");
  const auto ns=t.sec()*1000000000LL+t.nsec();require(ns>0,"positive cutoff/record time");
  auto pose=[](const gz::msgs::Pose &p){
    require(!p.has_header()&&p.id()==0&&p.name().empty()&&p.has_position()&&p.has_orientation(),"complete canonical pose without invented identity/header");
    const auto &v=p.position();const auto &q=p.orientation();require(!v.has_header()&&!q.has_header(),"absent nested headers");
    const double values[]={v.x(),v.y(),v.z(),q.x(),q.y(),q.z(),q.w()};
    for(double value:values)require(std::isfinite(value),"finite actual LOCAL pose");
    require(std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1)<=1e-12,"unit LOCAL quaternion");
  };
  pose(m.pose());
  require(m.joint_size()==(slot%2==0?2:1),"independent full/filter-deduplicated selected set");
  unsigned previous=0;
  for(int index=0;index<m.joint_size();++index){
    const auto &j=m.joint(index);require(j.id()>previous&&j.id()!=m.id(),"strict Rust u32 joint identity order");previous=j.id();
    const bool base=slot%2==0&&index==0;
    require(j.name()==(base?"base_hinge":"tip_hinge")&&j.parent()==(base?"world":"base")&&j.child()==(base?"base":"tip"),"exact scoped owner's construction joint metadata");
    require(!j.has_header()&&j.has_pose()&&j.has_axis1()&&!j.has_axis2(),"canonical single-axis joint fields only");pose(j.pose());
    const auto &a=j.axis1();require(!a.has_header()&&a.has_xyz()&&!a.xyz().has_header()&&a.xyz_expressed_in().empty(),"canonical resolved axis metadata");
    require(a.xyz().x()==0&&a.xyz().y()==0&&a.xyz().z()==1,"retained revolute axis");
    require(a.limit_lower()==-INFINITY&&a.limit_upper()==INFINITY&&a.damping()==0,"unchanged unbounded joint metadata");
    require(std::isfinite(a.position())&&std::isfinite(a.velocity())&&a.force()==0,"finite measured q/v and omitted force");
  }
  return {m.SerializeAsString(),ns};
}
void packet(unsigned slot,const Packet &m){constexpr char d[]="0123456789abcdef";std::string out;out.reserve(m.wire.size()*2);for(unsigned char c:m.wire){out+=d[c>>4];out+=d[c&15];}std::cout<<"PACKET "<<slot<<' '<<out<<'\n';}
enum class Mode{Frozen,Prime,Record};
struct State{std::mutex mutex;std::condition_variable changed;Mode mode=Mode::Frozen;Packet prime;std::deque<Packet> fifo;std::shared_ptr<std::atomic<bool>> active;bool watching=false,have=false,bad=false;std::string failure;long long target=0,last=0;};
struct Peer{
  gz::transport::Node node;std::vector<std::shared_ptr<State>> states;
  explicit Peer(unsigned count){for(unsigned i=0;i<count;++i)states.push_back(std::make_shared<State>());}
  std::string topic(unsigned slot){return "/m57/joint"+std::to_string(slot);}
  std::string type(unsigned slot){(void)slot;return "gz.msgs.Model";}
  void ready(unsigned slot){const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,s;node.TopicInfo(topic(slot),p,s);if(p.size()==1&&s.size()==1&&p[0].MsgTypeName()==type(slot)&&s[0].MsgTypeName()==type(slot))return;require(std::chrono::steady_clock::now()<deadline,"typed per-channel readiness deadline");std::this_thread::yield();}}
  void subscribe(unsigned slot,const std::shared_ptr<State> &s){auto lease=s->active;std::function<void(const Message &)> callback=[s,lease,slot](const Message &m){std::lock_guard<std::mutex> lock(s->mutex);if(!lease->load(std::memory_order_acquire)||s->bad)return;try{const auto value=validate(m,slot);if(s->mode==Mode::Prime&&value.stamp==s->target){s->prime=value;s->have=true;s->mode=Mode::Frozen;}else if(s->mode==Mode::Record){const long long period=slot%2==0?10000000:20000000;require(s->fifo.size()<64,"bounded FIFO before retained copy");require(value.stamp>s->last&&value.stamp%period==0,"strict actual independent channel stamp/cadence");s->fifo.push_back(value);s->last=value.stamp;}}catch(const std::exception &e){s->bad=true;s->failure=e.what();}s->changed.notify_all();};require(node.Subscribe(topic(slot),callback),"typed per-owner subscription");}
  void watch(unsigned slot){auto s=states.at(slot);{std::lock_guard<std::mutex> lock(s->mutex);require(!s->watching&&!s->bad&&s->fifo.empty(),"fresh callback lease");s->active=std::make_shared<std::atomic<bool>>(true);s->mode=Mode::Frozen;s->have=false;}subscribe(slot,s);s->watching=true;ready(slot);}
  void drop(unsigned slot){auto s=states.at(slot);{std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->fifo.empty(),"drain before disconnect");s->active->store(false,std::memory_order_release);s->mode=Mode::Frozen;}require(node.Unsubscribe(topic(slot)),"actual channel unsubscribe");s->watching=false;}
};
}
int main(int argc,char **argv){try{
  require(argc==2,"expected bounded owner count");const auto count=number(argv[1],8);require(count>=1,"positive peer count");Peer peer(static_cast<unsigned>(count));for(unsigned slot=0;slot<peer.states.size();++slot)peer.watch(slot);std::cout<<"READY multiple-joint-states "<<count<<std::endl;
  for(unsigned phase=0;phase<16384;++phase){char line[97]{};require(static_cast<bool>(std::cin.getline(line,sizeof(line))),"bounded phase or EOF");std::istringstream in(line);std::string command,indexText,arg,extra;in>>command;
    if(command=="finish"){require(!(in>>extra),"finish arity");for(const auto &s:peer.states){std::lock_guard<std::mutex> lock(s->mutex);require(!s->watching&&!s->bad&&s->fifo.empty(),"all callbacks drained");}std::cout<<"PASS canonical multiple joint states"<<std::endl;return 0;}
    require(static_cast<bool>(in>>indexText),"channel slot required");const auto slot=static_cast<unsigned>(number(indexText,count-1));auto s=peer.states.at(slot);
    if(command=="prime"){require(static_cast<bool>(in>>arg)&&!(in>>extra),"prime arity");const auto target=number(arg,4000000000LL);std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->fifo.empty()&&s->mode==Mode::Frozen,"prime outside recording");s->have=false;s->target=target;s->mode=Mode::Prime;std::cout<<"PRIMING "<<slot<<std::endl;
    }else if(command=="cutoff"){require(!(in>>extra),"cutoff arity");std::unique_lock<std::mutex> lock(s->mutex);require(s->changed.wait_for(lock,5s,[&]{return s->bad||s->have;}),"actual same-channel cutoff deadline");require(!s->bad&&s->have,"actual cutoff: "+s->failure);packet(slot,s->prime);std::cout<<"PRIMED "<<slot<<' '<<s->prime.stamp<<std::endl;
    }else if(command=="arm"){require(!(in>>extra),"arm arity");std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->have&&s->mode==Mode::Frozen&&s->fifo.empty(),"no-step arm after actual cutoff");s->last=s->prime.stamp;s->mode=Mode::Record;std::cout<<"ARMED "<<slot<<std::endl;
    }else if(command=="drain"){require(static_cast<bool>(in>>arg)&&!(in>>extra),"drain arity");const auto n=number(arg,1);std::unique_lock<std::mutex> lock(s->mutex);require(s->watching&&s->mode==Mode::Record,"armed channel drain");require(s->changed.wait_for(lock,5s,[&]{return s->bad||s->fifo.size()>=static_cast<std::size_t>(n);}),"exact FIFO deadline");require(!s->bad&&s->fifo.size()==static_cast<std::size_t>(n),"exact channel count: "+s->failure);for(const auto &m:s->fifo)packet(slot,m);s->fifo.clear();std::cout<<"DRAINED "<<slot<<' '<<n<<std::endl;
    }else if(command=="drop"){require(!(in>>extra),"drop arity");peer.drop(slot);std::cout<<"DROPPED "<<slot<<std::endl;
    }else if(command=="watch"){require(!(in>>extra),"watch arity");peer.watch(slot);std::cout<<"WATCHING "<<slot<<std::endl;
    }else if(command=="retired"){require(!(in>>extra)&&!s->watching,"drop before retirement");const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,q;peer.node.TopicInfo(peer.topic(slot),p,q);if(p.empty())break;require(std::chrono::steady_clock::now()<deadline,"actual owned publisher retirement");std::this_thread::yield();}std::cout<<"RETIRED "<<slot<<std::endl;
    }else throw std::runtime_error("unknown finite phase");
  }throw std::runtime_error("finite phase cap");
}catch(const std::exception &e){std::cerr<<"multiple JointStatePublisher peer: "<<e.what()<<'\n';return 1;}}
