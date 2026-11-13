// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Typed canonical Rust lexical Pose_V peer; distinct from unordered native reference.
#include <gz/msgs/pose_v.pb.h>
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
using Message=gz::msgs::Pose_V;
void require(bool value,const std::string &why){if(!value)throw std::runtime_error(why);}
long long number(const std::string &s,long long cap){require(!s.empty()&&s.find_first_not_of("0123456789")==std::string::npos,"decimal phase field");std::size_t used=0;auto n=std::stoll(s,&used);require(used==s.size()&&n<=cap,"phase field bound");return n;}
struct Packet{std::string wire;long long stamp;};
Packet validate(const Message &m,unsigned slot){require(m.ByteSizeLong()<=65536&&!m.has_header()&&m.pose_size()>0&&m.pose_size()<=128,"bounded nonempty Pose_V/absent outer header");long long ns=-1;std::string previous;for(const auto &p:m.pose()){require(p.id()==0&&p.has_header()&&p.header().has_stamp()&&p.has_position()&&p.has_orientation(),"zero wire ID/complete actual Pose");const auto &h=p.header();require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(1).key()=="child_frame_id"&&h.data(1).value_size()==1&&h.data(1).value(0)==p.name(),"exact ordered per-Pose frame headers");const std::string owner="rig"+std::to_string(slot/2);require(p.name()==owner||p.name().rfind(owner+"::",0)==0,"independent owner frame");require(p.name()>previous,"strict Rust lexical child-frame order");previous=p.name();require(h.data(0).value(0)==(p.name()==owner?"default":owner),"actual local parent frame");const auto &t=h.stamp();require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000&&t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"bounded stamp");const auto stamp=t.sec()*1000000000LL+t.nsec();require(stamp>0&&(ns<0||ns==stamp),"same positive per-vector time");ns=stamp;const auto &v=p.position();const auto &q=p.orientation();require(!v.has_header()&&!q.has_header(),"absent nested headers");const double values[]={v.x(),v.y(),v.z(),q.x(),q.y(),q.z(),q.w()};for(double value:values)require(std::isfinite(value),"finite actual local pose");require(std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1)<=1e-12,"unit local orientation");}return {m.SerializeAsString(),ns};}
void packet(unsigned slot,const Packet &m){constexpr char d[]="0123456789abcdef";std::string out;out.reserve(m.wire.size()*2);for(unsigned char c:m.wire){out+=d[c>>4];out+=d[c&15];}std::cout<<"PACKET "<<slot<<' '<<out<<'\n';}
enum class Mode{Frozen,Prime,Record};
struct State{std::mutex mutex;std::condition_variable changed;Mode mode=Mode::Frozen;Packet prime;std::deque<Packet> fifo;std::shared_ptr<std::atomic<bool>> active;bool watching=false,have=false,bad=false;std::string failure;long long target=0,last=0;};
struct Peer{
  gz::transport::Node node;std::vector<std::shared_ptr<State>> states;
  explicit Peer(unsigned count){for(unsigned i=0;i<2*count;++i)states.push_back(std::make_shared<State>());}
  std::string topic(unsigned slot){return "/m55/pose"+std::to_string(slot/2)+(slot%2==0?"":"_static");}
  std::string type(unsigned slot){(void)slot;return "gz.msgs.Pose_V";}
  void ready(unsigned slot){const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,s;node.TopicInfo(topic(slot),p,s);if(p.size()==1&&s.size()==1&&p[0].MsgTypeName()==type(slot)&&s[0].MsgTypeName()==type(slot))return;require(std::chrono::steady_clock::now()<deadline,"typed per-channel readiness deadline");std::this_thread::yield();}}
  void subscribe(unsigned slot,const std::shared_ptr<State> &s){auto lease=s->active;std::function<void(const Message &)> callback=[s,lease,slot](const Message &m){std::lock_guard<std::mutex> lock(s->mutex);if(!lease->load(std::memory_order_acquire)||s->bad)return;try{const auto value=validate(m,slot);if(s->mode==Mode::Prime&&value.stamp==s->target){s->prime=value;s->have=true;s->mode=Mode::Frozen;}else if(s->mode==Mode::Record){const long long period=slot%2==0?(slot/2%2==0?10000000:20000000):(slot/2%2==0?40000000:10000000);require(s->fifo.size()<64,"bounded FIFO before retained copy");require(value.stamp>s->last&&value.stamp%period==0,"strict actual independent channel stamp/cadence");s->fifo.push_back(value);s->last=value.stamp;}}catch(const std::exception &e){s->bad=true;s->failure=e.what();}s->changed.notify_all();};require(node.Subscribe(topic(slot),callback),"typed paired-channel subscription");}
  void watch(unsigned slot){auto s=states.at(slot);{std::lock_guard<std::mutex> lock(s->mutex);require(!s->watching&&!s->bad&&s->fifo.empty(),"fresh callback lease");s->active=std::make_shared<std::atomic<bool>>(true);s->mode=Mode::Frozen;s->have=false;}subscribe(slot,s);s->watching=true;ready(slot);}
  void drop(unsigned slot){auto s=states.at(slot);{std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->fifo.empty(),"drain before disconnect");s->active->store(false,std::memory_order_release);s->mode=Mode::Frozen;}require(node.Unsubscribe(topic(slot)),"actual channel unsubscribe");s->watching=false;}
};
}
int main(int argc,char **argv){try{
  require(argc==2,"expected bounded owner count");const auto count=number(argv[1],8);require(count>=1,"positive peer count");Peer peer(static_cast<unsigned>(count));for(unsigned slot=0;slot<peer.states.size();++slot)peer.watch(slot);std::cout<<"READY multiple-pose-publishers "<<count<<std::endl;
  for(unsigned phase=0;phase<16384;++phase){char line[97]{};require(static_cast<bool>(std::cin.getline(line,sizeof(line))),"bounded phase or EOF");std::istringstream in(line);std::string command,indexText,arg,extra;in>>command;
    if(command=="finish"){require(!(in>>extra),"finish arity");for(const auto &s:peer.states){std::lock_guard<std::mutex> lock(s->mutex);require(!s->watching&&!s->bad&&s->fifo.empty(),"all callbacks drained");}std::cout<<"PASS canonical multiple pose publishers"<<std::endl;return 0;}
    require(static_cast<bool>(in>>indexText),"channel slot required");const auto slot=static_cast<unsigned>(number(indexText,2*count-1));auto s=peer.states.at(slot);
    if(command=="prime"){require(static_cast<bool>(in>>arg)&&!(in>>extra),"prime arity");const auto target=number(arg,4000000000LL);std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->fifo.empty()&&s->mode==Mode::Frozen,"prime outside recording");s->have=false;s->target=target;s->mode=Mode::Prime;std::cout<<"PRIMING "<<slot<<std::endl;
    }else if(command=="cutoff"){require(!(in>>extra),"cutoff arity");std::unique_lock<std::mutex> lock(s->mutex);require(s->changed.wait_for(lock,5s,[&]{return s->bad||s->have;}),"actual same-channel cutoff deadline");require(!s->bad&&s->have,"actual cutoff: "+s->failure);packet(slot,s->prime);std::cout<<"PRIMED "<<slot<<' '<<s->prime.stamp<<std::endl;
    }else if(command=="arm"){require(!(in>>extra),"arm arity");std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->have&&s->mode==Mode::Frozen&&s->fifo.empty(),"no-step arm after actual cutoff");s->last=s->prime.stamp;s->mode=Mode::Record;std::cout<<"ARMED "<<slot<<std::endl;
    }else if(command=="drain"){require(static_cast<bool>(in>>arg)&&!(in>>extra),"drain arity");const auto n=number(arg,1);std::unique_lock<std::mutex> lock(s->mutex);require(s->watching&&s->mode==Mode::Record,"armed channel drain");require(s->changed.wait_for(lock,5s,[&]{return s->bad||s->fifo.size()>=static_cast<std::size_t>(n);}),"exact FIFO deadline");require(!s->bad&&s->fifo.size()==static_cast<std::size_t>(n),"exact channel count: "+s->failure);for(const auto &m:s->fifo)packet(slot,m);s->fifo.clear();std::cout<<"DRAINED "<<slot<<' '<<n<<std::endl;
    }else if(command=="drop"){require(!(in>>extra),"drop arity");peer.drop(slot);std::cout<<"DROPPED "<<slot<<std::endl;
    }else if(command=="watch"){require(!(in>>extra),"watch arity");peer.watch(slot);std::cout<<"WATCHING "<<slot<<std::endl;
    }else if(command=="retired"){require(!(in>>extra)&&!s->watching,"drop before retirement");const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,q;peer.node.TopicInfo(peer.topic(slot),p,q);if(p.empty())break;require(std::chrono::steady_clock::now()<deadline,"actual owned publisher retirement");std::this_thread::yield();}std::cout<<"RETIRED "<<slot<<std::endl;
    }else throw std::runtime_error("unknown finite phase");
  }throw std::runtime_error("finite phase cap");
}catch(const std::exception &e){std::cerr<<"multiple PosePublisher peer: "<<e.what()<<'\n';return 1;}}
