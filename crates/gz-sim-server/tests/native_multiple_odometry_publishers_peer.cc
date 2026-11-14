// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Rust canonical transport consumer only; no original System or estimator.
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>
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
void require(bool value,const std::string &why){if(!value)throw std::runtime_error(why);}
long long number(const std::string &s,long long cap){require(!s.empty()&&s.find_first_not_of("0123456789")==std::string::npos,"decimal phase field");std::size_t used=0;auto n=std::stoll(s,&used);require(used==s.size()&&n<=cap,"phase field bound");return n;}
struct Packet{long long time=0;std::string wire;};
long long header(const gz::msgs::Header &h,unsigned owner){
 require(h.has_stamp()&&h.data_size()==2,"stamp and exactly two frame maps, no sequence");const auto&t=h.stamp();require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000&&t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"stamp bounds");
 require(h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)=="m56/odom"+std::to_string(owner)&&h.data(1).key()=="child_frame_id"&&h.data(1).value_size()==1&&h.data(1).value(0)=="m56/base"+std::to_string(owner),"per-owner exact frame metadata");return t.sec()*1000000000LL+t.nsec();
}
void pose(const gz::msgs::Pose&p){require(p.has_position()&&p.has_orientation()&&!p.has_header()&&p.name().empty()&&p.id()==0,"canonical nested pose shape");const auto&v=p.position();const auto&q=p.orientation();require(std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z())&&std::isfinite(q.x())&&std::isfinite(q.y())&&std::isfinite(q.z())&&std::isfinite(q.w())&&std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1)<=1e-12,"finite unit pose");}
void twist(const gz::msgs::Twist&t){require(t.has_linear()&&t.has_angular(),"complete twist");for(const auto*v:{&t.linear(),&t.angular()})require(std::isfinite(v->x())&&std::isfinite(v->y())&&std::isfinite(v->z()),"finite velocity");}
Packet checked(const gz::msgs::Odometry&m,unsigned owner){require(m.ByteSizeLong()<=8192&&m.has_header()&&m.has_pose()&&m.has_twist(),"bounded complete odometry");pose(m.pose());twist(m.twist());return{header(m.header(),owner),m.SerializeAsString()};}
Packet checked(const gz::msgs::OdometryWithCovariance&m,unsigned owner){require(m.ByteSizeLong()<=8192&&m.has_header()&&m.has_pose_with_covariance()&&m.has_twist_with_covariance(),"bounded complete covariance wrapper");const auto&p=m.pose_with_covariance();const auto&t=m.twist_with_covariance();require(p.has_pose()&&p.has_covariance()&&t.has_twist()&&t.has_covariance(),"nested covariance presence");pose(p.pose());twist(t.twist());for(const auto*c:{&p.covariance(),&t.covariance()}){require(c->data_size()==36,"all36 covariance words");for(float value:c->data())require(value==0,"ideal exact zero float covariance");}return{header(m.header(),owner),m.SerializeAsString()};}
Packet checked(const gz::msgs::Pose_V&m,unsigned owner){require(m.ByteSizeLong()<=8192&&!m.has_header()&&m.pose_size()==1&&m.pose(0).has_header(),"single TF with perPose header and absent outer header");auto p=m.pose(0);const auto time=header(p.header(),owner);p.clear_header();pose(p);return{time,m.SerializeAsString()};}
long long stamp(const Packet&p){return p.time;}
void packet(unsigned slot,const Packet&p){require(p.wire.size()<=8192,"wire bound");constexpr char digits[]="0123456789abcdef";std::string out;out.reserve(p.wire.size()*2);for(unsigned char c:p.wire){out+=digits[c>>4];out+=digits[c&15];}std::cout<<"PACKET "<<slot<<' '<<out<<'\n';}
enum class Mode{Frozen,Prime,Record};
struct State{std::mutex mutex;std::condition_variable changed;Mode mode=Mode::Frozen;Packet prime;std::deque<Packet>fifo;std::shared_ptr<std::atomic<bool>>active;bool watching=false,have=false,bad=false;std::string failure;long long target=0,last=0;};
struct Peer{
 gz::transport::Node node;std::vector<std::shared_ptr<State>>states;
 explicit Peer(unsigned count){for(unsigned i=0;i<count*3;++i)states.push_back(std::make_shared<State>());}
 std::string topic(unsigned slot){const char*names[]={"odom","cov","tf"};return "/m56/"+std::string(names[slot%3])+std::to_string(slot/3);}
 std::string type(unsigned slot){const char*names[]={"gz.msgs.Odometry","gz.msgs.OdometryWithCovariance","gz.msgs.Pose_V"};return names[slot%3];}
 void ready(unsigned slot){const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher>p,s;node.TopicInfo(topic(slot),p,s);if(p.size()==1&&s.size()==1&&p[0].MsgTypeName()==type(slot)&&s[0].MsgTypeName()==type(slot))return;require(std::chrono::steady_clock::now()<deadline,"typed channel discovery deadline");std::this_thread::yield();}}
 template<class M>void subscribe(unsigned slot,const std::shared_ptr<State>&s,const std::shared_ptr<std::atomic<bool>>&lease){std::function<void(const M&)>callback=[s,lease,slot](const M&m){std::lock_guard<std::mutex>lock(s->mutex);if(!lease->load(std::memory_order_acquire)||s->bad)return;try{const auto p=checked(m,slot/3);require(p.time>0,"positive acquisition stamp");if(s->mode==Mode::Prime&&p.time==s->target){s->prime=p;s->have=true;s->mode=Mode::Frozen;}else if(s->mode==Mode::Record){require(s->fifo.size()<64,"FIFO bound before copy");require(p.time>s->last,"strict per-channel acquisition order after cutoff");s->fifo.push_back(p);s->last=p.time;}}catch(const std::exception&e){s->bad=true;s->failure=e.what();}s->changed.notify_all();};require(node.Subscribe(topic(slot),callback),"typed canonical channel subscription");}
 void watch(unsigned slot){auto s=states.at(slot);{std::lock_guard<std::mutex>lock(s->mutex);require(!s->watching&&!s->bad&&s->fifo.empty(),"fresh subscriber lease");s->active=std::make_shared<std::atomic<bool>>(true);s->mode=Mode::Frozen;s->have=false;}switch(slot%3){case 0:subscribe<gz::msgs::Odometry>(slot,s,s->active);break;case 1:subscribe<gz::msgs::OdometryWithCovariance>(slot,s,s->active);break;default:subscribe<gz::msgs::Pose_V>(slot,s,s->active);}s->watching=true;ready(slot);}
 void drop(unsigned slot){auto s=states.at(slot);{std::lock_guard<std::mutex>lock(s->mutex);require(s->watching&&!s->bad&&s->fifo.empty(),"drain before disconnect");s->active->store(false,std::memory_order_release);s->mode=Mode::Frozen;}require(node.Unsubscribe(topic(slot)),"actual owned unsubscribe");s->watching=false;}
};
}
int main(int argc,char **argv){try{
  require(argc==2,"expected bounded sensor count");const auto count=number(argv[1],8);require(count>=1,"positive peer count");Peer peer(static_cast<unsigned>(count));for(unsigned i=0;i<peer.states.size();++i)peer.watch(i);std::cout<<"READY multiple-odometry-publishers "<<count<<std::endl;
  for(unsigned phase=0;phase<16384;++phase){char line[97]{};require(static_cast<bool>(std::cin.getline(line,sizeof(line))),"bounded phase or EOF");std::istringstream in(line);std::string command,indexText,arg,extra;in>>command;
    if(command=="finish"){require(!(in>>extra),"finish arity");for(const auto &s:peer.states){std::lock_guard<std::mutex> lock(s->mutex);require(!s->watching&&!s->bad&&s->fifo.empty(),"all owned callbacks drained");}std::cout<<"PASS canonical multiple odometry publishers"<<std::endl;return 0;}
    require(static_cast<bool>(in>>indexText),"sensor index required");const auto i=static_cast<unsigned>(number(indexText,count*3-1));auto s=peer.states.at(i);
    if(command=="prime"){require(static_cast<bool>(in>>arg)&&!(in>>extra),"prime arity");const auto target=number(arg,4000000000LL);std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->fifo.empty()&&s->mode==Mode::Frozen,"prime outside recording");s->have=false;s->target=target;s->mode=Mode::Prime;std::cout<<"PRIMING "<<i<<std::endl;
    }else if(command=="cutoff"){require(!(in>>extra),"cutoff arity");std::unique_lock<std::mutex> lock(s->mutex);require(s->changed.wait_for(lock,5s,[&]{return s->bad||s->have;}),"same-topic cutoff callback deadline");require(!s->bad&&s->have,"actual cutoff: "+s->failure);packet(i,s->prime);std::cout<<"PRIMED "<<i<<' '<<stamp(s->prime)<<std::endl;
    }else if(command=="arm"){require(!(in>>extra),"arm arity");std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->have&&s->mode==Mode::Frozen&&s->fifo.empty(),"arm after actual same-topic cutoff");s->last=stamp(s->prime);s->mode=Mode::Record;std::cout<<"ARMED "<<i<<std::endl;
    }else if(command=="drain"){require(static_cast<bool>(in>>arg)&&!(in>>extra),"drain arity");const auto n=number(arg,1);std::unique_lock<std::mutex> lock(s->mutex);require(s->watching&&s->mode==Mode::Record,"armed drain");require(s->changed.wait_for(lock,5s,[&]{return s->bad||s->fifo.size()>=static_cast<std::size_t>(n);}),"exact per-topic FIFO deadline");require(!s->bad&&s->fifo.size()==static_cast<std::size_t>(n),"exact packet count: "+s->failure);for(const auto &m:s->fifo)packet(i,m);s->fifo.clear();std::cout<<"DRAINED "<<i<<' '<<n<<std::endl;
    }else if(command=="drop"){require(!(in>>extra),"drop arity");peer.drop(i);std::cout<<"DROPPED "<<i<<std::endl;
    }else if(command=="watch"){require(!(in>>extra),"watch arity");peer.watch(i);std::cout<<"WATCHING "<<i<<std::endl;
    }else if(command=="retired"){require(!(in>>extra)&&!s->watching,"drop before retirement");const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,q;peer.node.TopicInfo(peer.topic(i),p,q);if(p.empty())break;require(std::chrono::steady_clock::now()<deadline,"owned publisher retirement deadline");std::this_thread::yield();}std::cout<<"RETIRED "<<i<<std::endl;
    }else throw std::runtime_error("unknown finite phase");
  }throw std::runtime_error("finite phase cap");
}catch(const std::exception &e){std::cerr<<"multiple-odometry peer: "<<e.what()<<'\n';return 1;}}
