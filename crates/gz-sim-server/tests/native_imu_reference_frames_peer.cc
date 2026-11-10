// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Canonical transport consumer only; not a native sensor/Physics oracle.
#include <gz/msgs/imu.pb.h>
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
using Message=gz::msgs::IMU;
constexpr const char *topic="/m43/imu";
void require(bool c,const std::string &s){if(!c)throw std::runtime_error(s);}
long long stamp(const Message &m){
  require(m.has_header()&&m.header().has_stamp(),"IMU header");const auto &t=m.header().stamp();
  require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000&&t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"stamp bounds");return t.sec()*1000000000LL+t.nsec();
}
std::uint64_t sequence(const Message &m){
  const auto &h=m.header();require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)=="rig/body/imu"&&h.data(1).key()=="seq"&&h.data(1).value_size()==1,"exact frame/sequence");
  const auto &s=h.data(1).value(0);require(!s.empty()&&s.find_first_not_of("0123456789")==std::string::npos,"decimal sequence");std::size_t used=0;auto n=std::stoull(s,&used);require(used==s.size()&&std::to_string(n)==s,"canonical sequence");return n;
}
void validate(const Message &m,bool enabled){
  require(m.ByteSizeLong()<=8192,"canonical cap before retained copy");require(stamp(m)>0,"actual positive acquisition time");(void)sequence(m);
  require(m.entity_name()=="rig::body::imu"&&m.has_orientation()==enabled&&m.has_angular_velocity()&&m.has_linear_acceleration(),"exact IMU field presence/name");
  const auto finite=[](const gz::msgs::Vector3d &v){return std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z());};
  require(finite(m.angular_velocity())&&finite(m.linear_acceleration()),"finite motion channels");
  if(enabled){const auto &q=m.orientation();const double n=q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w();require(std::isfinite(n)&&std::abs(n-1)<1e-12,"unit actual orientation");}
  require(m.has_orientation_covariance()&&m.has_angular_velocity_covariance()&&m.has_linear_acceleration_covariance(),"all covariance fields present");
  require(m.orientation_covariance().data_size()==9&&m.angular_velocity_covariance().data_size()==9&&m.linear_acceleration_covariance().data_size()==9,"full covariance shapes");
  for(int i=0;i<9;++i){require(m.orientation_covariance().data(i)==0&&std::isfinite(m.angular_velocity_covariance().data(i))&&std::isfinite(m.linear_acceleration_covariance().data(i)),"finite covariances/unknown orientation variance");}
}
void packet(const Message &m){const auto bytes=m.SerializeAsString();require(bytes.size()<=8192,"packet cap");constexpr char d[]="0123456789abcdef";std::string out;out.reserve(bytes.size()*2);for(unsigned char b:bytes){out+=d[b>>4];out+=d[b&15];}std::cout<<"PACKET "<<out<<'\n';}
enum class Mode {Frozen,Prime,Record};
struct State{std::mutex mutex;std::condition_variable changed;std::deque<Message> fifo;Message prime;Mode mode=Mode::Frozen;bool have=false,bad=false;long long target=-1,last=0;std::uint64_t seq=0;std::string failure;};
struct Peer{
  gz::transport::Node node;std::shared_ptr<State> state=std::make_shared<State>();std::shared_ptr<std::atomic<bool>> active;bool watching=false;const bool orientation;explicit Peer(bool enabled):orientation(enabled){}
  void watch(){require(!watching,"already subscribed");auto lease=std::make_shared<std::atomic<bool>>(true);active=lease;
    std::function<void(const Message &)> cb=[s=state,lease,enabled=orientation](const Message &m){std::lock_guard<std::mutex> lock(s->mutex);if(!lease->load(std::memory_order_acquire)||s->bad)return;
      try{validate(m,enabled);const auto t=stamp(m);const auto q=sequence(m);
        if(s->mode==Mode::Prime&&(s->target<0||t==s->target)){s->prime=m;s->have=true;s->mode=Mode::Frozen;}
        else if(s->mode==Mode::Record){require(s->fifo.size()<256,"FIFO cap before copy");require(t>s->last&&t%10000000==0&&s->seq!=UINT64_MAX&&q==s->seq+1,"subsequent exact rate-grid/sequence");s->fifo.push_back(m);s->last=t;s->seq=q;}
      }catch(const std::exception &e){s->bad=true;s->failure=e.what();}s->changed.notify_all();};
    require(node.Subscribe(topic,cb),"typed IMU subscriber");watching=true;
  }
  void ready(){const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,s;node.TopicInfo(topic,p,s);if(p.size()==1&&s.size()==1&&p[0].MsgTypeName()=="gz.msgs.IMU"&&s[0].MsgTypeName()=="gz.msgs.IMU")return;require(std::chrono::steady_clock::now()<deadline,"typed discovery deadline");std::this_thread::yield();}}
  void drop(){require(watching,"already dropped");{std::lock_guard<std::mutex> lock(state->mutex);require(!state->bad&&state->fifo.empty(),"drain before disconnect");active->store(false,std::memory_order_release);state->mode=Mode::Frozen;}require(node.Unsubscribe(topic),"subscriber retirement");watching=false;}
};
long long number(const std::string &s,long long limit){require(!s.empty()&&s.find_first_not_of("0123456789")==std::string::npos,"bounded decimal phase");std::size_t used=0;auto n=std::stoll(s,&used);require(used==s.size()&&n<=limit,"phase integer bound");return n;}
}
int main(int argc,char **argv){try{
  require(argc==2&&(std::string(argv[1])=="enabled"||std::string(argv[1])=="disabled"),"expected enabled|disabled orientation contract");Peer p(std::string(argv[1])=="enabled");p.watch();p.ready();std::cout<<"READY imu-reference-frames"<<std::endl;
  for(unsigned phase=0;phase<4096;++phase){char line[65]{};require(static_cast<bool>(std::cin.getline(line,sizeof(line))),"bounded phase/EOF");const std::string command=line;
    if(command.rfind("prime ",0)==0){const auto arg=command.substr(6);const auto target=arg=="any"?-1:number(arg,2000000000LL);
      {std::lock_guard<std::mutex> lock(p.state->mutex);require(p.watching&&!p.state->bad&&p.state->mode==Mode::Frozen&&p.state->fifo.empty(),"prime outside strict recording");p.state->target=target;p.state->have=false;p.state->mode=Mode::Prime;}
      std::cout<<"PRIMING "<<arg<<std::endl;std::unique_lock<std::mutex> lock(p.state->mutex);require(p.state->changed.wait_for(lock,5s,[&]{return p.state->bad||p.state->have;}),"actual prime callback deadline");require(!p.state->bad&&p.state->have,"prime failure: "+p.state->failure);packet(p.state->prime);std::cout<<"PRIMED "<<stamp(p.state->prime)<<std::endl;
    }else if(command=="arm"){
      std::lock_guard<std::mutex> lock(p.state->mutex);require(p.watching&&!p.state->bad&&p.state->have&&p.state->mode==Mode::Frozen&&p.state->fifo.empty(),"arm after actual same-topic cutoff");p.state->last=stamp(p.state->prime);p.state->seq=sequence(p.state->prime);p.state->mode=Mode::Record;std::cout<<"ARMED"<<std::endl;
    }else if(command.rfind("drain ",0)==0){const auto count=number(command.substr(6),1);std::unique_lock<std::mutex> lock(p.state->mutex);require(p.watching&&p.state->mode==Mode::Record,"armed drain");require(p.state->changed.wait_for(lock,5s,[&]{return p.state->bad||p.state->fifo.size()>=static_cast<std::size_t>(count);}),"FIFO deadline");require(!p.state->bad&&p.state->fifo.size()==static_cast<std::size_t>(count),"exact FIFO count: "+p.state->failure);for(const auto &m:p.state->fifo)packet(m);p.state->fifo.clear();std::cout<<"DRAINED "<<count<<std::endl;
    }else if(command=="drop"){p.drop();std::cout<<"DROPPED"<<std::endl;
    }else if(command=="watch"){p.watch();p.ready();std::cout<<"WATCHING"<<std::endl;
    }else if(command=="retired"){require(!p.watching,"drop before retirement");const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> pubs,subs;p.node.TopicInfo(topic,pubs,subs);if(pubs.empty())break;require(std::chrono::steady_clock::now()<deadline,"publisher retirement deadline");std::this_thread::yield();}std::cout<<"RETIRED"<<std::endl;
    }else if(command=="finish"){require(!p.watching,"drop before finish");std::lock_guard<std::mutex> lock(p.state->mutex);require(!p.state->bad&&p.state->fifo.empty(),"all actual callbacks consumed");std::cout<<"PASS canonical IMU reference frames"<<std::endl;return 0;
    }else throw std::runtime_error("unknown phase");
  }throw std::runtime_error("finite phase cap");
}catch(const std::exception &e){std::cerr<<"native IMU reference peer: "<<e.what()<<'\n';return 1;}}
