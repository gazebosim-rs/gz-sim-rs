// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Transport-only canonical consumer; no native sensor or Physics oracle.
#include <gz/msgs/fluid_pressure.pb.h>
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
using Message=gz::msgs::FluidPressure;
void require(bool value,const std::string &why){if(!value)throw std::runtime_error(why);}
long long number(const std::string &s,long long cap){require(!s.empty()&&s.find_first_not_of("0123456789")==std::string::npos,"decimal phase field");std::size_t used=0;auto n=std::stoll(s,&used);require(used==s.size()&&n<=cap,"phase field bound");return n;}
long long stamp(const Message &m){require(m.has_header()&&m.header().has_stamp(),"canonical header/stamp");const auto &t=m.header().stamp();require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000&&t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"stamp bounds");return t.sec()*1000000000LL+t.nsec();}
std::uint64_t validate(const Message &m,unsigned index){
  require(m.ByteSizeLong()<=8192,"callback payload cap before retained copy");require(stamp(m)>0,"positive actual acquisition");
  require(std::isfinite(m.pressure())&&m.pressure()>0&&std::isfinite(m.variance())&&m.variance()>=0,"finite independent channels");
  const auto &h=m.header();require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)=="rig"+std::to_string(index)+"/body/air_pressure"&&h.data(1).key()=="seq"&&h.data(1).value_size()==1,"exact per-topic frame/sequence");
  const auto &text=h.data(1).value(0);require(!text.empty()&&text.find_first_not_of("0123456789")==std::string::npos,"decimal sequence");std::size_t used=0;const auto sequence=std::stoull(text,&used);require(used==text.size()&&std::to_string(sequence)==text,"canonical sequence");return sequence;
}
void packet(unsigned index,const Message &m){const auto wire=m.SerializeAsString();require(wire.size()<=8192,"wire cap");constexpr char d[]="0123456789abcdef";std::string out;out.reserve(wire.size()*2);for(unsigned char c:wire){out+=d[c>>4];out+=d[c&15];}std::cout<<"PACKET "<<index<<' '<<out<<'\n';}
enum class Mode{Frozen,Prime,Record};
struct State{std::mutex mutex;std::condition_variable changed;Mode mode=Mode::Frozen;Message prime;std::deque<Message> fifo;std::shared_ptr<std::atomic<bool>> active;bool watching=false,have=false,bad=false;std::string failure;long long target=0,last=0;std::uint64_t sequence=0;};
struct Peer{
  gz::transport::Node node;std::vector<std::shared_ptr<State>> states;
  explicit Peer(unsigned count){for(unsigned i=0;i<count;++i)states.push_back(std::make_shared<State>());}
  std::string topic(unsigned i){return "/m50/air_pressure"+std::to_string(i);}
  void ready(unsigned i){const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,s;node.TopicInfo(topic(i),p,s);if(p.size()==1&&s.size()==1&&p[0].MsgTypeName()=="gz.msgs.FluidPressure"&&s[0].MsgTypeName()=="gz.msgs.FluidPressure")return;require(std::chrono::steady_clock::now()<deadline,"typed per-topic discovery deadline");std::this_thread::yield();}}
  void watch(unsigned i){auto s=states.at(i);{std::lock_guard<std::mutex> lock(s->mutex);require(!s->watching&&!s->bad&&s->fifo.empty(),"fresh subscriber lease");s->active=std::make_shared<std::atomic<bool>>(true);s->mode=Mode::Frozen;s->have=false;}auto lease=s->active;
    std::function<void(const Message &)> callback=[s,lease,i](const Message &m){std::lock_guard<std::mutex> lock(s->mutex);if(!lease->load(std::memory_order_acquire)||s->bad)return;try{
      const auto seq=validate(m,i);const auto t=stamp(m);
      if(s->mode==Mode::Prime&&t==s->target){s->prime=m;s->have=true;s->mode=Mode::Frozen;}
      else if(s->mode==Mode::Record){const long long periods[]={10000000,20000000,40000000,10000000,20000000,40000000,10000000,20000000};require(s->fifo.size()<64,"bounded FIFO before copy");require(t>s->last&&t%periods[i]==0&&s->sequence!=UINT64_MAX&&seq==s->sequence+1,"strict per-topic stamp/sequence after cutoff");s->fifo.push_back(m);s->last=t;s->sequence=seq;}
    }catch(const std::exception &e){s->bad=true;s->failure=e.what();}s->changed.notify_all();};require(node.Subscribe(topic(i),callback),"canonical FluidPressure subscription");s->watching=true;ready(i);
  }
  void drop(unsigned i){auto s=states.at(i);{std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->fifo.empty(),"drain before owned disconnect");s->active->store(false,std::memory_order_release);s->mode=Mode::Frozen;}require(node.Unsubscribe(topic(i)),"actual subscription retirement");s->watching=false;}
};
}
int main(int argc,char **argv){try{
  require(argc==2,"expected bounded sensor count");const auto count=number(argv[1],8);require(count>=1,"positive peer count");Peer peer(static_cast<unsigned>(count));for(unsigned i=0;i<peer.states.size();++i)peer.watch(i);std::cout<<"READY multiple-air-pressures "<<count<<std::endl;
  for(unsigned phase=0;phase<16384;++phase){char line[97]{};require(static_cast<bool>(std::cin.getline(line,sizeof(line))),"bounded phase or EOF");std::istringstream in(line);std::string command,indexText,arg,extra;in>>command;
    if(command=="finish"){require(!(in>>extra),"finish arity");for(const auto &s:peer.states){std::lock_guard<std::mutex> lock(s->mutex);require(!s->watching&&!s->bad&&s->fifo.empty(),"all owned callbacks drained");}std::cout<<"PASS canonical multiple air pressures"<<std::endl;return 0;}
    require(static_cast<bool>(in>>indexText),"sensor index required");const auto i=static_cast<unsigned>(number(indexText,count-1));auto s=peer.states.at(i);
    if(command=="prime"){require(static_cast<bool>(in>>arg)&&!(in>>extra),"prime arity");const auto target=number(arg,4000000000LL);std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->fifo.empty()&&s->mode==Mode::Frozen,"prime outside recording");s->have=false;s->target=target;s->mode=Mode::Prime;std::cout<<"PRIMING "<<i<<std::endl;
    }else if(command=="cutoff"){require(!(in>>extra),"cutoff arity");std::unique_lock<std::mutex> lock(s->mutex);require(s->changed.wait_for(lock,5s,[&]{return s->bad||s->have;}),"same-topic cutoff callback deadline");require(!s->bad&&s->have,"actual cutoff: "+s->failure);packet(i,s->prime);std::cout<<"PRIMED "<<i<<' '<<stamp(s->prime)<<std::endl;
    }else if(command=="arm"){require(!(in>>extra),"arm arity");std::lock_guard<std::mutex> lock(s->mutex);require(s->watching&&!s->bad&&s->have&&s->mode==Mode::Frozen&&s->fifo.empty(),"arm after actual same-topic cutoff");s->last=stamp(s->prime);s->sequence=validate(s->prime,i);s->mode=Mode::Record;std::cout<<"ARMED "<<i<<std::endl;
    }else if(command=="drain"){require(static_cast<bool>(in>>arg)&&!(in>>extra),"drain arity");const auto n=number(arg,1);std::unique_lock<std::mutex> lock(s->mutex);require(s->watching&&s->mode==Mode::Record,"armed drain");require(s->changed.wait_for(lock,5s,[&]{return s->bad||s->fifo.size()>=static_cast<std::size_t>(n);}),"exact per-topic FIFO deadline");require(!s->bad&&s->fifo.size()==static_cast<std::size_t>(n),"exact packet count: "+s->failure);for(const auto &m:s->fifo)packet(i,m);s->fifo.clear();std::cout<<"DRAINED "<<i<<' '<<n<<std::endl;
    }else if(command=="drop"){require(!(in>>extra),"drop arity");peer.drop(i);std::cout<<"DROPPED "<<i<<std::endl;
    }else if(command=="watch"){require(!(in>>extra),"watch arity");peer.watch(i);std::cout<<"WATCHING "<<i<<std::endl;
    }else if(command=="retired"){require(!(in>>extra)&&!s->watching,"drop before retirement");const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,q;peer.node.TopicInfo(peer.topic(i),p,q);if(p.empty())break;require(std::chrono::steady_clock::now()<deadline,"owned publisher retirement deadline");std::this_thread::yield();}std::cout<<"RETIRED "<<i<<std::endl;
    }else throw std::runtime_error("unknown finite phase");
  }throw std::runtime_error("finite phase cap");
}catch(const std::exception &e){std::cerr<<"native multiple-air_pressure peer: "<<e.what()<<'\n';return 1;}}
