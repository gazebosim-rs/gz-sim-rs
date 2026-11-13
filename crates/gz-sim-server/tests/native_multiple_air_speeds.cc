// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Fresh original AirSpeed System multiplicity only. Controlled ECM hierarchy,
// SENSOR WORLD pose / POINT WORLD velocity / WORLD wind are input records,
// not a native Physics trajectory. Public relativeVel records a native handoff
// witness, not an observation of the private sensor getter.
// Prospective Source schedule: pair31 calls/60 inputs/11 packets, eight31/240/37,
// removal17/31/7. Counts become evidence only after actual bounded capture+DATA.
#include <gz/msgs/air_speed.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocitySeed.hh>
#include <gz/sim/components/AirSpeedSensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/common/Console.hh>
#include <sdf/Plugin.hh>
#include <sdf/Sensor.hh>
#include <sdf/sdf.hh>
#include <algorithm>
#include <array>
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
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
using namespace std::chrono_literals;
namespace {
using Message=gz::msgs::AirSpeed;
void require(bool value,const std::string &why){if(!value)throw std::runtime_error(why);}
std::string hex(const std::string &s){constexpr char d[]="0123456789abcdef";std::string out;out.reserve(s.size()*2);for(unsigned char c:s){out+=d[c>>4];out+=d[c&15];}return out;}
std::string bits(double value){std::uint64_t n;std::memcpy(&n,&value,sizeof(n));std::ostringstream out;out<<std::hex<<std::setfill('0')<<std::setw(16)<<n;return out.str();}
void poseBits(const gz::math::Pose3d &p){std::cout<<"[\""<<bits(p.Pos().X())<<"\",\""<<bits(p.Pos().Y())<<"\",\""<<bits(p.Pos().Z())<<"\",\""<<bits(p.Rot().X())<<"\",\""<<bits(p.Rot().Y())<<"\",\""<<bits(p.Rot().Z())<<"\",\""<<bits(p.Rot().W())<<"\"]";}
void maps(const std::string &path){std::ifstream input("/proc/self/maps");require(bool(input),"maps open");std::string text,line;while(std::getline(input,line)){require(text.size()+line.size()+1<=4*1024*1024,"maps cap");text+=line+'\n';}require(input.eof(),"complete maps read");const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);require(fd>=0,"exclusive maps output");std::size_t offset=0;while(offset<text.size()){const auto n=::write(fd,text.data()+offset,text.size()-offset);if(n<=0){::close(fd);throw std::runtime_error("maps write");}offset+=static_cast<std::size_t>(n);}const bool flushed=::fsync(fd)==0;const bool closed=::close(fd)==0;require(flushed&&closed,"maps durable completion");}
long long stamp(const Message &m){require(m.has_header()&&m.header().has_stamp(),"actual header/stamp");const auto &t=m.header().stamp();require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000&&t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"stamp bounds");return t.sec()*1000000000LL+t.nsec();}
struct Received{std::mutex mutex;std::condition_variable changed;std::vector<Message> messages;std::shared_ptr<std::atomic<bool>> active=std::make_shared<std::atomic<bool>>(true);bool bad=false;std::string failure;};
void subscribe(gz::transport::Node &node,const std::string &topic,const std::shared_ptr<Received> &s){auto lease=s->active;std::function<void(const Message &)> callback=[s,lease](const Message &m){std::lock_guard<std::mutex> lock(s->mutex);if(!lease->load(std::memory_order_acquire)||s->bad)return;try{require(m.ByteSizeLong()<=8192&&s->messages.size()<8,"callback count/bytes before copy");require(std::isfinite(m.diff_pressure())&&std::isfinite(m.temperature())&&m.temperature()>0,"finite actual output");(void)stamp(m);s->messages.push_back(m);}catch(const std::exception &e){s->bad=true;s->failure=e.what();}s->changed.notify_all();};require(node.Subscribe(topic,callback),"typed air_speed subscription");}
void ready(gz::transport::Node &node,const std::string &topic,bool publisher){const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,s;node.TopicInfo(topic,p,s);if(publisher?(p.size()==1&&s.size()==1&&p[0].MsgTypeName()=="gz.msgs.AirSpeed"&&s[0].MsgTypeName()=="gz.msgs.AirSpeed"):p.empty())return;require(std::chrono::steady_clock::now()<deadline,publisher?"all-connected typed readiness":"actual removed publisher retirement");std::this_thread::yield();}}
void vectorBits(const gz::math::Vector3d &v){std::cout<<"[\""<<bits(v.X())<<"\",\""<<bits(v.Y())<<"\",\""<<bits(v.Z())<<"\"]";}
struct Sensor{unsigned index;unsigned period;gz::sim::Entity model;gz::sim::Entity entity;std::string topic;std::shared_ptr<Received> received;bool alive=true;unsigned consumed=0;};
gz::math::Pose3d modelInput(const Sensor &s,long long ns){const double t=std::chrono::duration<double>(std::chrono::nanoseconds(ns)).count();return gz::math::Pose3d(2+s.index,-1,3+s.index+(1+.25*s.index)*t+.5*t*t,.1*t,-.2*t,(.3+.05*s.index)*t);}
gz::math::Vector3d pointInput(const Sensor &s,long long ns){const double t=std::chrono::duration<double>(std::chrono::nanoseconds(ns)).count();return gz::math::Vector3d((s.index%2==0?1:-1)*(3+.25*s.index)+.1*t,-.5+.1*s.index,.25-.05*s.index);}
void observation(const std::string &mode,Sensor &s,long long ns,const gz::math::Pose3d &world){
  std::unique_lock<std::mutex> lock(s.received->mutex);const auto wanted=s.consumed+1;
  require(s.received->changed.wait_for(lock,5s,[&]{return s.received->bad||s.received->messages.size()>=wanted;}),"actual per-topic acquisition deadline");require(!s.received->bad&&s.received->messages.size()==wanted,"exact per-topic callbacks: "+s.received->failure);const auto &m=s.received->messages[s.consumed];
  require(stamp(m)==ns&&std::isfinite(world.Pos().Z())&&std::isfinite(m.diff_pressure())&&std::isfinite(m.temperature())&&m.temperature()>0,"actual signed noiseless input/output scope");
  const auto &h=m.header();require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)=="m51/native/"+std::to_string(s.index)&&h.data(1).key()=="seq"&&h.data(1).value_size()==1&&h.data(1).value(0)==std::to_string(s.consumed),"exact independent frame/sequence");
  std::cout<<"{\"kind\":\"observation\",\"mode\":"<<std::quoted(mode)<<",\"sensor\":"<<s.index<<",\"time_ns\":"<<ns<<",\"sequence\":"<<std::quoted(h.data(1).value(0))<<",\"diff_pressure_bits\":\""<<bits(m.diff_pressure())<<"\",\"temperature_bits\":\""<<bits(m.temperature())<<"\",\"header_wire\":"<<std::quoted(hex(h.SerializeAsString()))<<",\"wire\":"<<std::quoted(hex(m.SerializeAsString()))<<"}\n";++s.consumed;
}
}
int main(int argc,char **argv){bool captured=false;try{
  require(argc==4,"expected ORIGINAL_DSO MODE MAPS_PATH");const std::string mode=argv[2];require(mode=="pair"||mode=="eight"||mode=="remove","declared multiplicity mode");const unsigned count=mode=="eight"?8:2;const std::filesystem::path mapPath(argv[3]);require(mapPath.is_absolute(),"owned absolute maps path");require(gz::common::Console::Init(mapPath.parent_path().string(),mapPath.filename().string()+".console.log"),"owned Console Init");require(gz::common::Console::Directory()==mapPath.parent_path().string(),"public owned Console directory");
  sdf::Plugin config(argv[1],"gz::sim::systems::AirSpeed");gz::sim::SystemLoader loader;auto loaded=loader.LoadPlugin(config);require(bool(loaded),"existing original AirSpeed System DSO");auto *configure=(*loaded)->QueryInterface<gz::sim::ISystemConfigure>();auto *pre=(*loaded)->QueryInterface<gz::sim::ISystemPreUpdate>();auto *post=(*loaded)->QueryInterface<gz::sim::ISystemPostUpdate>();require(configure&&pre&&post,"actual original interfaces");
  gz::sim::EntityComponentManager ecm;const auto world=ecm.CreateEntity();ecm.CreateComponent(world,gz::sim::components::World());ecm.CreateComponent(world,gz::sim::components::Name("default"));ecm.CreateComponent(world,gz::sim::components::Pose(gz::math::Pose3d::Zero));
  // Original Load finds this marker and Update dereferences its velocity seed.
  const gz::math::Vector3d wind(1,-2,.5);const auto windEntity=ecm.CreateEntity();ecm.CreateComponent(windEntity,gz::sim::components::Wind());ecm.CreateComponent(windEntity,gz::sim::components::WorldLinearVelocitySeed(wind));
  gz::sim::EventManager events;configure->Configure(world,{},ecm,events);
  gz::transport::Node node;std::vector<Sensor> sensors;
  for(unsigned i=0;i<count;++i){const unsigned periods[]={5,10,20,5,10,20,5,10};const unsigned period=periods[i];const std::string topic="/m51/native/air_speed/"+mode+"/"+std::to_string(i);
    const auto model=ecm.CreateEntity();ecm.CreateComponent(model,gz::sim::components::Model());ecm.CreateComponent(model,gz::sim::components::Name("rig"+std::to_string(i)));ecm.CreateComponent(model,gz::sim::components::ParentEntity(world));ecm.CreateComponent(model,gz::sim::components::Pose(gz::math::Pose3d(2+i,-1,3+i,0,0,0)));
    const auto link=ecm.CreateEntity();ecm.CreateComponent(link,gz::sim::components::Link());ecm.CreateComponent(link,gz::sim::components::Name("body"));ecm.CreateComponent(link,gz::sim::components::ParentEntity(model));ecm.CreateComponent(link,gz::sim::components::Pose(gz::math::Pose3d::Zero));
    const double yaw=i%2==0?.3:-.4;
    std::ostringstream xml;xml<<std::setprecision(17)<<"<sdf version='1.12'><model name='rig'><link name='body'><sensor name='air_speed' type='air_speed'><pose>.25 -.5 "<<(.5+.25*i)<<" .1 -.2 "<<yaw<<"</pose><update_rate>"<<(1000/period)<<"</update_rate><topic>"<<topic<<"</topic><frame_id>m51/native/"<<i<<"</frame_id><air_speed/></sensor></link></model></sdf>";
    sdf::SDFPtr parsed(new sdf::SDF);sdf::init(parsed);require(sdf::readString(xml.str(),parsed),"finite per-sensor XML");sdf::Sensor dom;require(dom.Load(parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor")).empty(),"actual sensor DOM load");
    require(dom.AirSpeedSensor()!=nullptr&&dom.UpdateRate()==1000/period&&dom.Topic()==topic&&dom.FrameId()=="m51/native/"+std::to_string(i),"actual parsed independent air_speed configuration");
    const auto entity=ecm.CreateEntity();ecm.CreateComponent(entity,gz::sim::components::Sensor());ecm.CreateComponent(entity,gz::sim::components::AirSpeedSensor(dom));ecm.CreateComponent(entity,gz::sim::components::Name("air_speed"));ecm.CreateComponent(entity,gz::sim::components::ParentEntity(link));ecm.CreateComponent(entity,gz::sim::components::Pose(dom.RawPose()));
    const auto initialWorld=gz::sim::worldPose(entity,ecm);ecm.CreateComponent(entity,gz::sim::components::WorldPose(initialWorld));ecm.CreateComponent(entity,gz::sim::components::WorldLinearVelocity(gz::math::Vector3d::Zero));
    auto received=std::make_shared<Received>();subscribe(node,topic,received);sensors.push_back({i,period,model,entity,topic,received,true,0});
    std::cout<<"{\"kind\":\"configuration\",\"mode\":"<<std::quoted(mode)<<",\"sensor\":"<<i<<",\"topic\":"<<std::quoted(topic)<<",\"frame_id\":\"m51/native/"<<i<<"\",\"rate\":"<<(1000/period)<<",\"period_ns\":"<<period*1000000LL<<",\"initial_world_pose_bits\":";poseBits(initialWorld);std::cout<<",\"sensor_local_pose_bits\":";poseBits(dom.RawPose());std::cout<<",\"world_wind_bits\":";vectorBits(wind);std::cout<<",\"sdf_xml_hex\":"<<std::quoted(hex(xml.str()))<<"}\n";
  }
  std::cout<<std::setprecision(17);gz::sim::UpdateInfo info;info.paused=true;info.simTime=0ns;info.dt=0ns;pre->PreUpdate(info,ecm);post->PostUpdate(info,ecm);ecm.ClearNewlyCreatedEntities();
  for(auto &s:sensors)ready(node,s.topic,true);
  unsigned calls=1,observations=0,inputs=0;long long removedAt=-1,retiredAt=-1;
  for(unsigned tick=1;tick<=30;++tick){if(mode=="remove"&&tick>15&&tick!=20)continue;const long long ns=tick*1000000LL;
    if(mode=="remove"&&tick==20){auto &s=sensors[0];{std::lock_guard<std::mutex> lock(s.received->mutex);require(!s.received->bad&&s.received->messages.size()==s.consumed,"drained removed subscriber");s.received->active->store(false,std::memory_order_release);}require(node.Unsubscribe(s.topic),"owned removed subscriber retirement");ecm.RequestRemoveEntity(s.entity);s.alive=false;removedAt=15000000;
      std::cout<<"{\"kind\":\"removal\",\"mode\":\"remove\",\"sensor\":0,\"after_time_ns\":15000000,\"next_system_time_ns\":20000000,\"peer_connected_required\":true}\n";
    }
    // These are explicit per-live-owner controller input records. They are not
    // an inventory of every internal Sensor::Update call after family wakeup.
    for(auto &s:sensors)if(s.alive){ecm.SetComponentData<gz::sim::components::Pose>(s.model,modelInput(s,ns));const auto p=gz::sim::worldPose(s.entity,ecm);const auto velocity=pointInput(s,ns);ecm.SetComponentData<gz::sim::components::WorldPose>(s.entity,p);ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(s.entity,velocity);const auto relative=gz::sim::relativeVel(s.entity,ecm);const auto actualWind=ecm.Component<gz::sim::components::WorldLinearVelocitySeed>(windEntity);require(actualWind!=nullptr&&bits(actualWind->Data().X())==bits(wind.X())&&bits(actualWind->Data().Y())==bits(wind.Y())&&bits(actualWind->Data().Z())==bits(wind.Z()),"actual constant WORLD wind seed");
      std::cout<<"{\"kind\":\"input\",\"mode\":"<<std::quoted(mode)<<",\"sensor\":"<<s.index<<",\"time_ns\":"<<ns<<",\"world_pose_bits\":";poseBits(p);std::cout<<",\"world_point_velocity_bits\":";vectorBits(velocity);std::cout<<",\"world_wind_bits\":";vectorBits(actualWind->Data());std::cout<<",\"native_relative_velocity_handoff_bits\":";vectorBits(relative);std::cout<<"}\n";++inputs;}
    info.paused=false;info.iterations=tick;info.simTime=std::chrono::nanoseconds(ns);info.dt=std::chrono::nanoseconds(mode=="remove"&&tick==20?5000000:1000000);
    const auto markedPresent=[&]{bool marked=false;unsigned pending=0;ecm.EachRemoved<gz::sim::components::AirSpeedSensor>([&](const gz::sim::Entity &entity,const gz::sim::components::AirSpeedSensor *component){require(++pending<=count,"bounded pending air_speed removal view");if(entity==sensors[0].entity){require(component!=nullptr,"marked air_speed component present");marked=true;}return true;});require(marked&&ecm.HasEntity(sensors[0].entity)&&ecm.Component<gz::sim::components::AirSpeedSensor>(sensors[0].entity)!=nullptr,"marked entity/AirSpeedSensor retained through original systems");};
    if(mode=="remove"&&tick==20)markedPresent();pre->PreUpdate(info,ecm);if(mode=="remove"&&tick==20)markedPresent();post->PostUpdate(info,ecm);++calls;
    if(mode=="remove"&&tick==20){markedPresent();ecm.ProcessRemoveEntityRequests();require(!ecm.HasEntity(sensors[0].entity),"actual physical ECM removal after systems");ecm.ClearRemovedComponents();}
    if(!captured){maps(argv[3]);captured=true;}
    for(auto &s:sensors)if(s.alive&&(tick==1||tick%s.period==0)){observation(mode,s,ns,gz::sim::worldPose(s.entity,ecm));++observations;}
    if(mode=="remove"&&tick==20){ready(node,sensors[0].topic,false);retiredAt=ns;}
  }
  for(auto &s:sensors)if(s.alive){{std::lock_guard<std::mutex> lock(s.received->mutex);require(!s.received->bad&&s.received->messages.size()==s.consumed,"exact final per-topic observation inventory");s.received->active->store(false,std::memory_order_release);}require(node.Unsubscribe(s.topic),"closing subscriber retirement");}
  require(captured&&observations==(mode=="pair"?11u:mode=="eight"?37u:7u),"closed prospective packet ledger observed");
  require(calls==(mode=="remove"?17u:31u)&&inputs==(mode=="pair"?60u:mode=="eight"?240u:31u),"complete prospective call/input ledger observed");
  std::cout<<"{\"kind\":\"summary\",\"mode\":"<<std::quoted(mode)<<",\"configuration_count\":"<<count<<",\"system_calls\":"<<calls<<",\"input_rows\":"<<inputs<<",\"canonical_packets\":"<<observations<<",\"removal_after_time_ns\":"<<removedAt<<",\"retirement_observed_system_time_ns\":"<<retiredAt<<",\"native_family_demand_policy\":true,\"native_physics_executed\":false,\"old_native_modes_replayed\":false}\n";return 0;
}catch(const std::exception &e){if(argc==4&&!captured){try{maps(argv[3]);}catch(...){}}std::cerr<<"original multiple-air-speed system: "<<e.what()<<'\n';return 1;}}
