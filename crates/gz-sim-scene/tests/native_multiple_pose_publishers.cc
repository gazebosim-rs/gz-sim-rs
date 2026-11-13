// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// NEW direct-original-object controlled System reference; never loads the old DSO.
// Compile/link resolution is a future actual gate. No private implementation copy.
#include "PosePublisher.hh"
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/common/Console.hh>
#include <sdf/Plugin.hh>
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
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
using namespace std::chrono_literals;
namespace {
using Message=gz::msgs::Pose_V;
using Entity=gz::sim::Entity;
namespace c=gz::sim::components;
void require(bool value,const std::string &why){if(!value)throw std::runtime_error(why);}
std::string hex(const std::string &s){constexpr char d[]="0123456789abcdef";std::string out;out.reserve(s.size()*2);for(unsigned char ch:s){out+=d[ch>>4];out+=d[ch&15];}return out;}
std::string bits(double value){require(std::isfinite(value),"finite input/output bits");std::uint64_t n;std::memcpy(&n,&value,sizeof(n));std::ostringstream out;out<<std::hex<<std::setfill('0')<<std::setw(16)<<n;return out.str();}
void poseBits(const gz::math::Pose3d &p){std::cout<<"[\""<<bits(p.Pos().X())<<"\",\""<<bits(p.Pos().Y())<<"\",\""<<bits(p.Pos().Z())<<"\",\""<<bits(p.Rot().X())<<"\",\""<<bits(p.Rot().Y())<<"\",\""<<bits(p.Rot().Z())<<"\",\""<<bits(p.Rot().W())<<"\"]";}
void maps(const std::string &path){std::ifstream input("/proc/self/maps");require(bool(input),"maps open");std::string text,line;while(std::getline(input,line)){require(line.find("pose-publisher-system")==std::string::npos,"original PosePublisher DSO must not be mapped");require(text.size()+line.size()+1<=4*1024*1024,"maps cap");text+=line+'\n';}require(input.eof(),"complete maps read");const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);require(fd>=0,"exclusive maps output");std::size_t offset=0;while(offset<text.size()){const auto n=::write(fd,text.data()+offset,text.size()-offset);if(n<=0){::close(fd);throw std::runtime_error("maps write");}offset+=static_cast<std::size_t>(n);}const bool flushed=::fsync(fd)==0;const bool closed=::close(fd)==0;require(flushed&&closed,"maps durable completion");}
long long stamp(const gz::msgs::Pose &p){require(p.has_header()&&p.header().has_stamp(),"actual per-Pose header/stamp");const auto &t=p.header().stamp();require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000&&t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"stamp bounds");return t.sec()*1000000000LL+t.nsec();}
void validate(const Message &m,long long ns){require(m.ByteSizeLong()<=65536&&!m.has_header()&&m.pose_size()>0&&m.pose_size()<=128,"bounded nonempty vector/absent outer header");std::set<std::string> children;for(const auto &p:m.pose()){require(stamp(p)==ns&&p.id()==0&&p.has_position()&&p.has_orientation(),"actual matching stamp/zero wire ID/full local pose");const auto &h=p.header();require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&!h.data(0).value(0).empty()&&h.data(1).key()=="child_frame_id"&&h.data(1).value_size()==1&&p.name()==h.data(1).value(0)&&children.insert(p.name()).second,"exact ordered frame roles and unique child");const auto &v=p.position();const auto &q=p.orientation();require(!v.has_header()&&!q.has_header(),"no nested headers");const double values[]={v.x(),v.y(),v.z(),q.x(),q.y(),q.z(),q.w()};for(double value:values)require(std::isfinite(value),"finite actual local pose");require(std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1)<=1e-12,"actual unit orientation");}}
struct Packet{unsigned call;bool fixed;Message message;};
struct Received{std::mutex mutex;std::condition_variable changed;std::vector<Packet> packets;std::shared_ptr<std::atomic<bool>> active=std::make_shared<std::atomic<bool>>(true);unsigned call=0;long long ns=0;bool armed=false,bad=false;std::string failure;};
void subscribe(gz::transport::Node &node,const std::string &topic,bool fixed,const std::shared_ptr<Received> &s){auto lease=s->active;std::function<void(const Message &)> callback=[s,lease,fixed](const Message &m){std::lock_guard<std::mutex> lock(s->mutex);if(!lease->load(std::memory_order_acquire)||s->bad)return;try{require(s->armed&&s->packets.size()<8,"armed bounded callback before copy");validate(m,s->ns);s->packets.push_back({s->call,fixed,m});}catch(const std::exception &e){s->bad=true;s->failure=e.what();}s->changed.notify_all();};require(node.Subscribe(topic,callback),"typed Pose_V channel subscription");}
void ready(gz::transport::Node &node,const std::string &topic){const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,s;node.TopicInfo(topic,p,s);if(p.size()==1&&s.size()==1&&p[0].MsgTypeName()=="gz.msgs.Pose_V"&&s[0].MsgTypeName()=="gz.msgs.Pose_V")return;require(std::chrono::steady_clock::now()<deadline,"all-connected typed channel readiness");std::this_thread::yield();}}
Entity entity(gz::sim::EntityComponentManager &ecm,Entity parent,const std::string &name,const gz::math::Pose3d &pose){const auto id=ecm.CreateEntity();ecm.CreateComponent(id,c::Name(name));ecm.CreateComponent(id,c::Pose(pose));require(ecm.SetParentEntity(id,parent),"actual ECM parent graph");return id;}
struct Owner{unsigned index;Entity model,base,tip,joint;std::string name,topic;double dynamicRate,staticRate;std::unique_ptr<gz::sim::System> system;gz::sim::ISystemPostUpdate *post;std::shared_ptr<Received> received;unsigned consumed=0;};
gz::math::Pose3d input(unsigned owner,unsigned role,unsigned call){return gz::math::Pose3d(1+owner+.125*call+.25*role,2+3*owner-.2*call+.5*role,3+.1*call-.25*role,.03*call+.05*role,-.04*call+.02*owner,.2*owner+.1*call-.15*role);}
void nodeCard(gz::sim::EntityComponentManager &ecm,Entity id,const std::string &kind,bool selected,const std::string &declared){const auto *name=ecm.Component<c::Name>(id);const auto *parent=ecm.Component<c::ParentEntity>(id);const auto *pose=ecm.Component<c::Pose>(id);require(name&&parent&&pose,"complete configured role inputs");std::cout<<"{\"native_entity\":"<<id<<",\"parent_entity\":"<<parent->Data()<<",\"kind\":"<<std::quoted(kind)<<",\"name\":"<<std::quoted(name->Data())<<",\"selected\":"<<(selected?"true":"false")<<",\"declared_channel\":"<<std::quoted(declared)<<",\"canonical_link\":"<<(ecm.Component<c::CanonicalLink>(id)?"true":"false")<<",\"initial_local_pose_bits\":";poseBits(pose->Data());std::cout<<'}';}
void observation(const std::string &mode,const Owner &o,const Packet &p){const auto &m=p.message;std::cout<<"{\"kind\":\"observation\",\"mode\":"<<std::quoted(mode)<<",\"owner\":"<<o.index<<",\"call_index\":"<<p.call<<",\"channel\":"<<std::quoted(p.fixed?"static":"dynamic")<<",\"time_ns\":"<<stamp(m.pose(0))<<",\"records\":[";bool comma=false;for(const auto &value:m.pose()){if(comma)std::cout<<',';comma=true;std::cout<<"{\"name\":"<<std::quoted(value.name())<<",\"frame_id\":"<<std::quoted(value.header().data(0).value(0))<<",\"child_frame_id\":"<<std::quoted(value.header().data(1).value(0))<<",\"id\":"<<value.id()<<",\"pose_bits\":[\""<<bits(value.position().x())<<"\",\""<<bits(value.position().y())<<"\",\""<<bits(value.position().z())<<"\",\""<<bits(value.orientation().x())<<"\",\""<<bits(value.orientation().y())<<"\",\""<<bits(value.orientation().z())<<"\",\""<<bits(value.orientation().w())<<"\"],\"header_wire\":"<<std::quoted(hex(value.header().SerializeAsString()))<<",\"pose_wire\":"<<std::quoted(hex(value.SerializeAsString()))<<'}';}std::cout<<"],\"wire\":"<<std::quoted(hex(m.SerializeAsString()))<<"}\n";}
}
int main(int argc,char **argv){bool captured=false;try{
  require(argc==3,"expected MODE MAPS_PATH");const std::string mode=argv[1];require(mode=="pair"||mode=="eight","closed multiplicity modes");const unsigned count=mode=="pair"?2:8;const std::filesystem::path mapPath(argv[2]);require(mapPath.is_absolute(),"owned absolute maps path");require(gz::common::Console::Init(mapPath.parent_path().string(),mapPath.filename().string()+".console.log"),"owned Console Init");require(gz::common::Console::Directory()==mapPath.parent_path().string(),"actual owned Console directory");gz::common::Console::SetVerbosity(2);require(gz::common::Console::Verbosity()==2,"warning-visible actual Console verbosity");
  gz::sim::EntityComponentManager ecm;gz::sim::EventManager events;const auto world=ecm.CreateEntity();ecm.CreateComponent(world,c::World());ecm.CreateComponent(world,c::Name("default"));ecm.CreateComponent(world,c::Pose(gz::math::Pose3d::Zero));gz::transport::Node node;std::vector<Owner> owners;
  for(unsigned i=0;i<count;++i){const std::string name="rig"+std::to_string(i);const std::string topic="/m55/native/"+mapPath.parent_path().filename().string()+"/"+mode+"/pose"+std::to_string(i);const double dynamicRate=i%2==0?100:50,staticRate=i%2==0?25:100;const auto model=entity(ecm,world,name,input(i,0,0));ecm.CreateComponent(model,c::Model());const auto base=entity(ecm,model,"base",input(i,1,0));ecm.CreateComponent(base,c::Link());ecm.CreateComponent(base,c::CanonicalLink());ecm.CreateComponent(model,c::ModelCanonicalLink(base));const auto tip=entity(ecm,model,"tip",input(i,2,0));ecm.CreateComponent(tip,c::Link());const auto joint=entity(ecm,model,"hinge",gz::math::Pose3d::Zero);ecm.CreateComponent(joint,c::Joint());ecm.CreateComponent(joint,c::JointType(sdf::JointType::REVOLUTE));ecm.CreateComponent(joint,c::ParentLinkName("base"));ecm.CreateComponent(joint,c::ChildLinkName("tip"));require(ecm.EntityByComponents(c::Name("base"),c::Link(),c::ParentEntity(model))==base&&ecm.EntityByComponents(c::Name("tip"),c::Link(),c::ParentEntity(model))==tip,"real selected joint endpoint Links");
    std::ostringstream content;content<<std::setprecision(17)<<"<publish_link_pose>true</publish_link_pose><publish_model_pose>true</publish_model_pose><publish_nested_model_pose>false</publish_nested_model_pose><publish_visual_pose>false</publish_visual_pose><publish_collision_pose>false</publish_collision_pose><publish_sensor_pose>false</publish_sensor_pose><use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>true</static_publisher><update_frequency>"<<dynamicRate<<"</update_frequency><static_update_frequency>"<<staticRate<<"</static_update_frequency><topic>"<<topic<<"</topic>";
    sdf::Plugin plugin("direct-original-object-not-a-loader-input","gz::sim::systems::PosePublisher",content.str());const auto parsed=plugin.ToElement();require(bool(parsed)&&parsed->Get<bool>("use_pose_vector_msg")&&parsed->Get<bool>("static_publisher")&&parsed->Get<double>("update_frequency")==dynamicRate&&parsed->Get<double>("static_update_frequency")==staticRate&&parsed->Get<std::string>("topic")==topic,"actual nonnull parsed configuration inputs");
    // Only System is an ownership/deletion type. No concrete PImpl destructor
    // instantiation, replacement factory or original-provider rebuild here.
    std::unique_ptr<gz::sim::System> system(new gz::sim::systems::PosePublisher());auto *configure=dynamic_cast<gz::sim::ISystemConfigure*>(system.get());auto *post=dynamic_cast<gz::sim::ISystemPostUpdate*>(system.get());require(configure&&post,"original public checked interfaces");configure->Configure(model,parsed,ecm,events);auto received=std::make_shared<Received>();subscribe(node,topic,false,received);subscribe(node,topic+"_static",true,received);owners.push_back({i,model,base,tip,joint,name,topic,dynamicRate,staticRate,std::move(system),post,received,0});
    std::cout<<"{\"kind\":\"configuration\",\"mode\":"<<std::quoted(mode)<<",\"owner\":"<<i<<",\"native_entity\":"<<model<<",\"world_entity\":"<<world<<",\"world_name\":"<<std::quoted(ecm.Component<c::Name>(world)->Data())<<",\"world_pose_bits\":";poseBits(ecm.Component<c::Pose>(world)->Data());std::cout<<",\"name\":"<<std::quoted(name)<<",\"topic\":"<<std::quoted(topic)<<",\"static_topic\":"<<std::quoted(topic+"_static")<<",\"dynamic_rate_bits\":"<<std::quoted(bits(dynamicRate))<<",\"static_rate_bits\":"<<std::quoted(bits(staticRate))<<",\"static_publisher\":true,\"use_pose_vector_msg\":true,\"model_canonical_link\":"<<ecm.Component<c::ModelCanonicalLink>(model)->Data()<<",\"joint_type\":"<<static_cast<int>(ecm.Component<c::JointType>(joint)->Data())<<",\"parent_link_name\":"<<std::quoted(ecm.Component<c::ParentLinkName>(joint)->Data())<<",\"child_link_name\":"<<std::quoted(ecm.Component<c::ChildLinkName>(joint)->Data())<<",\"console_verbosity\":"<<gz::common::Console::Verbosity()<<",\"sdf_xml_hex\":"<<std::quoted(hex(parsed->ToString("")))<<",\"entities\":[";nodeCard(ecm,model,"model",true,"static");std::cout<<',';nodeCard(ecm,base,"link",true,"static");std::cout<<',';nodeCard(ecm,tip,"link",true,"dynamic");std::cout<<',';nodeCard(ecm,joint,"joint",false,"unselected");std::cout<<"]}\n";
  }
  for(const auto &o:owners){ready(node,o.topic);ready(node,o.topic+"_static");}
  const long long times[]={0,10000000,20000000,40000000};unsigned calls=0,observations=0,poseRecords=0;
  for(unsigned call=0;call<4;++call){for(auto &o:owners){const auto ns=times[call];{std::lock_guard<std::mutex> lock(o.received->mutex);require(!o.received->bad&&o.received->packets.size()==o.consumed,"previous callback fence before changing input");o.received->call=call;o.received->ns=ns;o.received->armed=true;}
      const Entity ids[]={o.model,o.base,o.tip};std::cout<<"{\"kind\":\"input\",\"mode\":"<<std::quoted(mode)<<",\"owner\":"<<o.index<<",\"call_index\":"<<call<<",\"time_ns\":"<<ns<<",\"dt_ns\":"<<(call?times[call]-times[call-1]:0)<<",\"paused\":false,\"local_poses\":[";for(unsigned role=0;role<3;++role){const auto value=input(o.index,role,call);(void)ecm.SetComponentData<c::Pose>(ids[role],value);const auto *actual=ecm.Component<c::Pose>(ids[role]);require(actual!=nullptr,"actual LOCAL pose getback");if(role)std::cout<<',';std::cout<<"{\"native_entity\":"<<ids[role]<<",\"input_pose_bits\":";poseBits(value);std::cout<<",\"getter_pose_bits\":";poseBits(actual->Data());std::cout<<'}';}std::cout<<"]}\n";
      gz::sim::UpdateInfo info;info.paused=false;info.iterations=call;info.simTime=std::chrono::nanoseconds(ns);info.dt=std::chrono::nanoseconds(call?times[call]-times[call-1]:0);o.post->PostUpdate(info,ecm);++calls;
      const bool dynamic=call==0||call==3||o.index%2==0||call==2;const bool fixed=call==0||call==3||o.index%2==1;const unsigned wanted=static_cast<unsigned>(dynamic)+static_cast<unsigned>(fixed);std::unique_lock<std::mutex> lock(o.received->mutex);require(o.received->changed.wait_for(lock,5s,[&]{return o.received->bad||o.received->packets.size()>=o.consumed+wanted;}),"actual per-call callback fence");require(!o.received->bad&&o.received->packets.size()==o.consumed+wanted,"exact callback count: "+o.received->failure);bool sawDynamic=false,sawStatic=false;for(unsigned n=0;n<wanted;++n){const auto &packet=o.received->packets[o.consumed++];require(packet.call==call&&((packet.fixed&&fixed&&!sawStatic)||(!packet.fixed&&dynamic&&!sawDynamic)),"unique actual owner/call/channel");std::set<std::string> actualNames;for(const auto &pose:packet.message.pose())actualNames.insert(pose.name());const std::set<std::string> selectedNames=packet.fixed?std::set<std::string>{o.name,o.name+"::base"}:std::set<std::string>{o.name+"::tip"};require(actualNames==selectedNames,"actual selected static/dynamic membership");if(packet.fixed)sawStatic=true;else sawDynamic=true;observation(mode,o,packet);poseRecords+=packet.message.pose_size();++observations;}require(sawDynamic==dynamic&&sawStatic==fixed,"actual both-channel membership");o.received->armed=false;
    }if(!captured){maps(argv[2]);captured=true;}ecm.ClearNewlyCreatedEntities();}
  for(auto &o:owners){{std::lock_guard<std::mutex> lock(o.received->mutex);require(!o.received->bad&&o.received->packets.size()==o.consumed,"final complete callback ledger");o.received->active->store(false,std::memory_order_release);}require(node.Unsubscribe(o.topic)&&node.Unsubscribe(o.topic+"_static"),"owned subscriber cleanup");}
  require(captured&&calls==4*count&&observations==(mode=="pair"?13u:52u)&&poseRecords==(mode=="pair"?19u:76u),"closed Source-derived actual ledger");
  std::cout<<"{\"kind\":\"summary\",\"mode\":"<<std::quoted(mode)<<",\"configuration_count\":"<<count<<",\"direct_post_update_calls\":"<<calls<<",\"input_rows\":"<<calls<<",\"observed_vectors\":"<<observations<<",\"canonical_packets\":"<<observations<<",\"pose_records\":"<<poseRecords<<",\"original_object_linked_System_scope\":true,\"original_Server_or_plugin_loader_executed\":false,\"native_Physics_executed\":false,\"native_demand_or_lifecycle_equivalence_claimed\":false,\"global_RNG_equivalence_claimed\":false,\"old_native_modes_replayed\":false}\n";std::cout.flush();require(bool(std::cout),"complete stdout");return 0;
}catch(const std::exception &e){if(!captured&&argc==3){try{maps(argv[2]);}catch(...){}}std::cerr<<"original direct-object multiple PosePublishers: "<<e.what()<<'\n';return 1;}}
