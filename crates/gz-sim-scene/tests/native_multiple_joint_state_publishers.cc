// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Direct unchanged original-object System; controlled components, no Physics/Loader.
#include "JointStatePublisher.hh"
#include <gz/common/Console.hh>
#include <gz/msgs/model.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
#include <array>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
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
namespace c = gz::sim::components;
using Entity = gz::sim::Entity;
static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559, "IEEE f64");
void require(bool value, const std::string &why) {
  if (!value) throw std::runtime_error(why);
}
std::uint64_t word(double value) {
  std::uint64_t result;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}
void text(std::ostream &out, const std::string &value) {
  require(value.size()<=131072, "bounded string or hex of64KiB wire");
  constexpr char digits[]="0123456789abcdef";
  out << '"';
  for (unsigned char ch:value) {
    if (ch=='"' || ch=='\\') out << '\\' << static_cast<char>(ch);
    else if (ch<32) out << "\\u00" << digits[ch>>4] << digits[ch&15];
    else out << static_cast<char>(ch);
  }
  out << '"';
}
std::string hex(const std::string &value) {
  constexpr char digits[]="0123456789abcdef";
  std::string result; result.reserve(value.size()*2);
  for (unsigned char ch:value) { result+=digits[ch>>4];result+=digits[ch&15]; }
  return result;
}
void bits(std::ostream &out, double value) {
  std::ostringstream encoded;
  encoded << std::hex << std::setfill('0') << std::setw(16) << word(value);
  text(out,encoded.str());
}
void finite(std::ostream &out, double value) {
  require(std::isfinite(value), "finite actual f64");bits(out,value);
}
void limit(std::ostream &out, double value, bool lower) {
  require(!std::isnan(value) && value!=(lower?INFINITY:-INFINITY), "signed limit metadata only");
  bits(out,value);
}
std::array<double,7> values(const gz::math::Pose3d &p) {
  return {p.Pos().X(),p.Pos().Y(),p.Pos().Z(),p.Rot().X(),p.Rot().Y(),p.Rot().Z(),p.Rot().W()};
}
void poseWords(std::ostream &out, const gz::math::Pose3d &p) {
  const auto v=values(p);
  require(std::abs(v[3]*v[3]+v[4]*v[4]+v[5]*v[5]+v[6]*v[6]-1)<=1e-12, "unit fixed input pose");
  out << '[';
  for (unsigned i=0;i<v.size();++i) { if(i)out<<',';finite(out,v[i]); }
  out << ']';
}
void samePose(const gz::math::Pose3d &a,const gz::math::Pose3d &b) {
  const auto x=values(a),y=values(b);
  for (unsigned i=0;i<x.size();++i) require(word(x[i])==word(y[i]), "fixed pose getter bits");
}
void array(std::ostream &out,const std::vector<double> &v) {
  require(v.size()<=1, "single-axis or empty input array");out<<'[';
  for(unsigned i=0;i<v.size();++i){if(i)out<<',';finite(out,v[i]);}out<<']';
}
template<class Message> void wire(std::ostream &out,const Message &m,std::size_t cap=65536) {
  require(m.ByteSizeLong()<=cap, "wire cap before serialization");text(out,hex(m.SerializeAsString()));
}
void header(std::ostream &out,bool present,const gz::msgs::Header &h) {
  if(!present){out<<"null";return;}
  require(h.ByteSizeLong()<=4096 && h.data_size()<=16, "actual Header bound");
  out<<"{\"stamp\":";
  if(h.has_stamp())out<<"{\"sec\":"<<h.stamp().sec()<<",\"nsec\":"<<h.stamp().nsec()<<'}';
  else out<<"null";
  out<<",\"data\":[";
  for(int i=0;i<h.data_size();++i){
    if(i)out<<',';const auto &datum=h.data(i);
    require(datum.key().size()<=256 && datum.value_size()<=8,"Header datum cap");
    out<<"{\"key\":";text(out,datum.key());out<<",\"values\":[";
    for(int j=0;j<datum.value_size();++j){if(j)out<<',';require(datum.value(j).size()<=1024,"Header value cap");text(out,datum.value(j));}
    out<<"]}";
  }
  out<<"],\"wire\":";wire(out,h,4096);out<<'}';
}
void vector(std::ostream &out,bool present,const gz::msgs::Vector3d &v) {
  if(!present){out<<"null";return;}
  out<<"{\"header\":";header(out,v.has_header(),v.header());out<<",\"xyz\":[";
  finite(out,v.x());out<<',';finite(out,v.y());out<<',';finite(out,v.z());out<<"],\"wire\":";wire(out,v);out<<'}';
}
void quaternion(std::ostream &out,bool present,const gz::msgs::Quaternion &q) {
  if(!present){out<<"null";return;}
  require(std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1)<=1e-12,"unit captured pose");
  out<<"{\"header\":";header(out,q.has_header(),q.header());out<<",\"xyzw\":[";
  finite(out,q.x());out<<',';finite(out,q.y());out<<',';finite(out,q.z());out<<',';finite(out,q.w());
  out<<"],\"wire\":";wire(out,q);out<<'}';
}
void pose(std::ostream &out,bool present,const gz::msgs::Pose &p) {
  if(!present){out<<"null";return;}
  out<<"{\"header\":";header(out,p.has_header(),p.header());out<<",\"name\":";text(out,p.name());out<<",\"id\":"<<p.id()<<",\"position\":";
  vector(out,p.has_position(),p.position());out<<",\"orientation\":";quaternion(out,p.has_orientation(),p.orientation());out<<",\"wire\":";wire(out,p);out<<'}';
}
void axis(std::ostream &out,bool present,const gz::msgs::Axis &a) {
  if(!present){out<<"null";return;}
  out<<"{\"header\":";header(out,a.has_header(),a.header());out<<",\"xyz\":";vector(out,a.has_xyz(),a.xyz());
  out<<",\"lower\":";limit(out,a.limit_lower(),true);out<<",\"upper\":";limit(out,a.limit_upper(),false);
  out<<",\"damping\":";finite(out,a.damping());out<<",\"position\":";finite(out,a.position());out<<",\"velocity\":";finite(out,a.velocity());
  out<<",\"wire\":";wire(out,a);out<<'}';
}
void joint(std::ostream &out,const gz::msgs::Joint &j) {
  out<<"{\"header\":";header(out,j.has_header(),j.header());out<<",\"name\":";text(out,j.name());
  out<<",\"id\":"<<j.id()<<",\"kind\":"<<j.type()<<",\"parent\":";text(out,j.parent());
  out<<",\"parent_id\":"<<j.parent_id()<<",\"child\":";text(out,j.child());out<<",\"child_id\":"<<j.child_id()<<",\"pose\":";
  pose(out,j.has_pose(),j.pose());out<<",\"axis1\":";axis(out,j.has_axis1(),j.axis1());out<<",\"axis2\":";axis(out,j.has_axis2(),j.axis2());
  out<<",\"wire\":";wire(out,j);out<<'}';
}
struct JointConfig {
  Entity entity;std::string name,parent,child;gz::math::Pose3d pose;sdf::JointAxis axis;bool selected;
};
struct Packet { unsigned call;std::uint64_t ordinal;std::string message; };
struct Received {
  std::mutex mutex;std::condition_variable changed;std::atomic<bool> active{true};
  bool armed=false,bad=false;unsigned call=0;std::uint64_t total=0;long long ns=0;
  std::vector<std::array<double,2>> input;
  std::vector<Packet> packets;std::string failure;
};
struct Owner {
  unsigned index;Entity model,world;std::string worldName,name,topic,requestedTopic,xml;
  gz::math::Pose3d pose;double rate;std::vector<std::string> requestedNames;std::vector<JointConfig> joints;
  std::unique_ptr<gz::sim::System> system;gz::sim::ISystemPostUpdate *post;
  std::shared_ptr<Received> received;
};
void validatePose(const gz::msgs::Pose &p,const gz::math::Pose3d &input) {
  require(p.has_position() && p.has_orientation(),"complete copied pose");const auto v=values(input);
  const double actual[]={p.position().x(),p.position().y(),p.position().z(),p.orientation().x(),p.orientation().y(),p.orientation().z(),p.orientation().w()};
  for(unsigned i=0;i<7;++i)require(word(actual[i])==word(v[i]),"pose getter-to-message bits");
}
std::string message(const gz::msgs::Model &m,const Owner &owner,const Received &state) {
  require(m.ByteSizeLong()<=65536 && m.has_header() && m.header().has_stamp(),"bounded complete actual Model");
  const auto &stamp=m.header().stamp();
  require(stamp.sec()>=0 && stamp.nsec()>=0 && stamp.nsec()<1000000000 && stamp.sec()<=(LLONG_MAX-stamp.nsec())/1000000000LL,"native stamp domain");
  require(stamp.sec()*1000000000LL+stamp.nsec()==state.ns && m.id()==owner.model && m.name()==owner.name,"actual owner/time fence");
  require(m.has_pose(),"actual Model LOCAL pose");validatePose(m.pose(),owner.pose);
  std::vector<const JointConfig*> selected;
  for(const auto &j:owner.joints)if(j.selected)selected.push_back(&j);
  require(m.joint_size()==static_cast<int>(selected.size()) && state.input.size()==owner.joints.size(),"complete selected input cardinality");
  Entity previous=0;
  for(int i=0;i<m.joint_size();++i){
    const auto &j=m.joint(i);const auto &cfg=*selected[static_cast<unsigned>(i)];
    const auto at=static_cast<std::size_t>(&cfg-owner.joints.data());
    require(j.id()>previous && j.id()==cfg.entity && j.name()==cfg.name && j.parent()==cfg.parent && j.child()==cfg.child,"untouched native ID order/metadata");previous=j.id();
    require(j.has_pose() && j.has_axis1() && !j.has_axis2(),"single-axis message presence");validatePose(j.pose(),cfg.pose);
    const auto &a=j.axis1();
    require(a.has_xyz() && word(a.xyz().x())==word(cfg.axis.Xyz().X()) && word(a.xyz().y())==word(cfg.axis.Xyz().Y()) && word(a.xyz().z())==word(cfg.axis.Xyz().Z()),"axis getter-to-message bits");
    require(word(a.limit_lower())==word(cfg.axis.Lower()) && word(a.limit_upper())==word(cfg.axis.Upper()) && word(a.damping())==word(cfg.axis.Damping()),"limit/damping getter-to-message bits");
    require(word(a.position())==word(state.input[at][0]) && word(a.velocity())==word(state.input[at][1]),"actual callback q/v input fence");
    require(a.force()==0,"empty input force is unmeasured, not a zero-force sample");
  }
  std::ostringstream out;
  out<<"{\"header\":";header(out,m.has_header(),m.header());out<<",\"name\":";text(out,m.name());
  out<<",\"id\":"<<m.id()<<",\"is_static\":"<<(m.is_static()?"true":"false")<<",\"deleted\":"<<(m.deleted()?"true":"false")
     <<",\"self_collide\":"<<(m.self_collide()?"true":"false")<<",\"pose\":";pose(out,m.has_pose(),m.pose());out<<",\"joints\":[";
  for(int i=0;i<m.joint_size();++i){if(i)out<<',';joint(out,m.joint(i));}
  out<<"],\"wire\":";wire(out,m);out<<'}';return out.str();
}
struct Readiness { bool ok;unsigned publishers,subscribers; };
Readiness ready(gz::transport::Node &node,const std::string &topic,bool initial) {
  const auto deadline=std::chrono::steady_clock::now()+5s;
  for(;;){
    std::vector<gz::transport::MessagePublisher> p,s;const bool ok=node.TopicInfo(topic,p,s);
    if(initial)return {ok,static_cast<unsigned>(p.size()),static_cast<unsigned>(s.size())};
    if(ok && p.size()==1 && s.size()==1 && p[0].MsgTypeName()=="gz.msgs.Model" && s[0].MsgTypeName()=="gz.msgs.Model")return {true,1,1};
    require(std::chrono::steady_clock::now()<deadline,"typed established publisher/subscriber readiness");std::this_thread::yield();
  }
}
void maps(const std::string &path) {
  std::ifstream input("/proc/self/maps");require(bool(input),"maps open");std::string data,line;
  while(std::getline(input,line)){
    require(line.find("joint-state-publisher-system")==std::string::npos,"original System DSO must never be mapped");
    require(data.size()+line.size()+1<=4194304,"maps cap");data+=line+'\n';
  }
  require(input.eof(),"complete maps input");const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);require(fd>=0,"exclusive owned maps");
  std::size_t at=0;
  while(at<data.size()){const auto n=::write(fd,data.data()+at,data.size()-at);if(n<=0){::close(fd);throw std::runtime_error("maps write");}at+=static_cast<std::size_t>(n);}
  const bool flushed=::fsync(fd)==0,closed=::close(fd)==0;require(flushed&&closed,"durable maps");
}
void emit(const std::string &row,unsigned &rows) {
  require(row.size()<=524288,"expanded owned row cap");std::cout<<row<<'\n';std::cout.flush();require(bool(std::cout),"complete owned stdout");++rows;
}
void emitConfiguration(const Owner &o,unsigned &rows) {
  std::ostringstream out;
  out<<"{\"kind\":\"configuration\",\"owner\":"<<o.index<<",\"world_entity\":"<<o.world<<",\"model_entity\":"<<o.model<<",\"world_name\":";text(out,o.worldName);
  out<<",\"model_name\":";text(out,o.name);out<<",\"full_name\":";text(out,o.worldName+"::"+o.name);
  out<<",\"requested_topic\":";text(out,o.requestedTopic);out<<",\"resolved_topic\":";text(out,o.topic);out<<",\"rate\":";finite(out,o.rate);
  out<<",\"xml\":";text(out,o.xml);out<<",\"model_pose\":";poseWords(out,o.pose);out<<",\"requested_names\":[";
  for(unsigned i=0;i<o.requestedNames.size();++i){if(i)out<<',';text(out,o.requestedNames[i]);}
  out<<"],\"joints\":[";
  for(unsigned i=0;i<o.joints.size();++i){if(i)out<<',';const auto &j=o.joints[i];
    out<<"{\"entity\":"<<j.entity<<",\"parent_entity\":"<<o.model<<",\"name\":";text(out,j.name);out<<",\"parent_link\":";text(out,j.parent);
    out<<",\"child_link\":";text(out,j.child);out<<",\"local_pose\":";poseWords(out,j.pose);out<<",\"axis\":[";
    finite(out,j.axis.Xyz().X());out<<',';finite(out,j.axis.Xyz().Y());out<<',';finite(out,j.axis.Xyz().Z());out<<"],\"lower\":";limit(out,j.axis.Lower(),true);
    out<<",\"upper\":";limit(out,j.axis.Upper(),false);out<<",\"damping\":";finite(out,j.axis.Damping());out<<",\"selected\":"<<(j.selected?"true":"false")<<'}';
  }
  out<<"]}";emit(out.str(),rows);
}
struct Call { long long ns,dt,real;bool paused; };
const std::array<Call,10> calls={{{0,0,0,true},{1000000,1000000,1000000,false},{10000000,9000000,10000000,false},
 {10000000,0,10000000,true},{11000000,1000000,11000000,false},{20000000,9000000,20000000,false},
 {20000000,0,20000000,true},{15000000,0,20000000,false},{30000000,15000000,35000000,false},{30000000,0,35000000,true}}};
bool wanted(double rate,unsigned call) {
  if(rate==0)return true;
  if(rate==100)return call==0||call==2||call==3||call==5||call==6||call==7||call==8||call==9;
  require(rate==50,"closed Source input rate");return call==0||call==5||call==6||call==7;
}
void input(Owner &o,gz::sim::EntityComponentManager &ecm,unsigned call,unsigned &rows) {
  const auto *mp=ecm.Component<c::Pose>(o.model);const auto *parent=ecm.Component<c::ParentEntity>(o.model);
  require(mp&&parent&&parent->Data()==o.world&&ecm.ParentEntity(o.model)==o.world,"fixed model graph/getter");samePose(mp->Data(),o.pose);
  std::ostringstream out;
  out<<"{\"kind\":\"input\",\"owner\":"<<o.index<<",\"call_index\":"<<call<<",\"sim_ns\":"<<calls[call].ns
     <<",\"dt_ns\":"<<calls[call].dt<<",\"real_ns\":"<<calls[call].real<<",\"iterations\":"<<call<<",\"paused\":"<<(calls[call].paused?"true":"false")<<",\"model_pose\":";
  poseWords(out,o.pose);out<<",\"model_pose_getback\":";poseWords(out,mp->Data());out<<",\"joints\":[";
  std::vector<std::array<double,2>> actual;
  for(unsigned j=0;j<o.joints.size();++j){
    const auto &cfg=o.joints[j];const std::vector<double> q={.125*(o.index+1)+.0625*j+.0078125*call};
    const std::vector<double> v={-.25*(o.index+1)+.03125*j-.00390625*call};const std::vector<double> force;
    (void)ecm.SetComponentData<c::JointPosition>(cfg.entity,q);(void)ecm.SetComponentData<c::JointVelocity>(cfg.entity,v);(void)ecm.SetComponentData<c::JointForce>(cfg.entity,force);
    const auto *qp=ecm.Component<c::JointPosition>(cfg.entity);const auto *vp=ecm.Component<c::JointVelocity>(cfg.entity);const auto *fp=ecm.Component<c::JointForce>(cfg.entity);
    const auto *jp=ecm.Component<c::Pose>(cfg.entity);const auto *pp=ecm.Component<c::ParentEntity>(cfg.entity);
    require(qp&&vp&&fp&&jp&&pp&&pp->Data()==o.model&&ecm.ParentEntity(cfg.entity)==o.model,"actual selected/unselected joint inputs/graph");
    require(qp->Data().size()==1&&vp->Data().size()==1&&fp->Data().empty()&&word(qp->Data()[0])==word(q[0])&&word(vp->Data()[0])==word(v[0]),"setter/getback q/v bits and empty unmeasured force");samePose(jp->Data(),cfg.pose);
    actual.push_back({qp->Data()[0],vp->Data()[0]});
    if(j)out<<',';out<<"{\"entity\":"<<cfg.entity<<",\"parent_entity\":"<<pp->Data()<<",\"local_pose\":";poseWords(out,cfg.pose);
    out<<",\"local_pose_getback\":";poseWords(out,jp->Data());out<<",\"position\":";array(out,q);out<<",\"position_getback\":";array(out,qp->Data());
    out<<",\"velocity\":";array(out,v);out<<",\"velocity_getback\":";array(out,vp->Data());out<<",\"force\":";array(out,force);out<<",\"force_getback\":";array(out,fp->Data());out<<'}';
  }
  out<<"]}";emit(out.str(),rows);
  std::lock_guard<std::mutex> lock(o.received->mutex);require(!o.received->armed&&o.received->packets.empty()&&!o.received->bad,"previous call fully fenced");o.received->input=std::move(actual);
}
void run(const std::string &mode,const std::filesystem::path &mapPath) {
  const unsigned count=mode=="pair"?2:8;const std::string worldName="m57_"+mode+"_"+mapPath.parent_path().filename().string();
  require(worldName.size()<=200,"owned unique world name");
  require(gz::common::Console::Init(mapPath.parent_path().string(),mapPath.filename().string()+".console.log"),"owned Console Init");
  require(gz::common::Console::Directory()==mapPath.parent_path().string(),"owned Console directory");gz::common::Console::SetVerbosity(2);require(gz::common::Console::Verbosity()==2,"warning-visible Console2");
  gz::sim::EntityComponentManager ecm;gz::sim::EventManager events;gz::transport::Node node;
  const Entity world=ecm.CreateEntity();ecm.CreateComponent(world,c::World());ecm.CreateComponent(world,c::Name(worldName));ecm.CreateComponent(world,c::Pose(gz::math::Pose3d::Zero));
  std::vector<Owner> owners;bool mapsAttempted=false;unsigned rows=0,inputs=0,postCalls=0,fences=0,observations=0,jointRecords=0;
  const auto captureMaps=[&]{if(!mapsAttempted){mapsAttempted=true;maps(mapPath.string());}};
  const auto deactivate=[&](bool strict){
    for(auto &o:owners){
      std::lock_guard<std::mutex> lock(o.received->mutex);
      if(strict)require(!o.received->bad&&!o.received->armed&&o.received->packets.empty(),"complete final callback fence");
      o.received->active.store(false,std::memory_order_release);
    }
    for(const auto &o:owners){const bool removed=node.Unsubscribe(o.topic);if(strict)require(removed,"owned subscriber teardown");}
  };
  try {
    for(unsigned i=0;i<count;++i){
      const std::string name="arm"+std::to_string(i);const Entity model=ecm.CreateEntity();require(model>0&&model<=UINT32_MAX,"nonzero native u32 model identity");
      const gz::math::Pose3d fixed(1+i,4*i,1,0,0,0);ecm.CreateComponent(model,c::Model());ecm.CreateComponent(model,c::Name(name));ecm.CreateComponent(model,c::Pose(fixed));require(ecm.SetParentEntity(model,world),"model graph parent");
      const double rate=mode=="pair"&&i==0?0:(i%2==0?100:50);
      // Pair's second owner uses100Hz, while eight alternates100/50Hz.
      const double configuredRate=mode=="pair"&&i==1?100:rate;
      const std::string requested=i==0?"":"/m57/native/"+mapPath.parent_path().filename().string()+"/"+mode+"/owner"+std::to_string(i);
      const std::string topic=requested.empty()?"/world/"+worldName+"/model/"+name+"/joint_state":requested;
      Owner o{i,model,world,worldName,name,topic,requested,"",fixed,configuredRate,{}, {},nullptr,nullptr,std::make_shared<Received>()};
      for(unsigned j=0;j<2;++j){
        const Entity id=ecm.CreateEntity();require(id>0&&id<=UINT32_MAX,"nonzero native u32 joint identity");
        const std::string jname=j==0?"base_hinge":"tip_hinge",parentName=j==0?"world":"base",childName=j==0?"base":"tip";
        sdf::JointAxis axis;require(axis.SetXyz(gz::math::Vector3d(0,0,1)).empty(),"public joint axis input");
        axis.SetLower(j==0?-2.0:-INFINITY);axis.SetUpper(j==0?2.0:INFINITY);axis.SetDamping(j==0?.125:0);
        const gz::math::Pose3d jointPose=gz::math::Pose3d::Zero;
        ecm.CreateComponent(id,c::Joint());ecm.CreateComponent(id,c::Name(jname));ecm.CreateComponent(id,c::Pose(jointPose));
        ecm.CreateComponent(id,c::JointAxis(axis));ecm.CreateComponent(id,c::ParentLinkName(parentName));ecm.CreateComponent(id,c::ChildLinkName(childName));
        ecm.CreateComponent(id,c::JointPosition(std::vector<double>{0}));ecm.CreateComponent(id,c::JointVelocity(std::vector<double>{0}));ecm.CreateComponent(id,c::JointForce(std::vector<double>{}));require(ecm.SetParentEntity(id,model),"joint graph parent");
        o.joints.push_back({id,jname,parentName,childName,jointPose,axis,i%2==0||j==1});
      }
      if(i%2!=0)o.requestedNames={"tip_hinge"};
      std::ostringstream content;if(!requested.empty())content<<"<topic>"<<requested<<"</topic>";content<<"<update_rate>"<<configuredRate<<"</update_rate>";
      for(const auto &jointName:o.requestedNames)content<<"<joint_name>"<<jointName<<"</joint_name>";
      sdf::Plugin plugin("gz-sim-joint-state-publisher-system","gz::sim::systems::JointStatePublisher",content.str());const auto parsed=plugin.ToElement();
      require(bool(parsed)&&parsed->Get<double>("update_rate")==configuredRate,"genuine nonnull parsed plugin input");o.xml=parsed->ToString("");
      o.system=std::unique_ptr<gz::sim::System>(new gz::sim::systems::JointStatePublisher());
      auto *configure=dynamic_cast<gz::sim::ISystemConfigure*>(o.system.get());o.post=dynamic_cast<gz::sim::ISystemPostUpdate*>(o.system.get());require(configure&&o.post,"checked original public Configure/PostUpdate interfaces only");
      configure->Configure(model,parsed,ecm,events);
      owners.push_back(std::move(o));
      // Build every owner before any callback captures its stable address.
    }
    for(const auto &o:owners){
      const auto state=o.received;
      // Stable owner address is borrowed only while active; no System ownership is captured.
      const Owner *owner=&o;
      std::function<void(const gz::msgs::Model&)> callback=[state,owner](const gz::msgs::Model &m){
        std::lock_guard<std::mutex> lock(state->mutex);if(!state->active.load(std::memory_order_acquire)||state->bad)return;
        try{require(state->armed&&state->packets.size()<3&&state->total<16,"bounded armed callback before copy");require(m.ByteSizeLong()<=65536,"Model cap before owned copy");
          const auto data=message(m,*owner,*state);require(data.size()<=524288,"expanded owned message cap");state->packets.push_back({state->call,state->total++,data});
        }catch(const std::exception &e){state->bad=true;state->failure=e.what();}state->changed.notify_all();
      };
      require(node.Subscribe(o.topic,callback),"actual typed owner subscriber");emitConfiguration(o,rows);
    }
    for(unsigned call=0;call<calls.size();++call){
      // ALL component inputs and getbacks precede ANY original call in the group.
      for(auto &o:owners){input(o,ecm,call,rows);++inputs;}
      for(auto &o:owners){
        const auto discovery=ready(node,o.topic,call==0);const unsigned expected=wanted(o.rate,call)?1:0;
        std::uint64_t before;
        {std::lock_guard<std::mutex> lock(o.received->mutex);require(!o.received->bad&&!o.received->armed&&o.received->packets.empty(),"empty previous callback fence");before=o.received->total;o.received->call=call;o.received->ns=calls[call].ns;o.received->armed=true;}
        gz::sim::UpdateInfo info;info.simTime=std::chrono::nanoseconds(calls[call].ns);info.realTime=std::chrono::nanoseconds(calls[call].real);info.dt=std::chrono::nanoseconds(calls[call].dt);info.iterations=call;info.paused=calls[call].paused;
        o.post->PostUpdate(info,ecm);++postCalls;
        std::unique_lock<std::mutex> lock(o.received->mutex);
        if(expected)require(o.received->changed.wait_for(lock,5s,[&]{return o.received->bad||o.received->packets.size()>=expected;}),"actual positive callback deadline");
        else (void)o.received->changed.wait_for(lock,25ms,[&]{return o.received->bad||!o.received->packets.empty();});
        require(!o.received->bad&&o.received->packets.size()==expected&&o.received->total-before==expected,"exact per-owner call fence: "+o.received->failure);
        for(const auto &packet:o.received->packets){require(packet.call==call,"actual call identity");std::ostringstream out;out<<"{\"kind\":\"observed\",\"owner\":"<<o.index<<",\"call_index\":"<<call<<",\"arrival_ordinal\":"<<packet.ordinal<<",\"message\":"<<packet.message<<'}';emit(out.str(),rows);++observations;for(const auto &cfg:o.joints)if(cfg.selected)++jointRecords;}
        std::ostringstream fence;fence<<"{\"kind\":\"fence\",\"owner\":"<<o.index<<",\"call_index\":"<<call<<",\"initial_lazy_publication\":"<<(call==0?"true":"false")<<",\"topic_info_ok\":"<<(discovery.ok?"true":"false")<<",\"publishers_before\":"<<discovery.publishers<<",\"subscribers_before\":"<<discovery.subscribers<<",\"arrivals_before\":"<<before<<",\"arrivals_after\":"<<o.received->total<<",\"expected_outputs\":"<<expected<<'}';emit(fence.str(),rows);++fences;o.received->packets.clear();o.received->armed=false;
      }
      if(call==0)captureMaps();ecm.ClearNewlyCreatedEntities();
    }
    const unsigned wantedModels=mode=="pair"?18:48,wantedJoints=mode=="pair"?28:80;
    require(inputs==count*10&&postCalls==inputs&&fences==inputs&&observations==wantedModels&&jointRecords==wantedJoints,"complete Source-defined mode ledger");
    deactivate(true);captureMaps();owners.clear();
    const unsigned rawRows=count+inputs+observations+fences+1;require(rows+1==rawRows,"complete raw row ledger");
    std::ostringstream summary;summary<<"{\"kind\":\"summary\",\"owners\":"<<count<<",\"configurations\":"<<count<<",\"global_call_groups\":10,\"input_rows\":"<<inputs<<",\"direct_post_update_calls\":"<<postCalls<<",\"fence_rows\":"<<fences<<",\"observed_models\":"<<observations<<",\"observed_joints\":"<<jointRecords<<",\"raw_rows\":"<<rawRows<<'}';emit(summary.str(),rows);
  }catch(...){
    const auto original=std::current_exception();
    try{deactivate(false);}catch(const std::exception &e){std::cerr<<"subscriber cleanup failure: "<<e.what()<<'\n';}
    try{captureMaps();}catch(const std::exception &e){std::cerr<<"maps retention failure: "<<e.what()<<'\n';}
    std::rethrow_exception(original);
  }
}
}
int main(int argc,char **argv) {
  try{require(argc==3,"expected MODE MAPS_PATH");const std::string mode=argv[1];require(mode=="pair"||mode=="eight","closed mode");const std::filesystem::path path(argv[2]);require(path.is_absolute(),"owned absolute maps path");run(mode,path);return 0;}
  catch(const std::exception &e){std::cerr<<"original direct-object multiple JointStatePublishers: "<<e.what()<<'\n';return 1;}
}
