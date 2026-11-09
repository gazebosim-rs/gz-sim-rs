// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Default: canonical input peer. Oracle: original packet-local system/ECM only.
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <utility>
using namespace std::chrono_literals;
namespace {
gz::msgs::Entity entity(std::uint64_t id) {
  gz::msgs::Entity value;value.set_id(id);value.set_type(gz::msgs::Entity::LINK);return value;
}
gz::msgs::EntityWrench wrench(std::uint64_t id,double force,double torque) {
  gz::msgs::EntityWrench value;*value.mutable_entity()=entity(id);
  value.mutable_wrench()->mutable_force()->set_x(force);
  value.mutable_wrench()->mutable_torque()->set_z(torque);return value;
}
void ready(const std::vector<gz::transport::Node::Publisher *> &publishers) {
  const auto deadline=std::chrono::steady_clock::now()+5s;
  for(;;) {
    bool connected=true;for(const auto *publisher:publishers) connected=connected && publisher->HasConnections();
    if(connected)return;
    if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("wrench subscriptions did not match");
    std::this_thread::yield();
  }
}
#ifndef GZ_APPLY_LINK_WRENCH_ORACLE
void next() {std::string command;if(!std::getline(std::cin,command) || command!="next")throw std::runtime_error("harness phase control");}
#endif
}
#ifndef GZ_APPLY_LINK_WRENCH_ORACLE
int main(int argc,char **argv) {
 try {
  if(argc!=2)throw std::runtime_error("expected explicit current public link ID");
  std::size_t used=0;const auto id=std::stoull(argv[1],&used);
  if(id==0 || used!=std::string(argv[1]).size() || std::string(argv[1]).find_first_not_of("0123456789")!=std::string::npos)throw std::runtime_error("invalid link ID");
  gz::transport::Node node;
  auto once=node.Advertise<gz::msgs::EntityWrench>("/world/default/wrench");
  auto persistent=node.Advertise<gz::msgs::EntityWrench>("/world/default/wrench/persistent");
  auto clear=node.Advertise<gz::msgs::Entity>("/world/default/wrench/clear");
  if(!once || !persistent || !clear)throw std::runtime_error("canonical publishers");
  ready({&once,&persistent,&clear});
  std::cout<<"READY wrench inputs\n"<<std::flush;next();
  if(!once.Publish(wrench(id,4,2)))throw std::runtime_error("once publish");
  std::cout<<"SENT once\n"<<std::flush;next();
  if(!persistent.Publish(wrench(id,1,2)) || !persistent.Publish(wrench(id,3,4)))throw std::runtime_error("persistent publish");
  std::cout<<"SENT persistent\n"<<std::flush;next();
  if(!clear.Publish(entity(id)))throw std::runtime_error("clear publish");
  std::cout<<"SENT clear\n"<<std::flush;next();
  std::cout<<"PASS canonical ApplyLinkWrench\n"<<std::flush;return 0;
 }catch(const std::exception &error){std::cerr<<"native wrench peer: "<<error.what()<<'\n';return 1;}
}
#else
#include <gz/math/Pose3.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <sdf/SDFImpl.hh>
#include <sdf/parser.hh>
#include <dlfcn.h>
#include <filesystem>
#include <iomanip>
#include <array>
#include <typeinfo>
#include <fstream>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <system_error>
namespace {
using Pair=std::array<double,2>;
std::string jsonString(const std::string &value) {
  std::string out="\"";for(char c:value){if(c=='\\' || c=='\"')out+='\\';out+=c;}return out+'\"';
}
void snapshotMaps(const std::string &path) {
  std::ifstream source("/proc/self/maps",std::ios::binary);
  std::vector<char> bytes(4*1024*1024+1);source.read(bytes.data(),static_cast<std::streamsize>(bytes.size()));
  const auto count=source.gcount();
  if(source.bad() || count<=0 || count>4*1024*1024)throw std::runtime_error("bounded actual maps read");
  const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC,0600);
  if(fd<0)throw std::system_error(errno,std::generic_category(),"exclusive maps output");
  std::size_t offset=0;
  while(offset<static_cast<std::size_t>(count)) {
    const auto written=::write(fd,bytes.data()+offset,static_cast<std::size_t>(count)-offset);
    if(written<0 && errno==EINTR)continue;
    if(written<=0){const int error=errno;::close(fd);throw std::system_error(error,std::generic_category(),"maps partial write");}
    offset+=static_cast<std::size_t>(written);
  }
  const int syncResult=::fsync(fd);const int syncError=errno;const int closeResult=::close(fd);
  if(syncResult!=0)throw std::system_error(syncError,std::generic_category(),"maps fsync");
  if(closeResult!=0)throw std::system_error(errno,std::generic_category(),"maps close");
}
struct Oracle {
  gz::sim::EntityComponentManager ecm;
  gz::sim::EventManager events;
  gz::sim::SystemLoader loader;
  gz::sim::SystemPluginPtr plugin;
  gz::sim::ISystemPreUpdate *update=nullptr;
  gz::sim::Entity world,a,b;
  std::string mode;
  std::size_t rows=0;
  std::int64_t timeNs=0;
  Oracle(const std::string &path,std::string selected):mode(std::move(selected)) {
    world=ecm.CreateEntity();ecm.CreateComponent(world,gz::sim::components::World());ecm.CreateComponent(world,gz::sim::components::Name("default"));
    auto model=ecm.CreateEntity();ecm.CreateComponent(model,gz::sim::components::Model());ecm.CreateComponent(model,gz::sim::components::Name("rig"));ecm.CreateComponent(model,gz::sim::components::ParentEntity(world));
    auto link=[&](const std::string &name){const auto id=ecm.CreateEntity();ecm.CreateComponent(id,gz::sim::components::Link());ecm.CreateComponent(id,gz::sim::components::Name(name));ecm.CreateComponent(id,gz::sim::components::ParentEntity(model));ecm.CreateComponent(id,gz::sim::components::Pose());ecm.CreateComponent(id,gz::sim::components::WorldPose());return id;};
    a=link("a");b=link("b");
    sdf::SDFPtr parsed(new sdf::SDF);sdf::init(parsed);
    const std::string xml="<sdf version='1.12'><world name='default'><plugin name='gz::sim::systems::ApplyLinkWrench' filename='unused'><verbose>false</verbose><topic>/m35/oracle/once</topic><topic_persistent>/m35/oracle/persistent</topic_persistent><topic_clear>/m35/oracle/clear</topic_clear></plugin></world></sdf>";
    if(!sdf::readString(xml,parsed))throw std::runtime_error("native plugin SDF");
    const auto element=parsed->Root()->GetElement("world")->GetElement("plugin");
    sdf::Plugin declaration;declaration.SetFilename(path);declaration.SetName("gz::sim::systems::ApplyLinkWrench");
    auto loaded=loader.LoadPlugin(declaration);if(!loaded)throw std::runtime_error("original system DSO");plugin=*loaded;
    auto *configure=plugin->QueryInterface<gz::sim::ISystemConfigure>();update=plugin->QueryInterface<gz::sim::ISystemPreUpdate>();
    if(!configure || !update)throw std::runtime_error("original system interfaces");
    Dl_info provider{};
    if(!dladdr(&typeid(*update),&provider) || !provider.dli_fname || std::filesystem::canonical(provider.dli_fname)!=std::filesystem::canonical(path))throw std::runtime_error("wrong original system provider");
    std::cout<<"{\"mode\":"<<jsonString(mode)<<",\"provider\":"<<jsonString(provider.dli_fname)<<"}\n";
    configure->Configure(world,element,ecm,events);
  }
  std::array<double,6> raw(gz::sim::Entity id) const {
    const auto *component=ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(id);
    if(!component)return {0,0,0,0,0,0};
    const auto &value=component->Data();
    return {value.force().x(),value.force().y(),value.force().z(),value.torque().x(),value.torque().y(),value.torque().z()};
  }
  Pair value(gz::sim::Entity id) const {const auto v=raw(id);return {v[0],v[5]};}
  std::array<Pair,2> tick(bool paused,std::int64_t dt=1000000) {
    if(++rows>10000)throw std::runtime_error("bounded oracle row cap");
    const auto effectiveDt=paused?0:dt;if(!paused)timeNs+=effectiveDt;
    gz::sim::UpdateInfo info;info.paused=paused;info.dt=std::chrono::nanoseconds(effectiveDt);info.simTime=std::chrono::nanoseconds(timeNs);
    update->PreUpdate(info,ecm);
    const std::array<Pair,2> result{value(a),value(b)};
    std::cout<<"{\"mode\":"<<jsonString(mode)<<",\"row\":"<<rows<<",\"paused\":"<<(paused?"true":"false")<<",\"dt_ns\":"<<effectiveDt<<",\"sim_time_ns\":"<<timeNs<<",\"target_ids\":["<<a<<','<<b<<"],\"wrenches\":[";
    bool firstTarget=true;
    for(const auto id:{a,b}){if(!firstTarget)std::cout<<',';firstTarget=false;const auto values=raw(id);std::cout<<'[';for(std::size_t i=0;i<values.size();++i){if(i)std::cout<<',';std::cout<<values[i];}std::cout<<']';}
    std::cout<<"]}\n";
    // Match the original Physics command-clear boundary without running Physics.
    for(const auto id:{a,b})if(auto *c=ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(id))c->Data().Clear();
    return result;
  }
  void await(Pair expected,bool zeroDt=false) {
    const auto deadline=std::chrono::steady_clock::now()+5s;
    do {if(tick(false,zeroDt?0:1000000)[0]==expected)return;std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<deadline);
    throw std::runtime_error("actual original wrench component deadline");
  }
};
}
int main(int argc,char **argv) {
 std::unique_ptr<Oracle> oracle;std::string maps;bool mapsWritten=false;
 try {
  if(argc!=4)throw std::runtime_error("oracle requires exact packet DSO, case and maps output");
  maps=argv[3];
  const std::string mode=argv[2];
  if(mode!="once" && mode!="paused-once" && mode!="persistent-add" && mode!="clear-paused" && mode!="independent" && mode!="zero-dt" && mode!="id-precedence" && mode!="name")throw std::runtime_error("unknown oracle mode");
  std::cout<<std::setprecision(17);oracle=std::make_unique<Oracle>(argv[1],mode);gz::transport::Node node;
  auto once=node.Advertise<gz::msgs::EntityWrench>("/m35/oracle/once");auto persistent=node.Advertise<gz::msgs::EntityWrench>("/m35/oracle/persistent");auto clear=node.Advertise<gz::msgs::Entity>("/m35/oracle/clear");
  if(!once || !persistent || !clear)throw std::runtime_error("oracle publishers");ready({&once,&persistent,&clear});
  if(mode=="persistent-add" || mode=="clear-paused") {
    if(!persistent.Publish(wrench(oracle->a,1,2)) || !persistent.Publish(wrench(oracle->a,3,4)))throw std::runtime_error("oracle persistent publish");
    oracle->await({4,6});if(oracle->tick(false)[0]!=Pair{4,6})throw std::runtime_error("persistent repeated addition");
    if(mode=="clear-paused") {
      if(!clear.Publish(entity(oracle->a)))throw std::runtime_error("oracle clear publish");
      for(int i=0;i<3;++i)if(oracle->tick(true)[0]!=Pair{0,0})throw std::runtime_error("paused force application");
      // Raw settling rows retain callback timing; no exact arrival-time claim.
      oracle->await({0,0});if(oracle->tick(false)[0]!=Pair{0,0})throw std::runtime_error("persistent clear continuation");
    }
  } else if(mode=="independent") {
    if(!once.Publish(wrench(oracle->a,4,2)) || !once.Publish(wrench(oracle->b,6,3)))throw std::runtime_error("independent publish");
    bool a=false,b=false;const auto deadline=std::chrono::steady_clock::now()+5s;
    do {const auto values=oracle->tick(false);a=a || values[0]==Pair{4,2};b=b || values[1]==Pair{6,3};if(a && b)break;std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<deadline);
    if(!a || !b || oracle->tick(false)!=std::array<Pair,2>{Pair{0,0},Pair{0,0}})throw std::runtime_error("independent once completion");
  } else {
    auto message=wrench(oracle->a,4,2);if(mode=="id-precedence")message.mutable_entity()->set_name("missing::wrong");
    if(mode=="name"){message.mutable_entity()->set_id(0);message.mutable_entity()->set_name("default::rig::a");}
    if(!once.Publish(message))throw std::runtime_error("oracle once publish");
    if(mode=="paused-once")for(int i=0;i<3;++i)if(oracle->tick(true)[0]!=Pair{0,0})throw std::runtime_error("paused once applied");
    oracle->await({4,2},mode=="zero-dt");if(oracle->tick(false)[0]!=Pair{0,0})throw std::runtime_error("once repeated");
  }
  snapshotMaps(maps);mapsWritten=true;
  std::cout<<"{\"mode\":"<<jsonString(mode)<<",\"complete\":true,\"rows\":"<<oracle->rows<<",\"native_physics_executed\":false}\n";
  return 0;
 }catch(const std::exception &error){
   if(oracle && !maps.empty() && !mapsWritten){try{snapshotMaps(maps);}catch(const std::exception &capture){std::cerr<<"maps failure: "<<capture.what()<<'\n';}}
   std::cerr<<"native wrench oracle: "<<error.what()<<'\n';return 1;
 }
}
#endif
