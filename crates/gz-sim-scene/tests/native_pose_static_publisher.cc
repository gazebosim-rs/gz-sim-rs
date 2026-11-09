// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Pinned gz-sim71c60db split PosePublisher: independent topics and keyed local poses.
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <tuple>
#include <vector>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocityReset.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
using namespace std::chrono_literals;
using Message=gz::msgs::Pose_V;
struct Received { int phase; bool fixed; Message message; };
struct Tick {int phase;long long time;bool paused;std::map<std::string,gz::math::Pose3d> poses;};
class Observe final:public gz::sim::System,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate,public gz::sim::ISystemReset
{
  public: std::atomic<int> phase{0};std::vector<Tick> ticks;std::string failure;unsigned resets=0;
  public: long long resetTimeNs=-1;bool resetPaused=false;bool jointless=false;bool fixed=false;bool velocitySet=false;
  public: void Reset(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &) override {
    ++resets;resetTimeNs=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();resetPaused=info.paused;
  }
  public: void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override
  {
    if(!failure.empty()||info.paused)return;
    try {
      if(jointless&&!velocitySet){
        ecm.Each<gz::sim::components::Link>([&](const auto &id,const auto *){
          ecm.CreateComponent(id,gz::sim::components::WorldLinearVelocityReset({1,0,0}));return true;
        });velocitySet=true;
      }
      if(!fixed)ecm.Each<gz::sim::components::Joint,gz::sim::components::Name>([&](const auto &id,const auto *,const auto *name){
        const double effort=name->Data()=="base_hinge"?1.0:.5;
        auto *cmd=ecm.Component<gz::sim::components::JointForceCmd>(id);
        if(cmd)cmd->Data()={effort};else ecm.CreateComponent(id,gz::sim::components::JointForceCmd({effort}));
        return true;
      });
    } catch(const std::exception &e){failure=std::string("PreUpdate: ")+e.what();}
    catch(...){failure="unknown PreUpdate failure";}
  }
  public: void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override
  {
    if(!failure.empty())return;
    try {
      const auto model=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("rig"));
      const auto *pose=ecm.Component<gz::sim::components::Pose>(model);
      if(!pose)throw std::runtime_error("missing rig local Pose");
      Tick tick{phase.load(),std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count(),info.paused,{{"rig",pose->Data()}}};
      ecm.Each<gz::sim::components::Link,gz::sim::components::Name,gz::sim::components::ParentEntity,gz::sim::components::Pose>(
        [&](const auto &,const auto *,const auto *name,const auto *parent,const auto *local){
          if(parent->Data()==model && !tick.poses.emplace("rig::"+name->Data(),local->Data()).second)
            throw std::runtime_error("duplicate native child-frame name");
          return true;
        });
      if(tick.poses.size()!=3 || !tick.poses.count("rig::base") || !tick.poses.count("rig::tip"))
        throw std::runtime_error("exact flat-model component membership");
      ticks.push_back(std::move(tick));if(ticks.size()>256)throw std::runtime_error("native tick cap");
    } catch(const std::exception &e){failure=std::string("PostUpdate: ")+e.what();}
    catch(...){failure="unknown PostUpdate failure";}
  }
};
void maps(const std::string &path){std::ifstream in("/proc/self/maps");std::ofstream out(path);if(!in||!out)throw std::runtime_error("maps open");out<<in.rdbuf();out.flush();if(!out)throw std::runtime_error("maps write");}
void emitPose(const gz::math::Pose3d &p){std::cout<<"\"position\":["<<p.Pos().X()<<','<<p.Pos().Y()<<','<<p.Pos().Z()<<"],\"rotation\":["<<p.Rot().X()<<','<<p.Rot().Y()<<','<<p.Rot().Z()<<','<<p.Rot().W()<<']';}
void run(const std::string &mode,const std::string &mapsPath)
{
  std::string xml;char ch;while(std::cin.get(ch)){if(xml.size()>=256*1024)throw std::runtime_error("fixture cap");xml+=ch;}
  const std::string original="<use_pose_vector_msg>true</use_pose_vector_msg><publish_model_pose>true</publish_model_pose><topic>/m31/pose</topic><update_frequency>100</update_frequency><static_publisher>true</static_publisher><static_update_frequency>25</static_update_frequency>";
  const auto where=xml.find(original);if(where==std::string::npos||xml.find(original,where+1)!=std::string::npos)throw std::runtime_error("exact fixture configuration");
  const bool rate=mode=="rate"||mode=="paused"||mode=="rewind"||mode=="inherit";
  if(!rate&&mode!="split"&&mode!="jointless"&&mode!="fixed")throw std::runtime_error("mode refused");
  std::string replacement="<use_pose_vector_msg>true</use_pose_vector_msg><publish_model_pose>true</publish_model_pose><topic>/m31/pose</topic><static_publisher>true</static_publisher>";
  if(rate)replacement+="<update_frequency>100</update_frequency>";
  if(rate&&mode!="inherit")replacement+="<static_update_frequency>25</static_update_frequency>";
  xml.replace(where,original.size(),replacement);
  if(mode=="jointless"){
    const auto first=xml.find("    <joint "),last=xml.find("    <plugin ");
    if(first==std::string::npos||last==std::string::npos||first>=last)throw std::runtime_error("jointless fixture selection");
    xml.erase(first,last-first);
  }
  if(mode=="fixed"){
    const std::string from="type=\"revolute\"",to="type=\"fixed\"";
    std::size_t pos=0;unsigned changed=0;while((pos=xml.find(from,pos))!=std::string::npos){xml.replace(pos,from.size(),to);pos+=to.size();++changed;}
    if(changed!=2)throw std::runtime_error("fixed fixture selection");
  }
  const std::string topic="/m31/pose";
  std::mutex mutex;std::condition_variable available;std::vector<Received> received;bool overflow=false;
  std::atomic<int> phase{0};int fenceSeen=0,fenceSent=0;
  gz::transport::Node node;
  for(bool fixed:{false,true})if(!node.Subscribe(topic+(fixed?"_static":""),std::function<void(const Message &)>([&,fixed](const Message &message){
    std::lock_guard<std::mutex> lock(mutex);if(received.size()>=512)overflow=true;else received.push_back({phase.load(),fixed,message});available.notify_all();
  })))throw std::runtime_error("canonical split subscriber");
  if(!node.Subscribe("/m31/capture_fence",std::function<void(const gz::msgs::Int32 &)>([&](const auto &message){
    std::lock_guard<std::mutex> lock(mutex);fenceSeen=message.data();available.notify_all();
  })))throw std::runtime_error("fence subscriber");
  auto fence=node.Advertise<gz::msgs::Int32>("/m31/capture_fence");if(!fence)throw std::runtime_error("fence publisher");
  const auto discovered=[&]{
    const auto deadline=std::chrono::steady_clock::now()+5s;
    while(std::chrono::steady_clock::now()<deadline){
      bool ready=true;
      for(bool fixed:{false,true}){
        std::vector<gz::transport::MessagePublisher> publishers,subscribers;
        node.TopicInfo(topic+(fixed?"_static":""),publishers,subscribers);
        ready=ready&&publishers.size()==1&&subscribers.size()==1
          &&publishers[0].MsgTypeName()=="gz.msgs.Pose_V"&&subscribers[0].MsgTypeName()=="gz.msgs.Pose_V";
      }
      if(ready)return;
      std::this_thread::yield();
    }
    throw std::runtime_error("current Pose_V publisher/subscriber discovery deadline");
  };
  const auto drain=[&](std::size_t expectedCount){
    gz::msgs::Int32 m;m.set_data(++fenceSent);
    if(!fence.Publish(m))throw std::runtime_error("fence publish");
    std::unique_lock<std::mutex> lock(mutex);
    // The self-fence alone cannot complete a different publisher's callbacks.
    // Keep the same five-second wait, requiring actual Pose_V arrival too.
    if(!available.wait_for(lock,5s,[&]{return overflow ||
          (fenceSeen==fenceSent&&received.size()>=expectedCount);}))
      throw std::runtime_error("Pose_V callback completion/FIFO drain deadline phase="+std::to_string(phase.load())
          +" expected="+std::to_string(expectedCount)+" received="+std::to_string(received.size())
          +" fence_sent="+std::to_string(fenceSent)+" fence_seen="+std::to_string(fenceSeen));
    if(overflow||received.size()!=expectedCount)
      throw std::runtime_error("unexpected actual Pose_V callback count at phase boundary");
  };
  gz::sim::ServerConfig config;if(!config.SetSdfString(xml))throw std::runtime_error("SDF");
  config.SetPhysicsEngine("gz-physics-dartsim-plugin");config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default","world",sdf::Plugin("gz-sim-physics-system","gz::sim::systems::Physics")));
  gz::sim::Server server(config);auto observe=std::make_shared<Observe>();observe->jointless=mode=="jointless";observe->fixed=mode=="fixed";server.AddSystem(observe);
  bool mapsAttempted=false;const auto capture=[&]{if(!mapsAttempted){mapsAttempted=true;maps(mapsPath);}};
  try {
    if(!server.RunOnce(true))throw std::runtime_error("paused discovery");discovered();drain(0);
    {std::lock_guard<std::mutex> lock(mutex);if(!received.empty())throw std::runtime_error("paused-zero must not publish");}
    if(mode=="paused"){for(unsigned i=0;i<20;++i)if(!server.RunOnce(true))throw std::runtime_error("paused-only run");drain(0);
      std::lock_guard<std::mutex> lock(mutex);if(!received.empty())throw std::runtime_error("paused-only publication");}
    phase=1;observe->phase=1;
    const unsigned advancing=100u;
    const bool emptyDynamic=mode=="jointless"||mode=="fixed";
    const std::size_t dynamicCount=emptyDynamic?0u:(rate?10u:100u);
    const std::size_t staticCount=rate?(mode=="inherit"?10u:2u):100u;
    const std::size_t advancingCount=dynamicCount+staticCount;
    if(!server.Run(true,advancing,false))throw std::runtime_error("finite advancing run");drain(advancingCount);
    phase=2;observe->phase=2;if(!server.RunOnce(true)||!server.RunOnce(true))throw std::runtime_error("paused updates");drain(advancingCount);
    if(mode=="rewind") {
      phase=3;observe->phase=3;gz::msgs::WorldControl request;request.set_pause(true);request.mutable_reset()->set_time_only(true);
      gz::msgs::Boolean reply;bool accepted=false;
      if(!node.Request("/world/default/control",request,5000,reply,accepted)||!accepted||!reply.data())throw std::runtime_error("time-only rewind request");
      // ACK only queues WorldControl. Step processes it after the old-time
      // PostUpdate; the following paused step actually performs Reset. Keeping
      // both steps paused prevents a reset-only iteration consuming advancing Run iterations.
      if(!server.RunOnce(true))throw std::runtime_error("consume queued rewind control");
      if(!server.RunOnce(true))throw std::runtime_error("apply paused rewind");
      if(observe->resets!=1 || observe->resetTimeNs!=0 || !observe->resetPaused)
        throw std::runtime_error("paused reset witness count="+std::to_string(observe->resets)
            +" time_ns="+std::to_string(observe->resetTimeNs)+" paused="+(observe->resetPaused?"true":"false"));
      discovered();drain(advancingCount);
      phase=4;observe->phase=4;if(!server.Run(true,40,false))throw std::runtime_error("post-rewind finite run");drain(advancingCount+5);
    }
    if(!observe->failure.empty())throw std::runtime_error(observe->failure);
    std::vector<Received> messages;{std::lock_guard<std::mutex> lock(mutex);if(overflow)throw std::runtime_error("native callback cap");messages=received;}
    if(observe->resets!=(mode=="rewind"?1u:0u))throw std::runtime_error("native reset lifecycle witness");
    std::map<std::tuple<bool,int,long long>,const Tick *> expected;
    long long dynamicLast=0,staticLast=0;bool reloaded=false;
    const long long dynamicPeriod=rate?10000000LL:0LL;
    const long long staticPeriod=rate?(mode=="inherit"?10000000LL:40000000LL):0LL;
    for(const auto &tick:observe->ticks)if(!tick.paused){
      if(tick.phase==4&&!reloaded){dynamicLast=0;staticLast=0;reloaded=true;}
      for(bool fixed:{true,false}){
        auto &last=fixed?staticLast:dynamicLast;const auto period=fixed?staticPeriod:dynamicPeriod;
        const auto diff=tick.time-last;if(diff>0&&diff<period)continue;last=tick.time;
        if(!fixed&&emptyDynamic)continue;
        if(!expected.emplace(std::make_tuple(fixed,tick.phase,tick.time),&tick).second)throw std::runtime_error("duplicate expected topic/time");
      }
    }
    const std::size_t count=advancingCount+(mode=="rewind"?5u:0u);
    if(expected.size()!=count||messages.size()!=count)throw std::runtime_error("native exact split cadence/count or paused suppression");
    bool moving=false;const auto &initial=observe->ticks.front().poses.at(mode=="jointless"?"rig":"rig::tip");
    for(const auto &tick:observe->ticks)if(!tick.poses.at(mode=="jointless"?"rig":"rig::tip").Equal(initial,1e-12))moving=true;
    if(mode!="fixed"&&!moving)throw std::runtime_error("missing actual moving component");
    for(const auto &receivedMessage:messages){
      const auto &message=receivedMessage.message;
      if(message.pose_size()==0||message.has_header())throw std::runtime_error("empty vector or outer header");
      const auto &stamp=message.pose(0).header().stamp();
      const long long time=stamp.sec()*1000000000LL+stamp.nsec();
      const auto foundTick=expected.find({receivedMessage.fixed,receivedMessage.phase,time});
      if(foundTick==expected.end())throw std::runtime_error("duplicate/unexpected native topic/time");
      const auto &tick=*foundTick->second;
      if(receivedMessage.phase!=tick.phase)throw std::runtime_error("native phase boundary");
      std::map<std::string,gz::math::Pose3d> selected=tick.poses;
      if(!emptyDynamic){if(receivedMessage.fixed)selected.erase("rig::tip");else{selected.erase("rig");selected.erase("rig::base");}}
      if(message.pose_size()!=static_cast<int>(selected.size()))throw std::runtime_error("native exact split classification");
      std::cout<<"{\"mode\":\""<<mode<<"\",\"channel\":\""<<(receivedMessage.fixed?"static":"dynamic")<<"\",\"phase\":"<<tick.phase<<",\"time_ns\":"<<tick.time<<",\"records\":[";
      bool first=true;for(const auto &pose:message.pose()){
        const auto found=selected.find(pose.name());if(found==selected.end())throw std::runtime_error("duplicate/unknown native frame");
        const auto &header=pose.header();const auto parent=pose.name()=="rig"?"default":"rig";
        if(!pose.has_header()||!header.has_stamp()||header.stamp().sec()*1000000000LL+header.stamp().nsec()!=tick.time
          ||header.data_size()!=2||header.data(0).key()!="frame_id"||header.data(1).key()!="child_frame_id"
          ||header.data(0).value_size()!=1||header.data(1).value_size()!=1||header.data(0).value(0)!=parent||header.data(1).value(0)!=pose.name()
          ||pose.id()!=0||!pose.has_position()||!pose.has_orientation())throw std::runtime_error("native pose header/name/ID fields");
        const auto actual=gz::msgs::Convert(pose);if(!actual.IsFinite()||!actual.Equal(found->second,1e-12))throw std::runtime_error("local packet/component mismatch");
        if(!first)std::cout<<',';first=false;std::cout<<"{\"name\":\""<<pose.name()<<"\",\"parent\":\""<<parent<<"\",\"id\":0,";emitPose(actual);std::cout<<'}';selected.erase(found);
      }
      if(!selected.empty())throw std::runtime_error("missing native frame");std::cout<<"]}\n";expected.erase(foundTick);
    }
    if(!expected.empty())throw std::runtime_error("missing native channel frame");
    capture();server.Stop();
  } catch(...) {
    const auto originalError=std::current_exception();try{server.Stop();}catch(...){std::cerr<<"native stop failed\n";}
    std::cerr<<"native pose observer: "<<observe->failure<<"; phase="<<phase.load()
        <<"; ticks="<<observe->ticks.size()<<"; resets="<<observe->resets
        <<"; reset_time_ns="<<observe->resetTimeNs<<"; reset_paused="<<(observe->resetPaused?"true":"false")<<'\n';
    try{capture();}catch(const std::exception &e){std::cerr<<"live maps failure: "<<e.what()<<'\n';}catch(...){std::cerr<<"unknown maps failure\n";}
    std::rethrow_exception(originalError);
  }
}
int main(int argc,char **argv) try {if(argc!=3)return 2;std::cout<<std::setprecision(17);run(argv[1],argv[2]);}
catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}

catch(...){std::cerr<<"unknown native pose failure\n";return 1;}
