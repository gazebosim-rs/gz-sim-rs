// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Contract: gz-sim71c60db3 JointStatePublisher.cc / joint_state_publisher_system.cc.
#include <algorithm>
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
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
using namespace std::chrono_literals;
using Message = gz::msgs::Model;
struct JointObservation
{
  gz::sim::Entity id;
  std::string name, parent, child;
  gz::math::Pose3d pose;
  sdf::JointAxis axis;
  double position, velocity;
};
struct Observation
{
  long long time;
  bool paused;
  gz::sim::Entity id;
  gz::math::Pose3d pose;
  std::vector<JointObservation> joints;
};
class Observe final : public gz::sim::System,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
  public: std::vector<Observation> rows;
  public: std::string failure;
  public: std::optional<Observation> warmup;
  public: void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (!failure.empty()) return;
    try
    {
      ecm.Each<gz::sim::components::Joint, gz::sim::components::Name>(
          [&](const gz::sim::Entity &entity, const auto *, const auto *name)
          {
            // Filtering the publisher does not request unselected measurements.
            // Empty components request actual Physics values for both observers;
            // never replace a present component or synthesize its data.
            gz::sim::Joint joint(entity);
            joint.EnablePositionCheck(ecm);
            joint.EnableVelocityCheck(ecm);
            if (info.paused) return true;
            const double effort = name->Data() == "base_hinge" ? 1.0 : .5;
            auto *command = ecm.Component<gz::sim::components::JointForceCmd>(entity);
            if (!command) ecm.CreateComponent(entity, gz::sim::components::JointForceCmd({effort}));
            else command->Data() = {effort};
            return true;
          });
    }
    catch (const std::exception &e) { failure = std::string("PreUpdate: ")+e.what(); }
    catch (...) { failure = "unknown PreUpdate failure"; }
  }
  public: void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    if (!failure.empty()) return;
    try
    {
      const auto model = ecm.EntityByComponents(gz::sim::components::Model(),
          gz::sim::components::Name("arm"));
      const auto *pose = ecm.Component<gz::sim::components::Pose>(model);
      if (!pose) throw std::runtime_error("missing model Pose");
      Observation row{std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count(),
          info.paused, model, pose->Data(), {}};
      // The single discovery tick can precede Physics-owned joint value arrays.
      if (info.paused && row.time == 0)
      {
        if (warmup) throw std::runtime_error("duplicate discovery tick");
        warmup = row;
        return;
      }
      ecm.Each<gz::sim::components::Joint, gz::sim::components::Name>(
          [&](const gz::sim::Entity &entity, const auto *, const auto *name)
          {
            const auto *p = ecm.Component<gz::sim::components::JointPosition>(entity);
            const auto *v = ecm.Component<gz::sim::components::JointVelocity>(entity);
            const auto *jp = ecm.Component<gz::sim::components::Pose>(entity);
            const auto *axis = ecm.Component<gz::sim::components::JointAxis>(entity);
            const auto *parent = ecm.Component<gz::sim::components::ParentLinkName>(entity);
            const auto *child = ecm.Component<gz::sim::components::ChildLinkName>(entity);
            if (!p || !v || p->Data().size()!=1 || v->Data().size()!=1 || !jp || !axis || !parent || !child)
              throw std::runtime_error("joint="+name->Data()+" entity="+std::to_string(entity)
                  +" time_ns="+std::to_string(row.time)
                  +" position_size="+(p ? std::to_string(p->Data().size()) : "missing")
                  +" velocity_size="+(v ? std::to_string(v->Data().size()) : "missing")
                  +" pose="+(jp ? "present" : "missing")+" axis="+(axis ? "present" : "missing")
                  +" parent="+(parent ? "present" : "missing")+" child="+(child ? "present" : "missing"));
            row.joints.push_back({entity,name->Data(),parent->Data(),child->Data(),jp->Data(),
                axis->Data(),p->Data()[0],v->Data()[0]});
            return true;
          });
      if (row.joints.size()!=2) throw std::runtime_error("native two-joint fixture selection failed");
      std::sort(row.joints.begin(),row.joints.end(),[](const auto &a,const auto &b){return a.id<b.id;});
      rows.push_back(std::move(row));
      if (rows.size()>256) throw std::runtime_error("native observation cap");
    }
    catch (const std::exception &e) { failure = std::string("PostUpdate: ")+e.what(); }
    catch (...) { failure = "unknown PostUpdate failure"; }
  }
};
void maps(const std::string &path)
{
  std::ifstream input("/proc/self/maps"); std::ofstream output(path);
  if (!input || !output) throw std::runtime_error("live maps open");
  output << input.rdbuf(); output.flush();
  if (!output) throw std::runtime_error("live maps write");
}
void poseEqual(const gz::msgs::Pose &message,const gz::math::Pose3d &expected)
{
  const auto actual=gz::msgs::Convert(message);
  if (!actual.IsFinite() || !expected.IsFinite() || !actual.Equal(expected,1e-12))
    throw std::runtime_error("native pose packet/component mismatch");
}
void emitPose(const gz::math::Pose3d &p)
{
  std::cout << '[' << p.Pos().X() << ',' << p.Pos().Y() << ',' << p.Pos().Z() << ','
      << p.Rot().X() << ',' << p.Rot().Y() << ',' << p.Rot().Z() << ',' << p.Rot().W() << ']';
}
void run(const std::string &mode,const std::string &mapsPath)
{
  std::string xml; char ch;
  while (std::cin.get(ch)) { if(xml.size()>=256*1024) throw std::runtime_error("fixture cap"); xml+=ch; }
  const std::string original="<topic>/m18/joint_state</topic><update_rate>100</update_rate>";
  const auto offset=xml.find(original);
  if(offset==std::string::npos || xml.find(original,offset+1)!=std::string::npos)
    throw std::runtime_error("exact native fixture configuration required");
  if(mode=="default") xml.replace(offset,original.size(),"");
  else if(mode=="filter") xml.replace(offset,original.size(),
      "<topic>/m18/joint_state</topic><joint_name>tip_hinge</joint_name><joint_name>tip_hinge</joint_name>");
  else if(mode!="rate") throw std::runtime_error("mode refused");
  std::mutex mutex; std::condition_variable ready; std::vector<Message> received;
  gz::transport::Node node;
  const std::string topic=mode=="default" ? "/world/default/model/arm/joint_state" : "/m18/joint_state";
  if(!node.Subscribe(topic,std::function<void(const Message &)>([&](const Message &m){
      std::lock_guard<std::mutex> lock(mutex);
      if(received.size()<256) received.push_back(m);
      ready.notify_all(); }))) throw std::runtime_error("subscribe");
  gz::sim::ServerConfig config;
  if(!config.SetSdfString(xml)) throw std::runtime_error("SDF refused");
  config.SetPhysicsEngine("gz-physics-dartsim-plugin");
  config.AddPlugin(gz::sim::ServerConfig::PluginInfo("default","world",
      sdf::Plugin("gz-sim-physics-system","gz::sim::systems::Physics")));
  gz::sim::Server server(config);
  auto observe=std::make_shared<Observe>(); server.AddSystem(observe);
  bool mapsAttempted=false;
  const auto capture=[&]{if(!mapsAttempted){mapsAttempted=true;maps(mapsPath);}};
  try
  {
    // Ordinary Run counts advancing iterations; RunOnce(true) explicitly
    // consumes one paused update. Discovery is separate from measured ticks.
    if(!server.RunOnce(true)) throw std::runtime_error("paused discovery run");
    if (!observe->failure.empty()) throw std::runtime_error(observe->failure);
    if (!observe->warmup) throw std::runtime_error("missing paused-zero component observation");
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (!ready.wait_for(lock,5s,[&]{return !received.empty();}) || received.size()!=1)
        throw std::runtime_error("single paused-zero callback discovery deadline/count");
      const auto &message = received[0];
      if (message.header().stamp().sec()!=0 || message.header().stamp().nsec()!=0
          || message.name()!="arm" || message.id()!=observe->warmup->id || !message.has_pose())
        throw std::runtime_error("paused-zero discovery packet metadata");
      poseEqual(message.pose(),observe->warmup->pose);
      received.clear();
    }
    if (!observe->rows.empty()) throw std::runtime_error("unexpected measured rows before finite run");
    if(!server.Run(true,100,false) || !server.RunOnce(true) || !server.RunOnce(true))
      throw std::runtime_error("finite native runs");
    if(!observe->failure.empty()) throw std::runtime_error(observe->failure);
    if(observe->rows.size()!=102) throw std::runtime_error("102 native component observations required");
    for (std::size_t i=0;i<observe->rows.size();++i)
    {
      const auto &row=observe->rows[i];
      const long long expectedTime=i<100 ? static_cast<long long>(i+1)*1000000LL : 100000000LL;
      if(row.time!=expectedTime || row.paused!=(i>=100))
        throw std::runtime_error("native observation time/paused sequence mismatch");
    }
    const std::size_t expected=mode=="rate" ? 12 : 102;
    std::vector<Message> messages;
    {std::unique_lock<std::mutex> lock(mutex);ready.wait_for(lock,2s,[&]{return received.size()>=expected;});messages=received;}
    if(messages.size()!=expected) throw std::runtime_error("native packet count mismatch");
    for(std::size_t index=0;index<messages.size();++index)
    {
      const auto &message=messages[index];
      const long long stamp=message.header().stamp().sec()*1000000000LL+message.header().stamp().nsec();
      const long long expectedStamp=index<(mode=="rate"?10u:100u)
          ? static_cast<long long>(index+1)*(mode=="rate"?10000000LL:1000000LL) : 100000000LL;
      if(stamp!=expectedStamp) throw std::runtime_error("native cadence including equal time mismatch");
      const std::size_t advancing=mode=="rate" ? 10 : 100;
      const std::size_t observationIndex=index<advancing
          ? static_cast<std::size_t>(stamp/1000000LL)-1 : 100+(index-advancing);
      const auto &row=observe->rows[observationIndex];
      if(row.time!=stamp || row.paused!=(index>=advancing))
        throw std::runtime_error("packet did not bind exact corresponding observation");
      if(message.name()!="arm" || message.id()!=row.id || !message.has_pose()) throw std::runtime_error("model metadata");
      poseEqual(message.pose(),row.pose);
      std::vector<JointObservation> selected;
      for(const auto &j:row.joints) if(mode!="filter" || j.name=="tip_hinge") selected.push_back(j);
      if(message.joint_size()!=static_cast<int>(selected.size())) throw std::runtime_error("joint filter/dedup");
      std::cout << "{\"mode\":\"" << mode << "\",\"time_ns\":" << stamp
          << ",\"paused\":" << (index>=(mode=="rate"?10u:100u)?"true":"false")
          << ",\"model_id\":" << row.id << ",\"model_pose\":"; emitPose(row.pose);
      std::cout << ",\"joints\":[";
      for(std::size_t i=0;i<selected.size();++i)
      {
        const auto &m=message.joint(static_cast<int>(i));const auto &j=selected[i];
        if(m.id()!=j.id || m.name()!=j.name || m.parent()!=j.parent || m.child()!=j.child || !m.has_pose() || !m.has_axis1())
          throw std::runtime_error("joint metadata");
        poseEqual(m.pose(),j.pose);
        const auto &a=m.axis1(); const auto xyz=gz::msgs::Convert(a.xyz());
        if(!std::isfinite(j.position) || !std::isfinite(j.velocity) || !std::isfinite(a.position()) || !std::isfinite(a.velocity())
            || std::abs(a.position()-j.position)>1e-12 || std::abs(a.velocity()-j.velocity)>1e-12
            || xyz!=j.axis.Xyz() || a.limit_lower()!=j.axis.Lower() || a.limit_upper()!=j.axis.Upper() || a.damping()!=j.axis.Damping())
          throw std::runtime_error("joint axis packet/component mismatch");
        if(i) std::cout << ',';
        std::cout << "{\"id\":" << j.id << ",\"name\":\"" << j.name << "\",\"parent\":\"" << j.parent
            << "\",\"child\":\"" << j.child << "\",\"pose\":";emitPose(j.pose);
        std::cout << ",\"axis\":[" << xyz.X() << ',' << xyz.Y() << ',' << xyz.Z() << ']'
            << ",\"lower\":\"" << a.limit_lower() << "\",\"upper\":\"" << a.limit_upper()
            << "\",\"damping\":" << a.damping() << ",\"position\":" << a.position() << ",\"velocity\":" << a.velocity() << '}';
      }
      std::cout << "]}\n";
    }
    if(std::abs(observe->rows[99].joints[0].position)<=1e-5 || std::abs(observe->rows[99].joints[0].velocity)<=1e-5)
      throw std::runtime_error("no actual joint motion witness");
    capture(); server.Stop();
  }
  catch(...)
  {
    const auto originalError=std::current_exception();
    try{server.Stop();}catch(...){std::cerr<<"native stop failed\n";}
    std::cerr<<"native joint-state observer: "<<observe->failure<<"; rows="<<observe->rows.size()<<'\n';
    try{capture();}catch(const std::exception &e){std::cerr<<"live maps failure: "<<e.what()<<'\n';}
    catch(...){std::cerr<<"unknown live maps failure\n";}
    std::rethrow_exception(originalError);
  }
}
int main(int argc,char **argv)
try
{
  if(argc!=3) return 2;
  std::cout<<std::setprecision(17);run(argv[1],argv[2]);
}
catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
