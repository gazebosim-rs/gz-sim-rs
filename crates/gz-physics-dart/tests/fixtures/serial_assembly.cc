// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Selected SDK final-tree observation, not a reproduction of the B03 refusal.
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <Eigen/Geometry>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
using Features=gz::physics::FeatureList<gz::physics::ForwardStep,
    gz::physics::AddLinkExternalForceTorque,gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel,gz::physics::GetJointFromModel,
    gz::physics::GetBasicJointState,gz::physics::LinkFrameSemantics,
    gz::physics::sdf::ConstructSdfWorld>;
static void require(bool ok,const char *why) { if(!ok) throw std::runtime_error(why); }
static void maps(const std::filesystem::path &path) {
  std::ifstream source("/proc/self/maps");std::ofstream saved(path);saved<<source.rdbuf();saved.flush();
  require(bool(source)&&bool(saved),"maps capture failed");
}
static void vector(const Eigen::Vector3d &value) {
  require(value.allFinite(),"nonfinite observation");
  std::cout<<value.x()<<','<<value.y()<<','<<value.z();
}
int main(int argc,char **argv) {
  if(argc!=3) return 2;
  const std::filesystem::path output(argv[2]);
  require(std::filesystem::create_directory(output),"output already exists");
  gz::plugin::Loader loader;loader.LoadLib(argv[1]);
  auto engine=gz::physics::RequestEngine3d<Features>::From(loader.Instantiate("gz::physics::dartsim::Plugin"));
  require(bool(engine),"engine missing requested features");maps(output/"before.maps");
  std::cout<<std::setprecision(17);
  for(unsigned scenario=0;scenario<2;++scenario) {
    const std::string z=scenario==0?"1":"0.19";
    const std::string floor=scenario==0?"":
      "<model name='floor'><static>true</static><link name='body'><collision name='plane'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model>";
    std::string links;
    for(const auto *name:{"root","middle","leaf"}) links+=
      std::string("<link name='")+name+"'><pose>0 0 "+z+" 0 0 0</pose><self_collide>false</self_collide><inertial><pose>0 0 0.1 0 0 0</pose><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='sphere'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link>";
    const std::string xml="<sdf version='1.11'><world name='w'><gravity>0 0 "+std::string(scenario==0?"0":"-9.81")+"</gravity>"+floor+
      "<model name='chain'><self_collide>false</self_collide>"+links+
      "<joint name='first' type='revolute'><parent>root</parent><child>middle</child><axis><xyz>0 0 1</xyz></axis></joint>"
      "<joint name='second' type='revolute'><parent>middle</parent><child>leaf</child><axis><xyz>0 0 1</xyz></axis></joint></model></world></sdf>";
    std::ofstream saved(output/(std::to_string(scenario)+".sdf"));saved<<xml;saved.close();require(bool(saved),"XML save failed");
    sdf::Root root;require(root.LoadSdfString(xml).empty()&&root.WorldByIndex(0),"SDF load failed");
    auto world=engine->ConstructWorld(*root.WorldByIndex(0));require(bool(world),"world construction failed");
    auto model=world->GetModel("chain");require(bool(model),"model missing");
    auto first=model->GetJoint("first"),second=model->GetJoint("second");require(bool(first)&&bool(second),"joint missing");
    auto origin=model->GetLink("root");require(bool(origin),"root missing");
    gz::physics::ForwardStep::Input input;gz::physics::ForwardStep::State state;gz::physics::ForwardStep::Output result;
    input.Get<std::chrono::steady_clock::duration>()=std::chrono::milliseconds(1);
    for(unsigned tick=0;tick<=100;++tick) {
      if(tick) {
        if(tick==1) {
          origin->AddExternalForce(Eigen::Vector3d(1,2,3),gz::physics::FrameID::World(),Eigen::Vector3d(0,0,0.1));
          origin->AddExternalTorque(Eigen::Vector3d(0.2,-0.1,0.3),gz::physics::FrameID::World());
        }
        world->Step(result,state,input);
      }
      if(tick!=0&&tick!=1&&tick!=10&&tick!=100) continue;
      std::cout<<"{\"scenario\":"<<scenario<<",\"tick\":"<<tick<<",\"requested_time_ns\":"<<tick*1000000<<",\"bodies\":[";
      bool comma=false;
      for(const auto *name:{"root","middle","leaf"}) {
        auto body=model->GetLink(name);require(bool(body),"link missing");const auto value=body->FrameDataRelativeToWorld();
        require(value.pose.matrix().allFinite(),"nonfinite pose");
        const Eigen::Quaterniond q(value.pose.linear());require(q.coeffs().allFinite()&&std::abs(q.squaredNorm()-1)<=1e-12,"orientation invalid");
        const Eigen::Vector3d local(0,0,0.1),com=value.pose*local;
        const Eigen::Vector3d velocity=value.linearVelocity+value.angularVelocity.cross(value.pose.linear()*local);
        if(comma) std::cout<<',';
        comma=true;
        std::cout<<"{\"name\":\""<<name<<"\",\"values\":[";vector(value.pose.translation());
        std::cout<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<',';vector(velocity);
        std::cout<<',';vector(value.angularVelocity);std::cout<<',';vector(value.linearVelocity);
        std::cout<<',';vector(com);std::cout<<',';vector(local);std::cout<<"]}";
      }
      const double q1=first->GetPosition(0),v1=first->GetVelocity(0),q2=second->GetPosition(0),v2=second->GetVelocity(0);
      require(std::isfinite(q1)&&std::isfinite(v1)&&std::isfinite(q2)&&std::isfinite(v2),"nonfinite joint state");
      std::cout<<"],\"joints\":[["<<q1<<','<<v1<<"],["<<q2<<','<<v2<<"]]}\n";
    }
  }
  maps(output/"after.maps");
}
