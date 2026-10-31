// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Usage: static-contacts <pinned dartsim plugin> <loaded-maps output>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

using Features = gz::physics::FeatureList<gz::physics::ForwardStep,
    gz::physics::AddLinkExternalForceTorque, gz::physics::SetFreeGroupWorldVelocity, gz::physics::GetBasicJointState, gz::physics::SetBasicJointState,
    gz::physics::GetJointFromModel, gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel, gz::physics::LinkFrameSemantics,
    gz::physics::sdf::ConstructSdfWorld>;

static void vector(const Eigen::Vector3d &v) {
  std::cout << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}
static std::string link(const std::string &name, const std::string &pose) {
  return "<link name='"+name+"'><pose>"+pose+"</pose><inertial><mass>1</mass>"
    "<inertia><ixx>0.004</ixx><iyy>0.004</iyy><izz>0.004</izz></inertia>"
    "</inertial><collision name='c'><geometry><sphere><radius>0.1</radius>"
    "</sphere></geometry></collision></link>";
}
static std::string joint(const std::string &name, const std::string &parent,
                         const std::string &child, const std::string &pose) {
  return "<joint name='"+name+"' type='revolute'><parent>"+parent+"</parent><child>"+
    child+"</child><pose>"+pose+"</pose><axis><xyz>0 1 0</xyz></axis></joint>";
}
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  gz::plugin::Loader loader;
  loader.LoadLib(argv[1]);
  auto engine=gz::physics::RequestEngine3d<Features>::From(loader.Instantiate("gz::physics::dartsim::Plugin"));
  if (!engine) return 3;
  std::cout << std::setprecision(17);
  for (int scenario : {0,3}) {
    std::string obstacle=scenario==3 ? "0.205 0 0 0 0 0" : "0.15 0 0 0 0 0";
    std::string text="<sdf version='1.11'><world name='w'><gravity>0 0 0</gravity><model name='m'>"+
      link("parent","0 0 0 0 0 0")+
      link("child","0 0 -2 0 0 0")+
      joint("leaf","parent","child","0 0 1 0 0 0")+
      "</model><model name='obstacle'><static>true</static>"+link("body",obstacle)+"</model></world></sdf>";
    sdf::Root root; if(!root.LoadSdfString(text).empty()) return 4;
    auto world=engine->ConstructWorld(*root.WorldByIndex(0)); if(!world) return 5;
    auto model=world->GetModel("m"); auto leaf=model->GetJoint("leaf");
    {
      auto group=model->GetLink("parent")->FindFreeGroup(); if(!group) return 6;
      group->SetWorldLinearVelocity(Eigen::Vector3d(scenario==3 ? 1 : .2,0,0));
      group->SetWorldAngularVelocity(Eigen::Vector3d(0,scenario==3 ? 0 : 1,0));
    }
    leaf->SetVelocity(0,scenario==3 ? 0 : 2);
    gz::physics::ForwardStep::Input input; gz::physics::ForwardStep::State state; gz::physics::ForwardStep::Output output;
    input.Get<std::chrono::steady_clock::duration>()=std::chrono::milliseconds(1);
    for(int tick=0;tick<=100;++tick) {
      if(tick>0) {
        leaf->SetForce(0,scenario==3 || tick==1 ? .7 : 0);
        if(scenario==0 && tick==1) {
          model->GetLink("parent")->AddExternalForce(Eigen::Vector3d(.3,0,0));
          model->GetLink("parent")->AddExternalTorque(Eigen::Vector3d(0,.2,0));
        }
        world->Step(output,state,input);
      }
      if(tick>6 && tick%10!=0) continue;
      std::cout<<"{\"scenario\":"<<scenario<<",\"tick\":"<<tick<<",\"bodies\":[";
      int index=0;
      for(const char *name:{"parent","child"}) {
        auto data=model->GetLink(name)->FrameDataRelativeToWorld(); Eigen::Quaterniond q(data.pose.linear());
        std::cout<<(index++ ? "," : "")<<"{\"position\":"; vector(data.pose.translation());
        std::cout<<",\"orientation\":["<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<"],\"velocity\":"; vector(data.linearVelocity);
        std::cout<<",\"angular_velocity\":"; vector(data.angularVelocity); std::cout<<'}';
      }
      std::cout<<"],\"q\":"<<leaf->GetPosition(0)<<",\"dq\":"<<leaf->GetVelocity(0)<<"}\n";
    }
  }
  std::ifstream maps("/proc/self/maps"); std::ofstream recorded(argv[2]); recorded<<maps.rdbuf();
  return recorded ? 0 : 7;
}
