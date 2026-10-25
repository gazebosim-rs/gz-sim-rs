// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <set>
#include <string>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Name.hh>
class Observe final : public gz::sim::System,
 public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate,
 public gz::sim::ISystemReset
{
 public: unsigned epoch{0};
 public: double speed{0};
 public: gz::sim::Entity link{gz::sim::kNullEntity}, joint{gz::sim::kNullEntity};
 public: void Enable(gz::sim::EntityComponentManager &_ecm)
 {
   link=_ecm.EntityByComponents(gz::sim::components::Link(),gz::sim::components::Name("body"));
   joint=_ecm.EntityByComponents(gz::sim::components::Joint(),gz::sim::components::Name("hinge"));
   if(link==gz::sim::kNullEntity || joint==gz::sim::kNullEntity) throw std::runtime_error("missing pendulum entities");
   gz::sim::Link(link).EnableVelocityChecks(_ecm);
   gz::sim::Joint(joint).EnablePositionCheck(_ecm);
   gz::sim::Joint(joint).EnableVelocityCheck(_ecm);
 }
 public: void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) override
 { ++epoch; Enable(_ecm); }
 public: void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
 {
   Enable(_ecm);
   if(speed != 0 && _info.iterations == 1)
     _ecm.CreateComponent(joint, gz::sim::components::JointVelocityReset({speed}));
 }
 public: void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override
 {
   const std::set<uint64_t> samples{0,1,10,100,500,1000,2000,4000};
   if(!samples.count(_info.iterations)) return;
   const auto p=gz::sim::Link(link).WorldPose(_ecm);
   const auto v=gz::sim::Link(link).WorldLinearVelocity(_ecm);
   const auto w=gz::sim::Link(link).WorldAngularVelocity(_ecm);
   const auto q=gz::sim::Joint(joint).Position(_ecm);
   const auto dq=gz::sim::Joint(joint).Velocity(_ecm);
   if(!p || !v || !w || !q || !dq || q->size()!=1 || dq->size()!=1) throw std::runtime_error("missing pendulum state");
   std::cout<<std::setprecision(17)<<"{\"epoch\":"<<epoch<<",\"iteration\":"<<_info.iterations
    <<",\"time_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count()
    <<",\"x\":"<<p->Pos().X()<<",\"z\":"<<p->Pos().Z()<<",\"qy\":"<<p->Rot().Y()<<",\"qw\":"<<p->Rot().W()
    <<",\"vx\":"<<v->X()<<",\"vz\":"<<v->Z()<<",\"wy\":"<<w->Y()<<",\"q\":"<<q->at(0)<<",\"dq\":"<<dq->at(0)<<"}\n";
 }
};
int main(int argc,char **argv)
{
 if(argc!=3)return 2;
 std::string xml((std::istreambuf_iterator<char>(std::cin)),{});
 gz::sim::ServerConfig config;if(!config.SetSdfString(xml))return 3;
 gz::sim::Server server(config);auto observer=std::make_shared<Observe>();
 observer->speed=std::stod(argv[2]);
 if(server.AddSystem(observer)!=std::optional<bool>(true))return 4;
 if(!server.Run(true,4000,false))return 5;
 if(!server.Reset(0) || !server.Run(true,1,true) || !server.Run(true,10,false))return 6;
 std::ifstream maps("/proc/self/maps");std::ofstream output(argv[1]);output<<maps.rdbuf();if(!output)return 7;
}
