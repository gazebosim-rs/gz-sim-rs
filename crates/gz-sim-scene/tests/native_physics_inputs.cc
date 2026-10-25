// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <memory>
#include <string>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/LinearVelocityReset.hh>
#include <gz/sim/components/AngularVelocityReset.hh>
class Inputs final : public gz::sim::System,
 public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
 public: std::string mode;
 public: gz::sim::Entity link{gz::sim::kNullEntity};
 public: void PreUpdate(const gz::sim::UpdateInfo &_info,
 gz::sim::EntityComponentManager &_ecm) override
 {
   link = _ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("body"));
   if (link == gz::sim::kNullEntity) throw std::runtime_error("missing body");
   gz::sim::Link body(link);
   body.EnableVelocityChecks(_ecm);
   if (_info.iterations != 1) return;
   if (mode == "velocity" || mode == "both") _ecm.CreateComponent(link, gz::sim::components::WorldLinearVelocityReset({1, 0, 0}));
   if (mode == "force" || mode == "both") body.AddWorldForce(_ecm, {2, 0, 0});
   if (mode == "angular") _ecm.CreateComponent(link, gz::sim::components::WorldAngularVelocityReset({0, 0, 1}));
   if (mode == "torque") body.AddWorldWrench(_ecm, {0, 0, 0}, {0, 0, 0.2});
 }
 public: void PostUpdate(const gz::sim::UpdateInfo &_info,
 const gz::sim::EntityComponentManager &_ecm) override
 {
   gz::sim::Link body(link);
   auto velocity = body.WorldLinearVelocity(_ecm);
   auto pose = body.WorldPose(_ecm);
   auto angular = body.WorldAngularVelocity(_ecm);
   if (!velocity || !pose || !angular) throw std::runtime_error("missing observations");
   std::cout << std::setprecision(17) << "{\"iteration\":" << _info.iterations
    << ",\"vx\":" << velocity->X() << ",\"x\":" << pose->Pos().X()
    << ",\"y\":" << pose->Pos().Y() << ",\"vy\":" << velocity->Y()
    << ",\"wz\":" << angular->Z() << "}\n";
 }
};
int main(int argc, char **argv)
{
 if(argc != 3) return 2;
 std::string xml((std::istreambuf_iterator<char>(std::cin)), {});
 gz::sim::ServerConfig config;
 if(!config.SetSdfString(xml)) return 3;
 gz::sim::Server server(config);
 auto input = std::make_shared<Inputs>(); input->mode=argv[1];
 if(server.AddSystem(input) != std::optional<bool>(true)) return 4;
 if(!server.Run(true,3,false)) return 5;
 std::ifstream maps("/proc/self/maps"); std::ofstream output(argv[2]);
 output << maps.rdbuf(); if(!output) return 6;
}
