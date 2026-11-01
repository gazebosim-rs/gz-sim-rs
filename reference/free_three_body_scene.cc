// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Native reference observer only; no Rust system or plugin architecture.
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <sdf/Plugin.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Joint.hh>
#include <sdf/Element.hh>
namespace sim = gz::sim;
namespace c = gz::sim::components;
void vector(std::ostream &o, const gz::math::Vector3d &v)
{ o << ' ' << v.X() << ' ' << v.Y() << ' ' << v.Z(); }
void pose(std::ostream &o, const gz::math::Pose3d &p)
{ vector(o,p.Pos()); o << ' ' << p.Rot().X() << ' ' << p.Rot().Y()
  << ' ' << p.Rot().Z() << ' ' << p.Rot().W(); }
struct Observe final : sim::System, sim::ISystemPreUpdate, sim::ISystemPostUpdate
{
  std::ofstream out;
  unsigned ticks{0};
  explicit Observe(const std::string &p): out(p) { out << std::setprecision(17); }
  void PreUpdate(const sim::UpdateInfo &, sim::EntityComponentManager &ecm) override
  {
    unsigned links=0,joints=0;
    ecm.Each<c::Link,c::Name>([&](auto id,const auto*,const auto *name)
    {
      ++links;
      sim::Link(id).EnableVelocityChecks(ecm);
      if (ticks==0)
      {
        out << "INITIAL_LINK " << std::quoted(name->Data()) << ' ' << id << ' '
          << ecm.Component<c::ParentEntity>(id)->Data();
        pose(out,ecm.Component<c::Pose>(id)->Data()); pose(out,sim::worldPose(id,ecm)); out << '\n';
      }
      return true;
    });
    ecm.Each<c::Joint,c::Name>([&](auto id,const auto*,const auto *name)
    {
      ++joints;
      const auto n=name->Data();
      if (n!="first" && n!="second") throw std::runtime_error("unexpected joint");
      const double effort=n=="first"?.7:-.2;
      auto *f=ecm.Component<c::JointForceCmd>(id);
      if (f) f->Data()={effort}; else ecm.CreateComponent(id,c::JointForceCmd({effort}));
      if (!ecm.Component<c::JointPosition>(id)) ecm.CreateComponent(id,c::JointPosition());
      if (!ecm.Component<c::JointVelocity>(id)) ecm.CreateComponent(id,c::JointVelocity());
      if (ticks==0)
      {
        out << "INITIAL_JOINT " << std::quoted(n) << ' ' << id << ' '
          << ecm.Component<c::ParentEntity>(id)->Data() << ' '
          << std::quoted(ecm.Component<c::ParentLinkName>(id)->Data()) << ' '
          << std::quoted(ecm.Component<c::ChildLinkName>(id)->Data());
        pose(out,ecm.Component<c::Pose>(id)->Data()); pose(out,sim::worldPose(id,ecm));
        const auto &axis=ecm.Component<c::JointAxis>(id)->Data();
        vector(out,axis.Xyz()); out << ' ' << std::quoted(axis.XyzExpressedIn()) << '\n';
      }
      out << "COMMAND " << ticks+1 << ' ' << std::quoted(n) << ' ' << effort << '\n';
      return true;
    });
    if (links!=3 || joints!=2) throw std::runtime_error("wrong graph size");
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm) override
  {
    ++ticks;
    if (info.paused || info.iterations!=ticks || info.simTime!=std::chrono::milliseconds(ticks))
      throw std::runtime_error("wrong time");
    ecm.Each<c::Link,c::Name>([&](auto id,const auto*,const auto *name)
    {
      auto v=ecm.Component<c::WorldLinearVelocity>(id);
      auto w=ecm.Component<c::WorldAngularVelocity>(id);
      if (!v || !w) throw std::runtime_error("missing velocity");
      out << "BODY " << ticks << ' ' << std::quoted(name->Data()) << ' ' << id;
      pose(out,sim::worldPose(id,ecm)); vector(out,v->Data()); vector(out,w->Data()); out << '\n';
      return true;
    });
    ecm.Each<c::Joint,c::Name,c::JointPosition,c::JointVelocity>(
      [&](auto id,const auto*,const auto *n,const auto *p,const auto *v)
      { if(p->Data().size()!=1 || v->Data().size()!=1) throw std::runtime_error("missing joint state");
        out << "JOINT " << ticks << ' ' << std::quoted(n->Data()) << ' ' << id
          << ' ' << p->Data()[0] << ' ' << v->Data()[0] << '\n'; return true; });
    out.flush();
  }
};
int main(int argc,char **argv)
{
  if(argc!=3) return 2;
  std::ifstream in(argv[1]); const std::string xml((std::istreambuf_iterator<char>(in)),{});
  // Independent native DOM resolution. Keep generic ECM traversal in the
  // existing state log separate from the semantic joint frame.
  sdf::Root dom;
  if (!dom.LoadSdfString(xml).empty() || dom.WorldCount()!=1) return 7;
  const auto *model=dom.WorldByIndex(0)->ModelByName("chain");
  if (!model || model->JointCount()!=2) return 7;
  std::ofstream semantic(std::string(argv[2])+".semantic");
  semantic << std::setprecision(17);
  for (uint64_t i=0;i<model->JointCount();++i)
  {
    const auto *joint=model->JointByIndex(i);
    const auto *child=model->LinkByName(joint->ChildName());
    if (!child) return 7;
    gz::math::Pose3d modelFrame,childFrame,childLocal;
    if (!joint->SemanticPose().Resolve(modelFrame,"__model__").empty() ||
        !joint->SemanticPose().Resolve(childLocal).empty() ||
        !child->SemanticPose().Resolve(childFrame,"__model__").empty()) return 7;
    semantic << "DOM_JOINT " << std::quoted(joint->Name());
    pose(semantic,joint->RawPose());
    semantic << ' ' << std::quoted(joint->PoseRelativeTo());
    pose(semantic,modelFrame); pose(semantic,childLocal);
    pose(semantic,childFrame*childLocal);
    semantic << ' ' << joint->Element()->LineNumber().value_or(-1) << ' '
      << std::quoted(joint->Element()->FilePath()) << '\n';
  }
  semantic.flush(); if (!semantic) return 7;
  sim::ServerConfig config;
  if(!config.SetSdfString(xml)) return 3;
  config.SetPhysicsEngine("gz-physics-dartsim-plugin");
  config.AddPlugin(sim::ServerConfig::PluginInfo("default","world",
    sdf::Plugin("gz-sim-physics-system","gz::sim::systems::Physics")));
  sim::Server server(config);
  auto obs=std::make_shared<Observe>(argv[2]);
  auto added=server.AddSystem(obs); if(!added || !*added) return 4;
  if(!server.Run(true,10,false) || obs->ticks!=10) return 5;
  std::ifstream maps("/proc/self/maps"); std::ofstream mapped(std::string(argv[2])+".maps"); mapped<<maps.rdbuf();
  return obs->out && mapped ? 0:6;
}
