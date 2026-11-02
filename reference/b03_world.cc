// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Actual Gazebo Server observer SOURCE. No standalone physics reconstruction.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <sdf/Element.hh>
#include <sdf/Plugin.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Light.hh>

namespace sim = gz::sim;
namespace c = gz::sim::components;
static void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
static void vector(std::ostream &out, const gz::math::Vector3d &v) {
  require(v.IsFinite(), "nonfinite observation");
  out << ' ' << v.X() << ' ' << v.Y() << ' ' << v.Z();
}
static void pose(std::ostream &out, const gz::math::Pose3d &p) {
  vector(out, p.Pos());
  require(p.Rot().IsFinite(), "nonfinite orientation");
  out << ' ' << p.Rot().X() << ' ' << p.Rot().Y() << ' ' << p.Rot().Z() << ' ' << p.Rot().W();
}
struct BodyRow {
  std::string name; sim::Entity id; gz::math::Pose3d pose;
  gz::math::Vector3d origin, com, angular, comPosition, localCOM;
};
struct JointRow { std::string name; sim::Entity id; double position, velocity; };
struct Frame { unsigned tick; std::int64_t nanos; std::vector<BodyRow> bodies; std::vector<JointRow> joints; };
struct Observe final : sim::System, sim::ISystemPreUpdate, sim::ISystemPostUpdate {
  bool prepared{false}; unsigned ticks{0}; unsigned bodyQueries{0}, jointQueries{0};
  std::map<std::string, sim::Entity> identities;
  std::vector<std::string> entityLines;
  std::vector<Frame> selected;
  template<class Kind> unsigned entities(const char *family, sim::EntityComponentManager &ecm) {
    unsigned count=0;
    ecm.Each<Kind,c::Name>([&](auto id,const auto*,const auto*) {
      const auto name=sim::scopedName(id,ecm,"::",false);
      require(identities.emplace(name,id).second,"duplicate scoped identity");
      const auto *parent=ecm.Component<c::ParentEntity>(id);
      std::ostringstream line;
      line << "ENTITY " << family << ' ' << std::quoted(name) << ' ' << id << ' ' << (parent?parent->Data():0);
      entityLines.push_back(line.str()); ++count; return true;
    });
    return count;
  }
  void PreUpdate(const sim::UpdateInfo &, sim::EntityComponentManager &ecm) override {
    if (prepared) return;
    require(entities<c::World>("WORLD",ecm)==1,"world count");
    require(entities<c::Model>("MODEL",ecm)==3,"model count");
    require(entities<c::Light>("LIGHT",ecm)==1,"sun count");
    require(entities<c::Link>("BODY",ecm)==7,"body count");
    require(entities<c::Joint>("JOINT",ecm)==4,"joint count");
    require(entities<c::Collision>("COLLISION",ecm)==15,"collision count");
    ecm.Each<c::Link>([&](auto id,const auto*) { sim::Link(id).EnableVelocityChecks(ecm); return true; });
    ecm.Each<c::Joint>([&](auto id,const auto*) {
      if(!ecm.Component<c::JointPosition>(id)) ecm.CreateComponent(id,c::JointPosition());
      if(!ecm.Component<c::JointVelocity>(id)) ecm.CreateComponent(id,c::JointVelocity());
      return true;
    });
    prepared=true;
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm) override {
    if(info.paused) { require(info.simTime==std::chrono::nanoseconds::zero(),"paused preparation advanced time"); return; }
    ++ticks;
    require(info.dt==std::chrono::milliseconds(1) && info.simTime==std::chrono::milliseconds(ticks),"step clock changed");
    Frame frame{ticks,std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count(),{}, {}};
    ecm.Each<c::Link,c::Inertial>([&](auto id,const auto*,const auto *inertia) {
      const auto name=sim::scopedName(id,ecm,"::",false);
      require(identities.at(name)==id,"body identity changed");
      sim::Link link(id);
      const auto p=link.WorldPose(ecm), cp=link.WorldInertialPose(ecm);
      const auto local=inertia->Data().Pose().Pos();
      const auto v=link.WorldLinearVelocity(ecm), cv=link.WorldLinearVelocity(ecm,local), w=link.WorldAngularVelocity(ecm);
      require(p && cp && v && cv && w,"missing native point observation");
      require(p->Pos().IsFinite() && p->Rot().IsFinite() && cp->Pos().IsFinite() &&
          local.IsFinite() && v->IsFinite() && cv->IsFinite() && w->IsFinite(),
          "nonfinite body query");
      frame.bodies.push_back({name,id,*p,*v,*cv,*w,cp->Pos(),local}); ++bodyQueries; return true;
    });
    ecm.Each<c::Joint,c::JointPosition,c::JointVelocity>([&](auto id,const auto*,const auto *p,const auto *v) {
      const auto name=sim::scopedName(id,ecm,"::",false);
      require(identities.at(name)==id && p->Data().size()==1 && v->Data().size()==1,"joint observation changed");
      require(std::isfinite(p->Data()[0]) && std::isfinite(v->Data()[0]),"nonfinite joint state");
      frame.joints.push_back({name,id,p->Data()[0],v->Data()[0]}); ++jointQueries; return true;
    });
    require(frame.bodies.size()==7 && frame.joints.size()==4,"query work changed");
    if(ticks==1 || ticks==10 || ticks==100) selected.push_back(std::move(frame));
  }
  void write(std::ostream &out) const {
    for(const auto &line:entityLines) out << line << '\n';
    for(const auto &frame:selected) {
      out << "FRAME " << frame.tick << ' ' << frame.nanos << '\n';
      for(const auto &b:frame.bodies) {
        out << "BODY " << frame.tick << ' ' << std::quoted(b.name) << ' ' << b.id;
        pose(out,b.pose); vector(out,b.origin); vector(out,b.com); vector(out,b.angular);
        vector(out,b.comPosition); vector(out,b.localCOM); out << '\n';
      }
      for(const auto &j:frame.joints) out << "JOINT " << frame.tick << ' ' << std::quoted(j.name) << ' ' << j.id << ' ' << j.position << ' ' << j.velocity << '\n';
    }
  }
};
int main(int argc,char **argv) {
  if(argc!=4) return 2; // exact input path, output prefix, correctness|measure
  const std::string mode=argv[3]; require(mode=="correctness" || mode=="measure","unknown mode");
  for(const auto *suffix:{".input.sdf", ".state", ".maps", ".metrics.json"})
    require(!std::filesystem::exists(std::string(argv[2])+suffix),"output prefix already used");
  std::ifstream input(argv[1]); require(bool(input),"missing B03 input");
  const std::string xml((std::istreambuf_iterator<char>(input)),{});
  sdf::Root dom; require(dom.Load(argv[1]).empty() && dom.WorldCount()==1,"native DOM load failed");
  const auto *world=dom.WorldByIndex(0);
  require(world->Name()=="default" && world->ModelCount()==3 && world->LightCount()==1 && world->LightByIndex(0)->Name()=="sun","changed B03 world");
  std::ofstream copied(std::string(argv[2])+".input.sdf"); copied << xml; copied.close(); require(bool(copied),"input copy failed");
  sim::ServerConfig config; require(config.SetSdfFile(argv[1]),"SetSdfFile failed");
  config.SetPhysicsEngine("gz-physics-dartsim-plugin");
  config.AddPlugin(sim::ServerConfig::PluginInfo("default","world",sdf::Plugin("gz-sim-physics-system","gz::sim::systems::Physics")));
  sim::Server server(config); auto observer=std::make_shared<Observe>();
  const auto added=server.AddSystem(observer); require(added && *added,"observer admission failed");
  // Build native physics and velocity query components without advancing time.
  require(server.RunOnce(true) && observer->prepared && observer->ticks==0,"paused preparation failed");
  const auto begin=std::chrono::steady_clock::now();
  require(server.Run(true,100,false),"unpaused Server run failed");
  const auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-begin).count();
  require(observer->ticks==100 && observer->bodyQueries==700 && observer->jointQueries==400 && observer->selected.size()==3,"incomplete B03 work");
  std::ofstream out(std::string(argv[2])+".state"); out << std::setprecision(17);
  out << "INPUT " << std::quoted("b03.sdf") << ' ' << std::quoted(config.SdfFile()) << ' ' << std::quoted(world->Element()->FilePath()) << ' ' << world->Element()->LineNumber().value_or(-1) << '\n';
  out << "GRAVITY"; vector(out,world->Gravity()); out << '\n'; observer->write(out); out.flush(); require(bool(out),"state output failed");
  std::ifstream maps("/proc/self/maps"); require(bool(maps),"maps unavailable");
  std::ofstream mapped(std::string(argv[2])+".maps"); mapped << maps.rdbuf(); mapped.flush(); require(bool(mapped),"maps output failed");
  std::ostringstream metrics;
  metrics << "{\"role\":\"gazebo-server\",\"mode\":\"" << mode << "\",\"steps\":100,\"dt_ns\":1000000,\"body_queries\":700,\"joint_queries\":400,\"window_wall_ns\":" << wall << ",\"scope\":\"contextual-loaded-world-physics-query\",\"intrinsic_server_network\":\"present-unmatched\",\"worlds\":1,\"bodies\":7,\"joints\":4,\"colliders\":15,\"paused_preparation_calls\":1,\"sensors\":0,\"rendering\":false}\n";
  std::ofstream metricFile(std::string(argv[2])+".metrics.json");
  metricFile << metrics.str(); metricFile.flush(); require(bool(metricFile),"metrics output failed");
  std::cout << metrics.str();
}
