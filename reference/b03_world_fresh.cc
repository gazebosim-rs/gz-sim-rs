// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// DIAGNOSTIC ONLY: actual Server, 101 frames and existing contact components.
// No standalone physics reconstruction or qualification claim.
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
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Physics.hh>
#include <sdf/Collision.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
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
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Sensor.hh>
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
  std::vector<std::string> contacts, settings;
  struct QueryFrames {
    sim::Entity link, origin, com; std::string name;
    gz::math::Vector3d localCOM;
  };
  std::vector<QueryFrames> queries;
  std::vector<std::string> queryLines, freshLines;
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
    unsigned requests=0;
    ecm.Each<c::Collision>([&](auto id,const auto*) {
      require(!ecm.Component<c::ContactSensorData>(id),"unexpected prior contact component");
      ecm.CreateComponent(id,c::ContactSensorData()); ++requests; return true;
    });
    require(requests==15,"contact request count changed");
    // Collect first: creating non-Link entities must not mutate this iteration.
    std::vector<std::pair<sim::Entity,gz::math::Vector3d>> links;
    ecm.Each<c::Link,c::Inertial>([&](auto id,const auto*,const auto *inertial) {
      links.emplace_back(id,inertial->Data().Pose().Pos()); return true;
    });
    require(links.size()==7,"query parent count changed");
    for(const auto &[link,localCOM]:links) {
      const auto name=sim::scopedName(link,ecm,"::",false);
      const auto makeQuery=[&](const char *role,const gz::math::Vector3d &offset) {
        const auto entity=ecm.CreateEntity();
        const gz::math::Pose3d local(offset,gz::math::Quaterniond::Identity);
        require(ecm.SetParentEntity(entity,link),"query parent admission failed");
        ecm.CreateComponent(entity,c::Name(std::string("__diagnostic_")+role));
        ecm.CreateComponent(entity,c::Pose(local));
        ecm.CreateComponent(entity,c::WorldPose());
        ecm.CreateComponent(entity,c::WorldLinearVelocity());
        ecm.CreateComponent(entity,c::WorldAngularVelocity());
        std::ostringstream row; row << std::setprecision(17) << "QUERY_FRAME " << std::quoted(role)
            << ' ' << std::quoted(name) << ' ' << link << ' ' << entity; pose(row,local);
        queryLines.push_back(row.str());
        return entity;
      };
      const auto origin=makeQuery("origin",gz::math::Vector3d::Zero);
      const auto com=makeQuery("declared_com",localCOM);
      queries.push_back({link,origin,com,name,localCOM});
    }
    prepared=true;
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm) override {
    if(info.paused) {
      require(info.simTime==std::chrono::nanoseconds::zero() && selected.empty(),"paused preparation changed");
    } else {
      ++ticks;
      require(info.dt==std::chrono::milliseconds(1) && info.simTime==std::chrono::milliseconds(ticks),"step clock changed");
    }
    if(info.paused) {
      ecm.Each<c::World,c::Physics>([&](auto id,const auto*,const auto *p) {
        std::ostringstream row; row << std::setprecision(17) << "SETTINGS_ECM_DOM " << p->Data().MaxStepSize() << ' ' << p->Data().MaxContacts();
        const auto *detector=ecm.Component<c::PhysicsCollisionDetector>(id);
        const auto *solver=ecm.Component<c::PhysicsSolver>(id);
        const auto *iterations=ecm.Component<c::PhysicsSolverIterations>(id);
        row << ' ' << std::quoted(detector?detector->Data():"MISSING") << ' ' << std::quoted(solver?solver->Data():"MISSING")
            << ' ' << (iterations?std::to_string(iterations->Data()):"MISSING");
        settings.push_back(row.str()); return true;
      });
    }
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
    selected.push_back(std::move(frame));
    // Physics' generic child-frame path invokes engine.Resolve each update;
    // unlike Link's cached components, it is not gated by ChangedWorldPoses.
    for(const auto &query:queries) {
      const auto point=[&](sim::Entity entity,const char *role) {
        require(!ecm.Component<c::Model>(entity) && !ecm.Component<c::Link>(entity) &&
            !ecm.Component<c::Collision>(entity) && !ecm.Component<c::Joint>(entity) &&
            !ecm.Component<c::Sensor>(entity),"query gained a physical/system kind");
        const auto *parent=ecm.Component<c::ParentEntity>(entity);
        const auto *p=ecm.Component<c::WorldPose>(entity);
        const auto *v=ecm.Component<c::WorldLinearVelocity>(entity);
        const auto *w=ecm.Component<c::WorldAngularVelocity>(entity);
        require(parent && parent->Data()==query.link && p && v && w,"query component identity changed");
        std::ostringstream row; row << std::setprecision(17) << "FRESH_POINT " << ticks << ' '
            << std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count() << ' '
            << std::quoted(role) << ' ' << std::quoted(query.name) << ' ' << query.link << ' ' << entity;
        pose(row,p->Data()); vector(row,v->Data()); vector(row,w->Data()); freshLines.push_back(row.str());
        return BodyRow{query.name,query.link,p->Data(),v->Data(),{},w->Data(),{},query.localCOM};
      };
      const auto origin=point(query.origin,"origin");
      const auto com=point(query.com,"declared_com");
      std::ostringstream row; row << std::setprecision(17) << "FRESH_BODY " << ticks << ' '
          << std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count() << ' '
          << std::quoted(query.name) << ' ' << query.link << ' ' << query.origin << ' ' << query.com;
      pose(row,origin.pose); vector(row,origin.origin); vector(row,com.origin); vector(row,origin.angular);
      vector(row,com.pose.Pos()); vector(row,query.localCOM); freshLines.push_back(row.str());
    }
    unsigned observed=0;
    ecm.Each<c::Collision,c::ContactSensorData>([&](auto owner,const auto*,const auto *data) {
      const auto ownerName=sim::scopedName(owner,ecm,"::",false); ++observed;
      std::ostringstream count; count << "CONTACT_MESSAGES " << ticks << ' ' << std::quoted(ownerName) << ' ' << data->Data().contact_size(); contacts.push_back(count.str());
      for(int m=0;m<data->Data().contact_size();++m) {
        const auto &contact=data->Data().contact(m);
        const auto first=sim::scopedName(contact.collision1().id(),ecm,"::",false);
        const auto second=sim::scopedName(contact.collision2().id(),ecm,"::",false);
        std::ostringstream header; header << "CONTACT_PAIR " << ticks << ' ' << std::quoted(ownerName) << ' ' << m
            << ' ' << std::quoted(first) << ' ' << std::quoted(second) << ' ' << contact.position_size()
            << ' ' << contact.normal_size() << ' ' << contact.depth_size() << ' ' << contact.wrench_size(); contacts.push_back(header.str());
        require(contact.normal_size()==contact.position_size() && contact.depth_size()==contact.position_size() && contact.wrench_size()==contact.position_size(),"DART contact fields missing");
        for(int p=0;p<contact.position_size();++p) {
          const auto &point=contact.position(p); const auto &normal=contact.normal(p);
          const auto &force=contact.wrench(p).body_1_wrench().force();
          const auto &opposite=contact.wrench(p).body_2_wrench().force();
          std::ostringstream row; row << std::setprecision(17) << "CONTACT_POINT " << ticks << ' ' << std::quoted(ownerName) << ' ' << m << ' ' << p
              << ' ' << point.x() << ' ' << point.y() << ' ' << point.z()
              << ' ' << normal.x() << ' ' << normal.y() << ' ' << normal.z() << ' ' << contact.depth(p)
              << ' ' << force.x() << ' ' << force.y() << ' ' << force.z()
              << ' ' << opposite.x() << ' ' << opposite.y() << ' ' << opposite.z(); contacts.push_back(row.str());
        }
      }
      return true;
    });
    require(observed==15,"contact observation owner missing");
  }
  void write(std::ostream &out) const {
    for(const auto &line:entityLines) out << line << '\n';
    for(const auto &line:settings) out << line << '\n';
    for(const auto &line:queryLines) out << line << '\n';
    for(const auto &line:freshLines) out << line << '\n';
    for(const auto &line:contacts) out << line << '\n';
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
  const std::string mode=argv[3]; require(mode=="correctness","unknown mode");
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
  require(observer->ticks==100 && observer->bodyQueries==707 && observer->jointQueries==404 && observer->selected.size()==101 && observer->queries.size()==7 && observer->queryLines.size()==14 && observer->freshLines.size()==2121,"incomplete B03 work");
  std::ofstream out(std::string(argv[2])+".state"); out << std::setprecision(17);
  out << "INPUT " << std::quoted("b03.sdf") << ' ' << std::quoted(config.SdfFile()) << ' ' << std::quoted(world->Element()->FilePath()) << ' ' << world->Element()->LineNumber().value_or(-1) << '\n';
  // Parsed DOM/default metadata, NOT direct getters of internal DART settings.
  for(std::uint64_t mi=0;mi<world->ModelCount();++mi) {
    const auto *model=world->ModelByIndex(mi);
    out << "DOM_MODEL " << std::quoted(model->Name()); pose(out,model->RawPose()); out << '\n';
    for(std::uint64_t li=0;li<model->LinkCount();++li) {
      const auto *link=model->LinkByIndex(li);
      out << "DOM_LINK " << std::quoted(model->Name()+"::"+link->Name()); pose(out,link->RawPose());
      out << ' ' << link->Inertial().MassMatrix().Mass(); pose(out,link->Inertial().Pose());
      const auto moi=link->Inertial().Moi(); for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)out << ' ' << moi(i,j); out << '\n';
      for(std::uint64_t ci=0;ci<link->CollisionCount();++ci) {
        const auto *collision=link->CollisionByIndex(ci);
        out << "DOM_COLLISION " << std::quoted(model->Name()+"::"+link->Name()+"::"+collision->Name()) << ' ' << ci;
        pose(out,collision->RawPose());
        gz::math::Pose3d resolved; require(collision->SemanticPose().Resolve(resolved,link->Name()).empty(),"local collision frame resolution failed"); pose(out,resolved); out << '\n';
      }
    }
    for(std::uint64_t ji=0;ji<model->JointCount();++ji) {
      const auto *joint=model->JointByIndex(ji);
      out << "DOM_JOINT " << std::quoted(model->Name()+"::"+joint->Name()); pose(out,joint->RawPose());
      const auto *axis=joint->Axis(0); require(axis!=nullptr,"missing B03 axis");
      vector(out,axis->Xyz()); gz::math::Vector3d resolvedAxis; require(axis->ResolveXyz(resolvedAxis).empty(),"joint axis resolve failed"); vector(out,resolvedAxis);
      out << ' ' << axis->Lower() << ' ' << axis->Upper() << ' ' << axis->Effort() << ' ' << axis->MaxVelocity()
          << ' ' << axis->Damping() << ' ' << axis->Friction() << ' ' << axis->SpringStiffness(); out << '\n';
    }
  }
  out << "GRAVITY"; vector(out,world->Gravity()); out << '\n'; observer->write(out); out.flush(); require(bool(out),"state output failed");
  std::ifstream maps("/proc/self/maps"); require(bool(maps),"maps unavailable");
  std::ofstream mapped(std::string(argv[2])+".maps"); mapped << maps.rdbuf(); mapped.flush(); require(bool(mapped),"maps output failed");
  std::ostringstream metrics;
  metrics << "{\"role\":\"gazebo-server\",\"mode\":\"" << mode << "\",\"steps\":100,\"dt_ns\":1000000,\"body_queries\":707,\"joint_queries\":404,\"window_wall_ns\":" << wall << ",\"scope\":\"diagnostic-observer-effect-check-required\",\"intrinsic_server_network\":\"present-unmatched\",\"worlds\":1,\"bodies\":7,\"joints\":4,\"colliders\":15,\"paused_preparation_calls\":1,\"sensors\":0,\"rendering\":false}\n";
  std::ofstream metricFile(std::string(argv[2])+".metrics.json");
  metricFile << metrics.str(); metricFile.flush(); require(bool(metricFile),"metrics output failed");
  std::cout << metrics.str();
}
