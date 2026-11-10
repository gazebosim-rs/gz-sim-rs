// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Default: transport-only 3D peer. Explicit oracle macro loads the original
// system DSO against finite MODEL-local ECM poses; no native Physics is run.
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <array>
#include <fcntl.h>
#include <unistd.h>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace {
void require(bool condition,const std::string &why) {if(!condition)throw std::runtime_error(why);}
long long stamp(const gz::msgs::Header &h) {
  require(h.has_stamp()&&h.stamp().sec()>=0&&h.stamp().nsec()>=0&&h.stamp().nsec()<1000000000,"stamp");
  require(h.stamp().sec()<=(LLONG_MAX-h.stamp().nsec())/1000000000LL,"stamp nanosecond overflow");
  return h.stamp().sec()*1000000000LL+h.stamp().nsec();
}
void header(const gz::msgs::Header &h) {
  require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)=="m38/odom"
    &&h.data(1).key()=="child_frame_id"&&h.data(1).value_size()==1&&h.data(1).value(0)=="m38/base","frame headers / no seq");
  (void)stamp(h);
}
void vectorFinite(const gz::msgs::Vector3d &v) {require(std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z()),"finite vector");}
void poseFinite(const gz::msgs::Pose &p) {
  require(p.has_position()&&p.has_orientation(),"pose fields");vectorFinite(p.position());
  const auto &q=p.orientation();require(std::isfinite(q.x())&&std::isfinite(q.y())&&std::isfinite(q.z())&&std::isfinite(q.w())
    &&std::abs(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w()-1.0)<=1e-12,"finite unit quaternion");
}
struct Triple {gz::msgs::Odometry odom;gz::msgs::OdometryWithCovariance cov;gz::msgs::Pose_V tf;bool o=false,c=false,t=false;};
struct Observed {std::mutex mutex;std::condition_variable changed;std::map<long long,Triple> rows;bool bad=false;bool haveClock=false;long long clockNs=0;};
struct Subscribers {
  gz::transport::Node node;
  std::shared_ptr<Observed> seen=std::make_shared<Observed>();
  // This subscription survives GT disconnects on the same owned Node/session.
  void startClock() {
    std::function<void(const gz::msgs::Clock &)> callback=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(!s->bad,"clock callback after retained failure");
      require(m.has_sim()&&m.sim().sec()>=0&&m.sim().nsec()>=0&&m.sim().nsec()<1000000000,"clock simulation stamp");
      require(m.sim().sec()<=(LLONG_MAX-m.sim().nsec())/1000000000LL,"clock nanosecond overflow");
      s->clockNs=m.sim().sec()*1000000000LL+m.sim().nsec();s->haveClock=true;
      }catch(...){s->bad=true;}s->changed.notify_all();};
    require(node.Subscribe("/world/default/clock",callback),"canonical Clock subscription");
  }
  void start() {
    std::function<void(const gz::msgs::Odometry &)> odom=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(!s->bad,"callback after retained failure");
      require(m.has_header(),"odom header");const auto n=stamp(m.header());require(s->rows.size()<=256&&(s->rows.find(n)!=s->rows.end()||s->rows.size()<256),"callback cap before insertion");auto &r=s->rows[n];require(!r.o,"duplicate odom");r.odom=m;r.o=true;
      }catch(...){s->bad=true;}s->changed.notify_all();};
    std::function<void(const gz::msgs::OdometryWithCovariance &)> cov=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(!s->bad,"callback after retained failure");
      require(m.has_header(),"cov header");const auto n=stamp(m.header());require(s->rows.size()<=256&&(s->rows.find(n)!=s->rows.end()||s->rows.size()<256),"callback cap before insertion");auto &r=s->rows[n];require(!r.c,"duplicate covariance");r.cov=m;r.c=true;
      }catch(...){s->bad=true;}s->changed.notify_all();};
    std::function<void(const gz::msgs::Pose_V &)> tf=[s=seen](const auto &m){std::lock_guard<std::mutex> lock(s->mutex);try {
      require(!s->bad,"callback after retained failure");
      require(!m.has_header()&&m.pose_size()==1&&m.pose(0).has_header(),"transform envelope");const auto n=stamp(m.pose(0).header());require(s->rows.size()<=256&&(s->rows.find(n)!=s->rows.end()||s->rows.size()<256),"callback cap before insertion");auto &r=s->rows[n];require(!r.t,"duplicate transform");r.tf=m;r.t=true;
      }catch(...){s->bad=true;}s->changed.notify_all();};
    require(node.Subscribe("/m38/odometry",odom)&&node.Subscribe("/m38/odometry_covariance",cov)&&node.Subscribe("/m38/transform",tf),"subscribe three canonical types");
  }
  void ready(bool includeClock=true) {
    const auto deadline=std::chrono::steady_clock::now()+5s;
    for(;;) {
      bool all=true;const std::vector<std::pair<std::string,std::string>> topics={{"/world/default/clock","gz.msgs.Clock"},{"/m38/odometry","gz.msgs.Odometry"},{"/m38/odometry_covariance","gz.msgs.OdometryWithCovariance"},{"/m38/transform","gz.msgs.Pose_V"}};
      for(const auto &entry:topics) {if(!includeClock&&entry.first=="/world/default/clock")continue;std::vector<gz::transport::MessagePublisher> pubs,subs;node.TopicInfo(entry.first,pubs,subs);
        all=all&&pubs.size()==1&&subs.size()==1&&pubs[0].MsgTypeName()==entry.second&&subs[0].MsgTypeName()==entry.second;}
      if (all) {
        return;
      }
      require(std::chrono::steady_clock::now()<deadline,"three typed provider/consumer readiness deadline");
      std::this_thread::yield();
    }
  }
  void drain(std::size_t count) {
    std::unique_lock<std::mutex> lock(seen->mutex);
    require(seen->changed.wait_for(lock,5s,[&]{if(seen->bad)return true;if(seen->rows.size()!=count)return false;for(const auto &r:seen->rows)if(!r.second.o||!r.second.c||!r.second.t)return false;return true;}),"three-channel callback completion deadline");
    require(!seen->bad&&seen->rows.size()==count,"exact callback count");
  }
};
void validate(const Triple &r) {
  require(r.o&&r.c&&r.t,"all channel receipts");header(r.odom.header());poseFinite(r.odom.pose());
  require(r.odom.has_twist(),"3D pose/twist");
  vectorFinite(r.odom.twist().linear());vectorFinite(r.odom.twist().angular());
  require(r.cov.header().SerializeAsString()==r.odom.header().SerializeAsString()&&r.tf.pose(0).header().SerializeAsString()==r.odom.header().SerializeAsString(),"equal acquisition headers");
  require(r.cov.has_pose_with_covariance()&&r.cov.has_twist_with_covariance(),"covariance wrappers");
  require(r.cov.pose_with_covariance().pose().SerializeAsString()==r.odom.pose().SerializeAsString()
    &&r.cov.twist_with_covariance().twist().SerializeAsString()==r.odom.twist().SerializeAsString(),"equal three-channel values");
  const auto &a=r.cov.pose_with_covariance().covariance();const auto &b=r.cov.twist_with_covariance().covariance();
  require(a.data_size()==36&&b.data_size()==36,"covariance dimensions");for(int i=0;i<36;++i)require(a.data(i)==0.0f&&b.data(i)==0.0f,"noiseless covariance");
  auto tf=r.tf.pose(0);tf.clear_header();require(tf.SerializeAsString()==r.odom.pose().SerializeAsString(),"transform pose / default name-id");
}
#ifndef GZ_SIM_NATIVE_ODOMETRY_3D_ORACLE
gz::msgs::Entity target(std::uint64_t id) {
  gz::msgs::Entity value;value.set_id(id);value.set_type(gz::msgs::Entity::LINK);return value;
}
gz::msgs::EntityWrench wrench(std::uint64_t id,const std::array<double,3> &force,const std::array<double,3> &torque) {
  gz::msgs::EntityWrench value;*value.mutable_entity()=target(id);
  auto *f=value.mutable_wrench()->mutable_force();f->set_x(force[0]);f->set_y(force[1]);f->set_z(force[2]);
  auto *t=value.mutable_wrench()->mutable_torque();t->set_x(torque[0]);t->set_y(torque[1]);t->set_z(torque[2]);return value;
}
void inputReady(const std::vector<gz::transport::Node::Publisher *> &publishers) {
  const auto deadline=std::chrono::steady_clock::now()+5s;
  for(;;) {
    bool all=true;for(const auto *p:publishers)all=all&&p->HasConnections();
    if(all)return;
    require(std::chrono::steady_clock::now()<deadline,"three input connections deadline");
    std::this_thread::yield();
  }
}
template<class T> void packet(const char *channel,const T &message) {
  const auto bytes=message.SerializeAsString();require(bytes.size()<=8192,"canonical packet cap");
  constexpr char digits[]="0123456789abcdef";std::string hex;hex.reserve(bytes.size()*2);
  for(unsigned char b:bytes){hex+=digits[b>>4];hex+=digits[b&15];}
  std::cout<<"PACKET "<<channel<<' '<<hex<<'\n';
}
#endif
}
#ifndef GZ_SIM_NATIVE_ODOMETRY_3D_ORACLE
int main(int argc,char **argv) {
  try {
    require(argc==2,"expected actual public link ID");
    const std::string argument=argv[1];std::size_t used=0;const auto id=std::stoull(argument,&used);
    require(id!=0&&used==argument.size()&&argument.find_first_not_of("0123456789")==std::string::npos,"invalid public link ID");
    Subscribers subscriptions;subscriptions.startClock();subscriptions.start();subscriptions.ready();bool watching=true;
    gz::transport::Node inputs;
    auto once=inputs.Advertise<gz::msgs::EntityWrench>("/world/default/wrench");
    auto persistent=inputs.Advertise<gz::msgs::EntityWrench>("/world/default/wrench/persistent");
    auto clear=inputs.Advertise<gz::msgs::Entity>("/world/default/wrench/clear");
    require(once&&persistent&&clear,"canonical input publishers");inputReady({&once,&persistent,&clear});
    std::cout<<"READY three-dimensional"<<std::endl;
    for(unsigned phase=0;phase<128;++phase) {
      char line[65]{};require(static_cast<bool>(std::cin.getline(line,sizeof(line))),"bounded phase input/EOF");
      const std::string command=line;
      if(command=="once-force"||command=="once-torque") {
        const bool force=command=="once-force";
        require(once.Publish(wrench(id,force?std::array<double,3>{4,2,6}:std::array<double,3>{0,0,0},force?std::array<double,3>{0,0,0}:std::array<double,3>{2,4,6})),"one-shot XYZ publish");
      } else if(command=="persistent-force"||command=="persistent-torque") {
        const bool force=command=="persistent-force";
        require(persistent.Publish(wrench(id,force?std::array<double,3>{1,.5,1.5}:std::array<double,3>{0,0,0},force?std::array<double,3>{0,0,0}:std::array<double,3>{2,1,3}))
          &&persistent.Publish(wrench(id,force?std::array<double,3>{3,1.5,4.5}:std::array<double,3>{0,0,0},force?std::array<double,3>{0,0,0}:std::array<double,3>{4,2,3})),"two additive XYZ publishes");
      } else if(command=="clear") {
        require(clear.Publish(target(id)),"clear publish");
      } else if(command.rfind("drain ",0)==0) {
        const auto number=command.substr(6);std::size_t consumed=0;const auto count=std::stoul(number,&consumed);
        require(watching&&consumed==number.size()&&count<=4,"finite drain count");
        subscriptions.drain(count);
        std::lock_guard<std::mutex> lock(subscriptions.seen->mutex);
        require(!subscriptions.seen->bad&&subscriptions.seen->rows.size()==count,"completed callback set");
        for(const auto &entry:subscriptions.seen->rows) {
          validate(entry.second);
          packet("odometry",entry.second.odom);packet("covariance",entry.second.cov);packet("transform",entry.second.tf);
        }
        subscriptions.seen->rows.clear();
        std::cout<<"DRAINED "<<count<<std::endl;continue;
      } else if(command=="drop") {
        require(watching,"already disconnected");
        {std::lock_guard<std::mutex> lock(subscriptions.seen->mutex);require(!subscriptions.seen->bad&&subscriptions.seen->rows.empty(),"drop before complete receipt drain");}
        require(subscriptions.node.Unsubscribe("/m38/odometry")&&subscriptions.node.Unsubscribe("/m38/odometry_covariance")&&subscriptions.node.Unsubscribe("/m38/transform"),"drop all three subscriptions");
        watching=false;std::cout<<"DROPPED"<<std::endl;continue;
      } else if(command=="clock-101") {
        require(!watching,"Clock fence only while GT is disconnected");
        std::unique_lock<std::mutex> lock(subscriptions.seen->mutex);
        require(subscriptions.seen->changed.wait_for(lock,5s,[&]{return subscriptions.seen->bad||(subscriptions.seen->haveClock&&subscriptions.seen->clockNs==101000000LL);}),"exact 101 ms Clock deadline");
        require(!subscriptions.seen->bad&&subscriptions.seen->haveClock&&subscriptions.seen->clockNs==101000000LL,"actual canonical Clock fence");
        std::cout<<"CLOCK 101000000"<<std::endl;continue;
      } else if(command=="watch") {
        require(!watching,"duplicate subscriptions");subscriptions.start();subscriptions.ready();inputReady({&once,&persistent,&clear});
        watching=true;std::cout<<"WATCHING"<<std::endl;continue;
      } else if(command=="retired") {
        require(!watching,"retirement requires drained/dropped subscriptions");
        const auto deadline=std::chrono::steady_clock::now()+5s;
        for(;;) {
          bool gone=!once.HasConnections()&&!persistent.HasConnections()&&!clear.HasConnections();
          for(const auto *topic:{"/m38/odometry","/m38/odometry_covariance","/m38/transform"}) {
            std::vector<gz::transport::MessagePublisher> pubs,subs;subscriptions.node.TopicInfo(topic,pubs,subs);gone=gone&&pubs.empty();
          }
          if(gone)break;
          require(std::chrono::steady_clock::now()<deadline,"owned endpoint retirement deadline");std::this_thread::yield();
        }
        std::cout<<"RETIRED"<<std::endl;continue;
      } else if(command=="finish") {
        require(!watching,"drop before finish");
        std::lock_guard<std::mutex> lock(subscriptions.seen->mutex);require(!subscriptions.seen->bad&&subscriptions.seen->rows.empty(),"unconsumed callbacks at finish");
        std::cout<<"PASS three-dimensional odometry"<<std::endl;return 0;
      } else {throw std::runtime_error("unknown finite phase command");}
      std::cout<<"SENT "<<command<<std::endl;
    }
    throw std::runtime_error("finite phase count cap");
  }catch(const std::exception &error){std::cerr<<"native three-dimensional peer: "<<error.what()<<'\n';return 1;}
}

#else
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Plugin.hh>
namespace {
void poseJson(const gz::math::Pose3d &p) {
  std::cout<<'['<<p.Pos().X()<<','<<p.Pos().Y()<<','<<p.Pos().Z()<<','<<p.Rot().X()<<','<<p.Rot().Y()<<','<<p.Rot().Z()<<','<<p.Rot().W()<<']';
}
void maps(const std::string &path) {
  std::ifstream input("/proc/self/maps");require(bool(input),"actual maps open");
  std::string data,line;while(std::getline(input,line)){require(data.size()+line.size()+1<=4*1024*1024,"maps cap");data+=line;data+='\n';}
  require(input.eof(),"actual maps read");const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);require(fd>=0,"exclusive ordinary maps output");
  std::size_t offset=0;while(offset<data.size()){const auto n=::write(fd,data.data()+offset,data.size()-offset);if(n<=0){::close(fd);throw std::runtime_error("actual maps write");}offset+=static_cast<std::size_t>(n);}
  const bool synced=::fsync(fd)==0;const bool closed=::close(fd)==0;require(synced&&closed,"maps flush/close");
}
std::string hex(const std::string &bytes) {
  constexpr char digits[]="0123456789abcdef";std::string out;out.reserve(bytes.size()*2);
  for(unsigned char c:bytes){out+=digits[c>>4];out+=digits[c&15];}return out;
}
gz::math::Pose3d inputPose(const std::string &mode,unsigned tick,long long ns) {
  const double t=ns*1e-9;
  if(mode=="identity")return gz::math::Pose3d(tick%2?-0.0:0.0,0.0,tick%2?-0.0:0.0,0,0,0);
  if(mode=="stationary_tilted"||mode=="reload")return gz::math::Pose3d(2,-1,3,.2,.3,.6);
  if(mode=="translate_x")return gz::math::Pose3d(t+t*t,0,0,0,0,0);
  if(mode=="translate_y")return gz::math::Pose3d(0,.5*t+t*t,0,0,0,0);
  if(mode=="translate_z"||mode=="micro")return gz::math::Pose3d(0,0,2*t+t*t,0,0,0);
  if(mode=="rotate_roll")return gz::math::Pose3d(2,-1,3,.2+.8*t,0,0);
  if(mode=="rotate_pitch")return gz::math::Pose3d(2,-1,3,0,.3+.6*t,0);
  if(mode=="rotate_yaw")return gz::math::Pose3d(2,-1,3,0,0,.6+.4*t);
  if(mode=="angular_threshold") {
    gz::math::Pose3d pose=gz::math::Pose3d::Zero;
    if(tick%2){const std::array<double,4> xs={std::nextafter(1e-6,0.0),1e-6,std::nextafter(1e-6,INFINITY),2e-6};const double x=xs[((tick-1)/2)%xs.size()];pose.Rot()=gz::math::Quaterniond(std::sqrt(1-x*x),x,0,0);}return pose;
  }
  gz::math::Pose3d pose(2+t+t*t,-1+.5*t,3+2*t,.2+.4*t,.3-.2*t,.6+.8*t);
  if(mode=="quaternion_sign"&&tick%2)pose.Rot()=-pose.Rot();return pose;
}
void rawInput(const std::string &mode,unsigned generation,unsigned tick,long long ns,long long dt,bool paused,const gz::math::Pose3d &pose) {
  std::cout<<"{\"kind\":\"input\",\"mode\":"<<std::quoted(mode)<<",\"generation\":"<<generation<<",\"tick\":"<<tick
    <<",\"time_ns\":"<<ns<<",\"dt_ns\":"<<dt<<",\"paused\":"<<(paused?"true":"false")<<",\"pose\":";poseJson(pose);std::cout<<"}\n";
}
}
int main(int argc,char **argv) {
  bool captured=false;
  try {
    require(argc==4,"oracle ORIGINAL_DSO MODE MAPS_PATH");const std::string mode=argv[2];
    const std::vector<std::string> modes={"identity","stationary_tilted","translate_x","translate_y","translate_z","rotate_roll","rotate_pitch","rotate_yaw","tilted_screw","quaternion_sign","angular_threshold","rate","micro","pause_equal","reload"};
    require(std::find(modes.begin(),modes.end(),mode)!=modes.end(),"declared finite oracle mode");
    const auto elapsed=[](long long ns){return std::chrono::duration<double>(std::chrono::nanoseconds(ns)).count();};
    require(elapsed(1000)<=1e-6&&elapsed(1500)>1e-6,"inclusive original microsecond boundary");std::cout<<std::setprecision(17);
    unsigned inputs=0,acquisitions=0;
    for(unsigned generation=0;generation<(mode=="reload"?2u:1u);++generation) {
      const bool rate=mode=="rate";const long long step=mode=="micro"?500:1000000;
      const unsigned last=mode=="reload"?16:mode=="rate"?40:mode=="micro"?34:32;
      sdf::Plugin config(argv[1],"gz::sim::systems::OdometryPublisher");
      require(config.InsertContent("<dimensions>3</dimensions><odom_publish_frequency>"+std::string(rate?"50":"0")+"</odom_publish_frequency><odom_topic>/m38/odometry</odom_topic><odom_covariance_topic>/m38/odometry_covariance</odom_covariance_topic><tf_topic>/m38/transform</tf_topic><odom_frame>m38/odom</odom_frame><robot_base_frame>m38/base</robot_base_frame>"),"original3D config");
      gz::sim::SystemLoader loader;auto loaded=loader.LoadPlugin(config);require(bool(loaded),"exact original system plugin load");
      auto *configure=(*loaded)->QueryInterface<gz::sim::ISystemConfigure>();auto *pre=(*loaded)->QueryInterface<gz::sim::ISystemPreUpdate>();auto *post=(*loaded)->QueryInterface<gz::sim::ISystemPostUpdate>();
      require(configure&&pre&&post,"original system interfaces");
      gz::sim::EntityComponentManager ecm;gz::sim::EventManager events;
      const auto world=ecm.CreateEntity();ecm.CreateComponent(world,gz::sim::components::World());ecm.CreateComponent(world,gz::sim::components::Name("default"));ecm.CreateComponent(world,gz::sim::components::Pose(gz::math::Pose3d::Zero));
      const auto model=ecm.CreateEntity();ecm.CreateComponent(model,gz::sim::components::Model());ecm.CreateComponent(model,gz::sim::components::Name("rig"));ecm.CreateComponent(model,gz::sim::components::ParentEntity(world));ecm.CreateComponent(model,gz::sim::components::Pose(gz::math::Pose3d::Zero));
      configure->Configure(model,config.ToElement(),ecm,events);
      Subscribers subscriptions;subscriptions.start();subscriptions.ready(false);
      long long history=-1,lastPub=0;std::map<long long,gz::math::Pose3d> expected;
      const auto call=[&](unsigned tick,long long ns,long long dt,bool paused,const gz::math::Pose3d &pose){
        ecm.SetComponentData<gz::sim::components::Pose>(model,pose);gz::sim::UpdateInfo info;info.simTime=std::chrono::nanoseconds(ns);info.dt=std::chrono::nanoseconds(dt);info.iterations=tick;info.paused=paused;
        pre->PreUpdate(info,ecm);post->PostUpdate(info,ecm);rawInput(mode,generation,tick,ns,dt,paused,pose);++inputs;
        if(!captured){maps(argv[3]);captured=true;}
        if(paused)return;if(history<0){history=ns;return;}if(elapsed(ns-history)<=1e-6)return;history=ns;
        if(!rate||ns-lastPub>=20000000){lastPub=ns;require(expected.emplace(ns,pose).second,"duplicate expected acquisition");}
      };
      for(unsigned tick=0;tick<=last;++tick){const long long ns=static_cast<long long>(tick)*step;const auto pose=inputPose(mode,tick,ns);
        if(mode=="pause_equal"&&(tick==0||tick==10))call(tick,ns,0,true,pose);
        call(tick,ns,tick?step:0,false,pose);
        if(mode=="pause_equal"&&(tick==8||tick==24))call(tick,ns,0,false,pose);
      }
      subscriptions.drain(expected.size());std::lock_guard<std::mutex> lock(subscriptions.seen->mutex);
      for(const auto &entry:subscriptions.seen->rows){validate(entry.second);const auto found=expected.find(entry.first);require(found!=expected.end(),"unexpected actual acquisition stamp");const auto &r=entry.second;const auto &p=r.odom.pose();const auto &q=p.orientation();const auto &l=r.odom.twist().linear();const auto &a=r.odom.twist().angular();
        std::cout<<"{\"kind\":\"acquisition\",\"mode\":"<<std::quoted(mode)<<",\"generation\":"<<generation<<",\"time_ns\":"<<entry.first<<",\"input\":";poseJson(found->second);
        std::cout<<",\"pose\":["<<p.position().x()<<','<<p.position().y()<<','<<p.position().z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<"],\"linear\":["<<l.x()<<','<<l.y()<<','<<l.z()<<"],\"angular\":["<<a.x()<<','<<a.y()<<','<<a.z()<<']';
        std::cout<<",\"odometry_hex\":"<<std::quoted(hex(r.odom.SerializeAsString()))<<",\"covariance_hex\":"<<std::quoted(hex(r.cov.SerializeAsString()))<<",\"transform_hex\":"<<std::quoted(hex(r.tf.SerializeAsString()))<<"}\n";++acquisitions;
      }
    }
    const unsigned wantedInputs=mode=="rate"?41:mode=="micro"?35:mode=="pause_equal"?37:mode=="reload"?34:33;
    const unsigned wantedAcquisitions=mode=="rate"?2:mode=="micro"?11:32;
    require(inputs==wantedInputs&&acquisitions==wantedAcquisitions&&captured,"complete finite original3D capture");return 0;
  }catch(const std::exception &error){if(argc==4&&!captured){try{maps(argv[3]);}catch(...){}}std::cerr<<"original3D odometry: "<<error.what()<<'\n';return 1;}
}
#endif
