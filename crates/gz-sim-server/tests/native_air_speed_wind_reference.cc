// Copyright (C) 2023, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Fresh Sensors component / Math reference only. No Sim or Physics is linked.
// Formula and handoff: Sensors8ddf AirSpeedSensor.cc, Math1316 Quaternion.hh.
#include <gz/sensors/AirSpeedSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/air_speed.pb.h>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <array>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace {
using V=gz::math::Vector3d;
using Q=gz::math::Quaterniond;
using Message=gz::msgs::AirSpeed;
static_assert(sizeof(double)==sizeof(std::uint64_t)&&std::numeric_limits<double>::is_iec559);
void require(bool condition,const std::string &why){if(!condition)throw std::runtime_error(why);}
std::string bits(double value){std::uint64_t u;std::memcpy(&u,&value,sizeof(u));std::ostringstream s;s<<std::hex<<std::setw(16)<<std::setfill('0')<<u;return s.str();}
std::string hex(const std::string &bytes){constexpr char d[]="0123456789abcdef";std::string out;out.reserve(bytes.size()*2);for(unsigned char c:bytes){out+=d[c>>4];out+=d[c&15];}return out;}
void vectorBits(const V &v){std::cout<<"[\""<<bits(v.X())<<"\",\""<<bits(v.Y())<<"\",\""<<bits(v.Z())<<"\"]";}
void poseBits(const gz::math::Pose3d &p){std::cout<<"[\""<<bits(p.Pos().X())<<"\",\""<<bits(p.Pos().Y())<<"\",\""<<bits(p.Pos().Z())<<"\",\""<<bits(p.Rot().X())<<"\",\""<<bits(p.Rot().Y())<<"\",\""<<bits(p.Rot().Z())<<"\",\""<<bits(p.Rot().W())<<"\"]";}
long long nanos(std::chrono::steady_clock::duration t){return std::chrono::duration_cast<std::chrono::nanoseconds>(t).count();}
struct Config {std::string mode;V requested{1,-2,.5};bool present=true,noise=false;double mean=0,rate=0;unsigned instances=1;};
Config configuration(const std::string &mode){
  Config c;c.mode=mode;
  if(mode=="calm-omitted"){c.requested=V::Zero;c.present=false;}
  else if(mode=="calm-signed-zero")c.requested=V(-0.0,0.0,-0.0);
  else if(mode=="headwind")c.requested=V(4,0,0);
  else if(mode=="tailwind")c.requested=V(-1,0,0);
  else if(mode=="cancelling"||mode=="gaussian-cancel")c.requested=V(3,0,0);
  else if(mode=="crosswind")c.requested=V(0,4,5);
  else if(mode=="tilted-world-z")c.requested=V(0,0,2);
  else if(mode=="tilted-mixed-zero")c.requested=V(-0.0,2,-0.0);
  else if(mode=="float-pressure")c.requested=V(1,0,0);
  else require(mode=="reverse-flow"||mode=="changing-world-frame"||mode=="float-height"||mode=="gaussian-zero"||mode=="gaussian-mean"||mode=="rate100"||mode=="active100"||mode=="reload","known finite mode");
  if(mode=="gaussian-zero"||mode=="gaussian-mean"||mode=="gaussian-cancel"||mode=="reload"){c.noise=true;c.mean=mode=="gaussian-zero"?0:.1;}
  if(mode=="rate100"||mode=="active100")c.rate=100;
  if(mode=="reload")c.instances=2;
  return c;
}
V effective(const V &w){return w.X()==0&&w.Y()==0&&w.Z()==0?V::Zero:w;}
struct Call {long long time;gz::math::Pose3d world;V velocity;bool active=true;};
std::vector<Call> calls(const Config &c){
  std::vector<Call> result;
  const std::array<long long,6> rateTimes={1,2,9,10,11,20};
  const std::array<long long,7> activeTimes={1,2,9,10,10,11,20};
  const std::array<bool,7> active={false,true,true,false,true,true,true};
  const unsigned count=c.mode=="rate100"?6:c.mode=="active100"?7:5;
  const std::array<V,5> eulers={V(0,0,0),V(.2,-.3,.4),V(-.4,.1,-.6),V(.7,-.2,1.1),V(-.1,.5,-.8)};
  const std::array<double,5> heights={0,3.5,1000,-100,std::nextafter(3.5,std::numeric_limits<double>::infinity())};
  const std::array<double,5> xs={1,std::nextafter(1.0,std::numeric_limits<double>::infinity()),1+std::ldexp(1.0,-20),1+std::ldexp(1.0,-10),4.125};
  for(unsigned i=0;i<count;++i){
    Call row;row.time=(i+1)*1000000LL;row.world=gz::math::Pose3d(V(2,-1,3.5),Q::Identity);row.velocity=V(3.125,2,-4);
    if(c.mode=="headwind"||c.mode=="tailwind"||c.mode=="cancelling"||c.mode=="crosswind"||c.mode=="gaussian-cancel")row.velocity=V(3,0,0);
    if(c.mode=="tilted-world-z"||c.mode=="tilted-mixed-zero"||c.mode=="reverse-flow"){
      row.world.Rot()=Q(.2,.3,.6);row.velocity=c.mode=="reverse-flow"?V(-3,2,-.5):V(3,-1,.5);
    }
    if(c.mode=="changing-world-frame")row.world.Rot()=Q(eulers[i]);
    if(c.mode=="float-height")row.world.Pos().Z()=heights[i];
    if(c.mode=="float-pressure")row.velocity=V(xs[i],0,0);
    if(c.mode=="rate100")row.time=rateTimes[i]*1000000LL;
    if(c.mode=="active100"){row.time=activeTimes[i]*1000000LL;row.active=active[i];}
    result.push_back(row);
  }
  return result;
}
struct Received {std::mutex mutex;std::condition_variable changed;std::vector<Message> values;bool bad=false;std::string failure;};
void maps(const std::string &path){
  std::ifstream in("/proc/self/maps");std::ofstream out(path,std::ios::binary);require(in&&out,"live maps open");
  std::array<char,4096> buffer{};std::size_t total=0;
  while(in){in.read(buffer.data(),buffer.size());const auto n=in.gcount();require(n>=0&&total+static_cast<std::size_t>(n)<=4*1024*1024,"live maps cap");out.write(buffer.data(),n);total+=n;}
  require(in.eof()&&total>0,"live maps read");out.flush();require(static_cast<bool>(out),"live maps write");
}
void check(const Message &m,unsigned sequence,long long time){
  require(m.ByteSizeLong()<=8192&&m.has_header()&&m.header().has_stamp(),"bounded canonical header");
  const auto &t=m.header().stamp();require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000&&t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"stamp range");
  require(t.sec()*1000000000LL+t.nsec()==time,"actual acquisition time");
  const auto &h=m.header();require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)=="rig/body/air_speed"&&h.data(1).key()=="seq"&&h.data(1).value_size()==1&&h.data(1).value(0)==std::to_string(sequence),"actual frame/sequence header");
  require(std::isfinite(m.diff_pressure())&&std::isfinite(m.temperature())&&m.temperature()>0,"finite valid native readings");
}
void run(const Config &c,const std::string &mapsPath){
  bool attempted=false;const auto capture=[&]{if(!attempted){attempted=true;maps(mapsPath);}};
  unsigned totalCalls=0,acquired=0,suppressed=0;
  try{
    for(unsigned instance=0;instance<c.instances;++instance){
      // Reload deliberately changes the new component configuration as well as
      // its topic/lease: default instance0, deterministic Gaussian instance1.
      const bool noisy=c.noise&&(c.mode!="reload"||instance!=0);
      const double mean=noisy?c.mean:0.0;
      const std::string topic="/m41/native_air_speed_wind/"+c.mode+"/"+std::to_string(instance);
      std::ostringstream xml;xml<<std::setprecision(17)<<"<sdf version='1.12'><model name='rig'><link name='body'><sensor name='air_speed' type='air_speed'><topic>"<<topic<<"</topic><frame_id>rig/body/air_speed</frame_id><update_rate>"<<c.rate<<"</update_rate><air_speed>";
      if(noisy)xml<<"<pressure><noise type='gaussian'><mean>"<<mean<<"</mean><stddev>0</stddev></noise></pressure>";
      xml<<"</air_speed></sensor></link></model></sdf>";
      sdf::SDFPtr parsed(new sdf::SDF);sdf::init(parsed);require(sdf::readString(xml.str(),parsed),"finite component SDF");
      gz::sensors::SensorFactory factory;auto sensor=factory.CreateSensor<gz::sensors::AirSpeedSensor>(parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));require(static_cast<bool>(sensor),"actual AirSpeed SensorFactory");
      auto received=std::make_shared<Received>();auto live=std::make_shared<std::atomic<bool>>(true);gz::transport::Node node;
      std::function<void(const Message &)> callback=[received,live](const Message &m){
        std::lock_guard<std::mutex> lock(received->mutex);if(!live->load(std::memory_order_acquire)||received->bad)return;
        try{require(m.ByteSizeLong()<=8192&&received->values.size()<10,"callback cap before copy");received->values.push_back(m);}
        catch(const std::exception &e){received->bad=true;received->failure=e.what();}received->changed.notify_all();
      };
      try {
      require(node.Subscribe(topic,callback),"actual typed subscriber");
      const auto deadline=std::chrono::steady_clock::now()+5s;
      while(!sensor->HasConnections()){require(std::chrono::steady_clock::now()<deadline,"component connection deadline");std::this_thread::sleep_for(1ms);}
      const V wind=effective(c.requested);
      // Omitted means the component's own zero initialization, not a guessed setter.
      if(c.present)sensor->SetWindVelocity(wind);
      std::cout<<"{\"kind\":\"configuration\",\"mode\":\""<<c.mode<<"\",\"instance\":"<<instance<<",\"topic\":\""<<topic<<"\",\"frame_id\":\"rig/body/air_speed\",\"rate\":"<<c.rate<<",\"wind_present\":"<<(c.present?"true":"false")<<",\"requested_wind_bits\":";vectorBits(c.requested);
      std::cout<<",\"effective_wind_bits\":";vectorBits(wind);std::cout<<",\"noise_kind\":\""<<(noisy?"gaussian":"none")<<"\",\"noise_mean_bits\":\""<<bits(mean)<<"\",\"noise_stddev_bits\":\""<<bits(0)<<"\",\"noise_bias_mean_bits\":\""<<bits(0)<<"\",\"noise_bias_stddev_bits\":\""<<bits(0)<<"\",\"noise_dynamic_bias_stddev_bits\":\""<<bits(0)<<"\",\"noise_dynamic_bias_correlation_time_bits\":\""<<bits(0)<<"\",\"noise_precision_bits\":\""<<bits(0)<<"\"}\n";
      unsigned sequence=0;const auto inputs=calls(c);
      for(unsigned index=0;index<inputs.size();++index){const auto &r=inputs[index];
        sensor->SetPose(r.world);sensor->SetVelocity(r.velocity);sensor->SetActive(r.active);
        const auto before=nanos(sensor->NextDataUpdateTime());const bool connected=sensor->HasConnections();require(connected,"connection retained during component calls");
        const bool updated=sensor->Update(std::chrono::nanoseconds(r.time),false);const auto after=nanos(sensor->NextDataUpdateTime());
        std::optional<Message> message;
        {std::unique_lock<std::mutex> lock(received->mutex);
          if(updated){require(received->changed.wait_for(lock,5s,[&]{return received->bad||received->values.size()>=sequence+1;}),"actual canonical callback deadline");}
          require(!received->bad,"callback failure: "+received->failure);require(received->values.size()==sequence+unsigned(updated),"exact actual callback count");
          if(updated)message=received->values[sequence];}
        if(message){check(*message,sequence,r.time);++sequence;++acquired;}else ++suppressed;
        ++totalCalls;
        const V rotated=r.world.Rot().RotateVectorReverse(wind);const V relative=r.velocity-rotated;
        std::cout<<"{\"kind\":\"call\",\"mode\":\""<<c.mode<<"\",\"instance\":"<<instance<<",\"index\":"<<index<<",\"time_ns\":"<<r.time<<",\"active\":"<<(r.active?"true":"false")<<",\"force\":false,\"has_connections\":true,\"updated\":"<<(updated?"true":"false")<<",\"next_before_ns\":"<<before<<",\"next_after_ns\":"<<after<<",\"world_pose_bits\":";poseBits(r.world);
        std::cout<<",\"velocity_sensor_bits\":";vectorBits(r.velocity);std::cout<<",\"native_math_wind_sensor_bits\":";vectorBits(rotated);std::cout<<",\"native_math_relative_air_bits\":";vectorBits(relative);
        if(message){std::cout<<",\"sequence\":\""<<message->header().data(1).value(0)<<"\""<<",\"pressure_bits\":\""<<bits(message->diff_pressure())<<"\",\"temperature_bits\":\""<<bits(message->temperature())<<"\",\"header_wire\":\""<<hex(message->header().SerializeAsString())<<"\",\"wire\":\""<<hex(message->SerializeAsString())<<"\"";}
        else std::cout<<",\"sequence\":null,\"pressure_bits\":null,\"temperature_bits\":null,\"header_wire\":null,\"wire\":null";
        std::cout<<"}\n";
      }
      const unsigned expected=c.mode=="rate100"||c.mode=="active100"?3:5;require(sequence==expected,"exact per-instance acquisition count");
      {std::lock_guard<std::mutex> lock(received->mutex);require(!received->bad&&received->values.size()==expected,"closing bounded callback inventory");}
      capture();
      {std::lock_guard<std::mutex> lock(received->mutex);require(!received->bad&&received->values.size()==expected,"final callback inventory before retirement");live->store(false,std::memory_order_release);}
      require(node.Unsubscribe(topic),"fresh subscriber retirement");
      } catch(...) {
        const auto original=std::current_exception();
        {std::lock_guard<std::mutex> lock(received->mutex);live->store(false,std::memory_order_release);}
        try{capture();}catch(const std::exception &e){std::cerr<<"live maps capture while component alive: "<<e.what()<<'\n';}
        std::rethrow_exception(original);
      }
    }
    const unsigned expectedCalls=c.mode=="rate100"?6:c.mode=="active100"?7:5*c.instances;
    const unsigned expectedAcquired=c.mode=="rate100"||c.mode=="active100"?3:5*c.instances;
    require(totalCalls==expectedCalls&&acquired==expectedAcquired&&suppressed==expectedCalls-expectedAcquired,"exact mode closing counts");
    std::cout<<"{\"kind\":\"summary\",\"mode\":\""<<c.mode<<"\",\"instances\":"<<c.instances<<",\"input_calls\":"<<totalCalls<<",\"observations\":"<<acquired<<",\"suppressed_calls\":"<<suppressed<<"}\n";
  }catch(...){const auto error=std::current_exception();try{capture();}catch(const std::exception &e){std::cerr<<"live maps capture: "<<e.what()<<'\n';}std::rethrow_exception(error);}
}
}
int main(int argc,char **argv){try{require(argc==3,"expected MODE MAPS_PATH");run(configuration(argv[1]),argv[2]);return 0;}catch(const std::exception &e){std::cerr<<"native AirSpeed wind component: "<<e.what()<<'\n';return 1;}}
