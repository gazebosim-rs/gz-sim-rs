// Copyright (C) 2023, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Component-only Sensors8ddf / Math1316 reference. No Sim/System/Physics run.
#include <gz/common/Console.hh>
#include <gz/sensors/ImuSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/imu.pb.h>
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
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace {
using V = gz::math::Vector3d;
using Q = gz::math::Quaterniond;
using Message = gz::msgs::IMU;
static_assert(sizeof(double)==sizeof(std::uint64_t) && std::numeric_limits<double>::is_iec559);
static_assert(sizeof(float)==sizeof(std::uint32_t) && std::numeric_limits<float>::is_iec559);
void require(bool value,const std::string &why){if(!value)throw std::runtime_error(why);}
std::string bits(double value){std::uint64_t u;std::memcpy(&u,&value,sizeof u);std::ostringstream s;s<<std::hex<<std::setw(16)<<std::setfill('0')<<u;return s.str();}
std::string bits32(float value){std::uint32_t u;std::memcpy(&u,&value,sizeof u);std::ostringstream s;s<<std::hex<<std::setw(8)<<std::setfill('0')<<u;return s.str();}
std::string hex(const std::string &bytes){constexpr char d[]="0123456789abcdef";std::string out;out.reserve(bytes.size()*2);for(unsigned char c:bytes){out+=d[c>>4];out+=d[c&15];}return out;}
void vectorBits(const V &v){std::cout<<"[\""<<bits(v.X())<<"\",\""<<bits(v.Y())<<"\",\""<<bits(v.Z())<<"\"]";}
void quaternionBits(const Q &q){std::cout<<"[\""<<bits(q.X())<<"\",\""<<bits(q.Y())<<"\",\""<<bits(q.Z())<<"\",\""<<bits(q.W())<<"\"]";}
void poseBits(const gz::math::Pose3d &p){std::cout<<"[\""<<bits(p.Pos().X())<<"\",\""<<bits(p.Pos().Y())<<"\",\""<<bits(p.Pos().Z())<<"\",\""<<bits(p.Rot().X())<<"\",\""<<bits(p.Rot().Y())<<"\",\""<<bits(p.Rot().Z())<<"\",\""<<bits(p.Rot().W())<<"\"]";}
long long nanos(std::chrono::steady_clock::duration time){return std::chrono::duration_cast<std::chrono::nanoseconds>(time).count();}
const std::array<std::string,17> modes={
  "custom-motion","enu-motion","ned-motion","nwu-motion",
  "custom-sign","enu-sign","ned-sign","nwu-sign",
  "custom-disabled","enu-disabled","ned-disabled","nwu-disabled",
  "custom-mean","enu-mean","ned-mean","nwu-mean","custom-explicit-motion"};
const std::array<V,6> bodies={V(.2,-.3,.6),V(.25,-.28,.64),V(-.15,.35,-.4),V(.4,-.2,.9),V(-.3,.15,-.7),V(.1,.45,1.1)};
const std::array<V,6> accelerations={V(.7,-1.1,2.3),V(-.4,.8,-1.2),V(1.5,.25,-.75),V(-1.3,2.1,.6),V(.05,-.2,1.7),V(2.2,-1.6,.9)};
const std::array<V,6> angular={V(.3,-.5,.8),V(-.2,.4,-.6),V(.7,.1,-.3),V(-.8,.6,.2),V(.15,-.25,.35),V(.4,-.7,.9)};
const std::array<long long,6> times={1000000,2000000,5000000,10000000,20000000,30000000};
const V localRpy(-.4,.1,.25),gravity(0,0,-9.81);
const std::array<double,6> means={.1,-.2,.3,-.04,.05,-.06};
struct Config{std::string mode,localization;bool sign=false,enabled=true,noise=false,referenceTag=false,worldFrameCall=false;};
Config configuration(const std::string &mode){
  bool known=false;for(const auto &entry:modes)known=known||entry==mode;require(known,"known finite mode");
  Config c;c.mode=mode;c.localization=mode.rfind("enu-",0)==0?"ENU":mode.rfind("ned-",0)==0?"NED":mode.rfind("nwu-",0)==0?"NWU":"CUSTOM";
  c.sign=mode.find("-sign")!=std::string::npos;c.enabled=mode.find("-disabled")==std::string::npos;c.noise=mode.find("-mean")!=std::string::npos;
  c.referenceTag=c.localization!="CUSTOM"||mode=="custom-explicit-motion";c.worldFrameCall=c.referenceTag;return c;
}
std::string xml(const Config &c,const std::string &topic){
  std::ostringstream out;out<<std::setprecision(17)<<"<sdf version='1.12'><model name='rig'><link name='body'><sensor name='imu' type='imu'><topic>"<<topic<<"</topic><frame_id>m43/native/imu</frame_id><update_rate>0</update_rate><always_on>true</always_on><imu><enable_orientation>"<<(c.enabled?"true":"false")<<"</enable_orientation>";
  if(c.referenceTag){out<<"<orientation_reference_frame><localization>"<<c.localization<<"</localization>";if(c.localization=="CUSTOM")out<<"<custom_rpy parent_frame=''>0 0 0</custom_rpy>";out<<"</orientation_reference_frame>";}
  if(c.noise){for(unsigned group=0;group<2;++group){out<<(group==0?"<linear_acceleration>":"<angular_velocity>");for(unsigned axis=0;axis<3;++axis){const char name="xyz"[axis];out<<'<'<<name<<"><noise type='gaussian'><mean>"<<means[group*3+axis]<<"</mean><stddev>0</stddev><bias_mean>0</bias_mean><bias_stddev>0</bias_stddev><dynamic_bias_stddev>0</dynamic_bias_stddev><dynamic_bias_correlation_time>0</dynamic_bias_correlation_time><precision>0</precision></noise></"<<name<<'>'; }out<<(group==0?"</linear_acceleration>":"</angular_velocity>");}}
  out<<"</imu></sensor></link></model></sdf>";return out.str();
}
struct Received{std::mutex mutex;std::condition_variable changed;std::vector<Message> messages;bool bad=false;std::string failure;};
void maps(const std::string &path){
  std::ifstream in("/proc/self/maps");std::ofstream out(path,std::ios::binary);require(in&&out,"live maps open");std::array<char,4096> buffer{};std::size_t total=0;
  while(in){in.read(buffer.data(),buffer.size());const auto n=in.gcount();require(n>=0&&total+static_cast<std::size_t>(n)<=4*1024*1024,"live maps cap");out.write(buffer.data(),n);total+=n;}
  require(in.eof()&&total>0,"live maps read");out.flush();require(static_cast<bool>(out),"live maps write");
}
void finiteVector(const gz::msgs::Vector3d &v){require(std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z()),"finite native vector");}
template<class C> void covarianceBits(const C &c){require(c.data_size()==9,"covariance shape");std::cout<<'[';for(int i=0;i<9;++i){require(std::isfinite(c.data(i))&&c.data(i)==0,"deterministic zero variance");if(i)std::cout<<',';std::cout<<'"'<<bits32(c.data(i))<<'"';}std::cout<<']';}
void check(const Message &m,unsigned sequence,long long time,bool enabled){
  require(m.ByteSizeLong()<=8192&&m.has_header()&&m.header().has_stamp(),"bounded native canonical header");const auto &t=m.header().stamp();
  require(t.sec()>=0&&t.nsec()>=0&&t.nsec()<1000000000&&t.sec()<=(LLONG_MAX-t.nsec())/1000000000LL,"stamp bounds");require(t.sec()*1000000000LL+t.nsec()==time,"actual acquisition stamp");
  const auto &h=m.header();require(h.data_size()==2&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)=="m43/native/imu"&&h.data(1).key()=="seq"&&h.data(1).value_size()==1&&h.data(1).value(0)==std::to_string(sequence),"actual frame/sequence");
  require(m.entity_name()=="imu"&&m.has_orientation()==enabled&&m.has_angular_velocity()&&m.has_linear_acceleration(),"native field presence");finiteVector(m.angular_velocity());finiteVector(m.linear_acceleration());
  if(enabled){const auto &q=m.orientation();const double n=q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w();require(std::isfinite(n)&&std::abs(n-1)<1e-12,"finite unit native orientation");}
  require(m.has_orientation_covariance()&&m.has_angular_velocity_covariance()&&m.has_linear_acceleration_covariance(),"native covariance presence");
  require(m.orientation_covariance().data_size()==9&&m.angular_velocity_covariance().data_size()==9&&m.linear_acceleration_covariance().data_size()==9,"native covariance shape");
}
void noiseBits(const Config &c){
  std::cout<<'[';for(unsigned axis=0;axis<6;++axis){if(axis)std::cout<<',';std::cout<<"{\"kind\":\""<<(c.noise?"gaussian":"none")<<"\",\"parameters_bits\":[\""<<bits(c.noise?means[axis]:0.0)<<'"';for(unsigned p=1;p<7;++p)std::cout<<",\""<<bits(0)<<'"';std::cout<<"]}";}std::cout<<']';
}
void run(const Config &c,const std::string &mapsPath){
  bool attempted=false;const auto capture=[&]{if(!attempted){attempted=true;maps(mapsPath);}};
  try{
    // Initialize an owned per-mode sink before any factory logging.
    const std::filesystem::path mapsFile(mapsPath);
    require(mapsFile.is_absolute(),"absolute owned maps path");
    const auto consoleName=mapsFile.filename().string()+".console.log";
    require(gz::common::Console::Init(mapsFile.parent_path().string(),consoleName),"owned Console initialization");
    require(gz::common::Console::Directory()==mapsFile.parent_path().string(),"actual owned Console directory");
    const std::string topic="/m43/native/imu_reference/"+c.mode;const auto source=xml(c,topic);
    sdf::SDFPtr parsed(new sdf::SDF);sdf::init(parsed);require(sdf::readString(source,parsed),"finite IMU SDF");gz::sensors::SensorFactory factory;
    auto sensor=factory.CreateSensor<gz::sensors::ImuSensor>(parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));require(static_cast<bool>(sensor),"actual IMU SensorFactory");
    const Q initial=Q(bodies[0])*Q(localRpy);sensor->SetOrientationReference(initial);const Q beforeReference=sensor->OrientationReference();
    // Explicit CUSTOM/empty-parent intentionally produces the pinned warning.
    // Absent CUSTOM tag skips this call and keeps the exact initial reference.
    if(c.mode=="custom-explicit-motion")gz::common::Console::SetVerbosity(2);
    if(c.worldFrameCall)sensor->SetWorldFrameOrientation(Q(0,0,0),gz::sensors::WorldFrameEnumType::ENU);
    const Q afterReference=sensor->OrientationReference();sensor->SetOrientationEnabled(c.enabled);sensor->SetGravity(gravity);sensor->SetActive(true);require(sensor->OrientationEnabled()==c.enabled,"actual orientation-enabled setter/getter");
    auto received=std::make_shared<Received>();auto active=std::make_shared<std::atomic<bool>>(true);gz::transport::Node node;
    std::function<void(const Message &)> callback=[received,active](const Message &m){std::lock_guard<std::mutex> lock(received->mutex);if(!active->load(std::memory_order_acquire)||received->bad)return;
      try{require(m.ByteSizeLong()<=8192&&received->messages.size()<6,"callback cap before copy");received->messages.push_back(m);}catch(const std::exception &e){received->bad=true;received->failure=e.what();}received->changed.notify_all();};
    try{
      require(node.Subscribe(topic,callback),"actual typed IMU subscriber");const auto deadline=std::chrono::steady_clock::now()+5s;
      while(!sensor->HasConnections()){require(std::chrono::steady_clock::now()<deadline,"component connection deadline");std::this_thread::sleep_for(1ms);}
      std::cout<<"{\"kind\":\"configuration\",\"mode\":\""<<c.mode<<"\",\"instance\":0,\"topic\":\""<<topic<<"\",\"sensor_name\":\"imu\",\"frame_id\":\"m43/native/imu\",\"rate\":0,\"localization\":\""<<c.localization<<"\",\"reference_tag_present\":"<<(c.referenceTag?"true":"false")<<",\"orientation_enabled\":"<<(c.enabled?"true":"false")<<",\"world_frame_call\":"<<(c.worldFrameCall?"true":"false")<<",\"expected_warning\":\""<<(c.mode=="custom-explicit-motion"?"CUSTOM_EMPTY_PARENT":"NONE")<<"\",\"sdf_xml_hex\":\""<<hex(source)<<"\",\"custom_parent\":\"\",\"custom_rpy_bits\":";vectorBits(V::Zero);
      std::cout<<",\"console_verbosity\":"<<gz::common::Console::Verbosity();
      std::cout<<",\"orientation_enabled_actual\":"<<(sensor->OrientationEnabled()?"true":"false");std::cout<<",\"initial_body_rpy_bits\":";vectorBits(bodies[0]);std::cout<<",\"sensor_rpy_bits\":";vectorBits(localRpy);std::cout<<",\"initial_sensor_world_quaternion_bits\":";quaternionBits(initial);std::cout<<",\"reference_before_bits\":";quaternionBits(beforeReference);std::cout<<",\"reference_after_bits\":";quaternionBits(afterReference);std::cout<<",\"noise\":";noiseBits(c);std::cout<<"}\n";
      const std::array<unsigned,6> signedRows={0,0,2,2,5,5};
      for(unsigned index=0;index<6;++index){const unsigned row=c.sign?signedRows[index]:index;const bool negative=c.sign&&index%2==1;
        Q q=Q(bodies[row])*Q(localRpy);if(negative)q=-q;const gz::math::Pose3d world(V(2,-1,3),q);
        const V accelerationSensor=q.RotateVectorReverse(accelerations[row]);const V angularSensor=q.RotateVectorReverse(angular[row]);
        // Reset inputs on every call: Update mutates the stored local vectors.
        sensor->SetWorldPose(world);sensor->SetGravity(gravity);sensor->SetAngularVelocity(angularSensor);sensor->SetLinearAcceleration(accelerationSensor);sensor->SetActive(true);
        const bool connected=sensor->HasConnections();require(connected,"connection retained");const auto nextBefore=nanos(sensor->NextDataUpdateTime());
        const bool updated=sensor->Update(std::chrono::nanoseconds(times[index]),true);const auto nextAfter=nanos(sensor->NextDataUpdateTime());require(updated,"every forced rate-zero call updates");
        Message message;
        {std::unique_lock<std::mutex> lock(received->mutex);require(received->changed.wait_for(lock,5s,[&]{return received->bad||received->messages.size()>=index+1;}),"actual native callback deadline");require(!received->bad&&received->messages.size()==index+1,"exact actual callback inventory: "+received->failure);message=received->messages[index];}
        check(message,index,times[index],c.enabled);const Q getter=sensor->Orientation();require(getter.IsFinite(),"finite actual orientation getter");
        std::cout<<"{\"kind\":\"call\",\"mode\":\""<<c.mode<<"\",\"instance\":0,\"index\":"<<index<<",\"input_row\":"<<row<<",\"negative_world_quaternion\":"<<(negative?"true":"false")<<",\"time_ns\":"<<times[index]<<",\"active\":true,\"force\":true,\"has_connections\":true,\"updated\":true,\"next_before_ns\":"<<nextBefore<<",\"next_after_ns\":"<<nextAfter<<",\"world_pose_bits\":";poseBits(world);
        std::cout<<",\"acceleration_world_bits\":";vectorBits(accelerations[row]);std::cout<<",\"angular_velocity_world_bits\":";vectorBits(angular[row]);std::cout<<",\"gravity_world_bits\":";vectorBits(gravity);
        std::cout<<",\"acceleration_sensor_setter_bits\":";vectorBits(accelerationSensor);std::cout<<",\"angular_velocity_sensor_setter_bits\":";vectorBits(angularSensor);std::cout<<",\"orientation_getter_bits\":";quaternionBits(getter);
        std::cout<<",\"sequence\":\""<<message.header().data(1).value(0)<<"\",\"entity_name\":\""<<message.entity_name()<<"\",\"orientation_present\":"<<(message.has_orientation()?"true":"false")<<",\"orientation_bits\":";
        if(message.has_orientation())quaternionBits(Q(message.orientation().w(),message.orientation().x(),message.orientation().y(),message.orientation().z()));else std::cout<<"null";
        std::cout<<",\"angular_velocity_bits\":";vectorBits(V(message.angular_velocity().x(),message.angular_velocity().y(),message.angular_velocity().z()));std::cout<<",\"linear_acceleration_bits\":";vectorBits(V(message.linear_acceleration().x(),message.linear_acceleration().y(),message.linear_acceleration().z()));
        std::cout<<",\"orientation_covariance_bits\":";covarianceBits(message.orientation_covariance());std::cout<<",\"angular_velocity_covariance_bits\":";covarianceBits(message.angular_velocity_covariance());std::cout<<",\"linear_acceleration_covariance_bits\":";covarianceBits(message.linear_acceleration_covariance());
        std::cout<<",\"header_wire\":\""<<hex(message.header().SerializeAsString())<<"\",\"wire\":\""<<hex(message.SerializeAsString())<<"\"}\n";
      }
      {std::lock_guard<std::mutex> lock(received->mutex);require(!received->bad&&received->messages.size()==6,"closing callback count");}capture();
      {std::lock_guard<std::mutex> lock(received->mutex);require(!received->bad&&received->messages.size()==6,"final callback count before retirement");active->store(false,std::memory_order_release);}require(node.Unsubscribe(topic),"actual subscriber retirement");
    }catch(...){const auto error=std::current_exception();{std::lock_guard<std::mutex> lock(received->mutex);active->store(false,std::memory_order_release);}try{capture();}catch(const std::exception &e){std::cerr<<"live maps while component alive: "<<e.what()<<'\n';}std::rethrow_exception(error);}
    std::cout<<"{\"kind\":\"summary\",\"mode\":\""<<c.mode<<"\",\"instances\":1,\"input_calls\":6,\"observations\":6,\"suppressed_calls\":0,\"canonical_packets\":6}\n";
  }catch(...){const auto error=std::current_exception();try{capture();}catch(const std::exception &e){std::cerr<<"live maps capture: "<<e.what()<<'\n';}std::rethrow_exception(error);}
}
}
int main(int argc,char **argv){try{require(argc==3,"expected MODE MAPS_PATH");run(configuration(argv[1]),argv[2]);return 0;}catch(const std::exception &e){std::cerr<<"native IMU reference component: "<<e.what()<<'\n';return 1;}}
