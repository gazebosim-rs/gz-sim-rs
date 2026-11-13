// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Fresh SensorFactory COMPONENT reference only. Explicit WORLD poses/model maps
// and derived one-argument Update; no System/ECM/demand/rate/removal/Physics proof.
// Source obligations: pair2/eight8 configurations, FOUR direct calls each,
// 40 explicit inputs/40 image+frustum pairs/80 canonical messages in total.
#include <gz/common/Console.hh>
#include <gz/math/Frustum.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/logical_camera_image.pb.h>
#include <gz/msgs/logical_camera_sensor.pb.h>
#include <gz/sensors/LogicalCameraSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <array>
#include <atomic>
#include <chrono>
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
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
using namespace std::chrono_literals;
namespace {
using Image=gz::msgs::LogicalCameraImage;
using Frustum=gz::msgs::LogicalCameraSensor;
using Pose=gz::math::Pose3d;
void require(bool value,const std::string &why){if(!value)throw std::runtime_error(why);}
std::string hex(const std::string &s){constexpr char d[]="0123456789abcdef";std::string out;out.reserve(s.size()*2);for(unsigned char c:s){out+=d[c>>4];out+=d[c&15];}return out;}
std::string bits(double value){require(std::isfinite(value),"finite recorded scalar");std::uint64_t n;std::memcpy(&n,&value,sizeof(n));std::ostringstream out;out<<std::hex<<std::setfill('0')<<std::setw(16)<<n;return out.str();}
void poseBits(const Pose &p){std::cout<<"[\""<<bits(p.Pos().X())<<"\",\""<<bits(p.Pos().Y())<<"\",\""<<bits(p.Pos().Z())<<"\",\""<<bits(p.Rot().X())<<"\",\""<<bits(p.Rot().Y())<<"\",\""<<bits(p.Rot().Z())<<"\",\""<<bits(p.Rot().W())<<"\"]";}
void poseBits(const gz::msgs::Pose &p){require(p.has_position()&&p.has_orientation(),"complete actual pose");std::cout<<"[\""<<bits(p.position().x())<<"\",\""<<bits(p.position().y())<<"\",\""<<bits(p.position().z())<<"\",\""<<bits(p.orientation().x())<<"\",\""<<bits(p.orientation().y())<<"\",\""<<bits(p.orientation().z())<<"\",\""<<bits(p.orientation().w())<<"\"]";}
void maps(const std::string &path){std::ifstream input("/proc/self/maps");require(bool(input),"maps open");std::string text,line;while(std::getline(input,line)){require(text.size()+line.size()+1<=4*1024*1024,"maps cap");text+=line+'\n';}require(input.eof(),"complete maps read");const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);require(fd>=0,"exclusive maps output");std::size_t offset=0;while(offset<text.size()){const auto n=::write(fd,text.data()+offset,text.size()-offset);if(n<=0){::close(fd);throw std::runtime_error("maps write");}offset+=static_cast<std::size_t>(n);}const bool flushed=::fsync(fd)==0;const bool closed=::close(fd)==0;require(flushed&&closed,"maps durable completion");}
template<class Message> struct Received{std::mutex mutex;std::condition_variable changed;std::vector<Message> messages;std::shared_ptr<std::atomic<bool>> active=std::make_shared<std::atomic<bool>>(true);bool bad=false;std::string failure;};
template<class Message> void subscribe(gz::transport::Node &node,const std::string &topic,const std::shared_ptr<Received<Message>> &state){auto active=state->active;std::function<void(const Message &)> callback=[state,active](const Message &m){std::lock_guard<std::mutex> lock(state->mutex);if(!active->load(std::memory_order_acquire)||state->bad)return;try{require(m.ByteSizeLong()<=65536&&state->messages.size()<4,"callback cap before copy");state->messages.push_back(m);}catch(const std::exception &e){state->bad=true;state->failure=e.what();}state->changed.notify_all();};require(node.Subscribe(topic,callback),"actual typed component subscription");}
void ready(gz::transport::Node &node,const std::string &topic,const std::string &type){const auto deadline=std::chrono::steady_clock::now()+5s;for(;;){std::vector<gz::transport::MessagePublisher> p,s;node.TopicInfo(topic,p,s);if(p.size()==1&&s.size()==1&&p[0].MsgTypeName()==type&&s[0].MsgTypeName()==type)return;require(std::chrono::steady_clock::now()<deadline,"typed dual-channel readiness");std::this_thread::yield();}}
template<class Message> Message receive(const std::shared_ptr<Received<Message>> &state,unsigned index){std::unique_lock<std::mutex> lock(state->mutex);require(state->changed.wait_for(lock,5s,[&]{return state->bad||state->messages.size()>index;}),"actual dual-channel callback deadline");require(!state->bad&&state->messages.size()==index+1,"exact callback inventory: "+state->failure);return state->messages[index];}
void header(const gz::msgs::Header &h,long long ns,unsigned sequence,const std::string &frame,bool image){require(h.has_stamp()&&h.stamp().sec()==0&&h.stamp().nsec()==ns,"actual exact acquisition stamp");require(h.data_size()==(image?2:1)&&h.data(0).key()=="frame_id"&&h.data(0).value_size()==1&&h.data(0).value(0)==frame,"actual frame data");if(image)require(h.data(1).key()=="seq"&&h.data(1).value_size()==1&&h.data(1).value(0)==std::to_string(sequence),"per-instance image sequence");}
void compare(const gz::msgs::Pose &a,const Pose &b){require(a.has_position()&&a.has_orientation(),"actual full pose");const double av[]={a.position().x(),a.position().y(),a.position().z(),a.orientation().x(),a.orientation().y(),a.orientation().z(),a.orientation().w()};const double bv[]={b.Pos().X(),b.Pos().Y(),b.Pos().Z(),b.Rot().X(),b.Rot().Y(),b.Rot().Z(),b.Rot().W()};for(unsigned i=0;i<7;++i)require(std::isfinite(av[i])&&std::abs(av[i]-bv[i])<=1e-12,"original full-pose bound");}
struct Camera{unsigned index;std::string topic,frame;std::unique_ptr<gz::sensors::LogicalCameraSensor> sensor;std::shared_ptr<Received<Image>> images;std::shared_ptr<Received<Frustum>> frustums;};
Pose cameraInput(unsigned i,unsigned row){return Pose(-1+.1*row,3*i+.05*row,3+.1*row,.03*row,-.02*row,(i%2==0?0:.3)+.04*row);}
std::map<std::string,Pose> modelInputs(unsigned count,unsigned row){std::map<std::string,Pose> models;if(row==2)return models;for(unsigned i=0;i<count;++i)models.emplace("rig"+std::to_string(i),Pose(.05*row,3*i,3,0,0,.1*i));models.emplace("target",Pose(4+.1*row,.2*row,3,.1,-.2,.5));models.emplace("behind",Pose(-4,0,3,0,0,-.3));return models;}
void outputModels(const std::map<std::string,Pose> &models){std::cout<<'[';bool first=true;for(const auto &[name,p]:models){if(!first)std::cout<<',';first=false;std::cout<<"{\"name\":"<<std::quoted(name)<<",\"world_pose_bits\":";poseBits(p);std::cout<<'}';}std::cout<<']';}
}
int main(int argc,char **argv){bool captured=false;try{
  require(argc==3,"expected MODE MAPS_PATH");const std::string mode=argv[1];require(mode=="pair"||mode=="eight","closed component modes");const unsigned count=mode=="pair"?2:8;const std::filesystem::path mapPath(argv[2]);require(mapPath.is_absolute(),"owned absolute maps path");require(gz::common::Console::Init(mapPath.parent_path().string(),mapPath.filename().string()+".console.log"),"owned Console Init");require(gz::common::Console::Directory()==mapPath.parent_path().string(),"owned Console directory");gz::common::Console::SetVerbosity(2);require(gz::common::Console::Verbosity()==2,"actual Console verbosity");
  gz::sensors::SensorFactory factory;gz::transport::Node node;std::vector<Camera> cameras;std::set<std::uint64_t> ids;
  for(unsigned i=0;i<count;++i){const std::string topic="/m53/native/"+mode+"/camera"+std::to_string(i),frame="m53/native/camera"+std::to_string(i),name="camera"+std::to_string(i);const double near=.5+.05*i,far=8+i,hfov=1.2+.02*i,aspect=1+.1*i,rate=i%2==0?100:50;
    std::ostringstream xml;xml<<std::setprecision(17)<<"<sdf version='1.12'><model name='rig'><link name='body'><sensor name='"<<name<<"' type='logical_camera'><pose>-1 0 0 0 0 "<<(i%2==0?0:.3)<<"</pose><update_rate>"<<rate<<"</update_rate><topic>"<<topic<<"</topic><frame_id>"<<frame<<"</frame_id><logical_camera><near>"<<near<<"</near><far>"<<far<<"</far><horizontal_fov>"<<hfov<<"</horizontal_fov><aspect_ratio>"<<aspect<<"</aspect_ratio></logical_camera></sensor></link></model></sdf>";
    sdf::SDFPtr parsed(new sdf::SDF);sdf::init(parsed);require(sdf::readString(xml.str(),parsed),"per-instance SDF load");auto element=parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor");auto sensor=factory.CreateSensor<gz::sensors::LogicalCameraSensor>(element);require(bool(sensor),"actual ElementPtr SensorFactory creation");require(ids.insert(sensor->Id()).second&&sensor->Name()==name&&sensor->Topic()==topic&&sensor->FrameId()==frame&&sensor->UpdateRate()==rate,"independent actual native identity/configuration");require(sensor->Near()==near&&sensor->Far()==far&&sensor->HorizontalFOV().Radian()==hfov&&sensor->AspectRatio()==aspect,"actual parsed frustum getters");
    auto images=std::make_shared<Received<Image>>();auto frustums=std::make_shared<Received<Frustum>>();subscribe(node,topic,images);subscribe(node,topic+"/frustum",frustums);
    std::cout<<"{\"kind\":\"configuration\",\"mode\":"<<std::quoted(mode)<<",\"sensor\":"<<i<<",\"native_id\":"<<sensor->Id()<<",\"name\":"<<std::quoted(sensor->Name())<<",\"topic\":"<<std::quoted(sensor->Topic())<<",\"frustum_topic\":"<<std::quoted(topic+"/frustum")<<",\"frame_id\":"<<std::quoted(sensor->FrameId())<<",\"rate_bits\":\""<<bits(sensor->UpdateRate())<<"\",\"config_bits\":[\""<<bits(sensor->Near())<<"\",\""<<bits(sensor->Far())<<"\",\""<<bits(sensor->HorizontalFOV().Radian())<<"\",\""<<bits(sensor->AspectRatio())<<"\"],\"sensor_local_pose_bits\":";poseBits(sensor->Pose());std::cout<<",\"console_verbosity\":"<<gz::common::Console::Verbosity()<<",\"sdf_xml_hex\":"<<std::quoted(hex(xml.str()))<<"}\n";
    cameras.push_back({i,topic,frame,std::move(sensor),images,frustums});
  }
  for(auto &c:cameras){ready(node,c.topic,"gz.msgs.LogicalCameraImage");ready(node,c.topic+"/frustum","gz.msgs.LogicalCameraSensor");const auto deadline=std::chrono::steady_clock::now()+5s;while(!c.sensor->HasImageConnections()||!c.sensor->HasFrustumConnections()){require(std::chrono::steady_clock::now()<deadline,"actual component dual connection getters");std::this_thread::yield();}}
  unsigned inputs=0,pairs=0;
  for(unsigned row=0;row<4;++row){const long long ns=1000000LL+10000000LL*row;const auto models=modelInputs(count,row);
    for(auto &c:cameras){const auto world=cameraInput(c.index,row);c.sensor->SetPose(world);c.sensor->SetModelPoses(std::map<std::string,Pose>(models));
      std::cout<<"{\"kind\":\"input\",\"mode\":"<<std::quoted(mode)<<",\"sensor\":"<<c.index<<",\"row\":"<<row<<",\"time_ns\":"<<ns<<",\"world_pose_bits\":";poseBits(c.sensor->Pose());std::cout<<",\"model_world_poses\":";outputModels(models);std::cout<<"}\n";++inputs;
      require(c.sensor->Update(std::chrono::nanoseconds(ns)),"explicit derived one-argument Update");const auto image=receive(c.images,row);const auto frustum=receive(c.frustums,row);require(image.has_header()&&frustum.has_header()&&image.has_pose(),"actual paired fields");header(image.header(),ns,row,c.frame,true);header(frustum.header(),ns,row,c.frame,false);compare(image.pose(),world);require(c.sensor->Image().SerializeAsString()==image.SerializeAsString(),"actual public Image getter and received wire");
      require(frustum.near_clip()==c.sensor->Near()&&frustum.far_clip()==c.sensor->Far()&&frustum.horizontal_fov()==c.sensor->HorizontalFOV().Radian()&&frustum.aspect_ratio()==c.sensor->AspectRatio(),"paired frustum getters");
      const gz::math::Frustum clip(c.sensor->Near(),c.sensor->Far(),c.sensor->HorizontalFOV(),c.sensor->AspectRatio(),world);std::vector<std::string> selected;for(const auto &[name,p]:models)if(clip.Contains(p.Pos()))selected.push_back(name);require(image.model_size()==static_cast<int>(selected.size()),"all supplied model origins considered");
      for(unsigned j=0;j<selected.size();++j){const auto &m=image.model(j);require(m.name()==selected[j]&&m.has_pose(),"lexical selected model origins");compare(m.pose(),world.Inverse()*models.at(m.name()));}
      if(row==0){bool owner=false;for(const auto &m:image.model())owner|=m.name()=="rig"+std::to_string(c.index);require(owner,"visible owner origin retained");}if(row==2)require(image.model_size()==0,"actual empty model-map frame");
      std::cout<<"{\"kind\":\"observation\",\"mode\":"<<std::quoted(mode)<<",\"sensor\":"<<c.index<<",\"row\":"<<row<<",\"time_ns\":"<<ns<<",\"sequence\":"<<row<<",\"camera_pose_bits\":";poseBits(image.pose());std::cout<<",\"models\":[";for(int j=0;j<image.model_size();++j){if(j)std::cout<<',';std::cout<<"{\"name\":"<<std::quoted(image.model(j).name())<<",\"relative_pose_bits\":";poseBits(image.model(j).pose());std::cout<<'}';}std::cout<<"],\"image_header_wire\":"<<std::quoted(hex(image.header().SerializeAsString()))<<",\"frustum_header_wire\":"<<std::quoted(hex(frustum.header().SerializeAsString()))<<",\"image_wire\":"<<std::quoted(hex(image.SerializeAsString()))<<",\"frustum_wire\":"<<std::quoted(hex(frustum.SerializeAsString()))<<"}\n";++pairs;
    }
  }
  maps(argv[2]);captured=true;
  for(auto &c:cameras){{std::lock_guard<std::mutex> lock(c.images->mutex);require(!c.images->bad&&c.images->messages.size()==4,"exact image callback inventory");c.images->active->store(false,std::memory_order_release);} {std::lock_guard<std::mutex> lock(c.frustums->mutex);require(!c.frustums->bad&&c.frustums->messages.size()==4,"exact frustum callback inventory");c.frustums->active->store(false,std::memory_order_release);}require(node.Unsubscribe(c.topic)&&node.Unsubscribe(c.topic+"/frustum"),"owned subscriber cleanup");}
  require(inputs==4*count&&pairs==4*count,"closed direct-input/pair inventory");std::cout<<"{\"kind\":\"summary\",\"mode\":"<<std::quoted(mode)<<",\"configuration_count\":"<<count<<",\"direct_update_calls\":"<<inputs<<",\"input_rows\":"<<inputs<<",\"observed_pairs\":"<<pairs<<",\"canonical_packets\":"<<2*pairs<<",\"original_system_executed\":false,\"native_physics_executed\":false,\"native_lifecycle_equivalence_claimed\":false,\"old_native_modes_replayed\":false}\n";return 0;
}catch(const std::exception &e){if(argc==3&&!captured){try{maps(argv[2]);}catch(...){}}std::cerr<<"multiple logical-camera component: "<<e.what()<<'\n';return 1;}}
