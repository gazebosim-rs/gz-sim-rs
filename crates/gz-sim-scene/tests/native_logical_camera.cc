// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Focused SensorFactory reference: gz-sensors8ddf4c17 LogicalCameraSensor.cc,
// gz-math1316ed33 Frustum/Plane inclusive model-origin predicate (OSRF Apache-2.0).
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <utility>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gz/math/Pose3.hh>
#include <gz/math/Frustum.hh>
#include <gz/math/Plane.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/logical_camera_image.pb.h>
#include <gz/msgs/logical_camera_sensor.pb.h>
#include <gz/sensors/LogicalCameraSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
using namespace std::chrono_literals;
using Image=gz::msgs::LogicalCameraImage;
using Frustum=gz::msgs::LogicalCameraSensor;
using Pose=gz::math::Pose3d;
struct Input {
  Pose camera;std::map<std::string,Pose> models;std::vector<std::string> expected;
  int witnessPlane,witnessSign;bool nominalEdge;
  Input(Pose p,std::map<std::string,Pose> m,std::vector<std::string> e,int plane=-1,int sign=0,bool nominal=false)
    :camera(std::move(p)),models(std::move(m)),expected(std::move(e)),witnessPlane(plane),witnessSign(sign),nominalEdge(nominal){}
};
gz::math::Frustum geometry(const Pose &pose){return gz::math::Frustum(1,8,gz::math::Angle(std::acos(-1.0)/2),1,pose);}
std::string diagnostic(const std::string &mode,unsigned index,const Input &input,const gz::math::Frustum &clip,
                       const std::vector<std::string> &expected,const Image &image,const Frustum &frustum,const std::string &reason){
  std::ostringstream out;out<<std::setprecision(17);
  const auto fullPose=[&out](const Pose &p){out<<"["<<p.Pos().X()<<","<<p.Pos().Y()<<","<<p.Pos().Z()<<","<<p.Rot().X()<<","<<p.Rot().Y()<<","<<p.Rot().Z()<<","<<p.Rot().W()<<"]";};
  out<<reason<<"; mode="<<mode<<" index="<<index<<" camera=";fullPose(input.camera);out
    <<" config=["<<frustum.near_clip()<<","<<frustum.far_clip()<<","<<frustum.horizontal_fov()<<","<<frustum.aspect_ratio()<<"] expected_count="<<expected.size()<<" actual_count="<<image.model_size()<<" actual_names=[";
  for(const auto &model:image.model())out<<model.name()<<",";out<<"] expected_names=[";for(const auto &name:expected)out<<name<<",";out<<"]";
  for(const auto &model:image.model()){out<<" actual_model="<<model.name()<<" pose=";if(model.has_pose()){const auto &p=model.pose();out<<"["<<p.position().x()<<","<<p.position().y()<<","<<p.position().z()<<","<<p.orientation().x()<<","<<p.orientation().y()<<","<<p.orientation().z()<<","<<p.orientation().w()<<"]";}else out<<"null";}
  for(const auto &[name,p]:input.models){out<<" input="<<name<<" pose=";fullPose(p);out<<" planes=[";
    for(unsigned n=0;n<6;++n){const auto plane=clip.Plane(static_cast<gz::math::Frustum::FrustumPlane>(n));const auto normal=plane.Normal();
      out<<"{normal:["<<normal.X()<<","<<normal.Y()<<","<<normal.Z()<<"],offset:"<<plane.Offset()<<",distance:"<<plane.Distance(p.Pos())<<"},";}out<<"]";}
  return out.str();
}
void maps(const std::string &path){std::ifstream in("/proc/self/maps");std::ofstream out(path);if(!in||!out)throw std::runtime_error("native live maps open");out<<in.rdbuf();out.flush();if(!out)throw std::runtime_error("native live maps write");}
void requireFinitePoseValue(double value){if(!std::isfinite(value))throw std::runtime_error("nonfinite native pose");}
void compare(const gz::msgs::Pose &actual,const Pose &expected){
  const auto &p=actual.position();const auto &q=actual.orientation();
  const double a[]={p.x(),p.y(),p.z(),q.x(),q.y(),q.z(),q.w()};
  const double b[]={expected.Pos().X(),expected.Pos().Y(),expected.Pos().Z(),expected.Rot().X(),expected.Rot().Y(),expected.Rot().Z(),expected.Rot().W()};
  for(unsigned i=0;i<7;++i){requireFinitePoseValue(a[i]);if(std::abs(a[i]-b[i])>1e-12)throw std::runtime_error("native full pose mismatch");}
}
void outputPose(const gz::msgs::Pose &p){std::cout<<"["<<p.position().x()<<","<<p.position().y()<<","<<p.position().z()<<","<<p.orientation().x()<<","<<p.orientation().y()<<","<<p.orientation().z()<<","<<p.orientation().w()<<"]";}
void outputPose(const Pose &p){std::cout<<"["<<p.Pos().X()<<","<<p.Pos().Y()<<","<<p.Pos().Z()<<","<<p.Rot().X()<<","<<p.Rot().Y()<<","<<p.Rot().Z()<<","<<p.Rot().W()<<"]";}
void header(const gz::msgs::Header &h,long long stamp,unsigned sequence,bool image){
  if(!h.has_stamp()||h.stamp().sec()*1000000000LL+h.stamp().nsec()!=stamp||h.data_size()!=(image?2:1))throw std::runtime_error("native header stamp/size");
  bool frame=false,seq=false;
  for(const auto &field:h.data()){
    if(field.key()=="frame_id"&&field.value_size()==1&&field.value(0)=="rig/body/logical_camera")frame=true;
    else if(image&&field.key()=="seq"&&field.value_size()==1&&field.value(0)==std::to_string(sequence))seq=true;
    else throw std::runtime_error("native unexpected header field");
  }
  if(!frame||(image&&!seq))throw std::runtime_error("native frame/sequence");
}
std::vector<Input> inputs(const std::string &mode){
  std::vector<Input> rows;const Pose identity(0,0,0,0,0,0);
  if(mode=="planes"){
    const double h=4*std::tan(std::acos(-1.0)/4),e=1e-9;
    const std::vector<std::vector<gz::math::Vector3d>> groups={
      {{1,0,0},{1+e,0,0},{1-e,0,0}},{{8,0,0},{8-e,0,0},{8+e,0,0}},
      {{4,h,0},{4,h-e,0},{4,h+e,0}},{{4,-h,0},{4,-h+e,0},{4,-h-e,0}},
      {{4,0,h},{4,0,h-e},{4,0,h+e}},{{4,0,-h},{4,0,-h+e},{4,0,-h-e}}};
    for(const auto &group:groups)for(unsigned i=0;i<group.size();++i){Input row{identity,{{"target",Pose(group[i],gz::math::Quaterniond::Identity)}},{},-1,0,rows.size()>=6&&i==0};if(i<2)row.expected={"target"};rows.push_back(row);}
    rows.push_back({identity,{{"behind",Pose(-2,0,0,0,0,0)}},{}});
    // Keep the original19 nominal inputs untouched. Only their four nominal
    // side edges are characterized by actual floating planes, not assumed zero.
    // Add native ComputePlanes face centers in the exact corner/sum order.
    const double nearWidth=2.0*std::tan(std::acos(-1.0)/4),farWidth=nearWidth*8;
    const gz::math::Vector3d nc(1,0,0),fc(8,0,0),un(0,0,nearWidth*.5),rn(0,-nearWidth*.5,0),uf(0,0,farWidth*.5),rf(0,-farWidth*.5,0);
    const auto ntl=nc+un-rn,ntr=nc+un+rn,nbl=nc-un-rn,nbr=nc-un+rn;
    const auto ftl=fc+uf-rf,ftr=fc+uf+rf,fbl=fc-uf-rf,fbr=fc-uf+rf;
    const std::vector<gz::math::Vector3d> centers={(ftl+ntl+fbl+nbl)/4.0,(ftr+ntr+fbr+nbr)/4.0,(ftr+ntr+ftl+ntl)/4.0,(fbr+nbr+fbl+nbl)/4.0};
    const auto clip=geometry(identity);
    for(unsigned face=0;face<centers.size();++face){const int plane=static_cast<int>(face)+2;const auto normal=clip.Plane(static_cast<gz::math::Frustum::FrustumPlane>(plane)).Normal();
      for(int sign:{0,1,-1}){const auto point=centers[face]+normal*(sign*e);rows.push_back({identity,{{"target",Pose(point,gz::math::Quaterniond::Identity)}},sign<0?std::vector<std::string>{}:std::vector<std::string>{"target"},plane,sign});}}

  }else if(mode=="rotated"){
    for(unsigned i=0;i<3;++i){const Pose camera(2,-3,4,.1*i,-.2*i,.7*i);Input row{camera,{}, {"owner","target"}};
      row.models.emplace("target",camera*Pose(4,.25,.5,.3,-.2,.4));row.models.emplace("owner",camera*Pose(2,0,0,0,0,.2));row.models.emplace("behind",camera*Pose(-2,0,0,0,0,0));rows.push_back(row);}
  }else if(mode=="empty"){rows.push_back({identity,{}, {}});
  }else if(mode=="order"){rows.push_back({identity,{{"z",Pose(4,0,0,0,0,.4)},{"owner",Pose(2,0,0,.2,0,0)},{"a",Pose(3,0,0,0,.3,0)}},{"a","owner","z"}});
  }else throw std::runtime_error("unknown logical-camera mode");
  return rows;
}
void run(const std::string &mode,const std::string &mapsPath){
  const auto rows=inputs(mode);
  const std::string xml="<sdf version='1.12'><model name='rig'><link name='body'><sensor name='logical_camera' type='logical_camera'>"
    "<topic>/m30/logical_camera</topic><frame_id>rig/body/logical_camera</frame_id><logical_camera><near>1</near><far>8</far>"
    "<horizontal_fov>1.5707963267948966</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor></link></model></sdf>";
  sdf::SDFPtr parsed(new sdf::SDF);sdf::init(parsed);if(!sdf::readString(xml,parsed))throw std::runtime_error("native SDF refusal");
  gz::sensors::SensorFactory factory;
  auto sensor=factory.CreateSensor<gz::sensors::LogicalCameraSensor>(parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));if(!sensor)throw std::runtime_error("native LogicalCamera creation");
  std::mutex mutex;std::condition_variable ready;std::vector<Image> images;std::vector<Frustum> frustums;bool overflow=false;
  gz::transport::Node node;
  bool attempted=false;const auto capture=[&]{attempted=true;maps(mapsPath);};
  try{
  if(!node.Subscribe("/m30/logical_camera",std::function<void(const Image &)>([&](const Image &m){std::lock_guard<std::mutex> lock(mutex);if(images.size()>=rows.size())overflow=true;else images.push_back(m);ready.notify_all();})))throw std::runtime_error("native image subscription");
  if(!node.Subscribe("/m30/logical_camera/frustum",std::function<void(const Frustum &)>([&](const Frustum &m){std::lock_guard<std::mutex> lock(mutex);if(frustums.size()>=rows.size())overflow=true;else frustums.push_back(m);ready.notify_all();})))throw std::runtime_error("native frustum subscription");
    const auto deadline=std::chrono::steady_clock::now()+5s;
    while(!sensor->HasImageConnections()||!sensor->HasFrustumConnections()){if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("native paired connection deadline");std::this_thread::sleep_for(1ms);}
    for(unsigned index=0;index<rows.size();++index){
      const auto &input=rows[index];sensor->SetPose(input.camera);sensor->SetModelPoses(std::map<std::string,Pose>(input.models));const long long stamp=(index+1)*1000000LL;
      if(!sensor->Update(std::chrono::nanoseconds(stamp)))throw std::runtime_error("native update refused");
      Image image;Frustum frustum;
      {std::unique_lock<std::mutex> lock(mutex);if(!ready.wait_for(lock,5s,[&]{return overflow||(images.size()>index&&frustums.size()>index);})||overflow)throw std::runtime_error("native paired callback deadline/overflow");image=images[index];frustum=frustums[index];}
      if(!image.has_header()||!frustum.has_header()||!image.has_pose())throw std::runtime_error("native paired canonical fields");
      const auto clip=geometry(input.camera);auto expected=input.expected;
      if(input.nominalEdge)expected=clip.Contains(input.models.at("target").Pos())?std::vector<std::string>{"target"}:std::vector<std::string>{};
      const auto fail=[&](const std::string &reason){throw std::runtime_error(diagnostic(mode,index,input,clip,expected,image,frustum,reason));};
      try{header(image.header(),stamp,index,true);header(frustum.header(),stamp,index,false);compare(image.pose(),input.camera);}catch(const std::exception &e){fail(e.what());}
      if(input.witnessPlane>=0){const auto plane=clip.Plane(static_cast<gz::math::Frustum::FrustumPlane>(input.witnessPlane));const auto point=input.models.at("target").Pos();const auto distance=plane.Distance(point);
        if(input.witnessSign==0){if(distance!=0.0||plane.Side(point)!=gz::math::Planed::NO_SIDE)fail("face center is not an exact signed-zero witness");}
        else if(input.witnessSign>0){if(!(distance>0.0)||plane.Side(point)!=gz::math::Planed::POSITIVE_SIDE)fail("inward witness sign");}
        else if(!(distance<0.0)||plane.Side(point)!=gz::math::Planed::NEGATIVE_SIDE)fail("outward witness sign");
      }
      if(frustum.near_clip()!=1||frustum.far_clip()!=8||frustum.horizontal_fov()!=std::acos(-1.0)/2||frustum.aspect_ratio()!=1||image.model_size()!=static_cast<int>(expected.size()))fail("native frustum/detection count");
      for(unsigned i=0;i<expected.size();++i){const auto &model=image.model(i);if(model.name()!=expected[i]||!model.has_pose())fail("native lexical origin inclusion");try{compare(model.pose(),input.camera.Inverse()*input.models.at(model.name()));}catch(const std::exception &e){fail(e.what());}}
      std::cout<<std::setprecision(17)<<"{\"mode\":\""<<mode<<"\",\"index\":"<<index<<",\"time_ns\":"<<stamp<<",\"sequence\":"<<index<<",\"frame\":\"rig/body/logical_camera\",\"frustum_sequence\":null,\"config\":[1,8,"<<frustum.horizontal_fov()<<",1],\"camera\":";outputPose(image.pose());std::cout<<",\"inputs\":[";bool first=true;
      for(const auto &[name,p]:input.models){if(!first)std::cout<<",";first=false;std::cout<<"{\"name\":\""<<name<<"\",\"pose\":";outputPose(p);std::cout<<"}";}
      std::cout<<"],\"models\":[";first=true;for(const auto &model:image.model()){if(!first)std::cout<<",";first=false;std::cout<<"{\"name\":\""<<model.name()<<"\",\"pose\":";outputPose(model.pose());std::cout<<"}";}std::cout<<"],\"nominal_side_edge\":"<<(input.nominalEdge?"true":"false")<<",\"witness_plane\":";if(input.witnessPlane<0)std::cout<<"null";else std::cout<<input.witnessPlane;
      std::cout<<",\"witness_sign\":";if(input.witnessPlane<0)std::cout<<"null";else std::cout<<input.witnessSign;
      std::cout<<",\"planes\":[";for(unsigned n=0;n<6;++n){if(n)std::cout<<",";const auto plane=clip.Plane(static_cast<gz::math::Frustum::FrustumPlane>(n));const auto normal=plane.Normal();std::cout<<"{\"normal\":["<<normal.X()<<","<<normal.Y()<<","<<normal.Z()<<"],\"offset\":"<<plane.Offset()<<",\"distances\":[";first=true;for(const auto &[name,p]:input.models){(void)name;if(!first)std::cout<<",";first=false;const auto distance=plane.Distance(p.Pos());requireFinitePoseValue(distance);std::cout<<distance;}std::cout<<"]}";}std::cout<<"]}\n";
    }
    {std::lock_guard<std::mutex> lock(mutex);if(overflow||images.size()!=rows.size()||frustums.size()!=rows.size())throw std::runtime_error("native paired final count");}
    capture();
  }catch(...){const auto original=std::current_exception();if(!attempted){try{capture();}catch(const std::exception &e){std::cerr<<"native maps failure: "<<e.what()<<"\n";}}std::rethrow_exception(original);}
}
int main(int argc,char **argv){try{if(argc!=3)throw std::runtime_error("MODE MAPS_PATH required");run(argv[1],argv[2]);return 0;}catch(const std::exception &e){std::cerr<<"native logical-camera reference: "<<e.what()<<"\n";return 1;}}
