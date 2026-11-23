// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Transport-only canonical consumer; no native sensor/physics oracle.
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
namespace {
void require(bool value,const char *why){if(!value)throw std::runtime_error(why);}
}
int main(int argc,char **)try{
  require(argc==1,"no spatial observer arguments");
  std::mutex mutex;std::condition_variable changed;bool done=false;std::string bytes,failure;
  gz::transport::Node node;
  auto callback=[&](const gz::msgs::LaserScan &m){std::lock_guard<std::mutex> lock(mutex);if(done)return;
    try {
      require(m.ByteSizeLong()<=8192,"packet cap");require(m.count()==3&&m.vertical_count()==3&&m.ranges_size()==9&&m.intensities_size()==9,"3x3 cardinality");
      require(m.angle_min()==-0.3&&m.angle_max()==0.3&&m.angle_step()==0.3,"horizontal metadata");
      require(m.vertical_angle_min()==-0.2&&m.vertical_angle_max()==0.2&&m.vertical_angle_step()==0.2,"vertical metadata");
      require(m.range_min()==0&&m.range_max()==20&&m.frame()=="grid_frame","range/frame metadata");
      require(m.has_header()&&m.header().has_stamp()&&m.header().stamp().sec()>=0&&m.header().stamp().nsec()>=0&&m.header().stamp().nsec()<1000000000,"acquisition stamp");
      require(m.header().stamp().sec()>0||m.header().stamp().nsec()>0,"positive acquisition");
      require(m.header().data_size()==2&&m.header().data(0).key()=="frame_id"&&m.header().data(0).value_size()==1&&m.header().data(0).value(0)=="grid_frame"&&m.header().data(1).key()=="seq"&&m.header().data(1).value_size()==1,"canonical header");
      const auto &sequence=m.header().data(1).value(0);require(!sequence.empty(),"sequence presence");
      for(char c:sequence)require(c>='0'&&c<='9',"canonical sequence digits");
      require(std::to_string(std::stoull(sequence))==sequence,"canonical sequence integer");
      require(m.has_world_pose()&&m.world_pose().has_position()&&m.world_pose().has_orientation(),"WORLD pose");
      const auto &p=m.world_pose().position();const auto &q=m.world_pose().orientation();require(p.x()==0&&p.y()==0&&p.z()==0&&q.x()==0&&q.y()==0&&q.z()==0&&q.w()==1,"actual static acquisition frame");
      for(unsigned v=0;v<3;++v)for(unsigned h=0;h<3;++h){const auto i=v*3+h;const double pitch=-0.2+v*0.2,yaw=-0.3+h*0.3;const double expected=4/(std::cos(pitch)*std::cos(yaw));require(std::isfinite(m.ranges(i))&&std::abs(m.ranges(i)-expected)<1e-10&&m.intensities(i)==0,"vertical-major actual wall range");}
      bytes=m.SerializeAsString();
    }catch(const std::exception &e){failure=e.what();}
    done=true;changed.notify_all();
  };
  require(node.Subscribe("/m108/scan",std::function<void(const gz::msgs::LaserScan &)>(callback)),"typed Subscribe");
  {std::unique_lock<std::mutex> lock(mutex);require(changed.wait_for(lock,std::chrono::seconds(10),[&]{return done;}),"spatial packet deadline");}
  node.Unsubscribe("/m108/scan");require(failure.empty(),failure.c_str());
  constexpr char digits[]="0123456789abcdef";std::string hex;hex.reserve(bytes.size()*2);for(unsigned char c:bytes){hex+=digits[c>>4];hex+=digits[c&15];}std::cout<<"PACKET "<<hex<<'\n';return 0;
}catch(const std::exception &e){std::cerr<<"native spatial lidar: "<<e.what()<<'\n';return 1;}
