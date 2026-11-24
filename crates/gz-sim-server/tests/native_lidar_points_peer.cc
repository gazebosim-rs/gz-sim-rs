// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Canonical transport decoder only; this does not execute a native lidar sensor.
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
namespace {
void require(bool value,const char *why){if(!value)throw std::runtime_error(why);}
float scalar(const std::string &data,std::size_t offset){
  const auto *p=reinterpret_cast<const unsigned char *>(data.data()+offset);
  const std::uint32_t bits=std::uint32_t(p[0])|(std::uint32_t(p[1])<<8)|(std::uint32_t(p[2])<<16)|(std::uint32_t(p[3])<<24);
  float value;static_assert(sizeof(value)==sizeof(bits));std::memcpy(&value,&bits,sizeof(value));return value;
}
}
int main(int argc,char **)try{
  require(argc==1,"no point observer arguments");
  std::mutex mutex;std::condition_variable changed;bool done=false;std::string bytes,failure;
  gz::transport::Node node;
  auto callback=[&](const gz::msgs::PointCloudPacked &m){std::lock_guard<std::mutex> lock(mutex);if(done)return;
    try{
      require(m.ByteSizeLong()<=4096,"cloud encoded cap");
      require(m.width()==3&&m.height()==3&&m.point_step()==18&&m.row_step()==54&&m.data().size()==162,"organized 3x3 layout");
      require(!m.is_bigendian()&&m.is_dense(),"finite little endian cloud");
      const char *names[]={"x","y","z","intensity","ring"};const unsigned offsets[]={0,4,8,12,16};
      require(m.field_size()==5,"five native fields");
      for(unsigned i=0;i<5;++i){const auto &f=m.field(i);require(f.name()==names[i]&&f.offset()==offsets[i]&&f.count()==1&&int(f.datatype())==(i==4?3:6),"native field descriptor");}
      require(m.has_header()&&m.header().has_stamp(),"acquisition header");
      require(m.header().stamp().sec()>=0&&m.header().stamp().nsec()>=0&&m.header().stamp().nsec()<1000000000,"valid stamp");
      require(m.header().stamp().sec()>0||m.header().stamp().nsec()>0,"positive acquisition");
      require(m.header().data_size()==2&&m.header().data(0).key()=="frame_id"&&m.header().data(0).value_size()==1&&m.header().data(0).value(0)=="grid_frame"&&m.header().data(1).key()=="seq"&&m.header().data(1).value_size()==1,"canonical frame and sequence");
      const auto &seq=m.header().data(1).value(0);require(!seq.empty(),"sequence presence");
      for(char c:seq){
        require(c>='0'&&c<='9',"sequence decimal");
      }
      require(std::to_string(std::stoull(seq))==seq,"canonical sequence");
      for(unsigned v=0;v<3;++v)for(unsigned h=0;h<3;++h){const unsigned i=v*3+h;const double yaw=-0.3+h*0.3,pitch=-0.2+v*0.2;
        const double expected[]={4.,4.*std::tan(yaw),4.*std::tan(pitch)/std::cos(yaw)};
        for(unsigned a=0;a<3;++a)require(std::isfinite(scalar(m.data(),i*18+a*4))&&std::abs(double(scalar(m.data(),i*18+a*4))-expected[a])<1e-6,"analytic sensor-frame wall coordinates");
        require(scalar(m.data(),i*18+12)==0,"zero intensity");
        const auto *ring=reinterpret_cast<const unsigned char *>(m.data().data()+i*18+16);
        require((unsigned(ring[0])|(unsigned(ring[1])<<8))==v,"vertical ring index");
      }
      bytes=m.SerializeAsString();
    }catch(const std::exception &e){failure=e.what();}
    done=true;changed.notify_all();
  };
  require(node.Subscribe("/m114/scan/points",std::function<void(const gz::msgs::PointCloudPacked &)>(callback)),"typed Subscribe");
  {std::unique_lock<std::mutex> lock(mutex);require(changed.wait_for(lock,std::chrono::seconds(10),[&]{return done;}),"point packet deadline");}
  node.Unsubscribe("/m114/scan/points");require(failure.empty(),failure.c_str());
  constexpr char digits[]="0123456789abcdef";std::string hex;hex.reserve(bytes.size()*2);
  for(unsigned char c:bytes){hex+=digits[c>>4];hex+=digits[c&15];}std::cout<<"PACKET "<<hex<<'\n';return 0;
}catch(const std::exception &e){std::cerr<<"native lidar points: "<<e.what()<<'\n';return 1;}
