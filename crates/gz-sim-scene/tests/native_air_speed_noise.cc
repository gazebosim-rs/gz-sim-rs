// Copyright (C) 2023, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// AirSpeedSensor.cc at8ddf4c17; GaussianNoiseModel.cc and native Noise_TEST
// hundred-sample/five-sigma policy. No Sim/plugin or accepted ideal replay.
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/air_speed.pb.h>
#include <gz/sensors/AirSpeedSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
using namespace std::chrono_literals;
using Message=gz::msgs::AirSpeed;
void maps(const std::string &path) {
  std::ifstream input("/proc/self/maps");std::ofstream output(path);
  if(!input||!output)throw std::runtime_error("native live maps open");
  output<<input.rdbuf();output.flush();if(!output)throw std::runtime_error("native live maps write");
}
void run(const std::string &mode,const std::string &mapsPath) {
  double mean=0,stddev=0,precision=0,biasMean=0,dynamicStddev=0,tau=0;
  std::string kind="gaussian";
  if(mode=="zero"){}
  else if(mode=="mean")mean=.1;
  else if(mode=="negative")mean=-1;
  else if(mode=="gaussian-round"||mode=="quantized"){
    mean=-.125;precision=.25;if(mode=="quantized")kind="gaussian_quantized";
  }
  else if(mode=="threshold"||mode=="active-boundary"){
    mean=1.75e-6;precision=mode=="threshold"?1e-6:std::nextafter(1e-6,1.0);
  }
  else if(mode=="bias-zero-dt"){biasMean=.3;dynamicStddev=100;tau=2;}
  else if(mode=="white"){mean=.1;stddev=.2;}
  else throw std::runtime_error("unknown native AirSpeed noise mode");
  std::ostringstream noise;noise<<std::setprecision(17)<<"<noise type='"<<kind<<"'><mean>"<<mean
    <<"</mean><stddev>"<<stddev<<"</stddev><bias_mean>"<<biasMean
    <<"</bias_mean><dynamic_bias_stddev>"<<dynamicStddev<<"</dynamic_bias_stddev><dynamic_bias_correlation_time>"<<tau
    <<"</dynamic_bias_correlation_time><precision>"<<precision<<"</precision></noise>";
  const std::string xml="<sdf version='1.12'><model name='rig'><link name='body'><sensor name='air_speed' type='air_speed'>"
    "<topic>/m28/air_speed</topic><frame_id>rig/body/air_speed</frame_id><air_speed><pressure>"+noise.str()+
    "</pressure></air_speed></sensor></link></model></sdf>";
  gz::math::Rand::Seed(42);sdf::SDFPtr parsed(new sdf::SDF);sdf::init(parsed);
  if(!sdf::readString(xml,parsed))throw std::runtime_error("native noise SDF refused");
  gz::sensors::SensorFactory factory;
  auto sensor=factory.CreateSensor<gz::sensors::AirSpeedSensor>(parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
  if(!sensor)throw std::runtime_error("native AirSpeed creation");
  std::mutex mutex;std::condition_variable ready;std::vector<Message> messages;bool overflow=false;
  gz::transport::Node node;
  if(!node.Subscribe("/m28/air_speed",std::function<void(const Message &)>([&](const Message &message){
    std::lock_guard<std::mutex> lock(mutex);if(messages.size()>=100)overflow=true;else messages.push_back(message);ready.notify_all();
  })))throw std::runtime_error("native canonical subscriber");
  bool attempted=false;const auto capture=[&]{attempted=true;maps(mapsPath);};
  try {
    const auto deadline=std::chrono::steady_clock::now()+5s;
    while(!sensor->HasConnections()){
      if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("native connection deadline");
      std::this_thread::sleep_for(1ms);
    }
    const float temperature=288.15f-.0065f*3.5f;
    const float density=1.225f/std::pow(288.15f/temperature,4.256f);
    const double x=mode=="zero"||mode=="mean"?3.125:0;
    const float precursor=static_cast<float>((x>0?1.0f:0.0f)*.005f*density*x*x);
    const unsigned count=mode=="white"?100:5;
    double signedBias=0;std::vector<double> residuals;
    for(unsigned index=0;index<count;++index){
      sensor->SetPose(gz::math::Pose3d(0,0,3.5,.2,-.3,.4));sensor->SetVelocity({x,0,0});sensor->SetWindVelocity({0,0,0});
      const long long time=(index+1)*(mode=="bias-zero-dt"?1000000000LL:1000000LL);
      if(!sensor->Update(std::chrono::nanoseconds(time)))throw std::runtime_error("native acquisition refused");
      Message message;
      {std::unique_lock<std::mutex> lock(mutex);
       if(!ready.wait_for(lock,5s,[&]{return overflow||messages.size()>=index+1;})||overflow)throw std::runtime_error("native message deadline/overflow");
       message=messages[index];}
      if(!message.has_header()||!message.header().has_stamp())throw std::runtime_error("native timestamp absent");
      std::string frame,sequence;
      for(const auto &field:message.header().data()){
        if(field.key()=="frame_id"&&field.value_size()==1)frame=field.value(0);
        if(field.key()=="seq"&&field.value_size()==1)sequence=field.value(0);
      }
      const auto &stamp=message.header().stamp();
      if(stamp.sec()*1000000000LL+stamp.nsec()!=time||frame!="rig/body/air_speed"||sequence!=std::to_string(index)
          ||!std::isfinite(message.diff_pressure())||message.temperature()!=temperature)
        throw std::runtime_error("native canonical/noisy fields");
      double expected=static_cast<double>(precursor)+mean;
      if(mode=="bias-zero-dt"){
        if(index==0)signedBias=message.diff_pressure()<0?-.3:.3;
        expected+=signedBias;
      }
      if(precision>1e-6)expected=std::round(expected/precision)*precision;
      const float expectedFloat=static_cast<float>(expected);
      if(mode!="white"&&message.diff_pressure()!=static_cast<double>(expectedFloat*100.0f))
        throw std::runtime_error("native pre100 noise/float scale/dt0 disagreement");
      residuals.push_back(message.diff_pressure());
      std::cout<<"{\"mode\":\""<<mode<<"\",\"sample\":"<<index<<",\"world_z\":3.5,\"sensor_x\":"<<x
        <<",\"precursor\":"<<precursor<<",\"mean\":"<<mean<<",\"stddev\":"<<stddev<<",\"precision\":"<<precision
        <<",\"signed_bias\":"<<signedBias<<",\"time_ns\":"<<time<<",\"sequence\":\""<<sequence
        <<"\",\"frame_id\":\""<<frame<<"\",\"diff_pressure\":"<<message.diff_pressure()<<",\"temperature\":"<<message.temperature()<<"}\n";
    }
    {std::lock_guard<std::mutex> lock(mutex);if(overflow||messages.size()!=count)throw std::runtime_error("native exact count");}
    if(mode=="white"){
      double average=0,variance=0;for(double v:residuals)average+=v/100.0;
      for(double v:residuals)variance+=(v-average)*(v-average)/99.0;
      if(std::abs(average-10)>5*20/std::sqrt(100.0)||std::abs(variance-400)>5*std::sqrt(2*std::pow(20.0,4)/99.0))
        throw std::runtime_error("native hundred sample/five sigma moments");
    }
    capture(); // Sensor and provider stay alive for actual maps.
  } catch(...){
    const auto original=std::current_exception();if(!attempted){try{capture();}catch(const std::exception &e){std::cerr<<"native maps: "<<e.what()<<'\n';}}
    std::rethrow_exception(original);
  }
}
int main(int argc,char **argv) try {
  if(argc!=3)throw std::runtime_error("expected MODE MAPS_PATH");std::cout<<std::setprecision(17);run(argv[1],argv[2]);return 0;
} catch(const std::exception &e){std::cerr<<"native AirSpeed noise: "<<e.what()<<'\n';return 1;}
