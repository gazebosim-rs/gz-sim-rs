// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Reference: gz-sensors8ddf4c17 MagnetometerSensor.cc,
// test/integration/magnetometer.cc and Noise_TEST.cc (OSRF, Apache-2.0).
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Rand.hh>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/sensors/MagnetometerSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

using namespace std::chrono_literals;
using Message=gz::msgs::Magnetometer;
std::array<double,3> rotated(const gz::math::Quaterniond &q)
{
  const double x=q.X(), y=q.Y(), z=q.Z(), w=q.W();
  const double a=2e-5,b=-3e-5,c=4e-5;
  return {(1-2*(y*y+z*z))*a+2*(x*y+z*w)*b+2*(x*z-y*w)*c,
          2*(x*y-z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z+x*w)*c,
          2*(x*z+y*w)*a+2*(y*z-x*w)*b+(1-2*(x*x+y*y))*c};
}
void maps(const std::string &path)
{
  std::ifstream input("/proc/self/maps");std::ofstream output(path);
  if (!input || !output) throw std::runtime_error("native live maps open failed");
  output << input.rdbuf();output.flush();
  if (!output) throw std::runtime_error("native live maps write failed");
}
void moments(const std::vector<double> &values,double expectedMean,double stddev)
{
  if (values.size()!=100) throw std::runtime_error("native moment count differs");
  double mean=0,variance=0;for (double v:values) mean+=v/100;
  for (double v:values) variance+=(v-mean)*(v-mean)/100;
  if (std::abs(mean-expectedMean)>5*stddev/std::sqrt(100.0) ||
      std::abs(variance-stddev*stddev)>5*std::sqrt(2*std::pow(stddev,4)/99))
    throw std::runtime_error("native white Gaussian moment bounds differ");
}
void run(const std::string &mode,const std::string &mapsPath)
{
  std::array<double,3> mean={0,0,0},deviation={0,0,0},precision={0,0,0};
  std::string noise;
  if (mode=="ideal") noise="";
  else if (mode=="zero") noise="<x><noise type='gaussian'/></x><y><noise type='gaussian_quantized'/></y>";
  else if (mode=="mean" || mode=="quantized")
  {
    mean={2.25,-2.25,.32};if (mode=="quantized") precision={.5,.5,.3};
    for (unsigned int axis=0;axis<3;++axis)
    {
      const std::string name=std::string(1,"xyz"[axis]);
      noise+="<"+name+"><noise type='"+std::string(mode=="quantized" ? "gaussian_quantized" : "gaussian")+"'><mean>"+
          std::to_string(mean[axis])+"</mean><precision>"+std::to_string(precision[axis])+"</precision></noise></"+name+">";
    }
  }
  else if (mode=="white")
  {
    mean={.001,-.002,.003};deviation={.02,.03,.04};
    for (unsigned int axis=0;axis<3;++axis)
    {
      const std::string name=std::string(1,"xyz"[axis]);
      noise+="<"+name+"><noise type='gaussian'><mean>"+std::to_string(mean[axis])+"</mean><stddev>"+
          std::to_string(deviation[axis])+"</stddev></noise></"+name+">";
    }
  }
  else if (mode=="bias-zero-dt")
  {
    for (unsigned int axis=0;axis<3;++axis)
    {
      const std::string name=std::string(1,"xyz"[axis]);
      noise+="<"+name+"><noise type='gaussian'><bias_mean>"+std::to_string(.1*(axis+1))+
          "</bias_mean><dynamic_bias_stddev>100</dynamic_bias_stddev><dynamic_bias_correlation_time>2</dynamic_bias_correlation_time></noise></"+name+">";
    }
  }
  else throw std::runtime_error("unknown native magnetic noise mode");
  gz::math::Rand::Seed(42);
  const std::string xml="<sdf version='1.12'><model name='rig'><link name='body'><sensor name='magnetometer' type='magnetometer'>"
      "<topic>/m23/magnetometer</topic><frame_id>rig/body/magnetometer</frame_id><magnetometer>"+noise+
      "</magnetometer></sensor></link></model></sdf>";
  sdf::SDFPtr parsed(new sdf::SDF);sdf::init(parsed);
  if (!sdf::readString(xml,parsed)) throw std::runtime_error("native magnetic noise SDF refused");
  gz::sensors::SensorFactory factory;
  auto sensor=factory.CreateSensor<gz::sensors::MagnetometerSensor>(parsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
  if (!sensor) throw std::runtime_error("native magnetic noise sensor creation failed");
  std::mutex mutex;std::condition_variable ready;std::vector<Message> messages;bool overflow=false;
  gz::transport::Node node;bool mapsAttempted=false;unsigned int current=0;
  auto capture=[&] {if (!mapsAttempted) {mapsAttempted=true;maps(mapsPath);}};
  try
  {
    if (!node.Subscribe("/m23/magnetometer",std::function<void(const Message &)>([&](const Message &message)
        {std::lock_guard<std::mutex> guard(mutex);if (messages.size()<100) messages.push_back(message);else overflow=true;ready.notify_all();})))
      throw std::runtime_error("native magnetic noise subscribe failed");
    const auto deadline=std::chrono::steady_clock::now()+5s;
    while (!sensor->HasConnections()) {if (std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("native connection deadline");std::this_thread::sleep_for(1ms);}
    sensor->SetWorldMagneticField(gz::math::Vector3d(2e-5,-3e-5,4e-5));
    const unsigned int count=mode=="white" ? 100 : 6;
    std::array<std::vector<double>,3> residuals;
    const std::array<gz::math::Pose3d,6> poses={gz::math::Pose3d(0,0,0,0,0,0),gz::math::Pose3d(5,-2,8,0,0,0),
      gz::math::Pose3d(0,0,0,.2,-.3,.4),gz::math::Pose3d(1,2,3,.7,.1,-.2),
      gz::math::Pose3d(0,0,0,-.4,.6,1.2),gz::math::Pose3d(-3,5,9,.3,.4,-.2)};
    for (current=0;current<count;++current)
    {
      const auto &pose=poses[mode=="white" ? 2 : current];sensor->SetWorldPose(pose);
      const long long timeNs=(current+1)*(mode=="bias-zero-dt" ? 1000000000LL : 1000000LL);
      if (!sensor->Update(std::chrono::nanoseconds(timeNs))) throw std::runtime_error("native magnetic noise update failed");
      std::unique_lock<std::mutex> guard(mutex);
      if (!ready.wait_for(guard,5s,[&]{return messages.size()>=current+1 || overflow;})) throw std::runtime_error("native magnetic noise message deadline");
      if (overflow || messages.size()!=current+1) throw std::runtime_error("native magnetic noise count differs");
      const auto &message=messages[current];std::string frame,sequence;
      for (const auto &entry:message.header().data()) {
        if (entry.key()=="frame_id" && entry.value_size()==1) frame=entry.value(0);
        if (entry.key()=="seq" && entry.value_size()==1) sequence=entry.value(0);
      }
      const long long actualTime=message.header().stamp().sec()*1000000000LL+message.header().stamp().nsec();
      if (!message.has_field_tesla() || frame!="rig/body/magnetometer" || sequence!=std::to_string(current) || actualTime!=timeNs)
        throw std::runtime_error("native canonical magnetic fields differ");
      const auto &field=message.field_tesla();const std::array<double,3> actual={field.x(),field.y(),field.z()};const auto base=rotated(pose.Rot());
      for (unsigned int axis=0;axis<3;++axis) {
        if (!std::isfinite(actual[axis])) throw std::runtime_error("native noisy vector is nonfinite");
        const double residual=actual[axis]-base[axis];residuals[axis].push_back(residual);
        double expected=base[axis]+mean[axis];
        if (mode=="bias-zero-dt") {if (std::abs(std::abs(residuals[axis][0])-.1*(axis+1))>1e-15) throw std::runtime_error("native signed bias differs");expected+=residuals[axis][0];}
        if (precision[axis]>1e-6) expected=std::round(expected/precision[axis])*precision[axis];
        if (mode!="white" && std::abs(actual[axis]-expected)>1e-15) throw std::runtime_error("native post-rotation noise/no-elapsed-bias differs");
      }
      const auto &q=pose.Rot();
      std::cout<<"{\"mode\":\""<<mode<<"\",\"time_ns\":"<<timeNs<<",\"sequence\":\""<<sequence<<"\",\"frame_id\":\""<<frame
        <<"\",\"orientation\":["<<q.X()<<','<<q.Y()<<','<<q.Z()<<','<<q.W()<<"],\"field_tesla\":["<<actual[0]<<','<<actual[1]<<','<<actual[2]<<"]}\n";
    }
    if (mode=="white") for (unsigned int axis=0;axis<3;++axis) moments(residuals[axis],mean[axis],deviation[axis]);
    capture();
  }
  catch (...) {
    const auto original=std::current_exception();std::cerr<<"native magnetic noise mode="<<mode<<" index="<<current<<'\n';
    try {capture();}catch(const std::exception &error){std::cerr<<"native live maps failure: "<<error.what()<<'\n';}catch(...){std::cerr<<"unknown native live maps failure\n";}
    std::rethrow_exception(original);
  }
}
int main(int argc,char **argv)
try {if (argc!=3) return 2;std::cout<<std::setprecision(17);run(argv[1],argv[2]);}
catch (const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
