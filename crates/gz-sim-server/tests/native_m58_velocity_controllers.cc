// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include "JointController.hh"
#include "native_m58_joint_controller_reference.hh"
int main(int argc,char **argv){
  try{m58::require(argc==3 && std::string(argv[1])=="two_owners","expected two_owners MAPS_PATH");
    const std::filesystem::path maps(argv[2]);m58::initConsole(maps);
    gz::sim::EntityComponentManager ecm;gz::sim::EventManager events;m58::NoRegistry registry;
    const std::vector<m58::Phase> phases={m58::Phase{1,0x3fe0000000000000ULL,{m58::Sample{{{0x0000000000000000ULL,0x3fc0000000000000ULL,0x0000000000000000ULL,0xbfd0000000000000ULL}},{{0x0000000000000000ULL,0x3fd0000000000000ULL,0x0000000000000000ULL,0xbfe0000000000000ULL}},1000000,false,false,0}}},
      m58::Phase{3,0xbfe4000000000000ULL,{m58::Sample{{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},0,true,false,1}}},
      m58::Phase{1,0xbfec000000000000ULL,{}},
      m58::Phase{3,0x3ff2000000000000ULL,{m58::Sample{{{0x0000000000000000ULL,0x3fc0000000000000ULL,0x0000000000000000ULL,0xbfd0000000000000ULL}},{{0x0000000000000000ULL,0x3fd0000000000000ULL,0x0000000000000000ULL,0xbfe0000000000000ULL}},1000000,false,false,2}}}};
    m58::run(ecm,events,registry,"velocity",{1,3},phases,maps,[]{return std::unique_ptr<gz::sim::System>(new gz::sim::systems::JointController());});return 0;
  }catch(const std::exception &e){std::cerr<<"M58 original velocity helper failure: "<<e.what()<<'\n';return 1;}
}
