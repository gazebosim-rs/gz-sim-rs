// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include "JointPositionController.hh"
#include "native_m58_joint_controller_reference.hh"
#include <gz/transport/parameters/Registry.hh>
#include <gz/transport/parameters/result.hh>
namespace {
struct Registry {
  static constexpr bool enabled=true;
  gz::transport::parameters::ParametersRegistry registry;
  std::string stable;std::array<std::string,2> prefixes;std::array<std::string,16> names;std::uint64_t gets=0;
  explicit Registry(const std::string &ns):registry(ns){}
  void bind(m58::ECM &ecm,const m58::Graph &g){
    const std::array<std::string,8> suffixes={"p_gain","i_gain","d_gain","i_max","i_min","cmd_max","cmd_min","cmd_offset"};
    std::set<std::string> unique;
    for(unsigned owner=0;owner<2;++owner){prefixes[owner]="JointPositionController."+gz::sim::scopedName(g.joints[owner*2],ecm,".",false)+".";m58::require(prefixes[owner].size()<=112,"actual scoped prefix cap");
      for(unsigned s=0;s<8;++s){auto &name=names[owner*8+s];name=prefixes[owner]+suffixes[s];m58::require(name.size()<=128 && unique.insert(name).second,"owner-distinct scoped parameter names");}}
  }
  std::string prefix(unsigned owner)const{return m58::quoted(prefixes[owner]);}
  std::string reference()const{return stable;}
  std::string values(unsigned count=16){
    std::vector<std::string> result;
    for(unsigned i=0;i<count;++i){gz::msgs::Double value;const auto actual=registry.Parameter(names[i],value);++gets;
      m58::require(actual.ResultType()==gz::transport::parameters::ParameterResultType::Success,"genuine typed Parameter Success");
      m58::require(actual.ParamName().size()<=128&&actual.ParamType().size()<=32,"actual ParameterResult fields cap");
      result.push_back("{\"parameter_index\":"+std::to_string(i)+",\"requested_name\":"+m58::quoted(names[i])+",\"result\":{\"type\":\"Success\",\"parameter_name\":"+m58::quoted(actual.ParamName())+",\"parameter_type\":"+m58::quoted(actual.ParamType())+"},\"value\":"+m58::doubleJson(value)+'}');
    }return m58::join(result);
  }
  std::string declarations(unsigned count){
    const auto list=registry.ListParameters();m58::require(list.parameter_declarations_size()==static_cast<int>(count),"complete actual declaration count");std::vector<std::string> rows;std::set<std::string> found;
    for(const auto &d:list.parameter_declarations()){
      m58::require(d.name().size()<=128&&d.type().size()<=32&&d.type()=="gz.msgs.Double"&&found.insert(d.name()).second,"actual declaration name/type uniqueness");
      rows.push_back("{\"name\":"+m58::quoted(d.name())+",\"type\":"+m58::quoted(d.type())+",\"wire\":"+m58::wire(d,256)+'}');
    }
    for(unsigned i=0;i<count;++i)m58::require(found.count(names[i])==1,"exact scoped registry declarations");
    return "{\"declarations\":"+m58::join(rows)+",\"wire\":"+m58::wire(list,16384)+'}';
  }
  void empty(m58::Output &out){out.emit("registry",",\"stage\":\"empty\",\"configure_parameters_call\":null,\"declaration_list\":"+declarations(0)+",\"values\":[]");}
  template<class Owners>void setup(m58::Output &out,Owners &owners,m58::ECM &ecm,const m58::Graph &g){
    for(unsigned i=0;i<2;++i){auto *api=dynamic_cast<gz::sim::ISystemConfigureParameters *>(owners[i].system.get());m58::require(api,"genuine public parameter interface");api->ConfigureParameters(registry,ecm);
      const auto count=(i+1)*8;const auto list=declarations(count);const auto actual=values(count);if(i==1)stable=actual;
      out.emit("registry",",\"stage\":"+m58::quoted(i?"after_position2":"after_position0")+",\"configure_parameters_call\":{\"index\":"+std::to_string(i)+",\"slot\":"+std::to_string(owners[i].slot)+",\"model_entity\":"+std::to_string(g.models[i])+",\"joint_entity\":"+std::to_string(g.joints[owners[i].slot])+",\"interface\":\"ISystemConfigureParameters\"},\"declaration_list\":"+list+",\"values\":"+actual);
    }
  }
};
}
int main(int argc,char **argv){
  try{m58::require(argc==3&&std::string(argv[1])=="two_owners","expected two_owners MAPS_PATH");
    const std::filesystem::path maps(argv[2]);m58::initConsole(maps);
    gz::sim::EntityComponentManager ecm;gz::sim::EventManager events;
    Registry registry("/m58/"+maps.filename().string()+"/parameters");
    const std::vector<m58::Phase> phases={m58::Phase{0,0x3fd0000000000000ULL,{m58::Sample{{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},500000000,false,false,0}}},
      m58::Phase{2,0xbfd8000000000000ULL,{m58::Sample{{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},1000000,false,false,1}}},
      m58::Phase{0,0xbfe8000000000000ULL,{m58::Sample{{{0x4020000000000000ULL,0x0000000000000000ULL,0xc01c000000000000ULL,0x0000000000000000ULL}},{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},1000000,false,false,2},m58::Sample{{{0x4020000000000000ULL,0x0000000000000000ULL,0xc01c000000000000ULL,0x0000000000000000ULL}},{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},0,true,false,3}}},
      m58::Phase{2,0x3ff0000000000000ULL,{m58::Sample{{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},{{0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL}},1000000,false,true,4}}}};
    m58::run(ecm,events,registry,"position_abs",{0,2},phases,maps,[]{return std::unique_ptr<gz::sim::System>(new gz::sim::systems::JointPositionController());});return 0;
  }catch(const std::exception &e){std::cerr<<"M58 original position helper failure: "<<e.what()<<'\n';return 1;}
}
