// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Test-only direct original-object ledger. Utilities follow the reviewed M57 helper.
#ifndef NATIVE_M58_JOINT_CONTROLLER_REFERENCE_HH_
#define NATIVE_M58_JOINT_CONTROLLER_REFERENCE_HH_
#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
namespace m58 {
using namespace std::chrono_literals;
namespace c=gz::sim::components;
using Entity=gz::sim::Entity;
using ECM=gz::sim::EntityComponentManager;
using Clock=std::chrono::steady_clock;
static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559,"IEEE binary64");
inline void require(bool v,const std::string &why){if(!v)throw std::runtime_error(why);}
inline std::uint64_t word(double v){std::uint64_t n;std::memcpy(&n,&v,8);return n;}
inline double number(std::uint64_t n){double v;std::memcpy(&v,&n,8);require(std::isfinite(v),"finite input");return v;}
inline std::string quoted(const std::string &s){
  require(s.size()<=32768,"bounded string");constexpr char h[]="0123456789abcdef";
  std::string r="\"";for(unsigned char x:s){require(x<128,"ASCII text");
    if(x=='"'||x=='\\'){r+='\\';r+=static_cast<char>(x);}
    else if(x<32){r+="\\u00";r+=h[x>>4];r+=h[x&15];}else r+=static_cast<char>(x);
  }return r+'"';
}
inline std::string hex(const std::string &s){constexpr char h[]="0123456789abcdef";std::string r;for(unsigned char x:s){r+=h[x>>4];r+=h[x&15];}return r;}
inline std::string bits(double v){require(std::isfinite(v),"finite actual value");std::ostringstream s;s<<std::hex<<std::setfill('0')<<std::setw(16)<<word(v);return m58::quoted(s.str());}
template<class Message>std::string wire(const Message &m,std::size_t cap){require(m.ByteSizeLong()<=cap,"wire cap");return m58::quoted(hex(m.SerializeAsString()));}
inline std::string doubleJson(const gz::msgs::Double &m){
  require(!m.has_header(),"unexpected actual Double Header in controlled campaign");
  return "{\"header\":null,\"data_bits\":"+bits(m.data())+",\"wire\":"+wire(m,32)+'}';
}
inline std::string join(const std::vector<std::string> &v){std::string r="[";for(std::size_t i=0;i<v.size();++i){if(i)r+=',';r+=v[i];}return r+']';}
struct Output {
  std::string family;std::uint64_t rows=0,bytes=0;
  std::uint64_t emit(const std::string &kind,const std::string &fields){
    const auto ordinal=rows;const std::string row="{\"schema\":\"M58_ISOLATED_CONTROLLER_COMPONENT_LEDGER_V1\",\"family\":"+m58::quoted(family)+",\"kind\":"+m58::quoted(kind)+",\"ordinal\":"+std::to_string(rows)+fields+"}\n";
    const auto cap=kind=="registry"?65536u:(kind=="round_input"||kind=="pre"||kind=="round_end"?8192u:4096u);
    require(row.size()<=cap && rows<4096 && bytes+row.size()<=16777216,"exact encoded ledger caps");
    std::cout<<row;std::cout.flush();require(bool(std::cout),"actual stdout write");++rows;bytes+=row.size();return ordinal;
  }
};
inline void initConsole(const std::filesystem::path &mapPath){
  require(mapPath.is_absolute(),"owned absolute maps path");
  require(gz::common::Console::Init(mapPath.parent_path().string(),mapPath.filename().string()+".console.log"),"owned Console Init");
  require(gz::common::Console::Directory()==mapPath.parent_path().string(),"owned Console directory");gz::common::Console::SetVerbosity(2);require(gz::common::Console::Verbosity()==2,"warning-visible Console2");
}
inline void saveMaps(const std::filesystem::path &path){
  std::ifstream in("/proc/self/maps");require(bool(in),"maps open");std::string all,line;
  while(std::getline(in,line)){
    require(line.find("joint-controller-system")==std::string::npos && line.find("joint-position-controller-system")==std::string::npos,"original controller DSO mapped");
    require(all.size()+line.size()+1<=2097152,"maps cap");all+=line+'\n';
  }require(in.eof(),"complete maps");const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);require(fd>=0,"exclusive maps");
  std::size_t n=0;while(n<all.size()){const auto wrote=::write(fd,all.data()+n,all.size()-n);if(wrote<=0){::close(fd);throw std::runtime_error("maps write");}n+=static_cast<std::size_t>(wrote);}
  const bool flushed=::fsync(fd)==0,closed=::close(fd)==0;require(flushed&&closed,"maps durable");
}
struct Graph {
  Entity world;std::array<Entity,2> models{};std::array<Entity,4> joints{};
  std::vector<Entity> order;std::vector<std::string> kinds;
  explicit Graph(ECM &e){
    auto add=[&](const std::string &name,const std::string &kind,Entity parent){
      const auto x=e.CreateEntity();e.CreateComponent(x,c::Name(name));
      if(kind=="World")e.CreateComponent(x,c::World());
      else if(kind=="Model")e.CreateComponent(x,c::Model());
      else if(kind=="Link")e.CreateComponent(x,c::Link());
      else e.CreateComponent(x,c::Joint());
      if(parent)require(e.SetParentEntity(x,parent),"actual graph parent");order.push_back(x);kinds.push_back(kind);return x;
    };
    world=add("default","World",gz::sim::kNullEntity);
    for(unsigned owner=0;owner<2;++owner){models[owner]=add(owner?"arm_right":"arm_left","Model",world);
      add("base","Link",models[owner]);add("tip","Link",models[owner]);
      joints[owner*2]=add("base_hinge","Joint",models[owner]);joints[owner*2+1]=add("tip_hinge","Joint",models[owner]);
    }
    for(auto j:joints){e.CreateComponent(j,c::JointPosition(std::vector<double>{0}));e.CreateComponent(j,c::JointVelocity(std::vector<double>{0}));}
  }
  std::string json(const ECM &e)const{
    std::vector<std::string> v;require(order.size()==11,"eleven actual graph nodes");
    for(std::size_t i=0;i<order.size();++i){const auto x=order[i];const auto name=e.Component<c::Name>(x);require(name && name->Data().size()<=32,"actual graph Name");
      const auto p=e.Component<c::ParentEntity>(x);std::string parent=p?std::to_string(p->Data()):"null";
      require((i==0 && !p)||(i!=0 && p),"actual graph parent presence");
      require(kinds[i]=="World"?e.Component<c::World>(x)!=nullptr:kinds[i]=="Model"?e.Component<c::Model>(x)!=nullptr:kinds[i]=="Link"?e.Component<c::Link>(x)!=nullptr:e.Component<c::Joint>(x)!=nullptr,"actual graph kind");
      v.push_back("{\"entity\":"+std::to_string(x)+",\"kind\":"+m58::quoted(kinds[i])+",\"name\":"+m58::quoted(name->Data())+",\"parent_entity\":"+parent+'}');
    }return join(v);
  }
};
struct Sample {std::array<std::uint64_t,4> q{},v{};std::int64_t dt=1000000000;bool paused=false,equal=false;unsigned measurement=0;};
struct Phase {unsigned slot;std::uint64_t command;std::vector<Sample> measurements;};
inline std::string infoJson(const gz::sim::UpdateInfo &i){
  return "{\"sim_time_ns\":"+std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(i.simTime).count())+",\"real_time_ns\":"+std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(i.realTime).count())+",\"dt_ns\":"+std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(i.dt).count())+",\"iterations\":"+std::to_string(i.iterations)+",\"paused\":"+(i.paused?"true":"false")+'}';
}
inline std::string inputs(ECM &e,const Graph &g,const Sample &s,bool write){
  std::vector<std::string> result;
  for(unsigned slot=0;slot<4;++slot){const auto j=g.joints[slot];
    if(write){(void)e.SetComponentData<c::JointPosition>(j,{number(s.q[slot])});(void)e.SetComponentData<c::JointVelocity>(j,{number(s.v[slot])});}
    const auto q=e.Component<c::JointPosition>(j);const auto v=e.Component<c::JointVelocity>(j);
    require(q&&v&&q->Data().size()==1&&v->Data().size()==1&&word(q->Data()[0])==s.q[slot]&&word(v->Data()[0])==s.v[slot],"all actual input getbacks");
    result.push_back("{\"slot\":"+std::to_string(slot)+",\"model_entity\":"+std::to_string(g.models[slot/2])+",\"joint_entity\":"+std::to_string(j)+",\"joint_position\":["+bits(q->Data()[0])+"],\"joint_velocity\":["+bits(v->Data()[0])+"]}");
  }return join(result);
}
using Commands=std::array<std::optional<std::vector<std::uint64_t>>,4>;
inline std::pair<std::string,Commands> commands(const ECM &e,const Graph &g,const std::array<unsigned,2> &selected){
  std::vector<std::string> result;Commands values;
  for(unsigned slot=0;slot<4;++slot){const auto j=g.joints[slot];const auto v=e.Component<c::JointVelocityCmd>(j);const auto f=e.Component<c::JointForceCmd>(j);
    require(!f,"force component must actually be absent");std::string vel="null";
    if(v){require(v->Data().size()<=1,"actual selected bounded command");require(slot==selected[0]||slot==selected[1],"nonselected command write");values[slot]=std::vector<std::uint64_t>{};std::vector<std::string> data;for(double x:v->Data()){values[slot]->push_back(word(x));data.push_back(bits(x));}vel=join(data);}
    result.push_back("{\"slot\":"+std::to_string(slot)+",\"joint_entity\":"+std::to_string(j)+",\"joint_velocity_command\":"+vel+",\"joint_force_command\":null}");
  }return {join(result),values};
}
struct Tap {unsigned slot,phase;std::uint64_t arrival,elapsed;gz::msgs::Double message;};
struct Received {
  std::mutex mutex;std::condition_variable changed;bool active=true,armed=false,bad=false;unsigned phase=0,slot=0;std::uint64_t expected=0,total=0;Clock::time_point began=Clock::now();std::vector<Tap> taps;
};
struct NoRegistry {
  static constexpr bool enabled=false;std::uint64_t gets=0;
  void bind(ECM &,const Graph &){}
  std::string prefix(unsigned)const{return "null";}
  void empty(Output &){}
  template<class Owners>void setup(Output &,Owners &,ECM &,const Graph &){}
  std::string values(){return "";}
  std::string reference()const{return "";}
};
struct Owner {
  unsigned slot;std::string topic,xml,discovery;std::shared_ptr<sdf::Element> plugin;
  std::unique_ptr<gz::sim::System> system;gz::sim::ISystemPreUpdate *pre=nullptr;
  std::optional<gz::transport::Node::Publisher> publisher;
};
inline std::string discover(gz::transport::Node &node,const Owner &owner){
  const auto deadline=Clock::now()+5s;
  for(;;){std::vector<gz::transport::MessagePublisher> p,s;const bool ok=node.TopicInfo(owner.topic,p,s);
    if(ok&&p.size()==1&&s.size()==2&&owner.publisher->HasConnections()){
      std::vector<std::string> ps,ss;bool valid=true;
      for(const auto &x:p){valid=valid&&x.MsgTypeName()=="gz.msgs.Double";ps.push_back(m58::quoted(x.MsgTypeName()));}
      for(const auto &x:s){valid=valid&&x.MsgTypeName()=="gz.msgs.Double";ss.push_back(m58::quoted(x.MsgTypeName()));}
      if(valid){require(Clock::now()<deadline,"typed discovery success deadline");return "{\"publisher_type_names\":"+join(ps)+",\"subscriber_type_names\":"+join(ss)+",\"has_connections\":true}";}
    }require(Clock::now()<deadline,"actual typed discovery deadline");std::this_thread::yield();
  }
}
template<class Factory,class Registry>void run(ECM &ecm,gz::sim::EventManager &events,Registry &registry,const std::string &family,const std::array<unsigned,2> &selected,const std::vector<Phase> &phases,const std::filesystem::path &mapPath,Factory factory){
  require(phases.size()==4 && mapPath.is_absolute() && !std::filesystem::exists(mapPath),"fixed mode/owned maps path");
  require(gz::common::Console::Directory()==mapPath.parent_path().string()&&gz::common::Console::Verbosity()==2,"preinitialized owned Console");
  Output out{family};Graph graph(ecm);const auto graphSnapshot=graph.json(ecm);registry.bind(ecm,graph);registry.empty(out);
  std::array<Owner,2> owners;gz::transport::Node node;auto received=std::make_shared<Received>();bool mapsAttempted=false;
  const auto captureMaps=[&]{if(!mapsAttempted){mapsAttempted=true;saveMaps(mapPath);}};
  std::uint64_t rounds=0,calls=0,publications=0,taps=0,fences=0;std::vector<unsigned> probes;
  try{
    for(unsigned index=0;index<2;++index){auto &o=owners[index];o.slot=selected[index];const auto owner=o.slot/2;const std::string model=owner?"arm_right":"arm_left",joint=o.slot%2?"tip_hinge":"base_hinge";
      const std::string filename=Registry::enabled?"gz-sim-joint-position-controller-system":"gz-sim-joint-controller-system",classname=Registry::enabled?"gz::sim::systems::JointPositionController":"gz::sim::systems::JointController";
      const std::string content=Registry::enabled?"<joint_name>base_hinge</joint_name><joint_index>0</joint_index><use_velocity_commands>true</use_velocity_commands><cmd_max>2</cmd_max><initial_position>0</initial_position>":"<joint_name>tip_hinge</joint_name><use_force_commands>false</use_force_commands><initial_velocity>0</initial_velocity>";
      sdf::Plugin plugin(filename,classname,content);o.plugin=plugin.ToElement();require(bool(o.plugin),"genuine parsed Plugin");require(o.plugin->Get<std::string>("joint_name")==joint&&ecm.Component<c::Name>(graph.models[owner])->Data()==model&&ecm.Component<c::Name>(graph.joints[o.slot])->Data()==joint,"actual parsed joint/model getbacks");o.xml=o.plugin->ToString("");require(o.xml.size()<=1024,"actual parsed XML cap");
      o.system=factory();require(bool(o.system),"immediate System base ownership");auto configure=dynamic_cast<gz::sim::ISystemConfigure *>(o.system.get());o.pre=dynamic_cast<gz::sim::ISystemPreUpdate *>(o.system.get());require(configure&&o.pre,"public interfaces");configure->Configure(graph.models[owner],o.plugin,ecm,events);
      o.topic="/model/"+model+"/joint/"+joint+(Registry::enabled?"/0/cmd_pos":"/cmd_vel");
      const auto slot=o.slot;std::function<void(const gz::msgs::Double &)> callback=[received,slot](const gz::msgs::Double &m){std::lock_guard<std::mutex> lock(received->mutex);if(!received->active)return;
        if(!received->armed||received->slot!=slot||m.ByteSizeLong()>32||m.has_header()||!std::isfinite(m.data())||word(m.data())!=received->expected||received->taps.size()>=1||received->total>=4){received->bad=true;received->changed.notify_all();return;}
        try{const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now()-received->began).count();received->taps.push_back({slot,received->phase,received->total,static_cast<std::uint64_t>(elapsed),m});++received->total;}catch(...){received->bad=true;}received->changed.notify_all();};
      require(node.Subscribe(o.topic,callback),"actual typed tap subscription");o.publisher=node.Advertise<gz::msgs::Double>(o.topic);require(bool(*o.publisher),"canonical publisher");o.discovery=discover(node,o);
      std::string settings;
      if constexpr(Registry::enabled){require(o.plugin->Get<unsigned>("joint_index")==0&&o.plugin->Get<bool>("use_velocity_commands"),"actual position settings");settings="{\"joint_index\":0,\"use_velocity_commands\":true,\"cmd_max_bits\":"+bits(o.plugin->Get<double>("cmd_max"))+",\"initial_target_bits\":"+bits(o.plugin->Get<double>("initial_position"))+'}';}
      else{require(!o.plugin->Get<bool>("use_force_commands"),"actual velocity settings");settings="{\"use_force_commands\":false,\"initial_target_bits\":"+bits(o.plugin->Get<double>("initial_velocity"))+'}';}
      out.emit("config",",\"slot\":"+std::to_string(o.slot)+",\"owner\":"+std::to_string(owner)+",\"model_entity\":"+std::to_string(graph.models[owner])+",\"joint_entity\":"+std::to_string(graph.joints[o.slot])+",\"model_name\":"+m58::quoted(model)+",\"joint_name\":"+m58::quoted(joint)+",\"topic\":"+m58::quoted(o.topic)+",\"plugin_xml\":"+m58::quoted(o.xml)+",\"settings\":"+settings+",\"graph\":"+graphSnapshot+",\"parameter_prefix_from_public_joint_scope\":"+registry.prefix(owner)+",\"configure_index\":"+std::to_string(index)+",\"interfaces\":"+(Registry::enabled?"[\"ISystemConfigure\",\"ISystemPreUpdate\",\"ISystemConfigureParameters\"]":"[\"ISystemConfigure\",\"ISystemPreUpdate\"]")+",\"discovery\":"+o.discovery);
    }
    registry.setup(out,owners,ecm,graph);
    std::string stableRegistry;if constexpr(Registry::enabled)stableRegistry=registry.reference();
    gz::sim::UpdateInfo info;std::int64_t sim=0;
    auto round=[&](const Sample &sample,const std::string &role,std::optional<unsigned> phase,std::optional<unsigned> probe){
      require(rounds<274 && calls<548,"bounded rounds/calls");for(auto slot:std::array<unsigned,2>{unsigned(1-selected[0]%2),unsigned(3-selected[0]%2)})require(sample.q[slot]==0&&sample.v[slot]==0,"nonselected zero inputs");
      info.dt=std::chrono::nanoseconds(sample.dt);if(!sample.equal)sim+=sample.dt;info.simTime=std::chrono::nanoseconds(sim);info.realTime=info.simTime;info.iterations=rounds;info.paused=sample.paused;
      const auto actualInputs=inputs(ecm,graph,sample,true);const auto before=commands(ecm,graph,selected);const auto infoText=infoJson(info);
      require(graph.json(ecm)==graphSnapshot,"complete graph unchanged");
      auto reg=[&](const char *key){if constexpr(Registry::enabled){const auto v=registry.values();require(v==stableRegistry,"actual Registry stable");return std::string(",\"")+key+"\":"+v;}else return std::string();};
      out.emit("round_input",",\"round_index\":"+std::to_string(rounds)+",\"phase\":"+(phase?std::to_string(*phase):"null")+",\"role\":"+m58::quoted(role)+",\"probe_index\":"+(probe?std::to_string(*probe):"null")+",\"measurement_index\":"+(role=="measurement"?std::to_string(sample.measurement):"null")+",\"info\":"+infoText+",\"inputs\":"+actualInputs+",\"commands_before\":"+before.first+reg("registry_before"));
      for(unsigned i=0;i<2;++i){const auto actualCall=calls;owners[i].pre->PreUpdate(info,ecm);++calls;const auto state=commands(ecm,graph,selected);require(inputs(ecm,graph,sample,false)==actualInputs&&graph.json(ecm)==graphSnapshot,"all input/graph getbacks after public call");
        out.emit("pre",",\"round_index\":"+std::to_string(rounds)+",\"call_index\":"+std::to_string(actualCall)+",\"slot\":"+std::to_string(owners[i].slot)+",\"info\":"+infoText+",\"inputs_after\":"+actualInputs+",\"commands_after\":"+state.first+reg("registry_after")+",\"selected_index\":"+std::to_string(i));}
      const auto end=commands(ecm,graph,selected);require(inputs(ecm,graph,sample,false)==actualInputs,"round-end input getters");out.emit("round_end",",\"round_index\":"+std::to_string(rounds)+",\"info\":"+infoText+",\"inputs_after\":"+actualInputs+",\"commands_after\":"+end.first+reg("registry_after"));++rounds;
      if(sample.paused)require(end.second==before.second,"actual paused command presence/vectors held");return end.second;
    };
    round(Sample{},"initial",std::nullopt,std::nullopt);
    for(unsigned phase=0;phase<4;++phase){const auto &program=phases[phase];auto found=std::find(selected.begin(),selected.end(),program.slot);require(found!=selected.end(),"selected publication slot");auto &owner=owners[static_cast<unsigned>(found-selected.begin())];
      const auto baselineIndex=rounds;const auto baseline=round(Sample{},"baseline",phase,std::nullopt);require(!baseline[program.slot]||baseline[program.slot]->size()!=1||baseline[program.slot]->front()!=program.command,"fresh marker differs from baseline");
      {std::lock_guard<std::mutex> lock(received->mutex);require(!received->bad&&received->taps.empty()&&!received->armed,"one outstanding publication");received->phase=phase;received->slot=program.slot;received->expected=program.command;received->armed=true;}
      const auto deadline=Clock::now()+5s;gz::msgs::Double message;message.set_data(number(program.command));require(owner.publisher->Publish(message),"genuine canonical Publish");const auto pubOrdinal=out.emit("publication",",\"phase\":"+std::to_string(phase)+",\"slot\":"+std::to_string(program.slot)+",\"topic\":"+m58::quoted(owner.topic)+",\"message\":"+doubleJson(message)+",\"publish_returned_true\":true");++publications;
      Tap tapped;
      {std::unique_lock<std::mutex> lock(received->mutex);require(received->changed.wait_until(lock,deadline,[&]{return received->bad||!received->taps.empty();}),"tap deadline");require(!received->bad&&received->taps.size()==1,"exact tap");tapped=received->taps.front();received->taps.clear();}
      require(tapped.message.SerializeAsString()==message.SerializeAsString(),"full canonical publication/tap wire");const auto tapOrdinal=out.emit("tap",",\"phase\":"+std::to_string(tapped.phase)+",\"slot\":"+std::to_string(tapped.slot)+",\"topic\":"+m58::quoted(owner.topic)+",\"arrival_ordinal\":"+std::to_string(tapped.arrival)+",\"arrival_elapsed_ns\":"+std::to_string(tapped.elapsed)+",\"message\":"+doubleJson(tapped.message));++taps;require(Clock::now()<deadline,"actual tap acceptance deadline");
      std::vector<std::string> actualProbes;bool complete=false;
      for(unsigned i=0;i<32;++i){require(Clock::now()<deadline,"component-effect fence deadline");const auto index=rounds;const auto after=round(Sample{},"probe",phase,i);actualProbes.push_back(std::to_string(index));
        {std::lock_guard<std::mutex> lock(received->mutex);require(!received->bad&&received->taps.empty(),"no duplicate/unexpected tap");}
        bool peers=true;for(unsigned slot=0;slot<4;++slot)if(slot!=program.slot)peers=peers&&after[slot]==baseline[slot];
        complete=after[program.slot]&&after[program.slot]->size()==1&&after[program.slot]->front()==program.command&&peers;if(complete){require(Clock::now()<deadline,"actual component-effect completion deadline");break;}std::this_thread::yield();
      }require(complete,"bounded original effect not observed");
      {std::lock_guard<std::mutex> lock(received->mutex);received->armed=false;}
      require(Clock::now()<deadline,"actual fence acceptance deadline");
      out.emit("fence",",\"phase\":"+std::to_string(phase)+",\"slot\":"+std::to_string(program.slot)+",\"publication_ordinal\":"+std::to_string(pubOrdinal)+",\"tap_ordinal\":"+std::to_string(tapOrdinal)+",\"baseline_round_index\":"+std::to_string(baselineIndex)+",\"probe_round_indices\":"+join(actualProbes)+",\"matched_probe_round_index\":"+actualProbes.back()+",\"completion\":\"OBSERVED_ORIGINAL_PREUPDATE_COMPONENT_EFFECT\",\"evidence_scope\":\"tap-is-not-private-callback-ack\"");++fences;require(Clock::now()<deadline,"actual fence emission deadline");probes.push_back(static_cast<unsigned>(actualProbes.size()));
      for(const auto &sample:program.measurements)round(sample,"measurement",phase,std::nullopt);
    }
    captureMaps();
    {std::lock_guard<std::mutex> lock(received->mutex);require(!received->bad&&received->taps.empty()&&received->total==4,"complete bounded tap ledger");received->active=false;}
    for(auto &o:owners){require(node.Unsubscribe(o.topic),"retire owned tap");o.publisher.reset();}
    const auto expectedRows=(Registry::enabled?18u:15u)+4*rounds;require(out.rows+1==expectedRows&&calls==2*rounds&&publications==4&&taps==4&&fences==4&&registry.gets==(Registry::enabled?24+64*rounds:0),"derived actual Source counts");
    std::vector<std::string> pp;for(auto n:probes)pp.push_back(std::to_string(n));
    out.emit("summary",",\"state\":\"ORIGINAL_M58_COMPONENT_CAPTURE_COMPLETED_PENDING_INDEPENDENT_DATA\",\"probe_round_counts\":"+join(pp)+",\"counts\":{\"configurations\":2,\"registry_records\":"+std::to_string(Registry::enabled?3:0)+",\"round_inputs\":"+std::to_string(rounds)+",\"pre_update_calls\":"+std::to_string(calls)+",\"round_ends\":"+std::to_string(rounds)+",\"publications\":"+std::to_string(publications)+",\"tap_records\":"+std::to_string(taps)+",\"fences\":"+std::to_string(fences)+",\"command_state_getbacks\":"+std::to_string(16*rounds)+",\"helper_registry_get_calls\":"+std::to_string(registry.gets)+",\"raw_rows\":"+std::to_string(expectedRows)+'}');
  }catch(...){
    std::cerr<<"M58 partial actual ledger: rows="<<out.rows<<" rounds="<<rounds<<" returned_PreUpdate_calls="<<calls<<" publications="<<publications<<" taps="<<taps<<" fences="<<fences<<" registry_gets="<<registry.gets<<'\n';
    {std::lock_guard<std::mutex> lock(received->mutex);received->active=false;}
    for(auto &o:owners){if(!o.topic.empty())node.Unsubscribe(o.topic);o.publisher.reset();}
    try{captureMaps();}catch(const std::exception &e){std::cerr<<"maps retention failure: "<<e.what()<<'\n';}throw;
  }
}
} // namespace m58
#endif
