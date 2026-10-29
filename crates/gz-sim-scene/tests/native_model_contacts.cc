// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <vector>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <string>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
using Features = gz::physics::FeatureList<gz::physics::ForwardStep,
  gz::physics::GetShapeFromLink, gz::physics::GetContactsFromLastStepFeature, gz::physics::GetModelFromWorld, gz::physics::GetLinkFromModel, gz::physics::LinkFrameSemantics,
  gz::physics::sdf::ConstructSdfWorld>;
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  for (int mode = 0; mode < 6; ++mode) {
    gz::plugin::Loader loader; loader.LoadLib(argv[1]);
    auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate("gz::physics::dartsim::Plugin"));
    if (!engine) return 3;
    std::string xml = "<sdf version='1.11'><world name='w'><gravity>0 0 0</gravity><model name='a'>";
    if (mode == 3 || mode == 5) xml += "<self_collide>true</self_collide>";
    for (int i = 0; i < (mode >= 4 ? 3 : 2); ++i) {
      if (mode == 1 && i == 1) xml += "</model><model name='b'>";
      xml += "<link name='l" + std::to_string(i) + "'><pose>" + std::to_string(mode >= 4 ? 0.0 : i * .75)
        + " 0 0 0 0 0</pose>";
      if (mode == 2) xml += "<self_collide>true</self_collide>";
      xml += "<collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link>";
    }
    if (mode >= 4) {
      xml += "<joint name='ab' type='revolute'><parent>l0</parent><child>l1</child><axis><xyz>0 1 0</xyz></axis></joint>"
        "<joint name='bc' type='revolute'><parent>l1</parent><child>l2</child><axis><xyz>0 1 0</xyz></axis></joint>";
    }
    xml += "</model></world></sdf>";
    sdf::Root root; auto errors = root.LoadSdfString(xml);
    if (!errors.empty()) { std::cerr << errors[0].Message(); return 4; }
    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    auto first = world->GetModel("a")->GetLink("l0");
    auto second = world->GetModel(mode == 1 ? "b" : "a")->GetLink(mode >= 4 ? "l2" : "l1");
    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;
    input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
    for (int i = 0; i < 100; ++i) world->Step(output, state, input);
    using Point = gz::physics::GetContactsFromLastStepFeature::World<gz::physics::FeaturePolicy3d, Features>::ContactPoint;
    std::vector<std::string> pairs;
    for (const auto &contact : world->GetContactsFromLastStep()) {
      const auto &point = contact.Get<Point>();
      auto a = point.collision1->GetLink()->GetName();
      auto b = point.collision2->GetLink()->GetName();
      if (a > b) std::swap(a, b);
      pairs.push_back(a + ":" + b);
    }
    std::sort(pairs.begin(), pairs.end());
    std::cout << std::setprecision(17) << mode << " "
      << first->FrameDataRelativeToWorld().pose.translation().x() << " "
      << second->FrameDataRelativeToWorld().pose.translation().x() << " " << pairs.size();
    for (const auto &pair : pairs) std::cout << " " << pair;
    std::cout << "\n";
    if (mode >= 4) {
      std::ifstream maps("/proc/self/maps");
      std::ofstream output(argv[2]); output << maps.rdbuf();
      if (!output) return 7;
    }
  }
}
