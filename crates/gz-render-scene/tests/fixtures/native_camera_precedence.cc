// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

// This invokes the pinned sensor library's actual calibration method on an
// Ogre camera. It creates native rendering resources; coordinate before running.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Utils.hh>
#include <gz/sensors/CameraSensor.hh>
#include <sdf/Camera.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>
#include <sdf/World.hh>

int main(int argc, char **argv) {
  if (argc != 2 || argv[1][0] != '/')
    throw std::runtime_error("expected absolute engine plugin path");
  auto *engine = gz::rendering::engine(argv[1], {{"headless", "1"}});
  if (!engine) throw std::runtime_error("ogre2 unavailable");
  if (!engine->Headless()) throw std::runtime_error("engine is not headless");
  auto scene = engine->CreateScene("camera-precedence-probe");
  if (!scene) throw std::runtime_error("scene unavailable");
  const std::string k = "<intrinsics><fx>40</fx><fy>41</fy><cx>20</cx><cy>21</cy><s>0</s></intrinsics>";
  const std::string p = "<projection><p_fx>50</p_fx><p_fy>51</p_fy><p_cx>25</p_cx><p_cy>26</p_cy><tx>0</tx><ty>0</ty></projection>";
  const std::string lens[] = {"", k, p, k + p};
  gz::sensors::CameraSensor sensor;
  for (unsigned int i = 0; i < 4; ++i) {
    auto camera = scene->CreateCamera("camera-" + std::to_string(i));
    if (!camera) throw std::runtime_error("camera unavailable");
    camera->SetImageWidth(64);
    camera->SetImageHeight(64);
    camera->SetAspectRatio(1.0);
    camera->SetNearClipPlane(0.1);
    camera->SetFarClipPlane(10.0);
    sdf::Root root;
    const std::string xml = "<sdf version='1.12'><world name='w'><model name='m'><link name='l'><sensor name='c' type='camera'><camera><horizontal_fov>1.5707963267948966</horizontal_fov><image><width>64</width><height>64</height></image><clip><near>0.1</near><far>10</far></clip><lens>" + lens[i] + "</lens></camera></sensor></link></model></world></sdf>";
    const auto errors = root.LoadSdfString(xml);
    if (!errors.empty()) throw std::runtime_error(errors.front().Message());
    const auto *parsed = root.WorldByIndex(0)->ModelByIndex(0)->LinkByIndex(0)->SensorByIndex(0)->CameraSensor();
    if (!parsed) throw std::runtime_error("camera missing");
    sdf::Camera config = *parsed;
    if (config.HasLensIntrinsics() != (i == 1 || i == 3) ||
        config.HasLensProjection() != (i == 2 || i == 3))
      throw std::runtime_error("unexpected lens presence");
    camera->SetHFOV(GZ_PI / 2.0);
    sensor.UpdateLensIntrinsicsAndProjection(camera, config);
    const auto intrinsic = gz::rendering::projectionToCameraIntrinsic(camera->ProjectionMatrix(), 64, 64);
    const double expected = i >= 2 ? 50.0 : (i == 1 ? 40.0 : 32.0);
    if (std::abs(intrinsic(0, 0) - expected) > 1e-5)
      throw std::runtime_error("unexpected effective focal length");
    if (i == 3 && (config.LensIntrinsicsFx() != 40.0 || config.LensProjectionFx() != 50.0))
      throw std::runtime_error("K and P were not retained separately");
    std::cout << i << " effective_fx=" << intrinsic(0, 0)
              << " k_fx=" << config.LensIntrinsicsFx()
              << " p_fx=" << config.LensProjectionFx() << std::endl;
  }
  engine->DestroyScene(scene);
}
