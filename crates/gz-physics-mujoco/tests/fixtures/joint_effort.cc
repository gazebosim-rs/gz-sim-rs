// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mujoco/mujoco.h>

int main(int argc, char **argv) {
  if (argc != 2 || mj_version() != 3011000) return 2;
  char error[1024]{};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(mj_parseXMLString(
      "<mujoco><option timestep='0.001' gravity='0 0 0' integrator='implicitfast'/>"
      "<worldbody><body name='free' pos='3 0 0'><freejoint/>"
      "<inertial pos='0 0 0' mass='1' diaginertia='0.016 0.016 0.016'/>"
      "<geom type='sphere' size='0.2'/></body><body name='driven'>"
      "<inertial pos='0 0 0' mass='1' diaginertia='0.016 0.016 0.016'/>"
      "<joint name='hinge' type='hinge' axis='0 1 0' limited='false' damping='0'/>"
      "<geom type='sphere' size='0.2'/></body></worldbody></mujoco>",
      nullptr, error, sizeof(error)), mj_deleteSpec);
  if (!spec) { std::cerr << error; return 3; }
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_compile(spec.get(), nullptr), mj_deleteModel);
  if (!model || model->nv != 7 || model->nu != 0) return 4;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  if (!data) return 5;
  const int joint = mj_name2id(model.get(), mjOBJ_JOINT, "hinge");
  if (joint < 0) return 6;
  const int dof = model->jnt_dofadr[joint], position = model->jnt_qposadr[joint];
  if (dof != 6) return 7;
  std::cout << std::setprecision(17);
  for (double effort : {1.5, 0.0, -3.0, 0.0, 2.0, 0.0}) {
    data->qfrc_applied[dof] = effort;
    mj_step(model.get(), data.get());
    data->qfrc_applied[dof] = 0.0;
    for (int i = 0; i < dof; ++i) if (data->qvel[i] != 0.0) return 8;
    std::cout << "{\"effort\":" << effort << ",\"q\":" << data->qpos[position]
              << ",\"dq\":" << data->qvel[dof] << ",\"time\":" << data->time << "}\n";
  }
  std::ifstream maps("/proc/self/maps");
  std::ofstream recorded(argv[1]);
  recorded << maps.rdbuf();
  return recorded ? 0 : 9;
}
