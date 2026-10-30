// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#include "CameraSensorUtil.hh"

int main() {
  std::cout << std::setprecision(17);
  for (double skew : {0.0, 35.0}) {
    const auto matrix = gz::sensors::buildProjectionMatrix(640, 480, 400, 450, 270, 190, skew, 0.1, 100);
    std::cout << "skew " << skew << '\n';
    for (int row=0; row<4; ++row) {
      for (int col=0; col<4; ++col) std::cout << matrix(row,col) << ' ';
      std::cout << '\n';
    }
  }
}
