# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
# Verify this local base tag's image ID against the lock before and after build.
FROM gazebosim-rs-reference:headless-tpe-v6
COPY planning/baseline.json /reference/planning/baseline.json
COPY reference/build-reference.py reference/build-dart-reference.py reference/headless-dart.lock.json reference/Dockerfile.dart reference/Dockerfile.dart.dockerignore /reference/reference/
COPY reference/osrf-dart/ /reference/reference/osrf-dart/
ADD --checksum=sha256:7602bf8695cb1e673be9008ed17222e9ff1a08b3bbaa211a05a4ce73c579037e https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart-core+collisions+odelcpsolver6.13_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart-core+collisions+odelcpsolver6.13_6.13.2+ds1-1~osrf2~noble_amd64.deb
ADD --checksum=sha256:3b557b844dbb6c30160c81e7c722b8f00856425de092df2114c0a446c01f3b9e https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart-utils-urdf6.13_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart-utils-urdf6.13_6.13.2+ds1-1~osrf2~noble_amd64.deb
ADD --checksum=sha256:5e10c6f134f74f3e6100131f0db2b210affe20036de05539545fbf4e14fd7d35 https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart-utils6.13_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart-utils6.13_6.13.2+ds1-1~osrf2~noble_amd64.deb
ADD --checksum=sha256:77d4eb08a749799c7a3eaabf1e09416d6c84dcc0f75a7f571136ab2051873f41 https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart6.13-collision-bullet-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart6.13-collision-bullet-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb
ADD --checksum=sha256:a7a5687783f203d6d3e2cdb8e2f9692d2a15866f3df2d93de38745db207e36eb https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart6.13-collision-ode-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart6.13-collision-ode-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb
ADD --checksum=sha256:6ff3749ca0bf18b47604a2d480d9eadb301100641893fc1a5dcde4e4c8baaba2 https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart6.13-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart6.13-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb
ADD --checksum=sha256:9386428b8aeceb4a117fce4840633f90368569ba94cfa5bfc20311d8b7827e49 https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart6.13-external-convhull-3d-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart6.13-external-convhull-3d-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb
ADD --checksum=sha256:0b84a40ce11bad777c103297e92e23721c2ea14aa37df60a4257fc751a644f6a https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart6.13-external-odelcpsolver-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart6.13-external-odelcpsolver-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb
ADD --checksum=sha256:73f7f573aea947218098b5bb3ed88bccb488ac9e06a597f0ead58df0238df6dc https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart6.13-utils-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart6.13-utils-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb
ADD --checksum=sha256:3dd0348a294184ca3357d4f20089aa0a5d01a83e9d14325fb6314bbaadc905b2 https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/d/dart/libdart6.13-utils-urdf-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb /tmp/dart-debs/libdart6.13-utils-urdf-dev_6.13.2+ds1-1~osrf2~noble_amd64.deb
ARG BUILD_JOBS=2
RUN python3 /reference/reference/build-dart-reference.py --debs /tmp/dart-debs --jobs "$BUILD_JOBS"
ENV CMAKE_PREFIX_PATH=/reference/dart-install:/reference/install
ENV PKG_CONFIG_PATH=/reference/dart-install/lib/pkgconfig:/reference/install/lib/pkgconfig
ENV LD_LIBRARY_PATH=/reference/dart-install/lib:/reference/install/lib:/usr/lib/x86_64-linux-gnu
WORKDIR /reference
