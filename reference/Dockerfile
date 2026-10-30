# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
FROM docker.io/library/ubuntu@sha256:a2d43bffc46670d7108bb3c0468c67c5557c4c92e83e1e34cbff1322ad1b94dc AS reference-deps

# Signed Ubuntu metadata binds these public PEM anchors; no host trust store is copied.
ADD --checksum=sha256:641de77d8f142cfd62a1a6f964ba67b20754d3337c480efb529d086075a06c9a https://snapshot.ubuntu.com/ubuntu/20261024T000000Z/pool/main/c/ca-certificates/ca-certificates_20240203_all.deb /tmp/ca.deb
RUN mkdir /tmp/ca-bootstrap \
 && dpkg-deb --extract /tmp/ca.deb /tmp/ca-bootstrap \
 && mkdir -p /etc/ssl/certs \
 && cat /tmp/ca-bootstrap/usr/share/ca-certificates/mozilla/*.crt > /etc/ssl/certs/ca-certificates.crt \
 && rm -rf /tmp/ca-bootstrap /tmp/ca.deb

COPY reference/apt-sources.sh /reference/reference/apt-sources.sh
RUN sh /reference/reference/apt-sources.sh /etc/apt 20261024T000000Z

COPY reference/headless-tpe.lock.json /reference/reference/headless-tpe.lock.json
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt-get -o APT::Update::Error-Mode=any update \
 && apt-get install -y --no-install-recommends python3 ca-certificates \
 && python3 -c 'import json, subprocess; lock=json.load(open("/reference/reference/headless-tpe.lock.json")); subprocess.run(["apt-get", "install", "-y", "--no-install-recommends", *lock["apt_dependencies"]], check=True)' \
 && rm -rf /var/lib/apt/lists/*

FROM reference-deps AS headless-tpe
COPY planning/baseline.json /reference/planning/baseline.json
COPY reference/build-reference.py reference/Dockerfile reference/Dockerfile.dockerignore reference/ubuntu-bootstrap.json /reference/reference/
COPY scripts/inventory-native-tests.py /reference/scripts/inventory-native-tests.py
COPY planning/reference/non-gtest-registration-contracts.json /reference/planning/reference/
COPY LICENSE NOTICE /reference/
ARG BUILD_JOBS=2
RUN python3 /reference/reference/build-reference.py --jobs "$BUILD_JOBS"

ENV CMAKE_PREFIX_PATH=/reference/install
ENV PKG_CONFIG_PATH=/reference/install/lib/pkgconfig
ENV LD_LIBRARY_PATH=/reference/install/lib
WORKDIR /reference
