#!/bin/bash
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
sleep 0.2
printf '\027\000\000\000{"Ready":{"version":2}}'
IFS= read -r -n 1 _
printf '\045\000\000\000{"SnapshotAccepted":{"request_id":1}}'
exec sleep 60
