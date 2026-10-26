#!/bin/sh
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
set -eu
reference_apt_root=$1
reference_snapshot=$2
printf '%s\n' "$reference_snapshot" | grep -Eq '^[0-9]{8}T[0-9]{6}Z$'
reference_legacy="$reference_apt_root/sources.list"
if [ -e "$reference_legacy" ] || [ -L "$reference_legacy" ]; then
    if [ ! -f "$reference_legacy" ] || [ ! -r "$reference_legacy" ] || [ -L "$reference_legacy" ]; then
        echo 'Invalid legacy APT source' >&2
        exit 1
    fi
    if grep -Ev '^[[:space:]]*(#.*)?$' "$reference_legacy" > /dev/null; then
        echo 'Unexpected active legacy APT source' >&2
        exit 1
    else
        reference_status=$?
        [ "$reference_status" -eq 1 ] || exit "$reference_status"
    fi
fi
for reference_source in "$reference_apt_root"/sources.list.d/*.list "$reference_apt_root"/sources.list.d/*.sources; do
    if [ ! -e "$reference_source" ] && [ ! -L "$reference_source" ]; then
        continue
    fi
    if [ "$reference_source" != "$reference_apt_root/sources.list.d/ubuntu.sources" ] || [ -L "$reference_source" ]; then
        echo 'Unexpected inherited APT source' >&2
        exit 1
    fi
done
mkdir -p "$reference_apt_root/sources.list.d"
# Historical expiry is disabled; package signatures, hashes and HTTPS stay enabled.
printf '%s\n' 'Types: deb' \
    "URIs: https://snapshot.ubuntu.com/ubuntu/$reference_snapshot/" \
    'Suites: noble noble-updates noble-security' \
    'Components: main universe restricted multiverse' \
    'Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg' \
    'Snapshot: no' 'Check-Valid-Until: no' > "$reference_apt_root/sources.list.d/ubuntu.sources"
