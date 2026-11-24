// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Private-pipe capability negotiation for a solver's capacity and friction law.
use crate::physics::{NativeFrictionModel, NativeSolverProfile};

pub(super) struct NativeWireProfile {
    cold: &'static str,
    image: &'static str,
    pub(super) offer: Option<&'static [u8]>,
}
const STANDARD: NativeWireProfile = NativeWireProfile {
    cold: "--managed",
    image: "--managed-image",
    offer: None,
};
const EXTENDED: NativeWireProfile = NativeWireProfile {
    cold: "--managed-native64-v1",
    image: "--managed-native64-image-v1",
    offer: Some(super::EXTENDED_OFFER),
};
const POINT32: NativeWireProfile = NativeWireProfile {
    cold: "--managed-point32-v1",
    image: "--managed-point32-image-v1",
    offer: Some(b"GZNATIVE32/POINT/1\n"),
};
const POINT64: NativeWireProfile = NativeWireProfile {
    cold: "--managed-point64-v1",
    image: "--managed-point64-image-v1",
    offer: Some(b"GZNATIVE64/POINT/1\n"),
};
pub(super) fn wire_profile(p: NativeSolverProfile) -> &'static NativeWireProfile {
    match p {
        NativeSolverProfile::Standard32 => &STANDARD,
        NativeSolverProfile::Extended64 => &EXTENDED,
        NativeSolverProfile::Standard32PerPoint => &POINT32,
        NativeSolverProfile::Extended64PerPoint => &POINT64,
    }
}
impl NativeWireProfile {
    pub(super) fn mode(&self, image: bool) -> &'static str {
        if image { self.image } else { self.cold }
    }
}

/// Parse a managed child flag. The positive offer identifies the exact native
/// model before any seed or trusted native bytes leave the parent.
pub fn managed_worker_profile(flag: &str) -> Option<(NativeSolverProfile, bool)> {
    for capacity in [
        NativeSolverProfile::Standard32,
        NativeSolverProfile::Extended64,
    ] {
        for p in [
            capacity,
            capacity.with_friction(NativeFrictionModel::PerPoint),
        ] {
            let wire = wire_profile(p);
            if flag == wire.cold {
                return Some((p, false));
            }
            if flag == wire.image {
                return Some((p, true));
            }
        }
    }
    None
}
