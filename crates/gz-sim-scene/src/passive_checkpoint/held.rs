// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::physics::{BodyWrench, MAX_PENDING_IMPULSES};
use serde::{
    Deserializer,
    de::{Error, SeqAccess, Visitor},
};
use std::fmt;

/// Bound the contribution count during decoding, independently of the outer byte limit.
pub(super) fn decode<'de, D: Deserializer<'de>>(d: D) -> Result<Vec<BodyWrench>, D::Error> {
    struct OrderedWrenches;
    impl<'de> Visitor<'de> for OrderedWrenches {
        type Value = Vec<BodyWrench>;
        fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(f, "at most {MAX_PENDING_IMPULSES} ordered world wrenches")
        }
        fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
            let mut held = Vec::new();
            while let Some(value) = seq.next_element()? {
                if held.len() == MAX_PENDING_IMPULSES {
                    return Err(A::Error::invalid_length(held.len() + 1, &self));
                }
                held.push(value);
            }
            Ok(held)
        }
    }
    d.deserialize_seq(OrderedWrenches)
}
