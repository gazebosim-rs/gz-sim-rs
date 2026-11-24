// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Immutable public solver interval shared by all peers, in milliseconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SimulationStep {
    #[serde(rename = "1ms")]
    Ms1,
    #[serde(rename = "2ms")]
    Ms2,
    #[serde(rename = "5ms")]
    Ms5,
    #[default]
    #[serde(rename = "10ms")]
    Ms10,
}

impl SimulationStep {
    pub const fn duration(self) -> Duration {
        let millis = match self {
            Self::Ms1 => 1,
            Self::Ms2 => 2,
            Self::Ms5 => 5,
            Self::Ms10 => 10,
        };
        Duration::from_millis(millis)
    }

    pub fn from_duration(duration: Duration) -> Option<Self> {
        [Self::Ms1, Self::Ms2, Self::Ms5, Self::Ms10]
            .into_iter()
            .find(|s| s.duration() == duration)
    }

    pub(crate) fn is_default(&self) -> bool {
        *self == Self::Ms10
    }
}
