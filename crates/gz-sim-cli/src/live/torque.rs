// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One signed effort for each complete 1 ms physical interval of a driven hinge.
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::{
    io::{self, Read},
    sync::Arc,
};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Interpolation {
    Hold,
    Linear,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum End {
    Hold,
    Zero,
    Repeat,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Knot {
    tick: u64,
    torque_nm: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Program {
    version: u8,
    interpolation: Interpolation,
    end: End,
    knots: Vec<Knot>,
}

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
enum Waveform {
    Constant(f64),
    Scheduled(Program),
}

/// A constant or authored torque timeline. Sampling depends only on the
/// committed physics clock, so pause, restoration and worker placement
/// cannot advance an independent command cursor.
#[derive(Clone, bevy_ecs::prelude::Resource)]
pub struct JointTorqueDrive(Arc<Waveform>);

impl Serialize for JointTorqueDrive {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.0.as_ref().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for JointTorqueDrive {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        match Waveform::deserialize(deserializer)? {
            Waveform::Constant(effort) => Self::constant(effort),
            Waveform::Scheduled(program) => Self::program(program),
        }
        .map_err(serde::de::Error::custom)
    }
}

impl JointTorqueDrive {
    pub fn constant(torque_nm: f64) -> io::Result<Self> {
        if !torque_nm.is_finite() {
            return Err(io::Error::other("expected finite torque"));
        }
        Ok(Self(Arc::new(Waveform::Constant(torque_nm))))
    }

    pub fn read(reader: impl Read) -> io::Result<Self> {
        const MAX_BYTES: usize = 1 << 20;
        let mut bytes = Vec::new();
        reader
            .take((MAX_BYTES + 1) as u64)
            .read_to_end(&mut bytes)?;
        if bytes.len() > MAX_BYTES {
            return Err(io::Error::other("drive program exceeds 1 MiB"));
        }
        Self::program(serde_json::from_slice(&bytes)?)
    }

    fn program(p: Program) -> io::Result<Self> {
        if p.version != 1
            || !(2..=8192).contains(&p.knots.len())
            || p.knots[0].tick != 0
            || p.knots.iter().any(|k| !k.torque_nm.is_finite())
            || p.knots.windows(2).any(|k| k[0].tick >= k[1].tick)
        {
            return Err(io::Error::other(
                "expected version 1 and 2..8192 finite torque knots strictly increasing from tick zero",
            ));
        }
        Ok(Self(Arc::new(Waveform::Scheduled(p))))
    }

    /// Mean continuous torque over [tick, tick+1) in N·m. Integrating a
    /// linear segment at its midpoint preserves its angular impulse.
    pub fn interval_effort(&self, tick: u64) -> f64 {
        let p = match self.0.as_ref() {
            Waveform::Constant(effort) => return *effort,
            Waveform::Scheduled(p) => p,
        };
        let last = p.knots.last().expect("validated nonempty torque timeline");
        let tick = if tick >= last.tick {
            match p.end {
                End::Hold => return last.torque_nm,
                End::Zero => return 0.,
                End::Repeat => tick % last.tick,
            }
        } else {
            tick
        };
        let right = p.knots.partition_point(|k| k.tick <= tick);
        let left = &p.knots[right - 1];
        match p.interpolation {
            Interpolation::Hold => left.torque_nm,
            Interpolation::Linear => {
                let next = &p.knots[right];
                if left.torque_nm == next.torque_nm {
                    return left.torque_nm;
                }
                let fraction = ((tick - left.tick) as f64 + 0.5) / (next.tick - left.tick) as f64;
                left.torque_nm
                    .mul_add(1. - fraction, next.torque_nm * fraction)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn load(mode: &str, end: &str) -> JointTorqueDrive {
        let text = format!(
            r#"{{"version":1,"interpolation":"{mode}","end":"{end}","knots":[{{"tick":0,"torque_nm":-2}},{{"tick":4,"torque_nm":2}},{{"tick":6,"torque_nm":4}}]}}"#
        );
        JointTorqueDrive::read(text.as_bytes()).unwrap()
    }
    #[test]
    fn linear_impulse_and_period_wrap_follow_physics_clock() {
        let d = load("linear", "repeat");
        let cycle: Vec<_> = (0..6).map(|i| d.interval_effort(i)).collect();
        assert_eq!(cycle, [-1.5, -0.5, 0.5, 1.5, 2.5, 3.5]);
        assert_eq!(cycle.iter().sum::<f64>() * 0.001, 0.006);
        assert_eq!(d.interval_effort(u64::MAX), cycle[(u64::MAX % 6) as usize]);
    }
    #[test]
    fn discontinuities_and_end_policy_use_the_left_interval_boundary() {
        let hold = load("hold", "hold");
        assert_eq!(
            (
                hold.interval_effort(3),
                hold.interval_effort(4),
                hold.interval_effort(6)
            ),
            (-2., 2., 4.)
        );
        assert_eq!(load("hold", "zero").interval_effort(6), 0.);
        assert!(JointTorqueDrive::constant(f64::NAN).is_err());
    }
    #[test]
    fn malformed_sources_are_refused_before_installation() {
        for knots in [
            "[]",
            r#"[{"tick":1,"torque_nm":2},{"tick":2,"torque_nm":2}]"#,
            r#"[{"tick":0,"torque_nm":0},{"tick":0,"torque_nm":1}]"#,
        ] {
            let text =
                format!(r#"{{"version":1,"end":"repeat","interpolation":"hold","knots":{knots}}}"#);
            assert!(JointTorqueDrive::read(text.as_bytes()).is_err());
        }
    }
}
