// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Streaming per-solve comparison at identical simulation times.
use super::*;
use glam::{DQuat, DVec3};
use serde::de::DeserializeOwned;
use std::io::{BufRead, BufReader};

#[derive(Clone, Debug, PartialEq, serde::Deserialize, Serialize)]
struct Header {
    version: u8,
    mode: String,
    links: Vec<String>,
}
impl Header {
    fn new(mode: Mode) -> Self {
        Self {
            version: 1,
            mode: match mode {
                Mode::Dynamic => "dynamic",
                Mode::Adaptive => "adaptive",
            }
            .into(),
            links: LINKS.iter().map(|s| s.to_string()).collect(),
        }
    }
}
#[derive(Debug, PartialEq, serde::Deserialize, Serialize)]
struct End {
    commands: Receipt,
    first_resumed_ms: usize,
    mount_impulse_ns: [f64; 3],
}

#[derive(Debug, Default, PartialEq, Serialize)]
pub(super) struct Comparison {
    matched_solves: usize,
    max_position_m: f64,
    max_rotation_rad: f64,
    max_com_velocity_m_s: f64,
    max_angular_velocity_rad_s: f64,
    max_arm_linear_momentum_kg_m_s: f64,
    max_arm_angular_momentum_kg_m2_s: f64,
    max_pair_impulse_ns: f64,
    first_resumed_pair_impulse_error_ns: f64,
    first_resumed_mount_impulse_error_ns: f64,
}
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    DVec3::from(a).distance(DVec3::from(b))
}
impl Comparison {
    fn observe(&mut self, actual: &Sample, reference: &Sample, wake: bool) -> Result<()> {
        if actual.time_ms != reference.time_ms {
            return Err(format!("handoff trace clock differs at {} ms", actual.time_ms).into());
        }
        for (a, b) in actual.bodies.iter().zip(&reference.bodies) {
            self.max_position_m = self
                .max_position_m
                .max(distance(a.position_m, b.position_m));
            let r =
                DQuat::from_array(a.rotation_xyzw).conjugate() * DQuat::from_array(b.rotation_xyzw);
            self.max_rotation_rad = self
                .max_rotation_rad
                .max(2. * r.xyz().length().atan2(r.w.abs()));
            self.max_com_velocity_m_s = self
                .max_com_velocity_m_s
                .max(distance(a.com_velocity_m_s, b.com_velocity_m_s));
            self.max_angular_velocity_rad_s = self
                .max_angular_velocity_rad_s
                .max(distance(a.angular_velocity_rad_s, b.angular_velocity_rad_s));
        }
        self.max_arm_linear_momentum_kg_m_s = self.max_arm_linear_momentum_kg_m_s.max(distance(
            actual.arm_linear_momentum_kg_m_s,
            reference.arm_linear_momentum_kg_m_s,
        ));
        self.max_arm_angular_momentum_kg_m2_s =
            self.max_arm_angular_momentum_kg_m2_s.max(distance(
                actual.arm_angular_momentum_kg_m2_s,
                reference.arm_angular_momentum_kg_m2_s,
            ));
        let error = actual
            .pad_impulses_ns
            .into_iter()
            .zip(reference.pad_impulses_ns)
            .map(|(a, b)| (a - b).abs())
            .fold(
                (actual.support_impulse_ns - reference.support_impulse_ns).abs(),
                f64::max,
            );
        self.max_pair_impulse_ns = self.max_pair_impulse_ns.max(error);
        if wake {
            self.first_resumed_pair_impulse_error_ns = error;
        }
        self.matched_solves += 1;
        Ok(())
    }
}
type Lines = std::io::Lines<BufReader<File>>;
fn read<T: DeserializeOwned>(lines: &mut Lines) -> Result<T> {
    Ok(serde_json::from_str(
        &lines.next().ok_or("handoff reference ended early")??,
    )?)
}
fn write(out: &mut BufWriter<File>, value: &impl Serialize) -> Result<()> {
    serde_json::to_writer(&mut *out, value)?;
    out.write_all(b"\n")?;
    Ok(())
}

pub(super) struct Trace {
    output: Option<BufWriter<File>>,
    reference: Option<Lines>,
    comparison: Comparison,
    pub(super) wake_next: bool,
}
impl Trace {
    pub(super) fn open(path: Option<&Path>, reference: Option<&Path>, mode: Mode) -> Result<Self> {
        let mut reference = reference
            .map(File::open)
            .transpose()?
            .map(|f| BufReader::new(f).lines());
        if let Some(lines) = &mut reference
            && read::<Header>(lines)? != Header::new(Mode::Dynamic)
        {
            return Err(
                "expected a continuous dynamic reference for the six workcell links".into(),
            );
        }
        let mut output = path.map(File::create_new).transpose()?.map(BufWriter::new);
        if let Some(out) = &mut output {
            write(out, &Header::new(mode))?;
        }
        Ok(Self {
            output,
            reference,
            comparison: Comparison::default(),
            wake_next: false,
        })
    }
    pub(super) fn active(&self) -> bool {
        self.output.is_some() || self.reference.is_some()
    }
    pub(super) fn observe(&mut self, sample: &Sample) -> Result<()> {
        if let Some(out) = &mut self.output {
            write(out, sample)?;
        }
        if let Some(lines) = &mut self.reference {
            self.comparison
                .observe(sample, &read(lines)?, self.wake_next)?;
        }
        self.wake_next = false;
        Ok(())
    }
    pub(super) fn finish(
        &mut self,
        commands: &Receipt,
        first_resumed_ms: usize,
        mount_impulse_ns: [f64; 3],
    ) -> Result<Option<Comparison>> {
        let end = End {
            commands: commands.clone(),
            first_resumed_ms,
            mount_impulse_ns,
        };
        if let Some(out) = &mut self.output {
            write(out, &end)?;
            out.flush()?;
        }
        if let Some(lines) = &mut self.reference {
            let control: End = read(lines)?;
            if lines.next().transpose()?.is_some()
                || control.commands != end.commands
                || control.first_resumed_ms != first_resumed_ms
            {
                return Err(
                    "handoff reference command journal, wake clock or length differs".into(),
                );
            }
            let c = &mut self.comparison;
            c.first_resumed_mount_impulse_error_ns =
                distance(mount_impulse_ns, control.mount_impulse_ns);
            if c.max_position_m > 0.0005
                || c.max_rotation_rad > 0.001
                || c.max_arm_linear_momentum_kg_m_s > 0.005
                || c.max_arm_angular_momentum_kg_m2_s > 0.005
                || c.first_resumed_mount_impulse_error_ns > 0.005
                || c.first_resumed_pair_impulse_error_ns > 5e-6
            {
                return Err(format!("handoff accuracy gate failed: {c:?}").into());
            }
            return Ok(Some(std::mem::take(c)));
        }
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample() -> Sample {
        Sample {
            time_ms: 1,
            bodies: std::array::from_fn(|_| LinkState {
                position_m: [0.; 3],
                rotation_xyzw: [0., 0., 0., 1.],
                com_velocity_m_s: [0.; 3],
                angular_velocity_rad_s: [0.; 3],
            }),
            arm_linear_momentum_kg_m_s: [0.; 3],
            arm_angular_momentum_kg_m2_s: [0.; 3],
            support_impulse_ns: 0.00981,
            pad_impulses_ns: [0.; 2],
        }
    }
    #[test]
    fn stream_write_failures_and_invalid_trailing_bytes_propagate() -> Result<()> {
        let temp = tempfile::tempdir()?;
        let path = temp.path().join("journal");
        let end = Commands::default().receipt();
        let mut trace = Trace::open(Some(&path), None, Mode::Dynamic)?;
        trace.observe(&sample())?;
        trace.finish(&end, 1, [0.; 3])?;
        drop(trace);
        // A read-only descriptor gives a deterministic OS write error, including on flush.
        let failing = |cap| -> Result<Trace> {
            let mut trace = Trace::open(None, None, Mode::Dynamic)?;
            trace.output = Some(BufWriter::with_capacity(cap, File::open(&path)?));
            Ok(trace)
        };
        assert!(failing(0)?.observe(&sample()).is_err());
        assert!(failing(0)?.finish(&end, 1, [0.; 3]).is_err());
        assert!(failing(8192)?.finish(&end, 1, [0.; 3]).is_err());
        std::fs::OpenOptions::new()
            .append(true)
            .open(&path)?
            .write_all(&[0xff, 0xfe])?;
        let mut input = Trace::open(None, Some(&path), Mode::Adaptive)?;
        input.observe(&sample())?;
        assert!(input.finish(&end, 1, [0.; 3]).is_err());
        std::fs::write(&path, b"\xff")?;
        assert!(Trace::open(None, Some(&path), Mode::Adaptive).is_err());
        Ok(())
    }

    #[test]
    fn rejects_incomplete_wrong_clock_and_incompatible_references() -> Result<()> {
        let d = tempfile::tempdir()?;
        let path = d.path().join("reference.jsonl");
        assert!(Trace::open(None, Some(&path), Mode::Adaptive).is_err());
        let commands = Commands::default().receipt();
        let mut record = Trace::open(Some(&path), None, Mode::Dynamic)?;
        record.observe(&sample())?;
        record.finish(&commands, 1, [0.; 3])?;
        drop(record);
        let baseline = std::fs::read_to_string(&path)?;
        let validate = |sample: &Sample, commands: &Receipt, start: usize| -> Result<Comparison> {
            let mut check = Trace::open(None, Some(&path), Mode::Adaptive)?;
            check.wake_next = true;
            check.observe(sample)?;
            Ok(check.finish(commands, start, [0.; 3])?.unwrap())
        };
        assert_eq!(validate(&sample(), &commands, 1)?.matched_solves, 1);
        let mut bad_commands = Commands::default();
        bad_commands.accept(b'f', 0, 71, 1);
        assert!(validate(&sample(), &bad_commands.receipt(), 1).is_err());
        assert!(validate(&sample(), &commands, 2).is_err());
        for field in 0..6 {
            let mut s = sample();
            match field {
                0 => s.time_ms += 1,
                1 => s.bodies[0].position_m[0] = 0.01,
                2 => s.bodies[0].rotation_xyzw = DQuat::from_rotation_x(0.01).to_array(),
                3 => s.arm_linear_momentum_kg_m_s[1] = 0.01,
                4 => s.arm_angular_momentum_kg_m2_s[2] = 0.01,
                _ => s.pad_impulses_ns[0] = 0.01,
            }
            assert!(validate(&s, &commands, 1).is_err());
        }
        for malformed in [
            baseline.replace("dynamic", "adaptive"),
            baseline.lines().next().unwrap().to_owned(),
            format!("{baseline}{{}}\n"),
            "malformed\n".into(),
        ] {
            std::fs::write(&path, malformed)?;
            assert!(validate(&sample(), &commands, 1).is_err());
        }
        assert!(Trace::open(Some(&path), None, Mode::Dynamic).is_err());
        Ok(())
    }
}
