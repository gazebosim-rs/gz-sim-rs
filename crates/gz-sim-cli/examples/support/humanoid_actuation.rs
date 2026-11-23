// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exact bounded torques for the open-loop perturbation control.
use gz_physics::PhysicsError;
use gz_sim_scene::joint_posture::{PostureAxis, PostureCommand, PostureController};
use serde::{Deserialize, Serialize};
use std::{
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::Path,
    sync::Arc,
};
#[derive(PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Header {
    version: u8,
    interval_ns: u64,
    model: u64,
    axes: Vec<u64>,
}
impl Header {
    fn new(model: u64, axes: &[PostureAxis]) -> Self {
        Self {
            version: 1,
            interval_ns: 1_000_000,
            model,
            axes: axes.iter().map(|a| a.entity_id).collect(),
        }
    }
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Interval {
    tick: u64,
    effort_bits: Vec<u64>,
}
pub struct Recorder(BufWriter<File>);
impl Recorder {
    pub fn create(path: &Path, model: u64, axes: &[PostureAxis]) -> super::Result<Self> {
        let mut file = BufWriter::new(File::create(path)?);
        serde_json::to_writer(&mut file, &Header::new(model, axes))?;
        writeln!(file)?;
        Ok(Self(file))
    }
    pub fn append(&mut self, tick: u64, commands: &[PostureCommand]) -> super::Result<()> {
        serde_json::to_writer(
            &mut self.0,
            &Interval {
                tick,
                effort_bits: commands.iter().map(|c| c.effort_nm.to_bits()).collect(),
            },
        )?;
        writeln!(self.0)?;
        Ok(())
    }
    pub fn flush(&mut self) -> super::Result<()> {
        self.0.flush()?;
        Ok(())
    }
}
#[derive(Clone)]
pub struct Script {
    start: u64,
    inputs: Arc<Vec<Box<[f64]>>>,
}
impl Script {
    pub fn read(path: &Path, model: u64, axes: &[PostureAxis]) -> super::Result<Self> {
        let mut file = BufReader::new(File::open(path)?).lines();
        let header: Header = serde_json::from_str(&file.next().ok_or("empty torque file")??)?;
        if header != Header::new(model, axes) {
            return Err("torque recording does not match the model and joint identities".into());
        }
        let mut start = None;
        let mut inputs = Vec::new();
        for line in file {
            let row: Interval = serde_json::from_str(&line?)?;
            let first = *start.get_or_insert(row.tick);
            if first.checked_add(inputs.len() as u64) != Some(row.tick)
                || row.effort_bits.len() != axes.len()
            {
                return Err("nonconsecutive or incomplete torque recording".into());
            }
            let values = row
                .effort_bits
                .iter()
                .zip(axes)
                .map(|(&bits, axis)| {
                    let n = f64::from_bits(bits);
                    (n.is_finite() && n.abs() <= axis.max_effort)
                        .then_some(n)
                        .ok_or("recorded torque exceeds the authored cap")
                })
                .collect::<Result<Vec<_>, _>>()?;
            inputs.push(values.into_boxed_slice());
        }
        Ok(Self {
            start: start.ok_or("no torque intervals")?,
            inputs: Arc::new(inputs),
        })
    }
    pub fn contains(&self, tick: u64) -> bool {
        tick.checked_sub(self.start)
            .is_some_and(|i| i < self.inputs.len() as u64)
    }
    pub fn covers(&self, start: u64, end: u64) -> bool {
        start >= self.start
            && end
                .checked_sub(self.start)
                .is_some_and(|n| n <= self.inputs.len() as u64)
    }
    pub fn apply(
        &self,
        ctl: &mut PostureController,
        tick: u64,
        axes: &[PostureAxis],
    ) -> Result<(), PhysicsError> {
        let i = tick
            .checked_sub(self.start)
            .and_then(|n| usize::try_from(n).ok())
            .ok_or(PhysicsError::InvalidValue)?;
        let values = self.inputs.get(i).ok_or(PhysicsError::InvalidValue)?;
        for (axis, &effort) in axes.iter().zip(values) {
            ctl.set_direct_effort(axis.joint, Some(effort))?;
        }
        Ok(())
    }
}
