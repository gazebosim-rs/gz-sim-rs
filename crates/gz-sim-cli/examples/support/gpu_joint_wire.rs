// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validate each Gazebo wire packet before using it as measurement evidence.
use super::{Result, WAIT};
use gz_sim_proto::{
    msgs::{Clock, Double, Model, Time},
    prost::{Message, Name},
};
use std::{path::Path, time::Instant};
use zenoh::{
    Wait,
    handlers::FifoChannelHandler,
    pubsub::{Publisher, Subscriber},
    sample::Sample,
};

pub type Stream = Subscriber<FifoChannelHandler<Sample>>;

pub fn receive<M: Message + Name + Default>(stream: &Stream, deadline: Instant) -> Result<M> {
    let sample = stream
        .recv_timeout(deadline.saturating_duration_since(Instant::now()))?
        .ok_or("timed out waiting for Gazebo sample")?;
    if sample
        .attachment()
        .is_none_or(|ty| ty.to_bytes().as_ref() != M::full_name().as_bytes())
    {
        return Err("unexpected Gazebo sample type".into());
    }
    Ok(M::decode(sample.payload().to_bytes().as_ref())?)
}
fn millis(t: &Time) -> Result<u32> {
    if t.sec < 0 || !(0..1_000_000_000).contains(&t.nsec) || t.nsec % 1_000_000 != 0 {
        return Err("invalid millisecond simulation timestamp".into());
    }
    let ns = (t.sec as u128) * 1000 + u128::from(t.nsec as u32 / 1_000_000);
    Ok(ns.try_into()?)
}
pub fn wait_rewind(clock: &Stream, from: u32) -> Result<()> {
    let deadline = Instant::now() + WAIT;
    let mut saw_loaded = false;
    loop {
        let msg: Clock = receive(clock, deadline)?;
        let tick = millis(msg.sim.as_ref().ok_or("missing simulation time")?)?;
        saw_loaded |= tick == from;
        if tick == 0 && saw_loaded {
            return Ok(());
        }
    }
}
#[derive(Debug)]
pub struct Feedback {
    pub tick: u32,
    pub position: [f64; 2],
    pub velocity: [f64; 2],
}
impl Feedback {
    fn decode(model: Model) -> Result<Self> {
        let stamp = model
            .header
            .as_ref()
            .and_then(|h| h.stamp.as_ref())
            .ok_or("missing joint-state time")?;
        let tick = millis(stamp)?;
        let [shoulder, elbow] = model.joint.as_slice() else {
            return Err("joint-state packet must contain two axes".into());
        };
        let mut position = [0.; 2];
        let mut velocity = [0.; 2];
        for (i, (joint, name)) in [(shoulder, "shoulder"), (elbow, "elbow")]
            .into_iter()
            .enumerate()
        {
            if joint.name != name {
                return Err("unexpected joint-state owner".into());
            }
            let axis = joint.axis1.as_ref().ok_or("missing physical joint axis")?;
            if !axis.position.is_finite() || !axis.velocity.is_finite() {
                return Err("nonfinite joint-state reading".into());
            }
            position[i] = axis.position;
            velocity[i] = axis.velocity;
        }
        Ok(Self {
            tick,
            position,
            velocity,
        })
    }
    pub fn at(state: &Stream, expected: u32) -> Result<Self> {
        let deadline = Instant::now() + WAIT;
        loop {
            let packet = Self::decode(receive(state, deadline)?)?;
            if packet.tick == expected {
                return Ok(packet);
            }
            if packet.tick > expected {
                return Err("missing public joint-state tick".into());
            }
        }
    }
}
pub fn command(publisher: &Publisher<'_>, target: f64) -> Result<()> {
    publisher
        .put(
            Double {
                data: target,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .attachment(Double::full_name())
        .wait()?;
    Ok(())
}
pub fn save(path: &Path, value: &serde_json::Value) -> Result<()> {
    std::fs::write(path, value.to_string())?;
    Ok(())
}

/// Prove that a pending command and a moved physical state both reset.
pub fn reset_loaded(
    session: &zenoh::Session,
    prefix: &str,
    state: &Stream,
    publisher: &Publisher<'_>,
) -> Result<()> {
    use gz_sim_proto::msgs::{WorldControl, WorldReset};
    let control_key = format!("{prefix}/world/default/control");
    let clock = session
        .declare_subscriber(format!("{prefix}/world/default/clock"))
        .wait()?;
    command(publisher, -8.)?;
    super::advance(session, &control_key, 0)?;
    super::advance(session, &control_key, 100)?;
    let state_at_load = Feedback::at(state, 100)?;
    if state_at_load.position[0] >= -0.05 {
        return Err("arm did not load before reset".into());
    }
    // This queued alternative target must be cleared together with the physical state.
    command(publisher, 8.)?;
    super::advance(session, &control_key, 0)?;
    super::control(
        session,
        &control_key,
        WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
    )?;
    wait_rewind(&clock, 100)?;
    while state.try_recv()?.is_some() {}
    Ok(())
}
#[cfg(test)]
#[path = "../../tests/private/gpu_joint_wire.rs"]
mod tests;
