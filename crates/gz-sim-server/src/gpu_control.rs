// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A typed stable-link impulse service for the loaded SDF GPU world.
//! Keys use the same Gazebo partition, namespace, remapping and liveliness
//! conventions as world/control. The response acknowledges command admission.
use crate::{Counters, Error, world_io::ControlRequest};
use gz_sim_core::Simulation;
use gz_sim_proto::prost::Message;
use gz_sim_scene::gpu_scene::{BatchBoundary, batch_boundary, queue_link_impulse};
use std::{collections::BTreeMap, num::NonZeroUsize, sync::mpsc};
use zenoh::{liveliness::LivelinessToken, query::Queryable};

pub mod placement;
pub const REQUEST_TYPE: &str = "gz.sim.rs.GpuLinkImpulse";
pub const REPLY_TYPE: &str = "gz.sim.rs.GpuLinkImpulseAck";
pub(super) const MAX_REQUEST_BYTES: usize = 128;

/// Admission time, stable Gazebo link identity, and world center impulse in N·s.
/// Epoch and tick are explicitly present, including at time zero.
#[derive(Clone, PartialEq, Message)]
#[prost(prost_path = "gz_sim_proto::prost")]
pub struct LinkImpulse {
    #[prost(uint64, optional, tag = "1")]
    pub epoch: Option<u64>,
    #[prost(uint64, optional, tag = "2")]
    pub tick: Option<u64>,
    #[prost(uint64, tag = "3")]
    pub link_id: u64,
    #[prost(double, tag = "4")]
    pub x: f64,
    #[prost(double, tag = "5")]
    pub y: f64,
    #[prost(double, tag = "6")]
    pub z: f64,
}
/// For a successful request, epoch/tick identify its original admission barrier.
/// A duplicate of the last accepted command on that link reports the same barrier.
#[derive(Clone, PartialEq, Message)]
#[prost(prost_path = "gz_sim_proto::prost")]
pub struct LinkImpulseAck {
    #[prost(bool, tag = "1")]
    pub accepted: bool,
    #[prost(uint64, tag = "2")]
    pub epoch: u64,
    #[prost(uint64, tag = "3")]
    pub tick: u64,
    #[prost(bool, tag = "4")]
    pub duplicate: bool,
    #[prost(string, tag = "5")]
    pub reason: String,
}
pub(super) fn validate_key(
    sim: &Simulation,
    options: &crate::ServerOptions,
    key: &str,
) -> Result<(), Error> {
    if options.topic_key("/clock")? == key {
        return Err(Error::WorldNetwork(
            "GPU impulse endpoint conflicts with root clock".into(),
        ));
    }
    for e in sim.world().iter_entities() {
        let topic = e
            .get::<gz_sim_scene::ImuPublication>()
            .map(|s| &s.topic)
            .or_else(|| {
                e.get::<gz_sim_scene::AltimeterPublication>()
                    .map(|s| &s.topic)
            });
        if let Some(topic) = topic {
            let topic = crate::as_valid_topic(topic);
            if options.topic_key(&topic)? == key
                || options.topic_key(&format!("{topic}/set_rate"))? == key
            {
                return Err(Error::WorldNetwork(
                    "GPU impulse endpoint conflicts with a sensor or rate endpoint".into(),
                ));
            }
        }
    }
    Ok(())
}

pub(super) struct Endpoint {
    requests: mpsc::Receiver<ControlRequest>,
    _query: Queryable<()>,
    _presence: LivelinessToken,
}
impl Endpoint {
    pub(super) fn new(
        requests: mpsc::Receiver<ControlRequest>,
        query: Queryable<()>,
        presence: LivelinessToken,
    ) -> Self {
        Self {
            requests,
            _query: query,
            _presence: presence,
        }
    }
}
pub(super) struct Ingress {
    impulse: Endpoint,
    execution: Endpoint,
    epoch: u64,
    // One receipt per accepted link in the current world epoch.
    receipts: BTreeMap<u64, LinkImpulse>,
}
impl Ingress {
    pub(super) fn new(impulse: Endpoint, execution: Endpoint) -> Self {
        Self {
            impulse,
            execution,
            epoch: 0,
            receipts: BTreeMap::new(),
        }
    }
    fn command(&mut self, sim: &mut Simulation, payload: &[u8], resetting: bool) -> LinkImpulseAck {
        let at = batch_boundary(sim);
        if self.epoch != at.epoch {
            self.epoch = at.epoch;
            self.receipts.clear();
        }
        let mut ack = LinkImpulseAck {
            epoch: at.epoch,
            tick: at.tick,
            ..Default::default()
        };
        let result = (|| {
            if resetting || sim.is_faulted() {
                return Err("world reset or fault is pending".into());
            }
            let request = LinkImpulse::decode(payload)
                .map_err(|_| "malformed impulse protobuf".to_owned())?;
            let stamp = BatchBoundary {
                epoch: request.epoch.ok_or("missing epoch")?,
                tick: request.tick.ok_or("missing tick")?,
            };
            if stamp.epoch != at.epoch {
                return Err("stale scene epoch".into());
            }
            if self.receipts.get(&request.link_id) == Some(&request) {
                ack.tick = stamp.tick;
                ack.duplicate = true;
                return Ok(());
            }
            if stamp != at {
                return Err("stale scene iteration".into());
            }
            queue_link_impulse(
                sim,
                stamp,
                request.link_id,
                [request.x, request.y, request.z],
            )
            .map_err(|e| e.to_string())?;
            self.receipts.insert(request.link_id, request);
            Ok(())
        })();
        match result {
            Ok(()) => ack.accepted = true,
            Err(reason) => ack.reason = reason,
        }
        ack
    }
    pub(super) fn poll(
        &mut self,
        sim: &mut Simulation,
        budget: NonZeroUsize,
        counters: &mut Counters,
        resetting: bool,
    ) -> Result<(), Error> {
        for execution in [false, true] {
            for _ in 0..budget.get() {
                let endpoint = if execution {
                    &self.execution
                } else {
                    &self.impulse
                };
                let request = match endpoint.requests.try_recv() {
                    Ok(request) => request,
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => {
                        return Err(Error::WorldNetwork("GPU control receiver closed".into()));
                    }
                };
                let (accepted, bytes) = if execution {
                    let ack = placement::command(sim, &request.payload, resetting);
                    (ack.accepted, ack.encode_to_vec())
                } else {
                    let ack = self.command(sim, &request.payload, resetting);
                    (ack.accepted, ack.encode_to_vec())
                };
                if accepted {
                    counters.controls_accepted += 1
                } else {
                    counters.controls_rejected += 1
                }
                if request.reply(bytes).is_err() {
                    counters.response_failures += 1
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../tests/private/gpu_control.rs"]
mod tests;
