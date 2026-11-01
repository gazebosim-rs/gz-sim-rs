// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::{PendingResetRemoval, initial_imu_publications, reset_loaded_world};

impl Server {
    pub(super) fn validate_reset_steps(&self, count: u64) -> Result<(), Error> {
        let nanos = self
            .simulation
            .step_duration()
            .as_nanos()
            .checked_mul(u128::from(count))
            .ok_or(StepError::Overflow)?;
        if nanos > Duration::MAX.as_nanos() {
            return Err(StepError::Overflow.into());
        }
        time(nanos as i128)?;
        Ok(())
    }

    pub(super) fn reset_scene(&mut self) -> Result<UpdateInfo, Error> {
        // Reserve missing endpoints before changing the scene. Live shared leases survive.
        let existing: BTreeMap<_, _> = self
            .imus
            .iter()
            .map(|stream| (stream.endpoint.clone(), Arc::clone(&stream.publisher)))
            .collect();
        let mut publishers = BTreeMap::new();
        for metadata in initial_imu_publications(&self.simulation)? {
            let topic = crate::options::as_valid_topic(&metadata.topic);
            let key = crate::options::qualified_name(
                &self.partition,
                &self.namespace,
                self.network_config.topic_remap(&topic).unwrap_or(&topic),
            )?;
            if let std::collections::btree_map::Entry::Vacant(entry) = publishers.entry(key.clone())
            {
                let publisher = match existing.get(&key) {
                    Some(publisher) => Arc::clone(publisher),
                    None => self
                        .world_io
                        .imu_output(key.clone())
                        .map_err(|error| Error::WorldNetwork(error.to_string()))?,
                };
                entry.insert(publisher);
            }
        }
        let rates = &mut self.rates;
        let forces = &mut self.forces;
        let reset = reset_loaded_world(&mut self.simulation, |sim| {
            let mut streams = Vec::new();
            for entity in sim.world().iter_entities() {
                if entity.contains::<PendingResetRemoval>() {
                    continue;
                }
                let Some(metadata) = entity.get::<ImuPublication>() else {
                    continue;
                };
                let topic = crate::options::as_valid_topic(&metadata.topic);
                let endpoint = crate::options::qualified_name(
                    &self.partition,
                    &self.namespace,
                    self.network_config.topic_remap(&topic).unwrap_or(&topic),
                )?;
                let publisher = publishers.get(&endpoint).ok_or(Error::ResetStreams)?;
                streams.push(ImuStream {
                    entity: entity.id(),
                    metadata: metadata.clone(),
                    endpoint,
                    publisher: Arc::clone(publisher),
                    sequence: 0,
                    last_sample: None,
                });
            }
            let setup = rates.restart(&self.network_config, sim, &self.world_io.ingress_drops)?;
            self.world_io.install_rates(setup)?;
            // Disable old CPU handoff before preparing a replacement generation.
            *forces = None;
            let (replacement, setup) =
                forces::Forces::restart(&self.network_config, sim, &self.world_io.ingress_drops)?;
            *forces = replacement.into_active();
            self.world_io.install_forces(setup)?;
            Ok::<_, Error>((*sim.world().resource::<UpdateInfo>(), streams))
        });
        self.world_io.epoch.store(
            self.simulation.world().resource::<UpdateInfo>().epoch,
            std::sync::atomic::Ordering::Release,
        );
        let result = match reset {
            Ok(result) => result,
            Err(error @ gz_sim_scene::SceneResetError::CommittedReset) => {
                self.pending_reset = false;
                self.imus = Vec::new();
                self.rates.clear();
                self.forces = None;
                self.removals
                    .cancel_for_reset(&self.world_io, &mut self.counters);
                return Err(error.into());
            }
            Err(error) => return Err(error.into()),
        };
        // Reset is committed even if a custom handler invalidated the stream contract.
        self.pending_reset = false;
        self.active_time = ActiveTime::default();
        self.active_time
            .set_running(!self.simulation.is_paused(), Instant::now());
        self.removals
            .cancel_for_reset(&self.world_io, &mut self.counters);
        let (info, streams) = match result {
            Ok(result) => result,
            Err(error) => {
                self.reset_failed = true;
                self.imus = Vec::new();
                self.rates.clear();
                self.forces = None;
                return Err(error);
            }
        };
        self.imus = streams;
        Ok(info)
    }
}
