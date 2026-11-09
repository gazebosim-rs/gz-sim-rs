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
        validate_air_speed_and_pose_topics(&self.simulation, &self.world_io, &self.network_config)?;
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
        let lidar_publishers = lidar::prepare_reset(self)?;
        let drive_metadata = diff_drive::initial(&self.simulation)?;
        let controller_metadata = joint_controllers::initial(&self.simulation)?;
        let altimeter_publishers =
            if gz_sim_scene::initial_altimeter_publications(&self.simulation)?
                .next()
                .is_some()
            {
                let occupied = altimeter_reserved_keys(
                    &self.simulation,
                    &self.world_io,
                    &self.network_config,
                )?;
                altimeter::prepare_reset(self, &occupied)?
            } else {
                BTreeMap::new()
            };
        let air_pressure_publishers =
            if gz_sim_scene::initial_air_pressure_publications(&self.simulation)?
                .next()
                .is_some()
            {
                let occupied = air_pressure_reserved_keys(
                    &self.simulation,
                    &self.world_io,
                    &self.network_config,
                )?;
                air_pressure::prepare_reset(self, &occupied)?
            } else {
                BTreeMap::new()
            };
        let air_speed_publishers =
            if gz_sim_scene::initial_air_speed_publications(&self.simulation)?
                .next()
                .is_some()
            {
                let occupied = air_speed_reserved_keys(
                    &self.simulation,
                    &self.world_io,
                    &self.network_config,
                )?;
                air_speed::prepare_reset(self, &occupied)?
            } else {
                BTreeMap::new()
            };
        let magnetometer_publishers =
            if gz_sim_scene::initial_magnetometer_publications(&self.simulation)?
                .next()
                .is_some()
            {
                let occupied = magnetometer_reserved_keys(
                    &self.simulation,
                    &self.world_io,
                    &self.network_config,
                )?;
                magnetometer::prepare_reset(self, &occupied)?
            } else {
                BTreeMap::new()
            };
        let joint_state_publishers =
            if gz_sim_scene::initial_joint_state_publications(&self.simulation)?
                .next()
                .is_some()
            {
                let occupied = joint_state_reserved_keys(
                    &self.simulation,
                    &self.world_io,
                    &self.network_config,
                )?;
                joint_state::prepare_reset(self, &occupied)?
            } else {
                BTreeMap::new()
            };
        let logical_camera_publishers =
            if gz_sim_scene::initial_logical_camera_publications(&self.simulation)?
                .next()
                .is_some()
            {
                let occupied = logical_camera_reserved_keys(
                    &self.simulation,
                    &self.world_io,
                    &self.network_config,
                )?;
                logical_camera::prepare_reset(self, &occupied)?
            } else {
                BTreeMap::new()
            };
        let pose_publisher_publishers =
            if gz_sim_scene::initial_pose_publications(&self.simulation)?
                .next()
                .is_some()
            {
                let occupied = pose_publisher_reserved_keys(
                    &self.simulation,
                    &self.world_io,
                    &self.network_config,
                )?;
                pose_publisher::prepare_reset(self, &occupied)?
            } else {
                BTreeMap::new()
            };
        let drive = &mut self.diff_drive;
        let controllers = &mut self.joint_controllers;
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
            let lidars = lidar::streams(
                sim,
                &self.world_io,
                &self.network_config,
                &streams,
                Some(&lidar_publishers),
            )?;
            // Old command receivers and weak odometry jobs cannot enter the next epoch.
            *drive = None;
            *drive = diff_drive::stream(
                sim,
                &self.world_io,
                &self.network_config,
                &streams,
                &lidars,
                Some(&drive_metadata),
            )?;
            // Retire every old callback/receiver before installing fresh input generations.
            *controllers = None;
            *controllers = joint_controllers::prepare(
                sim,
                &self.world_io,
                &self.network_config,
                &controller_metadata,
            )?;
            // Disable old CPU handoff before preparing a replacement generation.
            *rates = None;
            let (replacement, setup) =
                rates::Rates::restart(&self.network_config, sim, &self.world_io.ingress_drops)?;
            *rates = replacement.into_active();
            self.world_io.install_rates(setup)?;
            // Disable old CPU handoff before preparing a replacement generation.
            *forces = None;
            let (replacement, setup) =
                forces::Forces::restart(&self.network_config, sim, &self.world_io.ingress_drops)?;
            *forces = replacement.into_active();
            self.world_io.install_forces(setup)?;
            let occupied = if sim
                .world()
                .iter_entities()
                .any(|entity| entity.contains::<gz_sim_scene::AltimeterPublication>())
            {
                altimeter_reserved_keys(sim, &self.world_io, &self.network_config)?
            } else {
                BTreeSet::new()
            };
            let altimeters = altimeter::streams(
                sim,
                &self.world_io,
                &self.network_config,
                &occupied,
                Some(&altimeter_publishers),
            )?;
            let occupied = if sim
                .world()
                .iter_entities()
                .any(|entity| entity.contains::<gz_sim_scene::AirPressurePublication>())
            {
                air_pressure_reserved_keys(sim, &self.world_io, &self.network_config)?
            } else {
                BTreeSet::new()
            };
            let air_pressures = air_pressure::streams(
                sim,
                &self.world_io,
                &self.network_config,
                &occupied,
                Some(&air_pressure_publishers),
            )?;
            let occupied = if sim
                .world()
                .iter_entities()
                .any(|entity| entity.contains::<gz_sim_scene::AirSpeedPublication>())
            {
                air_speed_reserved_keys(sim, &self.world_io, &self.network_config)?
            } else {
                BTreeSet::new()
            };
            let air_speeds = air_speed::streams(
                sim,
                &self.world_io,
                &self.network_config,
                &occupied,
                Some(&air_speed_publishers),
            )?;
            let occupied = if sim
                .world()
                .iter_entities()
                .any(|entity| entity.contains::<gz_sim_scene::MagnetometerPublication>())
            {
                magnetometer_reserved_keys(sim, &self.world_io, &self.network_config)?
            } else {
                BTreeSet::new()
            };
            let magnetometers = magnetometer::streams(
                sim,
                &self.world_io,
                &self.network_config,
                &occupied,
                Some(&magnetometer_publishers),
            )?;
            let occupied = if sim
                .world()
                .iter_entities()
                .any(|entity| entity.contains::<gz_sim_scene::JointStatePublication>())
            {
                joint_state_reserved_keys(sim, &self.world_io, &self.network_config)?
            } else {
                BTreeSet::new()
            };
            let joint_states = joint_state::streams(
                sim,
                &self.world_io,
                &self.network_config,
                &occupied,
                Some(&joint_state_publishers),
            )?;
            let occupied = if sim
                .world()
                .iter_entities()
                .any(|entity| entity.contains::<gz_sim_scene::PosePublication>())
            {
                pose_publisher_reserved_keys(sim, &self.world_io, &self.network_config)?
            } else {
                BTreeSet::new()
            };
            let camera_occupied = if sim
                .world()
                .iter_entities()
                .any(|entity| entity.contains::<gz_sim_scene::LogicalCameraPublication>())
            {
                logical_camera_reserved_keys(sim, &self.world_io, &self.network_config)?
            } else {
                BTreeSet::new()
            };
            let logical_cameras = logical_camera::streams(
                sim,
                &self.world_io,
                &self.network_config,
                &camera_occupied,
                Some(&logical_camera_publishers),
            )?;
            let pose_publishers = pose_publisher::streams(
                sim,
                &self.world_io,
                &self.network_config,
                &occupied,
                Some(&pose_publisher_publishers),
            )?;
            Ok::<_, Error>((
                *sim.world().resource::<UpdateInfo>(),
                streams,
                lidars,
                altimeters,
                air_pressures,
                air_speeds,
                logical_cameras,
                pose_publishers,
                magnetometers,
                joint_states,
            ))
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
                self.lidars = Vec::new();
                self.altimeters = Vec::new();
                self.air_pressures = Vec::new();
                self.air_speeds = Vec::new();
                self.logical_cameras = Vec::new();
                self.pose_publishers = Vec::new();
                self.magnetometers = Vec::new();
                self.joint_states = Vec::new();
                self.rates = None;
                self.forces = None;
                self.diff_drive = None;
                self.joint_controllers = None;
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
        let (
            info,
            streams,
            lidars,
            altimeters,
            air_pressures,
            air_speeds,
            logical_cameras,
            pose_publishers,
            magnetometers,
            joint_states,
        ) = match result {
            Ok(result) => result,
            Err(error) => {
                self.reset_failed = true;
                self.imus = Vec::new();
                self.lidars = Vec::new();
                self.altimeters = Vec::new();
                self.air_pressures = Vec::new();
                self.air_speeds = Vec::new();
                self.logical_cameras = Vec::new();
                self.pose_publishers = Vec::new();
                self.magnetometers = Vec::new();
                self.joint_states = Vec::new();
                self.rates = None;
                self.forces = None;
                self.diff_drive = None;
                self.joint_controllers = None;
                return Err(error);
            }
        };
        self.imus = streams;
        self.lidars = lidars;
        self.altimeters = altimeters;
        self.air_pressures = air_pressures;
        self.air_speeds = air_speeds;
        self.logical_cameras = logical_cameras;
        self.pose_publishers = pose_publishers;
        self.magnetometers = magnetometers;
        self.joint_states = joint_states;
        Ok(info)
    }
}
