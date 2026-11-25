// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Unloaded standby at the empty pickup station after a verified delivery.
use super::*;
use gz_physics::{JointId, PhysicsError};
use sha2::{Digest, Sha256};
#[path = "adaptive/trace.rs"]
mod trace;
use gz_sim_scene::{
    ArticulationTier, FidelityError, JointHandle, model_fidelity, set_model_fidelity,
    validate_model_demotion,
};
use std::{
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum, Serialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum Mode {
    Dynamic,
    Adaptive,
}

#[derive(Default, bevy_ecs::prelude::Resource)]
pub(super) struct Commands {
    force_writes: usize,
    target_writes: usize,
    drive_enable_writes: usize,
    model_removals: usize,
    digest: Sha256,
}
#[derive(Clone, Debug, PartialEq, Serialize, serde::Deserialize)]
struct Receipt {
    force_writes: usize,
    target_writes: usize,
    drive_enable_writes: usize,
    model_removals: usize,
    sha256: String,
}
impl Commands {
    fn accept(&mut self, kind: u8, time: usize, id: u64, value: u64) {
        self.digest.update([kind]);
        self.digest.update((time as u64).to_le_bytes());
        self.digest.update(id.to_le_bytes());
        self.digest.update(value.to_le_bytes());
    }
    fn receipt(&self) -> Receipt {
        Receipt {
            force_writes: self.force_writes,
            target_writes: self.target_writes,
            drive_enable_writes: self.drive_enable_writes,
            model_removals: self.model_removals,
            sha256: format!("{:x}", self.digest.clone().finalize()),
        }
    }
}
pub(super) fn retire(sim: &mut Simulation, id: u64) -> Result<()> {
    gz_sim_scene::remove_model(sim, id)?;
    let clock = ms(sim);
    if let Some(mut counts) = sim.world_mut().get_resource_mut::<Commands>() {
        counts.model_removals += 1;
        counts.accept(b'r', clock, id, 0);
    }
    Ok(())
}
pub(super) fn forces<const N: usize>(sim: &mut Simulation, values: [(u64, f64); N]) -> Result<()> {
    JointForceCommand::set_batch(sim, values)?;
    let clock = ms(sim);
    if let Some(mut counts) = sim.world_mut().get_resource_mut::<Commands>() {
        counts.force_writes += N;
        for (id, v) in values {
            counts.accept(b'f', clock, id, v.to_bits());
        }
    }
    Ok(())
}
pub(super) fn target(sim: &mut Simulation, id: u64, value: f64) -> Result<()> {
    set_joint_controller_command(sim, id, value)?;
    let clock = ms(sim);
    if let Some(mut counts) = sim.world_mut().get_resource_mut::<Commands>() {
        counts.target_writes += 1;
        counts.accept(b't', clock, id, value.to_bits());
    }
    Ok(())
}

pub(super) fn mark_workcell(xml: &str) -> Result<String> {
    let doc = roxmltree::Document::parse(xml)?;
    let gripper = doc
        .root_element()
        .children()
        .find(|w| w.has_tag_name("world") && w.attribute("name") == Some("default"))
        .and_then(|w| {
            w.children()
                .find(|m| m.has_tag_name("model") && m.attribute("name") == Some("gripper"))
        })
        .ok_or("missing serial gripper")?;
    let mut result = xml.to_owned();
    if !gripper.children().any(|p| {
        p.has_tag_name("plugin")
            && p.attribute("name") == Some("gz::sim::systems::SwitchableArticulation")
    }) {
        let end = xml[gripper.range()]
            .rfind("</")
            .ok_or("empty gripper model")?
            + gripper.range().start;
        result.insert_str(end,r#"<plugin name="gz::sim::systems::SwitchableArticulation" filename="gz-sim-switchable-articulation-system"/>"#);
    }
    Ok(result)
}

#[derive(Debug, PartialEq, Serialize)]
pub(super) struct Report {
    mode: Mode,
    pickup_retired_ms: usize,
    drives_suspended_ms: usize,
    eligible_since_ms: usize,
    standby_start_ms: usize,
    first_resumed_solve_ms: usize,
    prescribed_solves: usize,
    wake_reason: &'static str,
    support_loss: Option<adaptive_support::Loss>,
    commands: Receipt,
    first_resumed_mount_impulse_ns: [f64; 3],
    minimum_shelf_upward_impulse_during_return_ns: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    comparison: Option<trace::Comparison>,
    before_wake: Sample,
    first_resumed: Sample,
    final_sample: Sample,
}

struct Arm {
    model: u64,
    bodies: [BodyId; 6],
    mass_kg: f64,
    rail_joint: JointId,
    shelf: Part,
    shelf_body: BodyId,
    spindle: u64,
    wrist: u64,
}
#[derive(Debug, PartialEq, Serialize, serde::Deserialize)]
struct LinkState {
    position_m: [f64; 3],
    rotation_xyzw: [f64; 4],
    com_velocity_m_s: [f64; 3],
    angular_velocity_rad_s: [f64; 3],
}
impl From<gz_physics::BodyState> for LinkState {
    fn from(s: gz_physics::BodyState) -> Self {
        Self {
            position_m: s.position,
            rotation_xyzw: s.orientation,
            com_velocity_m_s: s.velocity,
            angular_velocity_rad_s: s.angular_velocity,
        }
    }
}
#[derive(Debug, PartialEq, Serialize, serde::Deserialize)]
struct Sample {
    time_ms: usize,
    bodies: [LinkState; 6],
    arm_linear_momentum_kg_m_s: [f64; 3],
    arm_angular_momentum_kg_m2_s: [f64; 3],
    support_impulse_ns: f64,
    pad_impulses_ns: [f64; 2],
}
const LINKS: [&str; 6] = [
    "gripper::carriage",
    "gripper::nut",
    "gripper::palm",
    "gripper::left",
    "gripper::right",
    "parcel::box",
];
impl Arm {
    fn bind(sim: &Simulation) -> Result<Self> {
        let mut bodies = Vec::new();
        for name in LINKS {
            bodies.push(part(sim, name)?.body(sim)?);
        }
        let rail = part(sim, "gripper::travel")?;
        let shelf = part(sim, "receiver::shelf")?;
        let rail_joint = sim
            .world()
            .get::<JointHandle>(rail.entity)
            .ok_or("missing rail joint")?
            .id();
        let mass_kg = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local solver")?
            .locked_subtree_inertia(rail_joint)?
            .mass_kg;
        Ok(Self {
            mass_kg,
            model: part(sim, "gripper")?.id,
            bodies: bodies.try_into().expect("six links"),
            rail_joint,
            shelf,
            shelf_body: shelf.body(sim)?,
            spindle: part(sim, "gripper::spindle")?.id,
            wrist: part(sim, "gripper::wrist")?.id,
        })
    }
    fn drives(&self, sim: &mut Simulation, enabled: bool) -> Result<()> {
        gz_sim_scene::set_joint_position_drives_enabled(sim, &[self.spindle, self.wrist], enabled)?;
        let clock = ms(sim);
        let mut receipt = sim.world_mut().resource_mut::<Commands>();
        receipt.drive_enable_writes += 2;
        for id in [self.spindle, self.wrist] {
            receipt.accept(b'e', clock, id, enabled as u64);
        }
        Ok(())
    }
    fn sample(&self, sim: &Simulation) -> Result<Sample> {
        let backend = sim.world().resource::<PhysicsBackend>();
        let native = backend
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local solver")?;
        let mut states = Vec::new();
        for body in self.bodies {
            states.push(LinkState::from(
                backend.state(body).ok_or("missing measured link")?,
            ));
        }
        let momentum = native.subtree_momentum_about(self.rail_joint, [0.; 3])?;
        let normal = |other| -> Result<f64> {
            Ok(native
                .normal_reaction_on(self.bodies[5], other)?
                .ok_or("missing pair reaction")?
                .magnitude)
        };
        let shelf_live = sim
            .world()
            .get::<BodyHandle>(self.shelf.entity)
            .is_some_and(|b| b.id() == self.shelf_body);
        Ok(Sample {
            time_ms: ms(sim),
            bodies: states.try_into().expect("six links"),
            arm_linear_momentum_kg_m_s: momentum.linear_kg_m_s,
            arm_angular_momentum_kg_m2_s: momentum.angular_kg_m2_s,
            support_impulse_ns: if shelf_live {
                normal(self.shelf_body)?
            } else {
                0.
            },
            pad_impulses_ns: [normal(self.bodies[3])?, normal(self.bodies[4])?],
        })
    }
}
fn ms(sim: &Simulation) -> usize {
    sim.elapsed().as_millis() as usize
}
struct Recorder<'a, F> {
    arm: &'a Arm,
    trace: trace::Trace,
    capture: &'a mut F,
    check_shelf: bool,
    min_shelf: f64,
}
impl<F: FnMut(&Simulation) -> Result<()>> Recorder<'_, F> {
    fn advance(&mut self, sim: &mut Simulation, event: bool) -> Result<()> {
        cycle::step(sim)?;
        if self.check_shelf {
            let native = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .ok_or("missing local solver")?;
            let load = native
                .normal_reaction_on(self.arm.bodies[5], self.arm.shelf_body)?
                .ok_or("missing shelf reaction")?
                .impulse[2];
            if load <= 0. {
                return Err(format!(
                    "placed parcel lost shelf support during return at {} ms",
                    ms(sim)
                )
                .into());
            }
            self.min_shelf = self.min_shelf.min(load);
        }
        if self.trace.active() {
            self.trace.observe(&self.arm.sample(sim)?)?;
        }
        if event || ms(sim).is_multiple_of(40) {
            (self.capture)(sim)?;
        }
        Ok(())
    }
    /// A transient load restarts the entire dwell. A stale binding aborts immediately.
    fn wait_for_standby(
        &mut self,
        sim: &mut Simulation,
        model: u64,
        max_solves: usize,
    ) -> Result<()> {
        let mut dwell = 0;
        let mut last_defer = None;
        for _ in 0..max_solves {
            self.advance(sim, false)?;
            match validate_model_demotion(sim, model) {
                Ok(()) => {
                    dwell += 1;
                    if dwell == 32 {
                        return Ok(());
                    }
                }
                Err(
                    e @ (FidelityError::PendingCommand
                    | FidelityError::Physics(PhysicsError::TransitionDeferred { .. })),
                ) => {
                    dwell = 0;
                    last_defer = Some(e);
                }
                Err(error) => return Err(error.into()),
            }
        }
        let cause =
            last_defer.map_or_else(|| "insufficient quiet solves".to_owned(), |e| e.to_string());
        Err(format!("no safe unloaded standby after {max_solves} solves: {cause}").into())
    }
}

pub(super) struct Options<'a> {
    pub mode: Mode,
    pub remove: bool,
    pub trace: Option<&'a Path>,
    pub reference: Option<&'a Path>,
    pub gravity: [f64; 3],
}
pub(super) fn run(
    sim: &mut Simulation,
    options: Options,
    release_tick: Option<usize>,
    capture: &mut impl FnMut(&Simulation) -> Result<()>,
) -> Result<Report> {
    if release_tick.is_none() || sim.step_duration() != Duration::from_millis(1) {
        return Err("standby requires a verified release and a 1 ms public step".into());
    }
    let Options {
        mode,
        remove,
        trace,
        reference,
        gravity,
    } = options;
    let arm = Arm::bind(sim)?;
    let mut recorder = Recorder {
        arm: &arm,
        trace: trace::Trace::open(trace, reference, mode)?,
        capture,
        check_shelf: true,
        min_shelf: f64::INFINITY,
    };
    let rail = part(sim, "gripper::travel")?;
    let tray = part(sim, "stand")?;

    let pickup_retired_ms = ms(sim);
    retire(sim, tray.id)?;
    (recorder.capture)(sim)?;
    // Clearing the parcel top needs more lift than opening above it. Raise vertically
    // before traversing, to avoid sweeping a finger through the delivered load.
    target(sim, arm.spindle, 1.98)?;
    for _ in 0..1500 {
        recorder.advance(sim, false)?;
    }
    // Return while raised. The issued rail target ends with zero commanded velocity.
    for elapsed in 1..=6000 {
        drive_rail(sim, rail, 0.46, 0., elapsed, 4000, 0.001)?;
        recorder.advance(sim, false)?;
    }
    let left = part(sim, "gripper::close_left")?.id;
    let right = part(sim, "gripper::close_right")?.id;
    // Lower under control to the authored gravity-bearing stop and close the empty
    // fingers before disengaging. Dropping the screw excites the undamped wrist.
    target(sim, arm.spindle, -0.10)?;
    forces(sim, [(left, 0.), (right, 0.)])?;
    for elapsed in 1..=5500 {
        drive_rail(sim, rail, 0., 0., elapsed, 1000, 0.001)?;
        recorder.advance(sim, false)?;
    }
    let drives_suspended_ms = ms(sim);
    forces(sim, [(rail.id, 0.)])?;
    arm.drives(sim, false)?;
    recorder.wait_for_standby(sim, arm.model, 20_000)?;
    recorder.check_shelf = false;
    let mut monitor = adaptive_support::Monitor::new(sim, remove)?;
    let standby_start_ms = ms(sim);
    if mode == Mode::Adaptive {
        set_model_fidelity(sim, arm.model, ArticulationTier::Prescribed)?;
    }
    (recorder.capture)(sim)?;
    let mut prescribed_solves = 0;
    let wake_reason = loop {
        let idle_tick = ms(sim) - standby_start_ms + 1;
        if monitor.before_idle_step(sim, idle_tick)? {
            (recorder.capture)(sim)?;
            break "receiver_support_lost";
        }
        if idle_tick == 501 {
            break "next_pickup_command";
        }
        recorder.advance(sim, false)?;
        match model_fidelity(sim, arm.model)? {
            ArticulationTier::Prescribed => prescribed_solves += 1,
            _ if mode == Mode::Adaptive => {
                return Err("unexpected wake during unloaded standby".into());
            }
            _ => {}
        }
    };
    let before_wake = arm.sample(sim)?;
    if mode == Mode::Adaptive {
        set_model_fidelity(sim, arm.model, ArticulationTier::Dynamic)?;
    }
    target(sim, arm.spindle, 1.6)?;
    arm.drives(sim, true)?;
    forces(sim, [(left, -20.), (right, -20.)])?;
    recorder.trace.wake_next = true;
    recorder.advance(sim, true)?;
    let first_resumed = arm.sample(sim)?;
    for tick in 1..=499 {
        recorder.advance(sim, tick == 499)?;
    }
    let commands = sim.world().resource::<Commands>().receipt();
    let first_resumed_mount_impulse_ns = std::array::from_fn(|i| {
        first_resumed.arm_linear_momentum_kg_m_s[i]
            - before_wake.arm_linear_momentum_kg_m_s[i]
            - arm.mass_kg * gravity[i] * sim.step_duration().as_secs_f64()
    });
    let comparison = recorder.trace.finish(
        &commands,
        first_resumed.time_ms,
        first_resumed_mount_impulse_ns,
    )?;
    Ok(Report {
        mode,
        pickup_retired_ms,
        drives_suspended_ms,
        eligible_since_ms: standby_start_ms - 31,
        standby_start_ms,
        first_resumed_solve_ms: first_resumed.time_ms,
        prescribed_solves,
        wake_reason,
        support_loss: monitor.loss,
        commands,
        comparison,
        first_resumed_mount_impulse_ns,
        minimum_shelf_upward_impulse_during_return_ns: recorder.min_shelf,
        before_wake,
        first_resumed,
        final_sample: arm.sample(sim)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fresh() -> Result<Simulation> {
        let xml = mark_workcell(Trial::ClearObstacle.xml())?;
        Ok(load_world_with_sdf_step(
            &xml,
            "standby.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?)
    }

    #[test]
    fn invalid_standby_inputs_cannot_retire_the_tray_or_command_the_arm() -> Result<()> {
        assert!(mark_workcell("<sdf").is_err());
        assert!(mark_workcell("<sdf><world name=\"default\"/></sdf>").is_err());
        assert!(
            mark_workcell("<sdf><world name=\"default\"><model name=\"gripper\"/></world></sdf>")
                .is_err()
        );
        let marked = mark_workcell(Trial::ClearObstacle.xml())?;
        assert_eq!(mark_workcell(&marked)?, marked);
        let mut sim = fresh()?;
        let opts = Options {
            mode: Mode::Adaptive,
            gravity: [0., 0., -9.81],
            trace: None,
            reference: None,
            remove: false,
        };
        assert!(run(&mut sim, opts, None, &mut |_| Ok(())).is_err());
        assert!(part(&sim, "stand").is_ok());
        assert!(forces(&mut sim, [(u64::MAX, 2.)]).is_err());
        assert!(target(&mut sim, u64::MAX, 0.).is_err());
        assert!(retire(&mut sim, u64::MAX).is_err());
        let arm = Arm::bind(&sim)?;
        let mut broken = arm;
        broken.wrist = u64::MAX;
        assert!(broken.drives(&mut sim, false).is_err());
        let parcel = part(&sim, "parcel")?;
        retire(&mut sim, parcel.id)?;
        assert!(Arm::bind(&sim).is_err());
        assert!(broken.sample(&sim).is_err());
        assert_eq!(sim.elapsed(), Duration::ZERO);
        Ok(())
    }

    #[test]
    fn missing_route_bindings_and_output_failures_stop_the_runner() -> Result<()> {
        let mut sim = fresh()?;
        let usable = || Options {
            mode: Mode::Adaptive,
            gravity: [0., 0., -9.81],
            trace: None,
            reference: None,
            remove: false,
        };
        let out = tempfile::tempdir()?;
        let record = out.path().join("existing");
        std::fs::write(&record, "previous recording")?;
        assert!(
            run(
                &mut sim,
                Options {
                    trace: Some(&record),
                    ..usable()
                },
                Some(1),
                &mut |_| Ok(())
            )
            .is_err()
        );
        assert!(part(&sim, "stand").is_ok());
        assert_eq!(std::fs::read_to_string(&record)?, "previous recording");
        assert!(
            run(&mut sim, usable(), Some(1), &mut |_| Err(
                "recorder stopped".into()
            ))
            .is_err()
        );
        assert!(part(&sim, "stand").is_err());
        assert!(run(&mut sim, usable(), Some(1), &mut |_| Ok(())).is_err());
        let mut sim = fresh()?;
        let shelf = part(&sim, "receiver")?;
        retire(&mut sim, shelf.id)?;
        assert!(Arm::bind(&sim).is_err());
        let mut sim = fresh()?;
        let rail = part(&sim, "gripper::travel")?;
        sim.world_mut()
            .entity_mut(rail.entity)
            .remove::<JointHandle>();
        assert!(Arm::bind(&sim).is_err());
        let mut sim = fresh()?;
        let arm = Arm::bind(&sim)?;
        // An unsupported parcel at the start of a workcell cannot masquerade as a delivery.
        let mut recorder = Recorder {
            arm: &arm,
            trace: trace::Trace::open(None, None, Mode::Dynamic)?,
            check_shelf: true,
            min_shelf: f64::INFINITY,
            capture: &mut |_: &Simulation| -> Result<()> { Ok(()) },
        };
        assert!(recorder.advance(&mut sim, false).is_err());
        assert_eq!(sim.elapsed(), Duration::from_millis(1));
        Ok(())
    }
    #[test]
    fn a_missing_or_replaced_link_cannot_supply_handoff_measurements() -> Result<()> {
        for (name, remove_body) in [
            ("gripper::palm", true),
            ("gripper::travel", false),
            ("gripper", false),
            ("gripper::spindle", false),
            ("gripper::wrist", false),
            ("receiver::shelf", true),
        ] {
            let mut sim = fresh()?;
            let e = part(&sim, name)?.entity;
            if remove_body {
                sim.world_mut().entity_mut(e).remove::<BodyHandle>();
            } else {
                sim.world_mut().entity_mut(e).remove::<Name>();
            }
            assert!(Arm::bind(&sim).is_err(), "{name}");
        }
        let mut sim = fresh()?;
        let mut arm = Arm::bind(&sim)?;
        arm.rail_joint = JointId::from_raw(u64::MAX);
        assert!(arm.sample(&sim).is_err());
        let mut arm = Arm::bind(&sim)?;
        let stale_shelf = arm.shelf_body;
        arm.shelf_body = arm.bodies[0]; // A reused entity cannot stand in for the retired receiver.
        assert_eq!(arm.sample(&sim)?.support_impulse_ns, 0.);
        arm.shelf_body = stale_shelf;
        let mut recorder = Recorder {
            arm: &arm,
            trace: trace::Trace::open(None, None, Mode::Dynamic)?,
            check_shelf: false,
            min_shelf: f64::INFINITY,
            capture: &mut |_: &Simulation| -> Result<()> { Err("display stopped".into()) },
        };
        assert!(recorder.advance(&mut sim, true).is_err());
        sim.world()
            .resource::<gz_sim_core::StepFault>()
            .report(std::io::Error::other("solver stopped"));
        assert!(recorder.advance(&mut sim, false).is_err());
        Ok(())
    }

    #[test]
    fn substituted_adapters_and_ambiguous_contact_identities_abort_measurement() -> Result<()> {
        let mut sim = fresh()?;
        for index in [3, 4] {
            let mut arm = Arm::bind(&sim)?;
            arm.bodies[index] = arm.bodies[5];
            assert!(arm.sample(&sim).is_err());
        }
        let mut arm = Arm::bind(&sim)?;
        arm.bodies[5] = arm.shelf_body;
        assert!(arm.sample(&sim).is_err());
        let mut recorder = Recorder {
            arm: &arm,
            trace: trace::Trace::open(None, None, Mode::Dynamic)?,
            check_shelf: true,
            min_shelf: f64::INFINITY,
            capture: &mut |_: &Simulation| -> Result<()> { Ok(()) },
        };
        assert!(recorder.advance(&mut sim, false).is_err());
        let arm = Arm::bind(&sim)?;
        sim.world_mut()
            .insert_resource(PhysicsBackend::from(MultiFidelityBackend::new([0.; 3])?));
        assert!(Arm::bind(&sim).is_err());
        let mut alternate = load_world_with_sdf_step(
            "<sdf version='1.12'><world name='default'/></sdf>",
            "other.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )?;
        let other = alternate
            .world_mut()
            .remove_resource::<PhysicsBackend>()
            .unwrap();
        sim.world_mut().insert_resource(other);
        assert!(Arm::bind(&sim).is_err());
        assert!(arm.sample(&sim).is_err());
        Ok(())
    }

    #[test]
    fn interruption_at_the_park_boundary_or_paused_support_loss_stops_before_the_next_solve()
    -> Result<()> {
        let xml = mark_workcell(&payload::configure(
            Trial::ClearObstacle.xml(),
            Some(1.),
            None,
        )?)?;
        let sdf = sdformat::load_str(&xml, "standby.sdf", &sdformat::NoResources)?;
        let args = Args::try_parse_from([
            "serial_transfer",
            "clear-obstacle",
            "unused",
            "--fidelity",
            "adaptive",
            "--remove-receiver-during-standby",
        ])?;
        for at in [31_032, 31_282] {
            let mut sim = load_world_with_sdf_step(
                &xml,
                "standby.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )?;
            let e = run_profile(&mut sim, &sdf, &args, &mut |s: &Simulation| {
                if ms(s) == at {
                    Err("recording interrupted".into())
                } else {
                    Ok(())
                }
            })
            .unwrap_err();
            assert_eq!(e.to_string(), "recording interrupted");
            assert_eq!(ms(&sim), at);
            assert_eq!(part(&sim, "receiver").is_ok(), at == 31_032);
        }
        Ok(())
    }

    #[test]
    fn a_moving_mount_cannot_accumulate_a_quiet_dwell() -> Result<()> {
        let mut sim = fresh()?;
        let arm = Arm::bind(&sim)?;
        let mut recorder = Recorder {
            arm: &arm,
            trace: trace::Trace::open(None, None, Mode::Dynamic)?,
            check_shelf: false,
            min_shelf: f64::INFINITY,
            capture: &mut |_: &Simulation| -> Result<()> { Ok(()) },
        };
        assert!(
            recorder
                .wait_for_standby(&mut sim, arm.model, 0)
                .unwrap_err()
                .to_string()
                .contains("insufficient quiet solves")
        );
        let err = recorder
            .wait_for_standby(&mut sim, arm.model, 4)
            .unwrap_err()
            .to_string();
        assert!(err.contains("no safe unloaded standby"), "{err}");
        assert_eq!(model_fidelity(&sim, arm.model)?, ArticulationTier::Dynamic);
        assert_eq!(sim.elapsed(), Duration::from_millis(4));
        assert!(recorder.wait_for_standby(&mut sim, u64::MAX, 4).is_err());
        assert_eq!(sim.elapsed(), Duration::from_millis(5));
        Ok(())
    }

    #[test]
    fn missing_feedback_at_a_stage_boundary_never_starts_the_next_motion() -> Result<()> {
        let xml = mark_workcell(&payload::configure(
            Trial::ClearObstacle.xml(),
            Some(1.),
            None,
        )?)?;
        let sdf = sdformat::load_str(&xml, "standby.sdf", &sdformat::NoResources)?;
        let args = Args::try_parse_from([
            "serial_transfer",
            "clear-obstacle",
            "unused",
            "--fidelity",
            "adaptive",
        ])?;
        for (at, name, component) in [
            (19_500, "gripper::travel", "rail_state"),
            (25_500, "gripper::close_left", "name"),
            (25_500, "gripper::close_right", "name"),
            (25_500, "gripper::spindle", "drive"),
            (31_032, "receiver", "name"),
            (31_033, "gripper", "id"),
        ] {
            let mut sim = load_world_with_sdf_step(
                &xml,
                "standby.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )?;
            let entity = part(&sim, name)?.entity;
            sim.add_systems(
                gz_sim_core::Phase::Finalize,
                move |world: &mut bevy_ecs::world::World| {
                    if world
                        .resource::<gz_sim_core::UpdateInfo>()
                        .sim_time
                        .as_millis()
                        == at
                    {
                        let mut e = world.entity_mut(entity);
                        match component {
                            "rail_state" => {
                                e.remove::<JointSample>();
                            }
                            "drive" => {
                                e.remove::<gz_sim_scene::JointControllerConfiguration>();
                            }
                            "id" => {
                                e.remove::<EntityId>();
                            }
                            _ => {
                                e.remove::<Name>();
                            }
                        }
                    }
                },
            );
            assert!(
                run_profile(&mut sim, &sdf, &args, &mut |_| Ok(())).is_err(),
                "{name} at {at}"
            );
            assert_eq!(sim.elapsed().as_millis(), at, "{name}");
        }
        Ok(())
    }

    #[test]
    fn reset_restores_the_empty_tray_and_receiver_and_repeats_the_support_loss_handoff()
    -> Result<()> {
        let xml = mark_workcell(&payload::configure(
            Trial::ClearObstacle.xml(),
            Some(1.),
            None,
        )?)?;
        let sdf = sdformat::load_str(&xml, "standby.sdf", &sdformat::NoResources)?;
        let mut sim = load_world_with_sdf_step(
            &xml,
            "standby.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let args = Args::try_parse_from([
            "serial_transfer",
            "clear-obstacle",
            "unused",
            "--parcel-kg",
            "1",
            "--fidelity",
            "adaptive",
            "--remove-receiver-during-standby",
        ])?;
        let run = |sim: &mut Simulation| -> Result<_> {
            let mut poses = Sha256::new();
            let outcome = run_profile(sim, &sdf, &args, &mut |state| {
                poses.update((ms(state) as u64).to_le_bytes());
                for name in LINKS {
                    let pose = part(state, name)?.pose(state)?;
                    for v in pose.translation.into_iter().chain(pose.rotation) {
                        poses.update(v.to_le_bytes());
                    }
                }
                Ok(())
            })?;
            Ok((outcome, poses.finalize()))
        };
        let first = run(&mut sim)?;
        let stats = first.0.standby.as_ref().unwrap();
        assert_eq!(stats.wake_reason, "receiver_support_lost");
        assert_eq!(stats.prescribed_solves, 250);
        assert!(stats.support_loss.as_ref().unwrap().last_upward_impulse_ns > 0.009);
        assert_eq!(stats.first_resumed.support_impulse_ns, 0.);
        assert!(stats.first_resumed.bodies[5].com_velocity_m_s[2] < -0.009);
        assert!(part(&sim, "receiver").is_err() && part(&sim, "stand").is_err());
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert!(part(&sim, "receiver").is_ok() && part(&sim, "stand").is_ok());
        assert_eq!(first, run(&mut sim)?);
        Ok(())
    }
}
