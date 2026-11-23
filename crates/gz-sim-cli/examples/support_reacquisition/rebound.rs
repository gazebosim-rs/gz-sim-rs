// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local moving-support rebound controls at the original Bevy and actuator clock.
use super::*;
use bevy_ecs::prelude::{Res, ResMut, Resource};
use gz_sim_core::{Phase, StepFault, UpdateInfo};
use rebound_gate::{Admission, ReboundGate, supported_gap};
use std::time::Instant;
#[path = "rebound_budget_continuous.rs"]
mod budget_continuous;
#[path = "rebound_compare.rs"]
mod compare;
#[path = "rebound_flight_detection.rs"]
mod flight_detection;
#[path = "weak_study.rs"]
pub(crate) mod weak_study;
pub(crate) use compare::budget_study::write as write_budget;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize)]
enum GridPhase {
    Aligned,
    HalfEarly,
    HalfLate,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize)]
enum Effort {
    Standard,
    Iterations(usize),
    Split(u32),
    Grid {
        calls: u32,
        iterations: usize,
        phase: GridPhase,
    },
}

impl Effort {
    fn native_steps(self) -> u32 {
        match self {
            Self::Split(n) | Self::Grid { calls: n, .. } => n,
            _ => 1,
        }
    }
    // The local fixture uses Rapier's four-iteration profile for ordinary solves.
    fn iterations(self) -> usize {
        match self {
            Self::Iterations(n) | Self::Grid { iterations: n, .. } => n,
            _ => 4,
        }
    }
    // Only the first and last intervals change on a phase-shifted grid.
    // Every interior boundary is translated by the same half period.
    fn validate_grid(self, dt: Duration) -> std::result::Result<(), PhysicsError> {
        let calls = self.native_steps();
        let shifted = matches!(
            self,
            Self::Grid {
                phase: GridPhase::HalfEarly | GridPhase::HalfLate,
                ..
            }
        );
        let divisor = u128::from(calls) * if shifted { 2 } else { 1 };
        if dt.is_zero()
            || calls == 0
            || (shifted && calls < 2)
            || dt.as_nanos() < u128::from(calls)
            || (shifted && !dt.as_nanos().is_multiple_of(divisor))
            || !(1..=32).contains(&self.iterations())
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }
    fn duration_at(self, dt: Duration, substep: u32) -> Duration {
        let calls = self.native_steps();
        // Keep exact equal durations on divisible and phase-shifted grids.
        if !dt.as_nanos().is_multiple_of(u128::from(calls)) {
            return gz_physics::NativeStepGrid::new(dt, calls)
                .expect("grid validated before advancing")
                .interval(substep - 1)
                .expect("one-based native solve index is in range");
        }
        let base = dt / calls;
        let Self::Grid { phase, .. } = self else {
            return base;
        };
        match (phase, substep == 1, substep == calls) {
            (GridPhase::HalfEarly, true, _) | (GridPhase::HalfLate, _, true) => base / 2,
            (GridPhase::HalfLate, true, _) | (GridPhase::HalfEarly, _, true) => base + base / 2,
            _ => base,
        }
    }
    fn solve(
        self,
        system: &mut MultiFidelityBackend,
        native_dt: Duration,
    ) -> std::result::Result<(), PhysicsError> {
        match self {
            Self::Iterations(n) | Self::Grid { iterations: n, .. } => {
                system.step_landing_with_iterations(native_dt, n)
            }
            _ => gz_physics::DynamicBackend::step(system, native_dt),
        }
    }
    fn increased(self) -> bool {
        self.native_steps() > 1 || self.iterations() > 4
    }
}

#[derive(Clone, Copy, Debug, Serialize)]
struct Budget {
    onset: Effort,
    follow: Effort,
    follow_ticks: u8,
}

impl Budget {
    fn effort(self, decision: Admission, remaining: &mut u8) -> Effort {
        match decision {
            Admission::Primary => {
                *remaining = 0;
                Effort::Iterations(16)
            }
            Admission::WeakStart => {
                *remaining = remaining.saturating_sub(1);
                Effort::Split(4)
            }
            Admission::SecondaryStart => {
                *remaining = self.follow_ticks;
                self.onset
            }
            _ if *remaining > 0 => {
                *remaining -= 1;
                self.follow
            }
            _ => Effort::Standard,
        }
    }
}

#[derive(Clone, Copy)]
enum Deck {
    Flat,
    Edge,
}
impl Deck {
    fn label(self) -> &'static str {
        match self {
            Self::Flat => "flat",
            Self::Edge => "edge",
        }
    }
    fn lane(self) -> Result<Lane> {
        match self {
            Self::Flat => Lane::new_at(STEP_US),
            Self::Edge => Lane::from_sdf_at(
                &SDF.replace("<pose>0 0 -0.06", "<pose>0.22 0 -0.06"),
                STEP_US,
            ),
        }
    }
}

#[derive(Clone, Copy)]
enum Mode {
    Legacy,
    Candidate,
    Coarse,
    Fixed { onset: u64 },
    FinerGrid { onset: u64, substeps: u32 },
    Budget(Budget),
    Weak(Budget),
    PrimaryOnset { secondary: Budget, primary: Effort },
    OverrideTick { tick: u64, effort: Effort },
    Uniform(Effort),
}
#[derive(Clone, Copy, Serialize)]
struct ReboundFrame {
    tick: u64,
    time_us: u64,
    foot: Body,
    upper: Body,
    pad: Body,
    normal_impulse_ns: f64,
    corner_ns: [f64; 4],
    selected: bool,
    secondary_start: bool,
    #[serde(skip)]
    weak_start: bool,
    #[serde(skip)]
    primary_start: bool,
    gap_m: f64,
    #[serde(skip)]
    native_steps: u32,
    #[serde(skip)]
    solver_iterations: usize,
    #[serde(skip)]
    first_loaded_substep: Option<u32>,
    #[serde(skip)]
    tangent_on_foot_ns: Option<[f64; 3]>,
}
impl ReboundFrame {
    fn with_effort(&self) -> impl Serialize + '_ {
        #[derive(Serialize)]
        struct Measured<'a> {
            #[serde(flatten)]
            frame: &'a ReboundFrame,
            native_steps: u32,
            solver_iterations: usize,
        }
        Measured {
            frame: self,
            native_steps: self.native_steps,
            solver_iterations: self.solver_iterations,
        }
    }
}
#[derive(Resource)]
struct Step {
    mode: Mode,
    effort: Effort,
    follow_remaining: u8,
    first_loaded_substep: Option<u32>,
    record_tangents: bool,
    point_recorder: Option<Box<weak_study::outer::points::Recorder>>,
    tangent_on_foot_ns: Option<[f64; 3]>,
    primary: Gate,
    candidate: ReboundGate,
    normal: f64,
    corners: [f64; 4],
    selected: bool,
    secondary_start: bool,
    weak_start: bool,
}
impl Step {
    fn step(
        &mut self,
        solver: &mut MultiFidelityBackend,
        foot: BodyId,
        pad: BodyId,
        tick: u64,
        dt: Duration,
    ) -> std::result::Result<(), PhysicsError> {
        let a = gz_physics::DynamicBackend::state(solver, foot).ok_or(PhysicsError::MissingBody)?;
        let b = gz_physics::DynamicBackend::state(solver, pad).ok_or(PhysicsError::MissingBody)?;
        self.secondary_start = false;
        self.weak_start = false;
        self.effort = match self.mode {
            Mode::Legacy => {
                if self.primary.advance(a, b, 2, dt.as_secs_f64()) {
                    Effort::Iterations(16)
                } else {
                    Effort::Standard
                }
            }
            Mode::Coarse => Effort::Standard,
            Mode::Uniform(effort) => effort,
            Mode::Fixed { onset } | Mode::FinerGrid { onset, .. } => {
                let n = if let Mode::FinerGrid { substeps, .. } = self.mode {
                    substeps
                } else {
                    4
                };
                if (onset..onset + 10).contains(&tick) {
                    Effort::Split(n)
                } else {
                    Effort::Standard
                }
            }
            Mode::Candidate
            | Mode::Budget(_)
            | Mode::Weak(_)
            | Mode::PrimaryOnset { .. }
            | Mode::OverrideTick { .. } => {
                let admission = self.candidate.admit(a, b, self.normal, 2, dt.as_secs_f64());
                self.secondary_start = admission == Admission::SecondaryStart;
                self.weak_start = admission == Admission::WeakStart;
                match self.mode {
                    Mode::PrimaryOnset { secondary, primary } => {
                        let ordinary = secondary.effort(admission, &mut self.follow_remaining);
                        if self.candidate.primary_started() {
                            primary
                        } else {
                            ordinary
                        }
                    }
                    Mode::Budget(budget) | Mode::Weak(budget) => {
                        budget.effort(admission, &mut self.follow_remaining)
                    }
                    Mode::OverrideTick { tick: when, effort } if when == tick => effort,
                    _ if admission != Admission::Ordinary => Effort::Iterations(16),
                    _ => Effort::Standard,
                }
            }
        };
        self.selected = self.effort.increased();
        let subs = self.effort.native_steps();
        self.effort.validate_grid(dt)?;
        self.normal = 0.;
        self.corners = [0.; 4];
        self.first_loaded_substep = None;
        let mut tangent = self.record_tangents.then_some(DVec3::ZERO);
        for substep in 1..=subs {
            let probe_before = self
                .point_recorder
                .as_ref()
                .map(|r| r.bodies(solver))
                .transpose()?;
            let native_dt = self.effort.duration_at(dt, substep);
            self.effort.solve(solver, native_dt)?;
            accumulate_reaction(
                solver,
                foot,
                pad,
                &mut self.corners,
                &mut self.normal,
                &mut tangent,
            )?;
            if let (Some(probe), Some(before)) = (&mut self.point_recorder, probe_before) {
                probe.observe(solver, before, tick, substep, native_dt)?;
            }
            if self.first_loaded_substep.is_none() && self.normal > 0. {
                self.first_loaded_substep = Some(substep);
            }
        }
        self.tangent_on_foot_ns = tangent.map(|v| v.to_array());
        Ok(())
    }
}
fn accumulate_reaction(
    solver: &MultiFidelityBackend,
    foot: BodyId,
    pad: BodyId,
    corners: &mut [f64; 4],
    normal: &mut f64,
    tangent: &mut Option<DVec3>,
) -> std::result::Result<(), PhysicsError> {
    let q = gz_physics::DynamicBackend::state(solver, foot)
        .ok_or(PhysicsError::MissingBody)?
        .orientation;
    let inverse = DQuat::from_array(q).conjugate();
    for patch in solver.rigid_contact_patches() {
        let first = (patch.a, patch.b) == (foot, pad);
        if !first && (patch.b, patch.a) != (foot, pad) {
            continue;
        }
        for point in patch.points {
            let r = inverse * DVec3::from(if first { point.lever_a } else { point.lever_b });
            corners[2 * usize::from(r.x > 0.) + usize::from(r.y > 0.)] += point.normal_impulse;
            *normal += point.normal_impulse;
            if let Some(sum) = tangent {
                if let Some(impulse) = point.tangent_impulse_on_a {
                    let impulse = DVec3::from(impulse);
                    *sum += if first { impulse } else { -impulse };
                } else {
                    // The grouped rigid-body solver may expose only a patch wrench.
                    *tangent = None;
                }
            }
        }
    }
    Ok(())
}
fn install(lane: &mut Lane, mode: Mode) -> Result<()> {
    let (foot, pad) = (lane.foot, lane.pad);
    let mut normal = 0.;
    let mut corners = [0.; 4];
    let backend = lane.sim.world().resource::<PhysicsBackend>();
    let solver = backend
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or("missing local solver")?;
    accumulate_reaction(solver, foot, pad, &mut corners, &mut normal, &mut None)?;
    if lane
        .sim
        .remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?
        != 1
    {
        return Err("fixture requires a single native step system".into());
    }
    lane.sim.world_mut().insert_resource(Step {
        mode,
        effort: Effort::Standard,
        follow_remaining: 0,
        first_loaded_substep: (normal > 0.).then_some(1),
        record_tangents: false,
        point_recorder: None,
        tangent_on_foot_ns: None,
        primary: Gate::default(),
        candidate: ReboundGate::with_weak_recontacts(matches!(
            mode,
            Mode::Weak(_) | Mode::PrimaryOnset { .. }
        )),
        normal,
        corners,
        selected: false,
        secondary_start: false,
        weak_start: false,
    });
    lane.sim.add_systems(
        Phase::PhysicsStep,
        move |t: Res<UpdateInfo>,
              fault: Res<StepFault>,
              mut backend: ResMut<PhysicsBackend>,
              mut receipt: ResMut<Step>| {
            if t.dt.is_zero() {
                return;
            }
            let solver = backend
                .downcast_mut::<MultiFidelityBackend>()
                .expect("rebound fixture backend");
            if let Err(e) = receipt.step(
                solver,
                foot,
                pad,
                t.iteration.saturating_sub(SETTLE_STEPS),
                t.dt,
            ) {
                fault.report(e);
            }
        },
    );
    lane.sim.initialize_schedules()?;
    Ok(())
}
fn state(b: Body) -> BodyState {
    BodyState {
        position: b.position,
        orientation: b.orientation,
        velocity: b.velocity,
        angular_velocity: b.angular_velocity,
    }
}
struct Rig {
    case: matched::Case,
    tick: u64,
    last_speed: f64,
    drive_history: Vec<f64>,
}
impl Rig {
    fn new(deck: Deck, switched: bool, mode: Mode) -> Result<Self> {
        Self::from_lane(deck.lane()?, switched, mode)
    }
    fn from_lane(mut lane: Lane, switched: bool, mode: Mode) -> Result<Self> {
        install(&mut lane, mode)?;
        let mut case = matched::Case::from_lane(
            if switched { 7 } else { 0 },
            (sweep::END_US / STEP_US + 1) as usize,
            lane,
        );
        case.advance(0, None)?;
        Ok(Self {
            case,
            tick: 0,
            last_speed: 0.,
            drive_history: Vec::with_capacity((sweep::END_US / STEP_US) as usize),
        })
    }
    fn advance(&mut self) -> Result<u64> {
        self.advance_at(sweep::command(self.tick * STEP_US, 0).1)
    }
    fn advance_at(&mut self, speed: f64) -> Result<u64> {
        if speed != self.last_speed {
            self.case.lane.drive(speed)?;
        }
        self.last_speed = speed;
        let start = Instant::now();
        self.case.lane.sim.step_exact(1)?;
        let elapsed = start.elapsed().as_nanos() as u64;
        self.drive_history.push(speed);
        self.tick += 1;
        self.case.observe(self.tick as usize)?;
        // A split step can deposit its entire reaction before the last substep.
        // Reconstruct the gate from the same outer receipt that it saw live.
        self.case.samples[self.tick as usize].normal_impulse_ns =
            self.case.lane.sim.world().resource::<Step>().normal;
        Ok(elapsed)
    }
    fn set_mode(&mut self, mode: Mode) {
        // Reconstruct the candidate's causal memory from the actual prefix. This
        // lets the comparison change treatment without charging shadow prediction
        // to the timed legacy steps or resetting a clearance witness at the cut.
        let mut step = self.case.lane.sim.world_mut().resource_mut::<Step>();
        if matches!(
            mode,
            Mode::Candidate
                | Mode::Budget(_)
                | Mode::Weak(_)
                | Mode::PrimaryOnset { .. }
                | Mode::OverrideTick { .. }
        ) {
            let mut remaining = 0;
            let mut gate = ReboundGate::with_weak_recontacts(matches!(
                mode,
                Mode::Weak(_) | Mode::PrimaryOnset { .. }
            ));
            for (sample, &speed) in self.case.samples.iter().zip(&self.drive_history) {
                let mut pad = state(sample.pad);
                pad.velocity[2] = speed;
                let decision = gate.admit(
                    state(sample.foot),
                    pad,
                    sample.normal_impulse_ns,
                    2,
                    STEP_US as f64 * 1e-6,
                );
                if let Mode::Budget(budget)
                | Mode::Weak(budget)
                | Mode::PrimaryOnset {
                    secondary: budget, ..
                } = mode
                {
                    budget.effort(decision, &mut remaining);
                }
            }
            step.follow_remaining = remaining;
            step.candidate = gate;
        }
        step.mode = mode;
    }
    fn frame(&self) -> Result<ReboundFrame> {
        let s = self.case.samples.last().ok_or("missing native boundary")?;
        let step = self.case.lane.sim.world().resource::<Step>();
        let gap_m =
            supported_gap(state(s.foot), state(s.pad)).ok_or("foot completely left the deck")?;
        Ok(ReboundFrame {
            tick: self.tick,
            time_us: self.tick * STEP_US,
            foot: s.foot,
            upper: s.upper,
            pad: s.pad,
            normal_impulse_ns: step.normal,
            corner_ns: step.corners,
            selected: step.selected,
            secondary_start: step.secondary_start,
            weak_start: step.weak_start,
            primary_start: matches!(step.mode, Mode::Weak(_) | Mode::PrimaryOnset { .. })
                && step.candidate.primary_started(),
            gap_m,
            native_steps: step.effort.native_steps(),
            solver_iterations: step.effort.iterations(),
            first_loaded_substep: step.first_loaded_substep,
            tangent_on_foot_ns: step.tangent_on_foot_ns,
        })
    }
}

fn continuous(deck: Deck, switched: bool, mode: Mode, out: Option<&Path>) -> Result<Value> {
    let mut rig = Rig::new(deck, switched, mode)?;
    let mut frames = Vec::new();
    let mut gap_peak = 0.0_f64;
    let mut landings = Vec::new();
    let mut secondary = Vec::new();
    let mut selected = Vec::new();
    let mut parked = 0;
    let mut prescribed = 0;
    let mut parked_ticks = 0;
    let mut unloaded_starts = Vec::new();
    let replay_cycle = match deck {
        Deck::Flat => 2,
        Deck::Edge => 1,
    };
    for tick in 1..=sweep::END_US / STEP_US {
        rig.advance()?;
        let f = rig.frame()?;
        let cycle = sweep::command((tick - 1) * STEP_US, 0).0;
        if f.normal_impulse_ns == 0. {
            gap_peak = gap_peak.max(f.gap_m);
        } else {
            if gap_peak > 1e-5 {
                let incoming = rig.case.samples[tick as usize - 1];
                let mut pad = incoming.pad;
                pad.velocity[2] = rig.last_speed;
                landings.push(json!({"cycle":cycle+1,"tick":tick,"peak_gap_m":gap_peak,"normal_ns":f.normal_impulse_ns,"selected":f.selected,"incoming_foot":incoming.foot,"incoming_pad":pad}));
            }
            gap_peak = 0.;
        }
        if f.secondary_start {
            secondary.push(tick);
            if f.normal_impulse_ns == 0. {
                unloaded_starts.push(tick);
            }
        }
        let is_parked = tick <= sweep::DEPART_US[0] / STEP_US
            || tick > sweep::DEPART_US[cycle] / STEP_US + STROKES[cycle].start_park();
        parked_ticks += usize::from(is_parked);
        if f.selected {
            selected.push(tick);
            parked += usize::from(is_parked);
        }
        prescribed += usize::from(rig.case.lane.tier()? == Prescribed);
        if out.is_some()
            && switched
            && cycle == replay_cycle
            && tick <= sweep::DEPART_US[cycle] / STEP_US + STROKES[cycle].start_park()
        {
            frames.push(f);
        }
    }
    let policy = if matches!(mode, Mode::Legacy) {
        "legacy"
    } else {
        "candidate"
    };
    if let Some(out) = out.filter(|_| switched) {
        #[derive(Serialize)]
        struct Trace {
            geometry: Value,
            frames: Vec<ReboundFrame>,
        }
        write_json(
            &out.join("traces")
                .join(format!("{}-{policy}.json", deck.label())),
            &Trace {
                geometry: json!({"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF}),
                frames,
            },
        )?;
    }
    Ok(
        json!({"deck":deck.label(),"switched":switched,"policy":policy,"landings":landings,"secondary_starts":secondary,"selected_ticks":selected,"parked_ticks":parked_ticks,"secondary_starts_without_load":unloaded_starts,"selected_parked":parked,"prescribed_boundaries":prescribed}),
    )
}
pub(crate) fn write_flight(out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    compare::flight::write(out)?;
    flight_detection::write(out)?;
    compare::free_control::write(out)
}

pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = Vec::new();
    for deck in [Deck::Flat, Deck::Edge] {
        for switched in [false, true] {
            for mode in [Mode::Legacy, Mode::Candidate] {
                rows.push(continuous(deck, switched, mode, Some(out))?);
            }
        }
    }
    fs::write(
        out.join("continuous.json"),
        serde_json::to_vec_pretty(&rows)?,
    )?;
    rebound_gate::write_event_audit(out)?;
    compare::write(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_new_secondary_restarts_its_budget_and_a_primary_cancels_pending_followup() {
        use Admission::*;
        let b = Budget {
            onset: Effort::Split(4),
            follow: Effort::Iterations(8),
            follow_ticks: 2,
        };
        let mut remaining = 0;
        let actual = [
            Ordinary,
            SecondaryStart,
            SecondaryFollow,
            SecondaryStart,
            SecondaryFollow,
            Ordinary,
            Ordinary,
            SecondaryStart,
            Primary,
            Ordinary,
        ]
        .map(|a| b.effort(a, &mut remaining));
        assert_eq!(
            actual,
            [
                Effort::Standard,
                b.onset,
                b.follow,
                b.onset,
                b.follow,
                b.follow,
                Effort::Standard,
                b.onset,
                Effort::Iterations(16),
                Effort::Standard
            ]
        );
    }

    #[test]
    fn native_budget_receipts_match_the_fixture_profile_and_restore_after_a_split() -> Result<()> {
        let mut rig = Rig::new(
            Deck::Flat,
            false,
            Mode::OverrideTick {
                tick: 165,
                effort: Effort::Split(4),
            },
        )?;
        // Native forks capture an airborne foot above a rising support. These
        // three ticks precede the first landing at tick 211.
        while rig.tick < 163 {
            rig.advance()?;
        }
        for steps in [1, 4, 1] {
            rig.advance()?;
            let f = rig.frame()?;
            assert_eq!(f.native_steps, steps);
            let lane = &rig.case.lane;
            let native = rig.case.backend().landing_fork(
                None,
                lane.foot,
                lane.pad,
                Duration::from_micros(STEP_US) / steps,
            )?;
            assert_eq!(f.solver_iterations, native.contact_settings().0);
            assert!((f.normal_impulse_ns - f.corner_ns.iter().sum::<f64>()).abs() < 1e-12);
        }
        Ok(())
    }
    #[test]
    fn weak_mode_refines_the_previously_missed_edge_tick_and_restores_ordinary_work() -> Result<()>
    {
        let budget = Budget {
            onset: Effort::Split(4),
            follow: Effort::Iterations(16),
            follow_ticks: 1,
        };
        let mut rig = Rig::new(Deck::Edge, false, Mode::Budget(budget))?;
        while rig.tick < 1484 {
            rig.advance()?;
        }
        assert_eq!(rig.frame()?.normal_impulse_ns, 0.);
        rig.set_mode(Mode::Weak(budget));
        rig.advance()?;
        let weak = rig.frame()?;
        assert!(weak.weak_start && !weak.secondary_start);
        assert_eq!((weak.native_steps, weak.solver_iterations), (4, 4));
        // Reconstruct the spent witness at the cut, even if the new solver
        // grid has delayed its first contact receipt to a later outer tick.
        rig.set_mode(Mode::Weak(budget));
        rig.advance()?;
        let after = rig.frame()?;
        assert!(!after.weak_start && !after.selected);
        assert_eq!((after.native_steps, after.solver_iterations), (1, 4));
        Ok(())
    }

    #[test]
    fn native_gate_selects_loaded_rebounds_and_stays_idle_through_park() -> Result<()> {
        for (deck, switched, target_tick) in [
            (Deck::Flat, true, 1454),
            (Deck::Flat, false, 1466),
            (Deck::Edge, true, 952),
            (Deck::Edge, false, 1469),
        ] {
            let row = continuous(deck, switched, Mode::Candidate, None)?;
            assert!(
                row["secondary_starts"]
                    .as_array()
                    .unwrap()
                    .contains(&target_tick.into())
            );
            assert_eq!(row["selected_parked"], 0);
            assert!(row["parked_ticks"].as_u64().unwrap() > 400);
            assert_eq!(row["secondary_starts_without_load"], json!([]));
            assert!(
                row["landings"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .any(|e| e["tick"] == target_tick
                        && e["selected"] == true
                        && e["peak_gap_m"].as_f64().unwrap() < 1e-3)
            );
        }
        Ok(())
    }
}
