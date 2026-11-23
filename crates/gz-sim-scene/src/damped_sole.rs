// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A compliant lower face on one independent rigid foot above finite deck panels.
//! The scene owns this contact domain for the entire public step, including flight.
//! Static collisionless observation models share the scene graph and can carry
//! fixed visuals, cameras and lidars. Articulated or free rigid guests require
//! an independently stepped island plus a swept cross-island contact check.
//!
//! Load the world using the local multifidelity engine. The foot is a free box
//! with principal moments along its link axes. Its COM may be offset. Each
//! fixed or prescribed support has a single box collision centered and aligned with its link. The panels may move
//! independently and share an edge. A lone support may rotate about its moving center. Their projected interiors must stay disjoint.
//!
//! ```xml
//! <plugin name="gz::sim::systems::DampedSole" filename="gz-sim-damped-sole-system">
//!   <foot>world::robot::sole</foot>
//!   <support>world::left::deck</support>
//!   <support deck_stiffness="3000000">world::right::deck</support>
//!   <foot_stiffness>1000000</foot_stiffness>
//!   <deck_stiffness>1000000</deck_stiffness>
//!   <damping>1</damping><friction>0.5</friction><slip_speed>0.002</slip_speed>
//!   <substeps>8</substeps><quadrature_depth>0</quadrature_depth>
//! </plugin>
//! ```
//!
//! A support may override `deck_stiffness`, `damping`, `friction` and
//! `slip_speed` as attributes. These coating properties govern registered
//! contacts. Layer stiffness uses N/m³, damping s/m and slip speed m/s. The fixed
//! midpoint grid keeps the actuator and sensor clocks on the SDF public tick.
//! To select a grid by convergence, add `max_substeps`, `max_quadrature_depth`,
//! `position_tolerance` (m), `rotation_tolerance` (rad), `impulse_tolerance`
//! (N·s), `angular_impulse_tolerance` (N·m·s) and `heat_tolerance` (J).
//! For multiple panels, `work_tolerance` (J) defaults to the heat tolerance.
//! A rejected trial leaves every body and the previous reaction intact.
//! Setting `flat_elastic_fast_path` to `true` enables the analytical solution
//! for centered, undamped, face-normal motion fully over a single support.
//!
//! Read each support's endpoint force, impulse and work with [panel_steps].
//! [last_step] also supplies the world COM trajectory and accepted solver grid.
use super::*;
use gz_physics::sole_contact::{
    SoleAccuracy, SoleAdaptiveStep, SoleBody, SoleDeck, SoleMaterial, SoleState, SoleStep,
};
use gz_physics::sole_supports::{
    PanelDifference, SolePanel, advance_panels, advance_panels_adaptive,
};
use gz_physics::sole_turntable::{
    AdaptiveTurntableStep, SoleTurntable, advance_turntable, advance_turntable_adaptive,
};
use gz_physics::{BodyState, DynamicBackend};
mod config;
pub(crate) use config::{Plan, is_plugin, observer_model, parse};
type SolveError = Box<dyn std::error::Error + Send + Sync>;

/// Integrated world-frame reaction of a single support on the foot.
#[derive(Clone, Copy, Debug)]
pub struct PanelStep {
    pub support: u64,
    /// Support pose and velocity at the end of the tick.
    pub deck: SoleDeck,
    pub step: SoleStep,
    /// Work supplied by motion of this support (J).
    pub support_work_j: f64,
    /// World angular velocity of the support about its center, in rad/s.
    pub angular_velocity: DVec3,
    /// Work supplied by support rotation, in joules.
    pub spin_work_j: f64,
}

/// Accepted shared grid and maximum error for any individual panel.
#[derive(Clone, Copy, Debug)]
pub struct PanelAccuracy {
    pub substeps: u32,
    pub depth: u8,
    pub trials: u32,
    pub temporal: PanelDifference,
    pub spatial: PanelDifference,
}

/// Foot state, public clock and first surviving support in stable public-ID order.
/// Use `panel_steps` for each support's separate load and work.
#[derive(Clone, Copy, Debug)]
pub struct DampedSoleStep {
    pub foot: u64,
    pub support: u64,
    pub time: UpdateInfo,
    pub state: SoleState,
    pub deck: SoleDeck,
    pub step: SoleStep,
    /// Single-support grid estimate when only contact tolerances were requested.
    pub adaptive: Option<SoleAdaptiveStep>,
    /// Across-panel estimate, including work on each panel.
    pub panel_accuracy: Option<PanelAccuracy>,
    /// Accepted rotating-support grid and observed errors, when adaptive.
    pub turntable_accuracy: Option<AdaptiveTurntableStep>,
    /// This tick used the analytical flat elastic contact trajectory.
    pub flat_elastic: bool,
    /// This fixed-grid tick used the spatially uniform normal damping path.
    pub flat_damped: bool,
}

#[derive(Clone, Copy)]
struct Support {
    public_id: u64,
    body_id: BodyId,
    half: DVec3,
    material: SoleMaterial,
}

#[derive(Resource)]
struct Coating {
    controls: Plan,
    foot: u64,
    foot_body: BodyId,
    supports: Vec<Support>,
    last: Option<DampedSoleStep>,
    rows: Vec<PanelStep>,
}

pub fn last_step(sim: &Simulation) -> Option<&DampedSoleStep> {
    sim.world().get_resource::<Coating>()?.last.as_ref()
}

/// Reactions for the last advancing tick, ordered by stable support link ID.
pub fn panel_steps(sim: &Simulation) -> &[PanelStep] {
    sim.world()
        .get_resource::<Coating>()
        .map_or(&[], |c| &c.rows)
}
pub(super) fn installed(world: &World) -> bool {
    world.contains_resource::<Coating>()
}
pub(super) fn translating_support(world: &World, id: u64) -> bool {
    world
        .get_resource::<Coating>()
        .is_some_and(|c| c.supports.len() > 1 && c.supports.iter().any(|p| p.public_id == id))
}
/// A lifecycle removal can occur while paused, before a new reaction is available.
pub(super) fn retire(world: &mut World, removed: &[u64]) {
    let Some(mut c) = world.get_resource_mut::<Coating>() else {
        return;
    };
    if removed.contains(&c.foot) {
        c.rows.clear();
        c.last = None;
        return;
    }
    c.rows.retain(|p| !removed.contains(&p.support));
    if let Some(first) = c.rows.first().copied() {
        if let Some(last) = &mut c.last {
            last.support = first.support;
            last.step = first.step;
            last.deck = first.deck;
        }
    } else {
        c.last = None
    }
}

pub(super) fn invalid(world: &str, reason: &str) -> LoadError {
    LoadError::DampedSole {
        world: world.into(),
        reason: reason.into(),
    }
}

fn sole_state(body: &SoleBody, foot: BodyState) -> SoleState {
    let orientation = DQuat::from_array(foot.orientation);
    SoleState {
        center: DVec3::from(foot.position) - orientation * body.foot_offset,
        orientation,
        velocity: foot.velocity.into(),
        spin: foot.angular_velocity.into(),
    }
}

fn panel(shape: Support, state: BodyState) -> SolePanel {
    SolePanel {
        id: shape.public_id,
        material: shape.material,
        deck: SoleDeck {
            center: state.position.into(),
            orientation: DQuat::from_array(state.orientation),
            half_extents: shape.half,
            velocity: state.velocity.into(),
        },
    }
}

pub(crate) fn install(
    sim: &mut Simulation,
    backend: PhysicsBackend,
    links: &BTreeMap<String, u64>,
    controls: Plan,
) -> Result<(), LoadError> {
    let native = backend
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or_else(|| invalid(&controls.world, "requires local multifidelity physics"))?;
    let resolve = |name: &str| -> Result<_, LoadError> {
        let id = *links
            .get(name)
            .ok_or_else(|| invalid(name, "missing public link"))?;
        let body = gz_sim_core::entity_by_id(sim.world(), id)
            .and_then(|e| sim.world().get::<BodyHandle>(e))
            .ok_or_else(|| invalid(name, "missing body binding"))?
            .id();
        Ok((id, body))
    };
    let (foot, f) = resolve(&controls.foot)?;
    let state = sole_state(
        &controls.body,
        native.state(f).ok_or(PhysicsError::MissingBody)?,
    );
    let mut supports = Vec::new();
    let mut ids = Vec::new();
    let mut panels = Vec::new();
    for (name, half, material) in &controls.supports {
        let (public_id, id) = resolve(name)?;
        let shape = Support {
            public_id,
            body_id: id,
            half: *half,
            material: *material,
        };
        let boundary = native.state(id).ok_or(PhysicsError::MissingBody)?;
        let p = panel(shape, boundary);
        let table = SoleTurntable {
            deck: p.deck,
            angular_velocity: boundary.angular_velocity.into(),
        };
        table
            .contact(&controls.body, &state, &p.material, controls.depth)
            .map_err(|e| invalid(&controls.world, &e.to_string()))?;
        supports.push(shape);
        panels.push(p);
        ids.push(id);
    }
    native.sole_wrench(f, ids.iter().copied())?;
    gz_physics::sole_supports::validate_panels(&panels, sim.step_duration().as_secs_f64())
        .map_err(|e| invalid(&controls.world, &e.to_string()))?;
    supports.sort_by_key(|p| p.public_id);
    gz_physics::install_with_step(sim, backend, advance)?;
    let rows = Vec::with_capacity(supports.len());
    sim.world_mut().insert_resource(Coating {
        controls,
        foot,
        foot_body: f,
        supports,
        last: None,
        rows,
    });
    sim.add_systems(
        Phase::Reset,
        (|mut c: ResMut<Coating>| {
            c.last = None;
            c.rows.clear()
        })
        .after(PhysicsReset),
    );
    Ok(())
}

#[derive(Default)]
struct Scratch {
    panels: Vec<SolePanel>,
    endpoints: Vec<(BodyId, BodyState)>,
    rows: Vec<PanelStep>,
}
impl Coating {
    fn solve(
        &mut self,
        native: &mut MultiFidelityBackend,
        foot: BodyId,
        time: UpdateInfo,
        gravity: DVec3,
        scratch: &mut Scratch,
    ) -> Result<(), SolveError> {
        let c = &self.controls;
        let (force, torque) = native.sole_wrench(foot, scratch.endpoints.iter().map(|p| p.0))?;
        let mut state = sole_state(
            &c.body,
            native.state(foot).ok_or(PhysicsError::MissingBody)?,
        );
        let dt = time.dt.as_secs_f64();
        let mut adaptive = None;
        let mut panel_accuracy = None;
        let mut turntable_accuracy = None;
        scratch.rows.clear();
        let mut row = |p: &SolePanel, step: SoleStep| {
            let mut deck = p.deck;
            deck.center += dt * deck.velocity;
            scratch.rows.push(PanelStep {
                support: p.id,
                deck,
                step,
                support_work_j: step.impulse.dot(deck.velocity),
                angular_velocity: DVec3::ZERO,
                spin_work_j: 0.,
            });
        };
        let exact = match scratch.panels.as_slice() {
            [p] if c.flat_elastic_fast_path
                && scratch.endpoints[0].1.angular_velocity == [0.; 3] =>
            {
                c.body.try_flat_elastic_step(
                    &mut state,
                    &p.deck,
                    &p.material,
                    gravity,
                    dt,
                    force.into(),
                    torque.into(),
                )?
            }
            _ => None,
        };
        let scalar = match scratch.panels.as_slice() {
            [p] if exact.is_none()
                && c.flat_damped_fast_path
                && c.accuracy.is_none()
                && scratch.endpoints[0].1.angular_velocity == [0.; 3] =>
            {
                c.body.try_flat_damped_step(
                    &mut state,
                    &p.deck,
                    &p.material,
                    gravity,
                    dt,
                    c.substeps,
                    force.into(),
                    torque.into(),
                )?
            }
            _ => None,
        };
        if let [p] = scratch.panels.as_slice()
            && scratch.endpoints[0].1.angular_velocity != [0.; 3]
        {
            let angular_velocity = DVec3::from(scratch.endpoints[0].1.angular_velocity);
            let table = SoleTurntable {
                deck: p.deck,
                angular_velocity,
            };
            let solution = if let Some(policy) = c.accuracy {
                let r = advance_turntable_adaptive(
                    &c.body,
                    &mut state,
                    &table,
                    &p.material,
                    gravity,
                    force.into(),
                    torque.into(),
                    dt,
                    &policy,
                    c.panel_work_tolerance.unwrap_or(policy.heat_j),
                )?;
                turntable_accuracy = Some(r);
                r.result
            } else {
                advance_turntable(
                    &c.body,
                    &mut state,
                    &table,
                    &p.material,
                    gravity,
                    force.into(),
                    torque.into(),
                    dt,
                    c.substeps,
                    c.depth,
                )?
            };
            scratch.rows.push(PanelStep {
                support: p.id,
                deck: table.at(dt),
                step: solution.on_sole,
                support_work_j: solution.support_work_j,
                spin_work_j: solution.spin_work_j,
                angular_velocity,
            });
        } else if let Some(step) = exact.or(scalar) {
            row(&scratch.panels[0], step);
        } else if let [p] = scratch.panels.as_slice()
            && c.panel_work_tolerance.is_none()
        {
            let step = if let Some(policy) = c.accuracy {
                let receipt = c.body.step_adaptive(
                    &mut state,
                    &p.deck,
                    &p.material,
                    gravity,
                    dt,
                    &policy,
                    force.into(),
                    torque.into(),
                )?;
                adaptive = Some(receipt);
                receipt.step
            } else {
                c.body.step_with_wrench(
                    &mut state,
                    &p.deck,
                    &p.material,
                    gravity,
                    dt,
                    c.substeps,
                    c.depth,
                    force.into(),
                    torque.into(),
                )?
            };
            row(p, step);
        } else {
            let reactions = if let (Some(policy), Some(work)) = (c.accuracy, c.panel_work_tolerance)
            {
                let result = advance_panels_adaptive(
                    &c.body,
                    &mut state,
                    &scratch.panels,
                    gravity,
                    force.into(),
                    torque.into(),
                    dt,
                    &policy,
                    work,
                )?;
                panel_accuracy = Some(PanelAccuracy {
                    substeps: result.substeps,
                    depth: result.depth,
                    trials: result.trials,
                    temporal: result.temporal,
                    spatial: result.spatial,
                });
                result.panels
            } else {
                advance_panels(
                    &c.body,
                    &mut state,
                    &scratch.panels,
                    gravity,
                    force.into(),
                    torque.into(),
                    dt,
                    c.substeps,
                    c.depth,
                )?
            };
            for (p, r) in scratch.panels.iter().zip(reactions) {
                debug_assert_eq!(p.id, r.id);
                row(p, r.on_sole);
            }
        }
        for (row, (_, end)) in scratch.rows.iter().zip(&mut scratch.endpoints) {
            end.position = row.deck.center.into();
            end.orientation = row.deck.orientation.to_array();
        }
        let end = BodyState {
            position: (state.center + state.orientation * c.body.foot_offset).into(),
            orientation: state.orientation.to_array(),
            velocity: state.velocity.into(),
            angular_velocity: state.spin.into(),
        };
        native.commit_sole_panels((foot, end), &scratch.endpoints)?;
        let first = scratch.rows[0];
        self.last = Some(DampedSoleStep {
            foot: self.foot,
            support: first.support,
            time,
            state,
            deck: first.deck,
            step: first.step,
            adaptive,
            panel_accuracy,
            turntable_accuracy,
            flat_elastic: exact.is_some(),
            flat_damped: scalar.is_some(),
        });
        std::mem::swap(&mut self.rows, &mut scratch.rows);
        Ok(())
    }
}
#[allow(clippy::too_many_arguments)]
fn advance(
    time: Res<UpdateInfo>,
    gravity: Res<Gravity>,
    ids: EntityLookup,
    bodies: Query<&BodyHandle>,
    mut coating: ResMut<Coating>,
    mut backend: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
    mut scratch: Local<Scratch>,
) {
    if time.dt.is_zero() {
        return;
    }
    let result = (|| -> Result<(), SolveError> {
        let native = backend
            .downcast_mut::<MultiFidelityBackend>()
            .ok_or(PhysicsError::MissingBody)?;
        let resolve = |id, expected| -> Result<Option<BodyId>, PhysicsError> {
            match ids.get(id) {
                Some(e) if bodies.get(e).is_ok_and(|h| h.id() == expected) => Ok(Some(expected)),
                None if native.state(expected).is_none() => Ok(None),
                _ => Err(PhysicsError::MissingBody),
            }
        };
        let foot = resolve(coating.foot, coating.foot_body)?;
        scratch.panels.clear();
        scratch.endpoints.clear();
        for &shape in &coating.supports {
            if let Some(id) = resolve(shape.public_id, shape.body_id)? {
                let initial = native.state(id).ok_or(PhysicsError::MissingBody)?;
                scratch.panels.push(panel(shape, initial));
                scratch.endpoints.push((id, initial));
            }
        }
        if let Some(foot) = foot
            && !scratch.panels.is_empty()
        {
            coating.solve(native, foot, *time, gravity.0.into(), &mut scratch)?;
        } else {
            // Without a surviving pair, remaining rigid bodies own ordinary free motion.
            native.step(time.dt)?;
            coating.rows.clear();
            coating.last = None;
        }
        Ok(())
    })();
    if let Err(e) = result {
        fault.report(std::io::Error::other(e))
    }
}
