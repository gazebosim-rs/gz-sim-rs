// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native revolute (or fixed control) carrying an independently integrated lower-face traction.
use super::*;
use gz_physics::{
    NativeStepGrid,
    sole_contact::{SoleBody, SoleContact, SoleDeck, SoleMaterial, SoleState, SoleStep},
};

#[derive(Clone, Copy, Debug)]
pub struct CoatedAnkle {
    pub foot: BodyId,
    pub shank: BodyId,
    pub floor: BodyId,
    pub material: SoleMaterial,
    pub substeps: u32,
    pub depth: u8,
}
#[derive(Clone, Copy, Debug)]
pub struct CoatedAnkleStep {
    pub deck: SoleDeck,
    /// All impulses act on the lower link. Moment is about its instantaneous COM.
    pub lower: SoleStep,
    /// Smallest verified clearance to the deck's lateral edges, in meters.
    pub edge_margin_m: f64,
    /// Smallest verified separation of the upper box above the deck.
    pub shank_margin_m: f64,
}
fn unsupported(capability: &'static str) -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "rapier",
        capability,
    }
}

#[derive(Clone, Copy)]
struct Shapes {
    foot: SoleBody,
    deck: SoleDeck,
    shank_half: Vector,
    rows: collision::SuppressedRows,
    hinge_radius: f64,
    root_radius: f64,
}
impl RapierBackend {
    fn ankle_shapes(&self, c: CoatedAnkle) -> Result<Shapes, PhysicsError> {
        let mut records = self.joints.records.values();
        let j = records.next().ok_or(PhysicsError::InvalidJoint)?.initial;
        if records.next().is_some()
            || j.parent != Some(c.foot)
            || j.child != c.shank
            || !matches!(j.motion, JointMotion::Angular)
            || !matches!(j.solver, Solver::Reduced)
            || self.bodies.len() != 3
            || self.world.bodies.len() != 3
            || self.world.colliders.len() != 3
            || self.curves.authored()
            || !self.held_wrenches.is_empty()
            || !self.parked_robots.is_empty()
            || !self.spinning_assemblies.is_empty()
        {
            return Err(unsupported(
                "coating requires an isolated lower-rooted dynamic ankle and one static deck",
            ));
        }
        let geometry = |id| -> Result<_, PhysicsError> {
            let (h, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
            let rb = &self.world.bodies[*h];
            let [ch] = rb.colliders() else {
                return Err(PhysicsError::InvalidValue);
            };
            let col = &self.world.colliders[*ch];
            let b = col
                .shape()
                .as_cuboid()
                .ok_or_else(|| unsupported("coated ankle needs three cuboids"))?;
            let pose = col
                .position_wrt_parent()
                .ok_or(PhysicsError::InvalidValue)?;
            if pose.translation != Vector::ZERO
                || pose.rotation != Rotation::IDENTITY
                || !col.is_enabled()
                || col.is_sensor()
            {
                return Err(unsupported(
                    "coated ankle needs centered, enabled collision boxes",
                ));
            }
            Ok((rb, *ch, b.half_extents))
        };
        let (lower, foot, half) = geometry(c.foot)?;
        let (upper, _, shank_half) = geometry(c.shank)?;
        let (floor, pad, deck_half) = geometry(c.floor)?;
        if c.floor == c.foot
            || c.floor == c.shank
            || !lower.is_dynamic()
            || !upper.is_dynamic()
            || !floor.is_fixed()
            || !collision::can_exchange_impulses(
                &self.world.colliders[foot],
                &self.world.colliders[pad],
                RigidBodyType::Fixed,
            )
            || [lower, upper]
                .iter()
                .any(|b| b.mass_properties().local_mprops.local_com != Vector::ZERO)
        {
            return Err(unsupported(
                "coated ankle requires COM-centered links over a static box",
            ));
        }
        let props = lower.mass_properties().local_mprops;
        Ok(Shapes {
            foot: SoleBody {
                mass: lower.mass(),
                inertia: props.inv_principal_inertia.recip().to_array().into(),
                half_extents: half.to_array().into(),
                foot_offset: Default::default(),
            },
            deck: SoleDeck {
                center: floor.translation().to_array().into(),
                orientation: *floor.rotation(),
                half_extents: deck_half.to_array().into(),
                velocity: Default::default(),
            },
            shank_half,
            rows: collision::SuppressedRows {
                foot,
                pad,
                second: None,
            },
            hinge_radius: j.frame2.translation.length() + shank_half.length(),
            root_radius: j.frame1.translation.length()
                + j.frame2.translation.length()
                + shank_half.length(),
        })
    }
    /// Validate the actual native construction and the initial finite contact domain.
    pub fn validate_coated_ankle(&self, c: CoatedAnkle, dt: Duration) -> Result<(), PhysicsError> {
        NativeStepGrid::new(dt, c.substeps)?;
        let sample = self.ankle_shapes(c)?.sample(self, c)?;
        if sample.edge <= 0. || sample.height <= 0. {
            return Err(unsupported(
                "ankle must start over its finite deck with shank contact clear",
            ));
        }
        Ok(())
    }
    /// Stage the entire active multibody and native contact caches. Upon success
    /// publish the candidate and consume its public forces/efforts exactly once.
    pub fn step_coated_ankle(
        &mut self,
        c: CoatedAnkle,
        dt: Duration,
    ) -> Result<CoatedAnkleStep, PhysicsError> {
        let grid = NativeStepGrid::new(dt, c.substeps)?;
        let shape = self.ankle_shapes(c)?;
        let mut trial = self.copy_passive_interval()?;
        trial.pending_wrenches = self.pending_wrenches.clone();
        let wrenches = [c.foot, c.shank].map(|id| {
            let rb = &self.world.bodies[self.bodies[&id].0];
            (id, rb.user_force().to_array(), rb.user_torque().to_array())
        });
        let mut receipt = CoatedAnkleStep {
            deck: shape.deck,
            lower: Default::default(),
            edge_margin_m: f64::INFINITY,
            shank_margin_m: f64::INFINITY,
        };
        // One temporal integration per material sample. With a free root and one
        // hinge the root translation and each angular coordinate follow one native drift.
        let parameters = trial.world.integration_parameters;
        trial.world.integration_parameters.num_solver_iterations = 1;
        let mut current = shape.sample(&trial, c)?;
        for (i, h) in grid.intervals().enumerate() {
            if i > 0 {
                for (id, f, t) in wrenches {
                    trial.add_world_wrench(id, f, t)?;
                }
                trial.joints.efforts.clone_from(&self.joints.efforts);
            }
            let contact = current.contact;
            trial.add_world_wrench(c.foot, contact.force.into(), contact.torque.into())?;
            trial.advance_coated_pair(h, shape.rows)?;
            let next = shape.sample(&trial, c)?;
            let reach = h.as_secs_f64() * (current.speed.max(next.speed));
            let edge = current.edge.min(next.edge) - reach;
            let shank = current.height.min(next.height) - reach;
            if edge <= 0. || shank <= 0. || !reach.is_finite() {
                return Err(unsupported(
                    "coated ankle sweep reaches the deck rim or unresolved shank contact",
                ));
            }
            receipt.edge_margin_m = receipt.edge_margin_m.min(edge);
            receipt.shank_margin_m = receipt.shank_margin_m.min(shank);
            let step = &mut receipt.lower;
            let t = h.as_secs_f64();
            step.impulse += contact.force * t;
            step.angular_impulse += contact.torque * t;
            step.normal_impulse += contact.normal_force * t;
            step.sliding_work += contact.sliding_power * t;
            step.dissipated_energy += contact.heat_power * t;
            step.peak_normal_force = step
                .peak_normal_force
                .max(contact.normal_force)
                .max(next.contact.normal_force);
            current = next;
        }
        receipt.lower.contact = current.contact;
        trial.world.integration_parameters = parameters;
        self.world = trial.world;
        self.joints = trial.joints;
        self.pending_wrenches = trial.pending_wrenches;
        self.welded_projection = trial.welded_projection;
        self.welded_exclusion = trial.welded_exclusion;
        self.fleet_clock = trial.fleet_clock;
        self.fleet_dispatch = trial.fleet_dispatch;
        Ok(receipt)
    }
}
struct Sample {
    contact: SoleContact,
    edge: f64,
    height: f64,
    speed: f64,
}
impl Shapes {
    fn sample(self, solver: &RapierBackend, c: CoatedAnkle) -> Result<Sample, PhysicsError> {
        let lo = solver.state(c.foot).ok_or(PhysicsError::MissingBody)?;
        let hi = solver.state(c.shank).ok_or(PhysicsError::MissingBody)?;
        let s = SoleState {
            center: lo.position.into(),
            orientation: Rotation::from_array(lo.orientation),
            velocity: lo.velocity.into(),
            spin: lo.angular_velocity.into(),
        };
        let contact = self
            .foot
            .contact(&s, &self.deck, &c.material, c.depth)
            .map_err(|_| unsupported("ankle contact exceeds the supported lower face"))?;
        let deck = Rotation::from_array(self.deck.orientation.to_array()).inverse();
        let d = Vector::from_array(self.deck.center.to_array());
        let half = Vector::from_array(self.deck.half_extents.to_array());
        let bounds = |state: BodyState, half: Vector| {
            let r = deck * Rotation::from_array(state.orientation);
            let extent = (r * Vector::X).abs() * half.x
                + (r * Vector::Y).abs() * half.y
                + (r * Vector::Z).abs() * half.z;
            (deck * (Vector::from_array(state.position) - d), extent)
        };
        let shoe = Vector::from_array(self.foot.half_extents.to_array());
        let (foot, fe) = bounds(lo, shoe);
        let (shank, se) = bounds(hi, self.shank_half);
        let edge = (half.x - foot.x.abs() - fe.x).min(half.y - foot.y.abs() - fe.y);
        let height = shank.z - se.z - half.z;
        let w = Vector::from_array(lo.angular_velocity);
        let upper_w = Vector::from_array(hi.angular_velocity);
        let speed = Vector::from_array(lo.velocity).length()
            + w.length() * self.root_radius.max(shoe.length())
            + (upper_w - w).length() * self.hinge_radius;
        Ok(Sample {
            contact,
            edge,
            height,
            speed,
        })
    }
}
