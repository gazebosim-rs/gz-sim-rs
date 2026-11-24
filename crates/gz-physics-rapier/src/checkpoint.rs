// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Pinned solver checkpoint for a complete partition of awake dynamic boxes.
use super::*;
use bincode::Options;
use serde::{Deserialize, Serialize};

/// Maximum dynamic members of one transferable solver.
pub const MAX_CHECKPOINT_BODIES: usize = 32;
/// Component capacity and rigid-contact friction for a distributed native graph.
/// All four identities have the same byte ceiling. The original two discriminants
/// remain stable in bincode records and historical grouped pool archives.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum NativeSolverProfile {
    #[default]
    Standard32,
    Extended64,
    Standard32PerPoint,
    Extended64PerPoint,
}
impl NativeSolverProfile {
    pub const fn max_bodies(self) -> usize {
        match self {
            Self::Standard32 | Self::Standard32PerPoint => MAX_CHECKPOINT_BODIES,
            Self::Extended64 | Self::Extended64PerPoint => 64,
        }
    }
    pub const fn friction(self) -> NativeFrictionModel {
        match self {
            Self::Standard32 | Self::Extended64 => NativeFrictionModel::Grouped,
            Self::Standard32PerPoint | Self::Extended64PerPoint => NativeFrictionModel::PerPoint,
        }
    }
    pub const fn with_friction(self, model: NativeFrictionModel) -> Self {
        match (self, model) {
            (Self::Standard32 | Self::Standard32PerPoint, NativeFrictionModel::Grouped) => Self::Standard32,
            (Self::Extended64 | Self::Extended64PerPoint, NativeFrictionModel::Grouped) => Self::Extended64,
            (Self::Standard32 | Self::Standard32PerPoint, NativeFrictionModel::PerPoint) => Self::Standard32PerPoint,
            (Self::Extended64 | Self::Extended64PerPoint, NativeFrictionModel::PerPoint) => Self::Extended64PerPoint,
        }
    }
    pub const fn capacity_profile(self) -> Self {
        self.with_friction(NativeFrictionModel::Grouped)
    }
    pub fn is_standard(&self) -> bool {
        *self == Self::Standard32
    }
}
/// Upper bound on the pinned native archive before allocation or decoding.
pub const PARTITION_CHECKPOINT_LIMIT: usize = 2 * 1024 * 1024;
const ORIENTED_GROUND: u32 = 1 << 31;
const EXTENDED_PROFILE: u32 = 1 << 30;
const PER_POINT_PROFILE: u32 = 1 << 29;
const FORMAT: u32 = 2;
const SOLVER_FORMAT: u32 = 3;
const FIXED_FORMAT: u32 = 4;
const GROUND_FORMAT: u32 = 5;
const MATERIAL_FORMAT:u32=6;
const PRIMITIVE_FORMAT:u32=7;
const CAPSULE_FORMAT:u32=8;
const CYLINDER_FORMAT:u32=9;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
enum BoxKind {
    Dynamic,
    Prescribed,
    Fixed,
}

impl BoxKind {
    fn native_type(self) -> RigidBodyType {
        match self {
            Self::Dynamic => RigidBodyType::Dynamic,
            Self::Prescribed => RigidBodyType::KinematicVelocityBased,
            Self::Fixed => RigidBodyType::Fixed,
        }
    }
}

/// Maximum speed at which the pinned box solver can honor an impulse without clamping.
pub fn partition_linear_speed_limit() -> f64 {
    IntegrationParameters::default().max_linear_velocity()
}

/// Free COM velocity change for a centered, unlocked partition body. Use the
/// same effective mass, summation and multiplication order as the native solve:
/// large opposing field and user forces can leave a representable residual.
pub fn partition_velocity_change(
    mass: f64,
    gravity: [f64; 3],
    force: [f64; 3],
    dt: f64,
) -> [f64; 3] {
    let mut mobility = RigidBodyMassProps::default();
    mobility.effective_inv_mass = Vector::splat(mass.recip());
    let mut forces = RigidBodyForces {
        user_force: Vector::from(force),
        ..Default::default()
    };
    forces.compute_effective_force_and_torque(Vector::from(gravity), mobility.effective_mass());
    (forces.force * mobility.effective_inv_mass * dt).to_array()
}

fn codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(PARTITION_CHECKPOINT_LIMIT as u64)
        .reject_trailing_bytes()
}

fn invalid() -> PhysicsError {
    PhysicsError::InvalidValue
}

#[derive(Serialize, Deserialize)]
struct BoxRecord {
    id: u64,
    handle: RigidBodyHandle,
    half_extents: [f64; 3],
    mass: f64,
    position: [f64; 3],
    velocity: [f64; 3],
    orientation: [f64; 4],
    angular_velocity: [f64; 3],
    has_wrench: bool,
}

fn validate_world(
    world: &PhysicsWorld,
    count: usize,
    profile: NativeSolverProfile,
) -> Result<(), PhysicsError> {
    if world.gravity != Vector::ZERO
        || world.bodies.len() != count
        || world.colliders.len()
            != count + usize::from(partition_ground::ground_of(world)?.is_some())
        || !world.impulse_joints.is_empty()
        || world.multibody_joints.iter().next().is_some()
        || !world.soft_bodies.is_empty()
    {
        return Err(invalid());
    }
    let defaults = IntegrationParameters {
        friction_model: profile.friction().native(),
        contact_recycling: false,
        ..IntegrationParameters::default()
    };
    let params = world.integration_parameters;
    // The distributed island sets dt during a step; a new island retains the default dt.
    if !(params.dt == defaults.dt || [0.001, 0.002, 0.005, 0.01].contains(&params.dt))
        || params
            != (IntegrationParameters {
                dt: params.dt,
                ..defaults
            })
    {
        return Err(invalid());
    }
    Ok(())
}

#[derive(Clone,Copy,Debug,PartialEq,Eq,Serialize,Deserialize)]
enum PrimitiveKind { Box, Sphere, Capsule, Cylinder }
impl PrimitiveKind {
    fn shape(self,dimensions:[f64;3])->Result<Shape,PhysicsError>{
        let [x,y,z]=dimensions;
        match self {
            Self::Box=>Ok(Shape::Box{half_extents:dimensions}),
            Self::Sphere if x==y && y==z=>Ok(Shape::Sphere{radius:x}),
            Self::Cylinder if x==y=>Ok(Shape::Cylinder{radius:x,half_height:z}),
            Self::Capsule if x==y=>Ok(Shape::Capsule{radius:x,half_length:z}),
            _=>Err(invalid()),
        }
    }
}

// Every format keeps three construction dimensions. A capsule stores its
// radius twice and its centerline half-length, independently of its end caps.
fn primitive_dimensions(c:&Collider)->Option<(PrimitiveKind,[f64;3])>{
    let shape=c.shape();
    if let Some(cube)=shape.as_cuboid(){Some((PrimitiveKind::Box,cube.half_extents.to_array()))}
    else if let Some(cylinder)=shape.as_cylinder(){Some((PrimitiveKind::Cylinder,[cylinder.radius,cylinder.radius,cylinder.half_height]))}
    else if let Some(ball)=shape.as_ball(){Some((PrimitiveKind::Sphere,[ball.radius;3]))}
    else{
        let capsule=shape.as_capsule()?;
        let h=capsule.segment.b.z;
        (h>0. && capsule.segment.b==Vector::Z*h && capsule.segment.a==-capsule.segment.b)
            .then_some((PrimitiveKind::Capsule,[capsule.radius,capsule.radius,h]))
    }
}
impl BoxRecord {
    fn validate(
        &self,
        world: &PhysicsWorld,
        expected: &Collider,
        kind: BoxKind,
    ) -> Result<(), PhysicsError> {
        let handle = self.handle;
        let live = world.bodies.get(handle).ok_or_else(invalid)?;
        let [collider] = live.colliders() else {
            return Err(invalid());
        };
        let collider = world.colliders.get(*collider).ok_or_else(invalid)?;
        let dimensions=primitive_dimensions(collider).ok_or_else(invalid)?;
        if !live.is_enabled()
            || live.body_type() != kind.native_type()
            || (kind != BoxKind::Dynamic && self.has_wrench)
            || (kind == BoxKind::Fixed && (self.mass.to_bits() != 0 || self.velocity != [0.;3] || self.angular_velocity != [0.;3] || live.linvel() != Vector::ZERO || live.angvel() != Vector::ZERO))
            // The native island manager marks an unmoving kinematic body
            // asleep after 0.5 s even with both sleep thresholds disabled.
            // Retain that state in the image; a subsequent twist wakes it.
            || (live.is_sleeping() && (kind == BoxKind::Dynamic || live.linvel() != Vector::ZERO || live.angvel() != Vector::ZERO))
            || !live.locked_axes().is_empty()
            || live.linear_damping() != 0.
            || live.angular_damping() != 0.
            || live.gravity_scale() != 1.
            || live.is_ccd_enabled()
            || live.soft_ccd_prediction() != 0.
            || live.activation().normalized_linear_threshold != -1.
            || live.activation().angular_threshold != -1.
            || !live.translation().is_finite()
            || rotation(live.rotation().to_array()).is_err()
            || !live.linvel().is_finite()
            || !live.angvel().is_finite()
            || !live.user_force().is_finite()
            || !live.user_torque().is_finite()
            || (!self.has_wrench
                && (live.user_force() != Vector::ZERO || live.user_torque() != Vector::ZERO))
            || !collider.is_enabled()
            || collider.is_sensor()
            || !expected.friction().is_finite() || expected.friction()<0.
            || collider.friction().to_bits()!=expected.friction().to_bits()
            || collider.friction_combine_rule()!=expected.friction_combine_rule()
            || collider.restitution_combine_rule()!=expected.restitution_combine_rule()
            || collider.restitution() != 0.
            || collider.collision_groups() != expected.collision_groups()
            || collider.solver_groups() != expected.solver_groups()
            || collider.active_hooks() != expected.active_hooks()
            || collider.contact_skin() != expected.contact_skin()
            || collider.active_collision_types() != expected.active_collision_types()
            || collider.parent() != Some(handle)
            || collider.position_wrt_parent() != Some(expected.position())
            || dimensions.1 != self.half_extents
            || Some(dimensions)!=primitive_dimensions(expected)
        {
            return Err(invalid());
        }
        if expected.mass_properties() != collider.mass_properties()
            || expected.mass_properties().mass() != live.mass()
        {
            return Err(invalid());
        }
        Ok(())
    }
}

impl RapierBackend {
    /// Configure a fresh distributed native solver before its first barrier.
    pub fn set_partition_step(
        &mut self,
        duration: std::time::Duration,
    ) -> Result<(), PhysicsError> {
        let dt = duration.as_secs_f64();
        if ![0.001, 0.002, 0.005, 0.01].contains(&dt) || !self.world.bodies.is_empty() {
            return Err(PhysicsError::InvalidValue);
        }
        self.world.integration_parameters.dt = dt;
        Ok(())
    }

    /// Return to the 10 ms public configuration after the final 1 ms solve.
    /// The next native step selects its dt before integrating. Current contact
    /// geometry and warm-start impulses remain those of the final fine solve.
    pub fn finish_refined_partition_step(&mut self) {
        self.world.integration_parameters.dt = 0.01;
    }

    /// Historical 10 ms cold seeds retain Rapier's default interval until their first tick.
    pub fn partition_step_matches(&self, duration: std::time::Duration) -> bool {
        let dt = self.world.integration_parameters.dt;
        dt == duration.as_secs_f64()
            || (duration == std::time::Duration::from_millis(10)
                && dt == IntegrationParameters::default().dt)
    }

    /// Select the world acceleration for the next native partition substeps.
    /// The worker's published checkpoints are taken between field intervals.
    pub fn set_partition_gravity(&mut self, gravity: [f64; 3]) -> Result<(), PhysicsError> {
        if !finite(gravity) {
            return Err(PhysicsError::InvalidValue);
        }
        self.world.gravity = Vector::from_array(gravity);
        Ok(())
    }

    /// Manifolds and native per-point normal impulses retained to warm-start the next solve.
    pub fn partition_warmstart(&self) -> (usize, Vec<u64>) {
        let mut count = 0;
        let mut impulses = Vec::new();
        for pair in self.world.narrow_phase.contact_pairs() {
            // Rapier processes collider removals on its next solve. These
            // retired edges are retained in the exact native image but no
            // longer belong to a live body or its reported warm-start data.
            if !self.world.colliders.contains(pair.collider1)
                || !self.world.colliders.contains(pair.collider2)
            {
                continue;
            }
            for manifold in pair.solver_manifolds() {
                count += 1;
                impulses.extend(
                    manifold
                        .points
                        .iter()
                        .map(|p| p.data.warmstart_impulse.to_bits()),
                );
            }
        }
        (count, impulses)
    }

    pub fn partition_box_count(&self) -> usize {
        self.bodies.len()
    }

    pub fn partition_box(&self, id: BodyId) -> Option<([f64; 3], f64)> {
        let (_, construction, ..) = self.bodies.get(&id)?;
        let half = construction.collider.shape().as_cuboid()?.half_extents;
        Some(((half * 2.).to_array(), construction.mass.unwrap_or(0.)))
    }

    pub fn partition_shape(&self,id:BodyId)->Option<(Shape,f64)>{
        let (_,construction,..)=self.bodies.get(&id)?;
        let (kind,dimensions)=primitive_dimensions(&construction.collider)?;
        let shape=kind.shape(dimensions).ok()?;
        Some((shape,construction.mass.unwrap_or(0.)))
    }

    pub fn partition_friction(&self,id:BodyId)->Option<f64>{
        self.bodies.get(&id).map(|b|b.1.collider.friction())
    }
    pub fn checkpoint_boxes(&self) -> Result<Vec<u8>, PhysicsError> {
        if self.bodies.len() < 2
            || self
                .bodies
                .values()
                .any(|entry| entry.1.kinematic || entry.1.mass.is_none() || entry.1.collider.shape().as_cuboid().is_none())
        {
            return Err(invalid());
        }
        self.checkpoint_solver()
    }

    /// Serialize one complete native worker solver. The extended format adds
    /// prescribed bodies and singletons. All-dynamic groups retain their
    /// existing two-or-more-box encoding and digest.
    pub fn checkpoint_solver(&self) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_solver_with_profile(NativeSolverProfile::Standard32)
    }

    pub fn checkpoint_solver_with_profile(
        &self,
        profile: NativeSolverProfile,
    ) -> Result<Vec<u8>, PhysicsError> {
        if self.curves.authored() || self.conserved_rotor.is_some() || self.join_queries_pending
            || self.native_profile.is_some_and(|bound| bound != profile)
            || !(1..=profile.max_bodies()).contains(&self.bodies.len())
            || self.fleet_clock != FleetClock::Coupled
            || !self.spinning_assemblies.is_empty()
            || !self.held_wrenches.is_empty()
        {
            return Err(invalid());
        }
        validate_world(&self.world, self.bodies.len(), profile)?;
        let record = |(&id, (handle, c, r, inertia, group)): (&BodyId, &BodyEntry)| {
            if c.can_sleep || c.articulation.is_some() || inertia.is_some() || group.is_some() {
                return Err(invalid());
            }
            let (_,half)=primitive_dimensions(&c.collider).ok_or_else(invalid)?;
            let record = BoxRecord {
                id: id.raw(),
                handle: *handle,
                half_extents: half,
                mass: c.mass.unwrap_or(0.),
                position: c.position,
                velocity: c.velocity,
                orientation: r.orientation,
                angular_velocity: r.angular_velocity,
                has_wrench: self.pending_wrenches.contains(&id),
            };
            let kind = if c.kinematic {
                BoxKind::Prescribed
            } else if c.mass.is_none() {
                BoxKind::Fixed
            } else {
                BoxKind::Dynamic
            };
            record.validate(&self.world, &c.collider, kind)?;
            Ok((kind, record))
        };
        let records = self
            .bodies
            .iter()
            .map(record)
            .collect::<Result<Vec<_>, _>>()?;
        let open = self.joints.construction_open();
        // Keep previous horizontal terrain images byte-identical. The flag is
        // independent of the body's box, sphere, or capsule record layout.
        let mut profile_flag = if profile.capacity_profile() == NativeSolverProfile::Extended64 {
            EXTENDED_PROFILE
        } else {
            0
        };
        if profile.friction() == NativeFrictionModel::PerPoint {
            profile_flag |= PER_POINT_PROFILE;
        }
        let terrain_flag = profile_flag
            | if self
                .partition_plane()?
                .is_some_and(|p| p.normal != [0., 0., 1.])
            {
                ORIENTED_GROUND
            } else {
                0
            };
        let cylinder = self
            .bodies
            .values()
            .any(|b| b.1.collider.shape().as_cylinder().is_some());
        if cylinder
            || self
                .bodies
                .values()
                .any(|b| b.1.collider.shape().as_capsule().is_some())
        {
            let format = if cylinder {
                CYLINDER_FORMAT
            } else {
                CAPSULE_FORMAT
            };
            let rows: Vec<_> = records
                .into_iter()
                .zip(self.bodies.values())
                .map(|((kind, record), b)| {
                    let (geometry, _) = primitive_dimensions(&b.1.collider).ok_or_else(invalid)?;
                    Ok((kind, b.1.collider.friction(), geometry, record))
                })
                .collect::<Result<_, PhysicsError>>()?;
            return codec()
                .serialize(&(format | terrain_flag, self.next_id, open, rows, &self.world))
                .map_err(|_| invalid());
        }
        if self.bodies.values().any(|b|b.1.collider.shape().as_ball().is_some()){
            let rows:Vec<_>=records.into_iter().zip(self.bodies.values())
                .map(|((kind,record),b)|(kind,b.1.collider.friction(),b.1.collider.shape().as_ball().is_some(),record)).collect();
            return codec().serialize(&(PRIMITIVE_FORMAT | terrain_flag,self.next_id,open,rows,&self.world)).map_err(|_|invalid());
        }
        if self.bodies.values().any(|b|b.1.collider.friction().to_bits()!=0) || self.partition_plane()?.is_some_and(|g|g.friction.to_bits()!=0) {
            let material:Vec<_>=records.into_iter().zip(self.bodies.values())
                .map(|((kind,record),entry)|(kind,entry.1.collider.friction(),record)).collect();
            codec().serialize(&(MATERIAL_FORMAT | terrain_flag,self.next_id,open,material,&self.world)).map_err(|_|invalid())
        } else if self.partition_plane()?.is_some() {
            codec()
                .serialize(&(GROUND_FORMAT | terrain_flag, self.next_id, open, records, &self.world))
                .map_err(|_| invalid())
        } else if records.iter().any(|(kind, _)| *kind == BoxKind::Fixed) {
            codec()
                .serialize(&(
                    FIXED_FORMAT | profile_flag,
                    self.next_id,
                    open,
                    records,
                    &self.world,
                ))
                .map_err(|_| invalid())
        } else if records.len() >= 2 && records.iter().all(|(kind, _)| *kind == BoxKind::Dynamic) {
            let physical: Vec<_> = records.into_iter().map(|(_, r)| r).collect();
            codec()
                .serialize(&(
                    FORMAT | profile_flag,
                    self.next_id,
                    open,
                    physical,
                    &self.world,
                ))
                .map_err(|_| invalid())
        } else {
            let legacy: Vec<_> = records
                .into_iter()
                .map(|(kind, r)| (kind == BoxKind::Prescribed, r))
                .collect();
            codec()
                .serialize(&(
                    SOLVER_FORMAT | profile_flag,
                    self.next_id,
                    open,
                    legacy,
                    &self.world,
                ))
                .map_err(|_| invalid())
        }
    }

    /// Only call with a complete, integrity-verified archive from the trusted source
    /// partition. Rapier's native serde models are version-specific trusted data.
    pub fn restore_trusted_boxes(bytes: &[u8]) -> Result<Self, PhysicsError> {
        let header: [u8; 4] = bytes.get(..4).and_then(|b| b.try_into().ok()).ok_or_else(invalid)?;
        if !matches!(u32::from_le_bytes(header) & !ORIENTED_GROUND, FORMAT | GROUND_FORMAT | MATERIAL_FORMAT) {
            return Err(invalid());
        }
        let solver = Self::restore_trusted_solver(bytes)?;
        if solver.bodies.len() < 2
            || solver
                .bodies
                .values()
                .any(|b| b.1.kinematic || b.1.mass.is_none())
        {
            return Err(invalid());
        }
        Ok(solver)
    }

    /// Hydrate a complete integrity-verified same-build worker solver.
    pub fn restore_trusted_solver(bytes: &[u8]) -> Result<Self, PhysicsError> {
        Self::restore_solver_with_profile(bytes, NativeSolverProfile::Standard32)
    }

    pub fn restore_solver_with_profile(
        bytes: &[u8],
        profile: NativeSolverProfile,
    ) -> Result<Self, PhysicsError> {
        if bytes.len() > PARTITION_CHECKPOINT_LIMIT {
            return Err(invalid());
        }
        // bincode 1.3.3's slice deserializer deliberately replaces its byte limit
        // with Infinite. The streaming reader honors the configured limit.
        let mut remaining = bytes;
        // The fixed-width bincode prefix ends in the record Vec's u64 length. Read
        // it before allocating records or invoking the native world's deserializer.
        let (format, next, open, count): (u32, u64, bool, u64) = codec()
            .deserialize_from(&mut remaining)
            .map_err(|_| invalid())?;
        let oriented = format & ORIENTED_GROUND != 0;
        let extended = format & EXTENDED_PROFILE != 0;
        if extended != (profile.capacity_profile() == NativeSolverProfile::Extended64)
            || (format & PER_POINT_PROFILE != 0) != (profile.friction() == NativeFrictionModel::PerPoint)
        {
            return Err(invalid());
        }
        let format = format & !(ORIENTED_GROUND | EXTENDED_PROFILE | PER_POINT_PROFILE);
        let minimum = match format {
            FORMAT => 2,
            SOLVER_FORMAT | FIXED_FORMAT | GROUND_FORMAT | MATERIAL_FORMAT | PRIMITIVE_FORMAT | CAPSULE_FORMAT | CYLINDER_FORMAT => 1,
            _ => return Err(invalid()),
        };
        if !(minimum..=profile.max_bodies() as u64).contains(&count) {
            return Err(invalid());
        }
        let records: Vec<(BoxKind, f64, PrimitiveKind, BoxRecord)> = (0..count)
            .map(|_| {
                let kind = match format {
                    FIXED_FORMAT | GROUND_FORMAT | MATERIAL_FORMAT | PRIMITIVE_FORMAT | CAPSULE_FORMAT | CYLINDER_FORMAT => codec()
                        .deserialize_from(&mut remaining)
                        .map_err(|_| invalid())?,
                    SOLVER_FORMAT
                        if codec()
                            .deserialize_from(&mut remaining)
                            .map_err(|_| invalid())? =>
                    {
                        BoxKind::Prescribed
                    }
                    _ => BoxKind::Dynamic,
                };
                let friction=if matches!(format,MATERIAL_FORMAT|PRIMITIVE_FORMAT|CAPSULE_FORMAT|CYLINDER_FORMAT) { codec().deserialize_from(&mut remaining).map_err(|_| invalid())? }else{0.};
                let geometry=match format {
                    CAPSULE_FORMAT|CYLINDER_FORMAT=>codec().deserialize_from(&mut remaining).map_err(|_|invalid())?,
                    PRIMITIVE_FORMAT if codec().deserialize_from(&mut remaining).map_err(|_|invalid())?=>PrimitiveKind::Sphere,
                    _=>PrimitiveKind::Box,
                };
                let record = codec()
                    .deserialize_from(&mut remaining)
                    .map_err(|_| invalid())?;
                Ok::<_, PhysicsError>((kind, friction, geometry, record))
            })
            .collect::<Result<_, _>>()?;
        let handles: BTreeSet<_> = records
            .iter()
            .map(|(_, _, _, r)| r.handle.into_raw_parts())
            .collect();
        if handles.len() != records.len()
            || records.windows(2).any(|p| p[0].3.id >= p[1].3.id)
            || records.last().unwrap().3.id >= next
        {
            return Err(invalid());
        }
        let world: PhysicsWorld = codec()
            .deserialize_from(&mut remaining)
            .map_err(|_| invalid())?;
        if !remaining.is_empty() {
            return Err(invalid());
        }
        validate_world(&world, records.len(), profile)?;
        let ground = partition_ground::ground_of(&world)?;
        if oriented != ground.is_some_and(|(_, p)| p.normal != [0., 0., 1.]) {
            return Err(invalid());
        }
        let ground_material=ground.is_some_and(|(c,_)|world.colliders[c].friction().to_bits()!=0);
        let has_material=ground_material || records.iter().any(|(_,f,_,_)|f.to_bits()!=0);
        if matches!(format,PRIMITIVE_FORMAT|CAPSULE_FORMAT|CYLINDER_FORMAT) {
            if format!=CYLINDER_FORMAT && records.iter().any(|r|r.2==PrimitiveKind::Cylinder){return Err(invalid());}
            let required=match format {CYLINDER_FORMAT=>PrimitiveKind::Cylinder,CAPSULE_FORMAT=>PrimitiveKind::Capsule,_=>PrimitiveKind::Sphere};
            if !records.iter().any(|r|r.2==required){return Err(invalid());}
        }else if (format==MATERIAL_FORMAT)!=has_material || (format!=MATERIAL_FORMAT && (format==GROUND_FORMAT)!=ground.is_some()){return Err(invalid());}
        let mut result = Self::new_multifidelity_with_profile([0.; 3], profile)?;
        let mut entries = BTreeMap::new();
        for (kind, friction, geometry,b) in records {
            let id = BodyId::from_raw(b.id);
            let handle = b.handle;
            let shape=geometry.shape(b.half_extents)?;
            let seed = result.spawn_partition_primitive(
                BodySpec {
                    shape,
                    mass: (kind != BoxKind::Fixed).then_some(b.mass),
                    position: b.position,
                    velocity: b.velocity,
                },
                RotationSpec {
                    orientation: b.orientation,
                    angular_velocity: b.angular_velocity,
                },
                kind == BoxKind::Prescribed,friction,
            )?;
            let mut entry = result.bodies.remove(&seed).unwrap();
            // Construction poses are relative to different insertion times. Validate
            // each seed in isolation; the archived current world owns their true poses.
            result.world.remove_body(entry.0);
            b.validate(&world, &entry.1.collider, kind)?;
            entry.0 = handle;
            entries.insert(id, entry);
            if b.has_wrench {
                result.pending_wrenches.insert(id);
            }
        }
        // Restore all solver graphs, manifolds, cached impulse accumulators, island and broad
        // phase state together. Rapier rebuilds only its annotated scratch pipeline workspaces.
        result.world = world;
        result.bodies = entries;
        result.next_id = next;
        result.refresh_max_boundary_spin();
        if !open {
            result.joints.close();
        }
        Ok(result)
    }
}

#[cfg(test)]
#[path = "../tests/private/native_checkpoint.rs"]
mod tests;
