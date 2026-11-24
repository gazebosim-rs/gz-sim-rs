// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent SDF, with separately identified physical controls derived from it.
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointSample, Name, PhysicsEngine, load_world_with_sdf_step, reset_loaded_world,
    set_joint_force,
};

const UPSTREAM: &str = include_str!("../../../examples/worlds/upstream_joint_force.sdf");
const TORQUE: f64 = 0.012;
const ROTOR_INERTIA: f64 = 0.00012;
const BASE_INERTIA: f64 = 5.;

struct Rig {
    sim: Simulation,
    joint: u64,
    base: BodyId,
    rotor: BodyId,
    floor: BodyId,
}

impl Rig {
    fn load(xml: &str) -> Self {
        let mut sim = load_world_with_sdf_step(
            xml,
            "upstream_joint_force.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let world = sim.world_mut();
        let mut entities = world.query::<(&Name, &EntityId, Option<&BodyHandle>)>();
        let mut named = |name: &str| {
            let (_, id, body) = entities.iter(world).find(|(n, _, _)| n.0 == name).unwrap();
            (id.get(), body.map(BodyHandle::id))
        };
        let (joint, _) = named("default::joint_force_test::j1");
        let (_, base) = named("default::joint_force_test::base_link");
        let (_, rotor) = named("default::joint_force_test::rotor");
        let (_, floor) = named("default::ground_plane::link");
        Self {
            sim,
            joint,
            base: base.unwrap(),
            rotor: rotor.unwrap(),
            floor: floor.unwrap(),
        }
    }

    fn command(&mut self, torque: f64) {
        set_joint_force(&mut self.sim, self.joint, torque).unwrap();
    }
    fn step(&mut self, ticks: u64) {
        self.sim.step_exact(ticks).unwrap();
    }
    fn joint(&self) -> (f64, f64) {
        let entity = entity_by_id(self.sim.world(), self.joint).unwrap();
        let state = self.sim.world().get::<JointSample>(entity).unwrap().state;
        (state.position, state.velocity)
    }
    fn body(&self, body: BodyId) -> BodyState {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .state(body)
            .unwrap()
    }
    fn support(&self) -> f64 {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| {
                (p.a == self.floor && p.b == self.base) || (p.b == self.floor && p.a == self.base)
            })
            .map(|p| p.normal_impulse)
            .sum()
    }
}

#[test]
fn upstream_rotor_accelerates_coasts_and_reverses_on_a_weight_bearing_base() {
    let mut rig = Rig::load(UPSTREAM);
    let mut replay = None;
    for _ in 0..2 {
        rig.step(50);
        let mut impulse = 0.;
        rig.command(TORQUE);
        for _ in 0..100 {
            rig.step(1);
            impulse += rig.support();
        }
        let driven = rig.joint();
        let expected_speed = TORQUE * 0.1 / ROTOR_INERTIA;
        let expected_impulse = (120. + 0.6) * 9.8 * 0.1;
        eprintln!(
            "grounded hinge {:?}, support impulse {impulse} N s, base z {}",
            driven,
            rig.body(rig.base).position[2]
        );
        assert!((driven.1 - expected_speed).abs() < 0.01);
        assert!((driven.0 - expected_speed * 0.05).abs() < 0.008);
        assert!((impulse - expected_impulse).abs() < expected_impulse * 0.01);
        assert!((rig.body(rig.base).position[2] - 0.005).abs() < 0.002);
        rig.command(0.);
        rig.step(50);
        let coast = rig.joint();
        assert!((coast.1 - driven.1).abs() < 0.01 && coast.0 > driven.0);
        rig.command(-TORQUE);
        rig.step(200);
        let reverse = rig.joint();
        assert!((reverse.1 + expected_speed).abs() < 0.02);
        if let Some(previous) = replay {
            assert_eq!((driven, coast, reverse, impulse), previous);
        }
        replay = Some((driven, coast, reverse, impulse));
        reset_loaded_world(&mut rig.sim, |_| ()).unwrap();
    }
}

#[test]
fn suspended_rotor_and_base_exchange_angular_momentum_about_the_authored_axis() {
    let inclined = UPSTREAM
        .replace(
            "<world name=\"default\">",
            "<world name=\"default\"><gravity>0 0 0</gravity>",
        )
        .replace("0 0 0.005 0 0 0", "0 0 3 0 0.6 0");
    let axis = DVec3::new(0.6_f64.sin(), 0., 0.6_f64.cos());
    for factor in [1., 4.] {
        let xml = inclined.replace(
            "<izz>0.00012</izz>",
            &format!("<izz>{}</izz>", factor * ROTOR_INERTIA),
        );
        let mut rig = Rig::load(&xml);
        rig.command(TORQUE);
        rig.step(100);
        let rotor = DVec3::from(rig.body(rig.rotor).angular_velocity);
        let base = DVec3::from(rig.body(rig.base).angular_velocity);
        let impulse = TORQUE * 0.1;
        let rotor_momentum = rotor * ROTOR_INERTIA * factor;
        let base_momentum = base * BASE_INERTIA;
        eprintln!(
            "suspended {factor}x inertia, rotor {} base {} rad/s, momentum residual {}",
            rotor.dot(axis),
            base.dot(axis),
            (rotor_momentum + base_momentum).length()
        );
        assert!((rotor_momentum - axis * impulse).length() < impulse * 1e-4);
        assert!((base_momentum + axis * impulse).length() < impulse * 1e-4);
        assert_eq!(rig.support(), 0.);
    }
}

#[test]
fn supporting_ground_rejects_lateral_force_below_friction_limit() {
    // A 1 m/s² horizontal gravity component supplies a known force on both links.
    let lateral = UPSTREAM.replace(
        "<world name=\"default\">",
        "<world name=\"default\"><gravity>1 0 -9.8</gravity>",
    );
    let elevated = lateral.replace("0 0 0.005 0 0 0", "0 0 3 0 0 0");
    for (xml, grounded) in [(&lateral, true), (&elevated, false)] {
        let mut rig = Rig::load(xml);
        rig.step(100);
        let a = rig.body(rig.base);
        let b = rig.body(rig.rotor);
        let vx = (120. * a.velocity[0] + 0.6 * b.velocity[0]) / 120.6;
        eprintln!("grounded={grounded} center-of-mass lateral speed {vx} m/s");
        if grounded {
            assert!(vx.abs() < 0.001 && a.position[0].abs() < 0.001);
        } else {
            assert!((vx - 0.1).abs() < 0.0001);
            assert_eq!(rig.support(), 0.);
        }
    }
}

#[test]
fn rotor_with_authored_travel_stops_reaches_both_bounds_under_load() {
    let xml = UPSTREAM.replace(
        "<xyz>0 0 1</xyz>",
        "<xyz>0 0 1</xyz><limit><lower>-0.25</lower><upper>0.25</upper></limit>",
    );
    let mut rig = Rig::load(&xml);
    for sign in [1., -1.] {
        rig.command(sign * TORQUE);
        for _ in 0..250 {
            rig.step(1);
            assert!(rig.joint().0.abs() < 0.26);
        }
        let (angle, speed) = rig.joint();
        eprintln!("travel stop under {sign} torque: {angle} rad, {speed} rad/s");
        assert!((angle - sign * 0.25).abs() < 0.005 && speed.abs() < 0.02);
    }
}

#[test]
fn torque_follows_the_base_as_it_rocks_onto_the_ground() {
    let tipped = UPSTREAM.replace("0 0 0.005 0 0 0", "0 0 0.14 0 0.55 0");
    let mut driven = Rig::load(&tipped);
    let mut coast = Rig::load(&tipped);
    let axis = |rig: &Rig| DQuat::from_array(rig.body(rig.base).orientation) * DVec3::Z;
    let original_axis = axis(&driven);
    driven.step(140);
    coast.step(140);
    let current_axis = axis(&driven);
    driven.command(TORQUE);
    driven.step(1);
    coast.step(1);
    let dw = DVec3::from(driven.body(driven.rotor).angular_velocity)
        - DVec3::from(coast.body(coast.rotor).angular_velocity);
    eprintln!(
        "rocking base moved {} rad, 1 ms torque response {} rad/s, axis agreement {}",
        original_axis.angle_between(current_axis),
        dw.length(),
        current_axis.dot(dw.normalize())
    );
    assert!(original_axis.angle_between(current_axis) > 0.05);
    assert!((dw.length() - TORQUE * 0.001 / ROTOR_INERTIA).abs() < 0.002);
    assert!(current_axis.dot(dw.normalize()) > 0.999);
}
