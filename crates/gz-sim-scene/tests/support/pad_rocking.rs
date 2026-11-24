// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{MASS, Rig, named};
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{MultiFidelityBackend, RigidContactPatch};
use gz_sim_core::EntityId;

const WORLD: &str = include_str!("../../../../examples/worlds/local_rocking_pad_grasp.sdf");
const INERTIA: DVec3 = DVec3::new(0.00001626666666666667, 0.0000246, 0.00002166666666666667);
const PRELOAD_US: u32 = 200_000;
const PULSE_US: u32 = 200;

#[derive(Clone, Copy, Default, Debug)]
pub(super) struct Patch {
    pub(super) selected: usize,
    pub(super) loaded: usize,
    pub(super) linear: DVec3,
    pub(super) angular: DVec3,
    pub(super) normal: f64,
}

pub(super) fn pad_impulses(rig: &Rig) -> [Patch; 2] {
    let engine = rig
        .sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    sum_pad_impulses(rig.parcel, rig.pads, engine.rigid_contact_patches())
}

pub(super) fn sum_pad_impulses(
    parcel: BodyId,
    pads: [BodyId; 2],
    patches: Vec<RigidContactPatch>,
) -> [Patch; 2] {
    let mut result = [Patch::default(); 2];
    for patch in patches {
        let parcel_a = patch.a == parcel;
        assert!(parcel_a || patch.b == parcel);
        let pad = if parcel_a { patch.b } else { patch.a };
        let side = pads.iter().position(|id| *id == pad).unwrap();
        let row = &mut result[side];
        let sign = if parcel_a { 1. } else { -1. };
        for point in patch.points {
            let impulse = sign
                * (DVec3::from(patch.normal_on_a) * point.normal_impulse
                    + DVec3::from(point.tangent_impulse_on_a.unwrap()));
            let lever = DVec3::from(if parcel_a {
                point.lever_a
            } else {
                point.lever_b
            });
            assert!(point.normal_impulse >= 0.0);
            row.normal += point.normal_impulse;
            row.linear += impulse;
            row.angular += lever.cross(impulse);
            row.selected += 1;
            row.loaded += usize::from(point.normal_impulse > 0.);
        }
    }
    result
}

fn angular_momentum(body: &BodyState) -> DVec3 {
    let orientation = DQuat::from_array(body.orientation);
    orientation * (INERTIA * (orientation.inverse() * DVec3::from(body.angular_velocity)))
}

#[derive(Clone, Copy, Default)]
struct Window {
    normal: [f64; 2],
    torque: [f64; 2],
    pitch: f64,
}

fn returning_edge(rig: &Rig, state: &BodyState) -> (f64, f64) {
    // Track the midpoint between the two positive-Z corners as they return to the left pad.
    let pad = rig
        .sim
        .world()
        .resource::<PhysicsBackend>()
        .state(rig.pads[0])
        .unwrap();
    let arm = DQuat::from_array(state.orientation) * DVec3::new(-0.015, 0., 0.012);
    let n = DQuat::from_array(pad.orientation) * DVec3::X;
    let offset = DVec3::from(state.position) + arm - DVec3::from(pad.position);
    let velocity = DVec3::from(state.velocity) + DVec3::from(state.angular_velocity).cross(arm)
        - DVec3::from(pad.velocity)
        - DVec3::from(pad.angular_velocity).cross(offset);
    (n.dot(offset) - 0.010, n.dot(velocity))
}

/// At first touch, compression of the two returning corners is zero. Their
/// initial reaction is viscous, computed independently from the fixture's masses
/// and moments of inertia. The other two points already carry their incident load.
fn returning_edge_damping(rig: &Rig, block: &BodyState) -> f64 {
    let engine = rig.sim.world().resource::<PhysicsBackend>();
    let pads = rig.pads.map(|id| engine.state(id).unwrap());
    let hinge = DVec3::new(0.0, 0.0, 0.38);
    let pad_mass = 0.08;
    // The wrist and both centered sliders rotate about the same vertical pin.
    // At this configuration the scalar slides are orthogonal to the yaw motion.
    let mut yaw_inertia = 0.000283333333333333;
    for pad in &pads {
        let offset = DVec3::from(pad.position) - hinge;
        yaw_inertia += 8.66666666666667e-6 + pad_mass * (offset.x.powi(2) + offset.y.powi(2));
    }
    let n = DQuat::from_array(pads[0].orientation) * DVec3::X;
    let rotation = DQuat::from_array(block.orientation);
    let mut load = 0.0;
    for y in [-0.010, 0.010] {
        let corner = DVec3::new(-0.015, y, 0.012);
        let arm = rotation * corner;
        let center = DVec3::from(block.position) + arm;
        let yaw_arm = (center - hinge).cross(n).z;
        let spin_arm = corner.cross(rotation.inverse() * n);
        let inv_mass = 1.0 / MASS
            + 1.0 / pad_mass
            + spin_arm.dot(spin_arm / INERTIA)
            + yaw_arm.powi(2) / yaw_inertia;
        let block_velocity =
            DVec3::from(block.velocity) + DVec3::from(block.angular_velocity).cross(arm);
        let pad_velocity = DVec3::from(pads[0].velocity)
            + DVec3::from(pads[0].angular_velocity).cross(center - DVec3::from(pads[0].position));
        let closing = n.dot(pad_velocity - block_velocity);
        // One quarter of the 30 kN/m face's damper at the material's ζ = 10.
        load += 0.25 * 20.0 * (30_000.0 / inv_mass).sqrt() * closing;
    }
    load
}

#[derive(Default)]
struct Trial {
    windows: Vec<Window>,
    impact_normal: [f64; 2],
    impact_moment: [f64; 2],
}

fn rock(step_us: u32) -> Trial {
    let mut rig = Rig::new(WORLD, step_us, 0.);
    rig.drive([3.; 2]);
    rig.sim.step_exact(u64::from(PRELOAD_US / step_us)).unwrap();
    let body = named(&rig.sim, "default::parcel::body");
    let id = rig.sim.world().get::<EntityId>(body).unwrap().get();
    let dt = rig.sim.step_duration().as_secs_f64();
    let mut previous = rig.state();
    let mut last = pad_impulses(&rig);
    assert!(
        last.iter()
            .all(|p| p.loaded == 4 && (p.normal / dt - 3.).abs() < 0.005)
    );
    let mut trial = Trial::default();
    let mut current = Window::default();
    let mut residual = [0.0f64; 2];
    let mut edge = [false; 2];
    let mut angle_peak = 0.0f64;
    let mut last_gap = returning_edge(&rig, &previous);
    let mut strike_peak = [0.0_f64; 2];
    let mut incident_peak = None;
    for tick in 1..=250_000 / step_us {
        let us = tick * step_us;
        let external = if us <= PULSE_US { 6. } else { 0. };
        if external != 0. {
            gz_sim_scene::queue_link_world_wrench(&mut rig.sim, id, [0.; 3], [0., external, 0.])
                .unwrap();
        }
        let near_return = us > 38_000 && us <= 40_000;
        let incoming = if near_return && incident_peak.is_none() {
            returning_edge_damping(&rig, &previous) + last[0].normal / dt
        } else {
            0.0
        };
        rig.sim.step_exact(1).unwrap();
        let state = rig.state();
        let reaction = pad_impulses(&rig);
        let gap = returning_edge(&rig, &state);
        // Compare both jaws over identical physical windows. A coarse public
        // step averages the short dashpot impulse as the far edge lands.
        if near_return {
            for (side, patch) in reaction.iter().enumerate() {
                trial.impact_normal[side] += patch.normal;
                trial.impact_moment[side] += patch.angular.y;
                strike_peak[side] = strike_peak[side].max(patch.normal / dt);
            }
            if reaction[0].loaded > 2 && last[0].loaded == 2 && incident_peak.is_none() {
                println!(
                    "rock_impact step_us={step_us} us={us} before_gap_m={} before_closing_m_s={} after_gap_m={}",
                    last_gap.0, -last_gap.1, gap.0
                );
                assert!(last_gap.0 >= -1e-6 && gap.0 <= 1e-6);
                assert!((0.55..0.65).contains(&-last_gap.1));
                incident_peak = Some(incoming);
            }
        }
        last_gap = gap;
        let parcel_impulse = reaction[0].linear + reaction[1].linear;
        let dv = DVec3::from(state.velocity) - DVec3::from(previous.velocity);
        residual[0] = residual[0].max((MASS * dv - parcel_impulse).length());
        let external_impulse = DVec3::Y * (external * dt);
        let angular = reaction[0].angular + reaction[1].angular;
        residual[1] = residual[1].max(
            (angular_momentum(&state) - angular_momentum(&previous) - external_impulse - angular)
                .length(),
        );
        let axis = DQuat::from_array(state.orientation) * DVec3::X;
        let pitch = (-axis.z).atan2(axis.x);
        angle_peak = angle_peak.max(pitch);
        for side in 0..2 {
            let row = reaction[side];
            current.normal[side] += row.normal / 0.001;
            current.torque[side] += row.angular.y / 0.001;
            edge[side] |= row.loaded == 2 && pitch > 0.1;
            if (row.loaded != last[side].loaded || row.selected != last[side].selected) && side == 0
            {
                println!(
                    "rock_edges step_us={step_us} elapsed_us={us} pitch_rad={pitch:.6} rows={}->{} load={}->{} force_before_after_n={:.8},{:.8} torque_before_after_nm={:.8},{:.8}",
                    last[side].selected,
                    row.selected,
                    last[side].loaded,
                    row.loaded,
                    last[side].normal / dt,
                    row.normal / dt,
                    last[side].angular.y / dt,
                    row.angular.y / dt
                );
            }
        }
        if us.is_multiple_of(1000) {
            current.pitch = pitch;
            trial.windows.push(current);
            current = Window::default();
        }
        last = reaction;
        previous = state;
    }
    let end_pitch = trial.windows.last().unwrap().pitch;
    println!(
        "rock_summary step_us={step_us} peak_deg={:.5} end_rad={end_pitch:.8} residual_ns={:e} residual_nms={:e}",
        angle_peak.to_degrees(),
        residual[0],
        residual[1]
    );
    println!(
        "rock_strike step_us={step_us} peak_n={strike_peak:?} impulse_ns={:?} moment_nms={:?}",
        trial.impact_normal, trial.impact_moment
    );
    assert_eq!(edge, [true; 2]);
    let expected = incident_peak.unwrap();
    println!(
        "rock_dashpot step_us={step_us} predicted_instant_n={expected} observed_tick_peak_n={}",
        strike_peak[0]
    );
    assert!((185.0..205.0).contains(&expected));
    if step_us == 1 {
        assert!((strike_peak[0] - expected).abs() < 0.015 * expected);
    }
    assert!((13.0..16.0).contains(&angle_peak.to_degrees()) && end_pitch.abs() < 5e-5);
    assert!(
        last.iter()
            .all(|p| p.loaded == 4 && (p.normal / dt - 3.).abs() < 0.005)
    );
    assert!(residual[0] < 1e-13 && residual[1] < 5e-9);
    trial
}

#[test]
fn block_rocks_from_face_to_edge_and_returns_under_load() {
    let reference = rock(1);
    // Bound the coarsest tick and require every subsequent step reduction
    // to decrease all three errors over matching 1 ms observation windows.
    let mut limits = [0.25_f64.to_radians(), 1.0, 0.020];
    for step_us in [200, 100, 50, 25, 10, 5, 2] {
        let observed = rock(step_us);
        let mut peak_error = [0.0f64; 3];
        for (a, b) in observed.windows.iter().zip(&reference.windows) {
            peak_error[0] = peak_error[0].max((a.pitch - b.pitch).abs());
            for side in 0..2 {
                peak_error[1] = peak_error[1].max((a.normal[side] - b.normal[side]).abs());
                peak_error[2] = peak_error[2].max((a.torque[side] - b.torque[side]).abs());
            }
        }
        // The impact can fall into different public ticks. Compare the total
        // linear and angular impulses of each jaw over the same two milliseconds.
        for side in 0..2 {
            let normal_error =
                (observed.impact_normal[side] / reference.impact_normal[side] - 1.0).abs();
            let moment_error =
                (observed.impact_moment[side] / reference.impact_moment[side] - 1.0).abs();
            println!(
                "rock_window_error step_us={step_us} side={side} normal_ratio_error={normal_error} moment_ratio_error={moment_error}"
            );
            assert!(normal_error < 0.01 && moment_error < 0.01);
        }
        assert!(
            peak_error
                .into_iter()
                .zip(limits)
                .all(|(error, limit)| error < limit),
            "step {step_us} µs: {peak_error:?}, bounds {limits:?}"
        );
        limits = peak_error;
        println!(
            "rock_fine step_us={step_us} max_angle_rad={} max_1ms_force_n={} max_1ms_torque_nm={}",
            peak_error[0], peak_error[1], peak_error[2]
        );
    }
}
