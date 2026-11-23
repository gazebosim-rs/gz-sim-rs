// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate the contact geometry contribution to finite-sphere rotational covariance.
#[path = "support/rough_contact_native.rs"]
mod native;
use clap::Parser;
use gz_physics_distributed::physics::{Body, ContactImpulse, GroundSupport, Primitive, Tier};
use rapier3d_f64::prelude::{Rotation, Vector};
use serde::Serialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const G: f64 = 9.81;
const RADIUS: f64 = 0.3;
const I: f64 = 0.4 * 2. * RADIUS * RADIUS;
#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t = 2)]
    step_ms: u64,
    /// An actual oblique-rest SDF report, to compare its exact parsed quaternions to direct trig.
    #[arg(long)]
    sdf_seeds: Option<PathBuf>,
    /// Diagnostic native velocity iterations per solver step; production uses 4.
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(1..=16))]
    solver_iterations: u8,
    out: PathBuf,
}
#[derive(Clone, Copy)]
struct Config {
    slab: f64,
    sphere: f64,
    drive: f64,
    width: f64,
    coast: bool,
}
#[derive(Clone, Serialize)]
struct Sample {
    position: [f64; 3],
    linear: [f64; 3],
    angular: [f64; 3],
    relative_rot: [f64; 4],
    contact: bool,
    normal: [f64; 3],
    tangent: [f64; 3],
    arm: [f64; 3],
    torque: [f64; 3],
    angular_balance: [f64; 3],
    lever_reconstruction_delta_m: f64,
    geometric_distance_m: Option<f64>,
    final_warmstart_normal_n_s: Option<f64>,
}
fn vec_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (Vector::from(a) - Vector::from(b)).length()
}
fn chord(a: [f64; 4], b: [f64; 4]) -> f64 {
    [1., -1.]
        .into_iter()
        .map(|s| {
            a.into_iter()
                .zip(b)
                .map(|(u, v)| (u - s * v).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .fold(f64::INFINITY, f64::min)
}
fn trajectory(
    c: Config,
    h: u64,
    loaded: Option<[f64; 4]>,
    iterations: u8,
) -> Result<(Vec<Sample>, String)> {
    let expected_basis = Rotation::from_rotation_z(c.drive);
    let orientation = |angle: f64| {
        if angle == 0. {
            Rotation::IDENTITY
        } else if let Some(q) = loaded {
            Rotation::from_xyzw(q[0], q[1], q[2], q[3])
        } else {
            Rotation::from_rotation_z(angle)
        }
    };
    let sphere_basis = orientation(c.sphere);
    let dir = expected_basis * Vector::X;
    let slab = Body {
        id: 1,
        primitive: Primitive::Box,
        tier: Tier::Prescribed,
        size: [10., c.width, 0.4],
        mass: 10.,
        friction: 0.65,
        position: [0., 0., -0.2],
        orientation: orientation(c.slab).to_array(),
        linear: [0.; 3],
        angular: [0.; 3],
    };
    let sphere = Body {
        id: 2,
        primitive: Primitive::Sphere,
        tier: Tier::Dynamic,
        size: [0.6; 3],
        mass: 2.,
        friction: 0.35,
        position: [0., 0., if c.coast { 100. } else { 0.35 }],
        orientation: sphere_basis.to_array(),
        linear: if c.coast {
            (2.5 * dir).to_array()
        } else {
            [0.; 3]
        },
        angular: if c.coast {
            (expected_basis * Vector::Y * (2.5 / RADIUS)).to_array()
        } else {
            [0.; 3]
        },
    };
    let mut bodies = [slab, sphere];
    let mut before = Vector::from(bodies[1].angular);
    let mut world = native::Native::new(&bodies, h as f64 / 1000., true);
    world.world.integration_parameters.num_solver_iterations = usize::from(iterations);
    let inv = expected_basis.conjugate();
    let mut out = vec![];
    let mut hash = Sha256::new();
    for tick in 1..=1200 / h {
        if tick * h == 200 + h && !c.coast {
            world.kick((7. * dir).to_array());
        }
        let r = world.step(&mut bodies);
        let pairs = r.active.then_some(ContactImpulse {
            a: 1,
            b: 2,
            normal_impulse: r.normal_n_s,
        });
        let floors = Vec::<GroundSupport>::new();
        hash.update(bincode::serialize(&(
            tick,
            &bodies,
            pairs.as_slice(),
            floors,
        ))?);
        assert!(r.points.len() <= 1);
        let reconstructed = r
            .points
            .first()
            .map(|p| Vector::from(p.pre_step_lever_arm_m))
            .unwrap_or_default();
        let arm = r
            .points
            .first()
            .map(|p| Vector::from(p.native_solver_lever_arm_m))
            .unwrap_or_default();
        let n = Vector::from(r.normal_on_sphere);
        let t = Vector::from(r.tangent_on_sphere);
        let torque = arm.cross(n + t);
        let b = &bodies[1];
        let w = Vector::from(b.angular);
        let dq = Rotation::from_xyzw(
            b.orientation[0],
            b.orientation[1],
            b.orientation[2],
            b.orientation[3],
        ) * sphere_basis.conjugate();
        out.push(Sample {
            position: (inv * Vector::from(b.position)).to_array(),
            linear: (inv * Vector::from(b.linear)).to_array(),
            angular: (inv * w).to_array(),
            relative_rot: (inv * dq * expected_basis).to_array(),
            contact: r.active,
            normal: (inv * n).to_array(),
            tangent: (inv * t).to_array(),
            arm: (inv * arm).to_array(),
            torque: (inv * torque).to_array(),
            angular_balance: (inv * (I * (w - before) - torque)).to_array(),
            lever_reconstruction_delta_m: (arm - reconstructed).length(),
            geometric_distance_m: r.points.first().map(|p| p.geometric_distance_m),
            final_warmstart_normal_n_s: r.points.first().map(|p| p.final_warmstart_normal_n_s),
        });
        before = w;
    }
    Ok((out, format!("{:x}", hash.finalize())))
}
#[derive(Default, Serialize)]
struct Difference {
    peak: f64,
    peak_tick: u64,
    first_over_1e_10: Option<u64>,
}
impl Difference {
    fn add(&mut self, v: f64, tick: u64) {
        if v > self.peak {
            self.peak = v;
            self.peak_tick = tick;
        }
        if v > 1e-10 {
            self.first_over_1e_10.get_or_insert(tick);
        }
    }
}
#[derive(Default, Serialize)]
struct Comparison {
    position_m: Difference,
    velocity_m_s: Difference,
    angular_rad_s: Difference,
    quaternion_chord: Difference,
    force_arm_m: Difference,
    normal_n_s: Difference,
    tangent_n_s: Difference,
    torque_n_m_s: Difference,
    angular_balance_n_m_s: Difference,
    solver_vs_reconstructed_arm_m: Difference,
    angular_difference_after_integrating_native_torques_n_m_s: Difference,
    cumulative_torque_from_lever_change: [f64; 3],
    cumulative_torque_from_impulse_change: [f64; 3],
    first_changed_arm: Option<Value>,
    first_changed_angular_velocity: Option<Value>,
    first_changed_tangent: Option<Value>,
    different_contact_ticks: usize,
}
fn compare(a: &[Sample], b: &[Sample]) -> Comparison {
    let mut c = Comparison::default();
    let mut d_arm = Vector::ZERO;
    let mut d_j = Vector::ZERO;
    for (i, (a, b)) in a.iter().zip(b).enumerate() {
        let tick = i as u64 + 1;
        c.position_m.add(vec_distance(a.position, b.position), tick);
        c.velocity_m_s.add(vec_distance(a.linear, b.linear), tick);
        c.angular_rad_s
            .add(vec_distance(a.angular, b.angular), tick);
        c.quaternion_chord
            .add(chord(a.relative_rot, b.relative_rot), tick);
        c.force_arm_m.add(vec_distance(a.arm, b.arm), tick);
        c.normal_n_s.add(vec_distance(a.normal, b.normal), tick);
        c.tangent_n_s.add(vec_distance(a.tangent, b.tangent), tick);
        c.torque_n_m_s.add(vec_distance(a.torque, b.torque), tick);
        c.angular_balance_n_m_s
            .add(Vector::from(b.angular_balance).length(), tick);
        c.solver_vs_reconstructed_arm_m
            .add(b.lever_reconstruction_delta_m, tick);
        c.different_contact_ticks += usize::from(a.contact != b.contact);
        let ja = Vector::from(a.normal) + Vector::from(a.tangent);
        let jb = Vector::from(b.normal) + Vector::from(b.tangent);
        let ra = Vector::from(a.arm);
        let rb = Vector::from(b.arm);
        // rb×jb − ra×ja = (rb−ra)×ja + rb×(jb−ja).
        d_arm += (rb - ra).cross(ja);
        d_j += rb.cross(jb - ja);
        let actual = I * (Vector::from(b.angular) - Vector::from(a.angular));
        c.angular_difference_after_integrating_native_torques_n_m_s
            .add((actual - d_arm - d_j).length(), tick);
        for (trigger, witness) in [
            (c.force_arm_m.first_over_1e_10, &mut c.first_changed_arm),
            (
                c.angular_rad_s.first_over_1e_10,
                &mut c.first_changed_angular_velocity,
            ),
            (c.tangent_n_s.first_over_1e_10, &mut c.first_changed_tangent),
        ] {
            if trigger == Some(tick) {
                *witness = Some(json!({"tick":tick,"reference":a,"case":b}));
            }
        }
    }
    c.cumulative_torque_from_lever_change = d_arm.to_array();
    c.cumulative_torque_from_impulse_change = d_j.to_array();
    c
}
fn main() -> Result<()> {
    let opts = Args::parse();
    if !matches!(opts.step_ms, 1 | 2 | 5 | 10) {
        return Err("choose 1, 2, 5 or 10 ms".into());
    }
    let h = opts.step_ms;
    let theta = 3f64.atan2(4.);
    let mut rows = vec![];
    let sdf_report = opts
        .sdf_seeds
        .map(fs::read)
        .transpose()?
        .as_deref()
        .map(serde_json::from_slice::<Value>)
        .transpose()?;
    let mut data = vec![("native_trigonometric", None)];
    if let Some(report) = &sdf_report {
        let q: [f64; 4] =
            serde_json::from_value(report["admitted_native_seeds"][0]["orientation"].clone())?;
        if report["admitted_native_seeds"][1]["orientation"]
            != report["admitted_native_seeds"][0]["orientation"]
        {
            return Err("different table and sphere seed yaw".into());
        }
        data.push(("SDF_loaded", Some(q)));
    }
    for (source, loaded) in data {
        for coast in [false, true] {
            let base = Config {
                slab: 0.,
                sphere: 0.,
                drive: 0.,
                width: 10.,
                coast,
            };
            let (reference, _) = trajectory(base, h, loaded, opts.solver_iterations)?;
            // A square avoids crossing a narrow side when rotating the drive alone.
            // The original 2 m-wide rectangle supplies additional end-to-end controls.
            for width in [2., 10.] {
                for slab in [0., theta] {
                    for sphere in [0., theta] {
                        for drive in [0., theta] {
                            if width == 2. && slab != drive {
                                continue;
                            }
                            let case = Config {
                                width,
                                slab,
                                sphere,
                                drive,
                                coast,
                            };
                            let (samples, sha) =
                                trajectory(case, h, loaded, opts.solver_iterations)?;
                            if opts.solver_iterations == 4
                                && source == "SDF_loaded"
                                && !coast
                                && width == 2.
                                && slab == theta
                                && sphere == theta
                                && drive == theta
                                && sdf_report.as_ref().unwrap()["step_ms"] == h
                                && sdf_report.as_ref().unwrap()["trajectory_sha256"] != sha
                            {
                                return Err(format!("directly reconstructed physical trajectory {sha} differs from the actual SDF run").into());
                            }
                            let c = compare(&reference, &samples);
                            let witnesses = if source == "SDF_loaded"
                                && h == 2
                                && !coast
                                && width == 2.
                                && slab == theta
                                && sphere == theta
                                && drive == theta
                            {
                                [40u64,49,50,51,52,53,100,101,178,245,246].into_iter().map(|tick|json!({"tick":tick,"axis":reference[tick as usize-1],"oblique":samples[tick as usize-1]})).collect::<Vec<_>>()
                            } else {
                                vec![]
                            };
                            rows.push(json!({"quaternion_source":source,"loaded_quaternion":loaded,"trajectory_sha256":sha,"event_samples":witnesses,"coast":coast,"width_m":width,"slab_yaw":slab,"sphere_yaw":sphere,"drive_yaw":drive,"difference":c}));
                        }
                    }
                }
            }
        }
    }
    fs::write(
        opts.out,
        serde_json::to_vec_pretty(
            &json!({"step_ms":h,"solver_iterations":opts.solver_iterations,"ticks":1200/h,"comparison":"uninterrupted native point-Coulomb vs stationary square with identical load schedule","cases":rows}),
        )?,
    )?;
    Ok(())
}
