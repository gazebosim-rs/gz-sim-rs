// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed-system momentum, kinetic energy and joint work in the driven four-body world.
#[path = "support/authored_hinge.rs"]
mod authored_hinge;
#[path = "support/actuated_convoy.rs"]
mod convoy;
#[path = "two_sphere_hinge/reference.rs"]
#[allow(dead_code)]
mod reference;
use authored_hinge::AuthoredHinge;
use clap::Parser;
use glam::{DMat3, DQuat, DVec3};
use gz_physics_distributed::multi_sphere::{MultiSphereObservation, MultiSphereSpec};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use serde::Serialize;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    seed: u64,
    /// Number of complete milliseconds of simulated time.
    #[arg(long)]
    ticks: u64,
    /// Ordinary public solver steps per millisecond (1, 2, 4 or 8).
    #[arg(long, default_value_t = 1)]
    divisor: u64,
    /// Optional single trajectory, sampled at every whole millisecond.
    #[arg(long)]
    trace: Option<PathBuf>,
    /// Record each complete physical state for native replay rendering.
    #[arg(long, requires = "trace")]
    trace_poses: bool,
}
struct Mechanics {
    linear: DVec3,
    angular: DVec3,
    energy: f64,
}
fn measure(spec: &MultiSphereSpec, state: &MultiSphereObservation) -> Mechanics {
    let links = [
        spec.hinge.links[0],
        spec.hinge.links[1],
        spec.spheres[0],
        spec.spheres[1],
    ];
    let states = [
        state.hinge.links[0],
        state.hinge.links[1],
        state.spheres[0],
        state.spheres[1],
    ];
    let center = std::array::from_fn::<_, 4, _>(|i| {
        DVec3::from(states[i].position)
            + DQuat::from_array(states[i].orientation) * DVec3::from(links[i].inertia_center)
    });
    let total_mass: f64 = links.iter().map(|l| l.mass).sum();
    let barycenter = center
        .iter()
        .zip(&links)
        .map(|(c, l)| *c * l.mass)
        .sum::<DVec3>()
        / total_mass;
    let mut m = Mechanics {
        linear: DVec3::ZERO,
        angular: DVec3::ZERO,
        energy: 0.,
    };
    for ((l, s), c) in links.into_iter().zip(states).zip(center) {
        let rotation =
            DQuat::from_array(s.orientation).normalize() * DQuat::from_array(l.inertia_orientation);
        let v = DVec3::from(s.velocity);
        let omega = DVec3::from(s.angular_velocity);
        let p = v * l.mass;
        let [xx, yy, zz, xy, xz, yz] = l.inertia_tensor;
        let inertia = DMat3::from_cols_array(&[xx, xy, xz, xy, yy, yz, xz, yz, zz]);
        let spin = rotation * (inertia * (rotation.conjugate() * omega));
        m.linear += p;
        m.angular += (c - barycenter).cross(p) + spin;
        m.energy += 0.5 * (v.dot(p) + omega.dot(spin));
    }
    m
}
#[derive(Serialize)]
struct Sample {
    time_ms: u64,
    energy_j: f64,
    joint_work_j: f64,
    linear_drift_ns: f64,
    angular_drift_nms: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    observation: Option<MultiSphereObservation>,
    interval_normal_impulses_ns: [f64; 2],
    q: f64,
    qdot: f64,
}
fn main() -> Result<()> {
    let a = Args::parse();
    if a.ticks == 0 || ![1, 2, 4, 8].contains(&a.divisor) {
        return Err("positive millisecond count and a divisor of 1, 2, 4 or 8 required".into());
    }
    let xml = convoy::geometry(a.seed);
    let spec = distributed_multi_sphere_spec(&xml, "convoy.sdf", "default")?;
    let fine = xml.replace(
        "<max_step_size>0.001</max_step_size>",
        &format!(
            "<max_step_size>{:.9}</max_step_size>",
            0.001 / a.divisor as f64
        ),
    );
    let mut world = AuthoredHinge::load(&fine, "convoy.sdf")?;
    let origin = reference::observe(&world, &spec, 0)?;
    let initial = measure(&spec, &origin);
    let mut prior_q = origin.hinge.q;
    let mut prior_ke = initial.energy;
    let mut out = a
        .trace
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let (mut work, mut peak_p, mut peak_l, mut free_work_residual, mut contact_work_residual) =
        (0., 0_f64, 0_f64, 0_f64, 0_f64);
    if let Some(w) = &mut out {
        let initial_sample = Sample {
            time_ms: 0,
            energy_j: initial.energy,
            joint_work_j: 0.,
            linear_drift_ns: 0.,
            angular_drift_nms: 0.,
            observation: a.trace_poses.then_some(origin),
            interval_normal_impulses_ns: [0.; 2],
            q: origin.hinge.q,
            qdot: origin.hinge.qdot,
        };
        serde_json::to_writer(&mut *w, &initial_sample)?;
        w.write_all(b"\n")?;
    }
    let mut began = [None, None];
    let mut loaded_steps = 0;
    let mut final_sample = None;
    for ms in 1..=a.ticks {
        let effort = convoy::drive(a.seed, ms) + convoy::extra(a.seed, ms).unwrap_or(0.);
        let mut impulses = [0.; 2];
        for sub in 1..=a.divisor {
            world.queue(spec.hinge.joint_id, effort)?;
            world.advance()?;
            let nanos = (ms - 1) * 1_000_000 + sub * (1_000_000 / a.divisor);
            let state = reference::observe(&world, &spec, nanos / 1_000_000)?;
            let loaded = [
                state.sphere_reaction.normal_impulse > 0.,
                state
                    .reactions
                    .iter()
                    .flatten()
                    .any(|r| r.normal_impulse > 0.),
            ];
            for (i, active) in loaded.into_iter().enumerate() {
                if active {
                    began[i].get_or_insert(nanos);
                }
            }
            impulses[0] += state.sphere_reaction.normal_impulse;
            impulses[1] += state
                .reactions
                .iter()
                .flatten()
                .map(|r| r.normal_impulse)
                .sum::<f64>();
            let m = measure(&spec, &state);
            let p = (m.linear - initial.linear).length();
            let l = (m.angular - initial.angular).length();
            let applied_work = effort * (state.hinge.q - prior_q);
            let remainder = m.energy - prior_ke - applied_work;
            if ![p, l, remainder].into_iter().all(f64::is_finite) {
                return Err(format!("invalid mechanics at {nanos} ns").into());
            }
            if loaded.contains(&true) {
                loaded_steps += 1;
                contact_work_residual += remainder;
            } else {
                free_work_residual += remainder;
            }
            peak_p = peak_p.max(p);
            peak_l = peak_l.max(l);
            work += applied_work;
            prior_ke = m.energy;
            prior_q = state.hinge.q;
            if sub == a.divisor {
                let s = Sample {
                    time_ms: ms,
                    energy_j: m.energy,
                    joint_work_j: work,
                    linear_drift_ns: p,
                    angular_drift_nms: l,
                    observation: a.trace_poses.then_some(state),
                    interval_normal_impulses_ns: impulses,
                    q: state.hinge.q,
                    qdot: state.hinge.qdot,
                };
                if let Some(w) = &mut out {
                    serde_json::to_writer(&mut *w, &s)?;
                    w.write_all(b"\n")?;
                }
                final_sample = Some(s);
            }
        }
    }
    if let Some(mut w) = out {
        w.flush()?;
    }
    println!(
        "{}",
        serde_json::json!({"seed":a.seed,"ticks":a.ticks,"steps_per_ms":a.divisor,"initial_energy_j":initial.energy,"max_linear_drift_ns":peak_p,"max_angular_drift_nms":peak_l,"first_pair_impulse_ns":began[0],"first_hinge_impulse_ns":began[1],"loaded_substeps":loaded_steps,"sum_work_residual_free_j":free_work_residual,"sum_work_residual_loaded_j":contact_work_residual,"final":final_sample})
    );
    Ok(())
}
