// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent pinned-Rapier controls for a floating locked cage and its free load.
use clap::Parser;
use rapier3d_f64::prelude::*;
use serde::Deserialize;
use serde_json::json;
use std::{error::Error, fs, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Deserialize)]
struct Ball {
    id: u64,
    radius: f64,
    mass: f64,
    position: [f64; 3],
    velocity: [f64; 3],
    inertia_center: [f64; 3],
    angular_velocity: [f64; 3],
}
#[derive(Deserialize)]
struct Joint {
    parent_id: u64,
    child_id: u64,
    anchor: [f64; 3],
    axis: [f64; 3],
}
#[derive(Deserialize)]
struct Tree {
    links: Vec<Ball>,
    joints: Vec<Joint>,
}
#[derive(Deserialize)]
struct Case {
    tree: Tree,
    payload: Ball,
}
#[derive(Parser)]
struct Args {
    construction: PathBuf,
    #[arg(long, default_value_t = 280)]
    ticks: u32,
    #[arg(long,default_value_t=1,value_parser=clap::value_parser!(u32).range(1..=64))]
    divisor: u32,
    /// Remove initial geometric interference by shrinking the load radius.
    #[arg(long, default_value_t = 0.)]
    shrink: f64,
    /// Record the two driven reduced-coordinate trajectories in a new directory.
    #[arg(long)]
    trace: Option<PathBuf>,
}
#[derive(Clone, Copy, Debug, PartialEq)]
enum Form {
    Compound,
    ReducedFixed,
    ReducedStopped,
    ReducedPinned,
    ImpulseFixed,
    ImpulseStopped,
}
fn sphere(w: &mut PhysicsWorld, b: &Ball, r: f64) -> RigidBodyHandle {
    w.insert(
        RigidBodyBuilder::dynamic()
            .can_sleep(false)
            .translation(b.position.into())
            .linvel(b.velocity.into())
            .angvel(b.angular_velocity.into()),
        ColliderBuilder::ball(r).mass(b.mass),
    )
    .0
}
fn run(a: &Args, s: &Case, form: Form, drive: bool) -> Result<()> {
    let mut w = PhysicsWorld::new();
    w.gravity = Vector::ZERO;
    w.integration_parameters.dt = 0.001 / f64::from(a.divisor);
    let p = sphere(&mut w, &s.payload, s.payload.radius - a.shrink);
    let m: f64 = s.tree.links.iter().map(|b| b.mass).sum();
    let c0 = s
        .tree
        .links
        .iter()
        .map(|b| Vector::from(b.position) * b.mass)
        .sum::<Vector>()
        / m;
    let mut links = Vec::new();
    let mut first_joint = None;
    if form == Form::Compound {
        let handle = w.insert_body(RigidBodyBuilder::dynamic().can_sleep(false).translation(c0));
        for b in &s.tree.links {
            w.insert_collider(
                ColliderBuilder::ball(b.radius)
                    .mass(b.mass)
                    .translation(Vector::from(b.position) - c0),
                Some(handle),
            );
        }
        links.push(handle);
    } else {
        links.extend(s.tree.links.iter().map(|b| sphere(&mut w, b, b.radius)));
        for j in &s.tree.joints {
            let lookup = |id| {
                s.tree
                    .links
                    .iter()
                    .position(|b| b.id == id)
                    .ok_or("missing joint member")
            };
            let i = lookup(j.parent_id)?;
            let k = lookup(j.child_id)?;
            let axis = Vector::from(j.axis);
            let orient = Rotation::from_rotation_arc(Vector::X, axis);
            let mask = if matches!(form, Form::ReducedFixed | Form::ImpulseFixed) {
                JointAxesMask::LOCKED_FIXED_AXES
            } else {
                JointAxesMask::LOCKED_REVOLUTE_AXES
            };
            let mut joint = GenericJointBuilder::new(mask)
                .contacts_enabled(false)
                .local_frame1(Pose::from_parts(
                    Vector::from(j.anchor) - Vector::from(s.tree.links[i].position),
                    orient,
                ))
                .local_frame2(Pose::from_parts(
                    Vector::from(j.anchor) - Vector::from(s.tree.links[k].position),
                    orient,
                ));
            if matches!(form, Form::ReducedStopped | Form::ImpulseStopped) {
                joint = joint.limits(JointAxis::AngX, [0., 0.]);
            }
            if matches!(
                form,
                Form::ReducedStopped | Form::ReducedFixed | Form::ReducedPinned
            ) {
                let handle = w
                    .insert_multibody_joint(links[i], links[k], joint)
                    .ok_or("failed native tree")?;
                if form == Form::ReducedPinned {
                    let (tree, index) = w.multibody_joints.get_mut(handle).unwrap();
                    tree.link_mut(index).unwrap().joint.kinematic = true;
                }
                first_joint.get_or_insert(handle);
            } else {
                w.insert_impulse_joint(links[i], links[k], joint);
            }
        }
    }
    let mt = m + s.payload.mass;
    let center = (c0 * m + Vector::from(s.payload.position) * s.payload.mass) / mt;
    let initial = Vector::from(s.payload.velocity) * s.payload.mass;
    let (mut peak_raw, mut peak_jac, mut peak_com, mut energy, mut final_jac) =
        (0_f64, 0_f64, 0_f64, 0_f64, Vector::ZERO);
    use std::io::{BufWriter, Write};
    let mut log = if drive && matches!(form, Form::ReducedStopped | Form::ReducedFixed | Form::ReducedPinned) {
        a.trace
            .as_ref()
            .map(|dir| {
                fs::File::create_new(dir.join(format!("{form:?}.jsonl"))).map(BufWriter::new)
            })
            .transpose()?
    } else {
        None
    };
    let record = |w: &PhysicsWorld,
                  log: &mut BufWriter<fs::File>,
                  tick: u32,
                  bary: Vector,
                  momentum: Vector|
     -> Result<()> {
        let balls: Vec<_> = links
            .iter()
            .chain(std::iter::once(&p))
            .map(|&h| {
                let body = &w.bodies[h];
                json!({"p":body.translation().to_array(),"q":body.rotation().to_array()})
            })
            .collect();
        let expected = center + initial / mt * (f64::from(tick) * 0.001);
        serde_json::to_writer(
            &mut *log,
            &json!({"tick":tick,"balls":balls,"barycenter":bary.to_array(),"expected_center":expected.to_array(),"momentum_residual":momentum.to_array()}),
        )?;
        writeln!(log)?;
        Ok(())
    };
    if let Some(log) = &mut log {
        record(&w, log, 0, center, Vector::ZERO)?;
    }
    for tick in 0..a.ticks {
        let j = tick as usize % s.tree.joints.len();
        for _ in 0..a.divisor {
            if drive && form != Form::Compound {
                let torque = if tick % 15 < 7 { 0.036 } else { -0.024 };
                let parent = links[j];
                let child = links[j + 1];
                let axis = *w.bodies[parent].rotation() * Vector::from(s.tree.joints[j].axis);
                w.bodies[parent].add_torque(-axis * torque, true);
                w.bodies[child].add_torque(axis * torque, true);
            }
            if let Some(h) = first_joint {
                let (tree, _) = w.multibody_joints.get_mut(h).unwrap();
                tree.damping_mut().fill(0.);
            }
            w.step();
            for &h in &links {
                w.bodies[h].reset_torques(false);
            }
        }
        let mut raw = w.bodies[p].linvel() * s.payload.mass;
        let mut bary = w.bodies[p].center_of_mass() * s.payload.mass;
        let mut kinetic = w.bodies[p].kinetic_energy();
        for &h in &links {
            let b = &w.bodies[h];
            raw += b.linvel() * b.mass();
            bary += b.center_of_mass() * b.mass();
            kinetic += b.kinetic_energy();
        }
        let mut jac = raw;
        if let Some(h) = first_joint {
            let (tree, _) = w.multibody_joints.get_mut(h).unwrap();
            tree.forward_kinematics(&w.bodies, true);
            jac = w.bodies[p].linvel() * s.payload.mass;
            for link in tree.links() {
                let v = tree.body_jacobian(link.link_id()) * tree.generalized_velocity();
                jac += Vector::new(v[0], v[1], v[2]) * w.bodies[link.rigid_body_handle()].mass();
            }
        }
        peak_raw = peak_raw.max(raw.distance(initial));
        peak_jac = peak_jac.max(jac.distance(initial));
        peak_com = peak_com
            .max((bary / mt).distance(center + initial / mt * ((f64::from(tick) + 1.) * 0.001)));
        energy = energy.max(kinetic);
        final_jac = jac - initial;
        if let Some(log) = &mut log {
            record(&w, log, tick + 1, bary / mt, final_jac)?;
        }
    }
    println!(
        "{}",
        json!({"form":format!("{form:?}"),"actuation":drive,"ticks":a.ticks,"subdivision":a.divisor,"payload_shrink_m":a.shrink,"initial_momentum_ns":initial.to_array(),"peak_native_momentum_drift_ns":peak_raw,"peak_jacobian_momentum_drift_ns":peak_jac,"final_jacobian_momentum_residual_ns":final_jac.to_array(),"peak_centroid_position_error_m":peak_com,"peak_native_kinetic_j":energy})
    );
    Ok(())
}
fn main() -> Result<()> {
    let a = Args::parse();
    let case: Case = serde_json::from_slice(&fs::read(&a.construction)?)?;
    if case.tree.links.len() < 2
        || case.tree.joints.len() + 1 != case.tree.links.len()
        || case
            .tree
            .links
            .iter()
            .any(|b| b.inertia_center != [0.; 3] || b.velocity != [0.; 3])
        || a.shrink < 0.
        || a.shrink >= case.payload.radius
    {
        return Err("expected the serial ball cage and a valid shrink".into());
    }
    if let Some(dir) = &a.trace {
        fs::create_dir(dir)?;
    }
    for form in [
        Form::Compound,
        Form::ReducedFixed,
        Form::ReducedStopped,
        Form::ReducedPinned,
        Form::ImpulseFixed,
        Form::ImpulseStopped,
    ] {
        for drive in [false, true] {
            if form == Form::Compound && drive {
                continue;
            }
            run(&a, &case, form, drive)?;
        }
    }
    Ok(())
}
