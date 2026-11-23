// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay a returning material contact on a revolving world-supported native pad.
#[path = "../../gz-sim-cli/examples/soft_replay/mod.rs"]
mod soft_replay;
use glam::{DQuat, DVec3};
use gz_physics_distributed::{
    articulation::ArticulationLinkState,
    floating_tree::{AutomaticImpactReceipt, AutomaticRotorMaterialSpec, RotorMaterialPool},
};
use gz_sim_scene::deformable::articulated_impact::distributed_rotor_material_contact_spec;
use serde_json::json;
use soft_replay::Frame;
use std::{
    collections::BTreeMap, env, error::Error, fs, io::Write, path::PathBuf, process::Command,
    time::Duration,
};
#[derive(Clone)]
struct Sample {
    ns: u64,
    pad: ArticulationLinkState,
    nodes: Vec<[f64; 3]>,
}
fn frame(s: &Sample, r: &AutomaticImpactReceipt, spec: &AutomaticRotorMaterialSpec) -> Frame {
    let mut f = Frame::default();
    let c = [0.14, 0.72, 0.72, 1.];
    let blue = [0.16, 0.35, 0.58, 1.];
    let gold = [0.98, 0.67, 0.12, 1.];
    let coral = [0.96, 0.32, 0.17, 1.];
    let anchor = DVec3::from(spec.pad.anchor);
    // Turn the vertical cloth face toward the default viewer while preserving all
    // lengths, angles and the common display frame for the pad, nodes and impulse.
    let basis = if s.nodes.len() == 4 {
        DQuat::from_rotation_z(1.15)
    } else {
        DQuat::IDENTITY
    };
    let view = |p: DVec3| basis * (p - anchor) + DVec3::new(0., -0.70, 0.57);
    let now = DVec3::from(s.pad.position);
    let radius = spec.pad.link.radius;
    f.add(
        "native_pad",
        view(now),
        basis * DQuat::from_array(s.pad.orientation),
        json!({"kind":"sphere","radius":radius}),
        c,
    );
    f.edge(
        "world_revolute_arm",
        view(anchor),
        view(now),
        anchor.distance(DVec3::from(spec.pad.link.position)),
        0.025,
        blue,
    );
    f.add(
        "stationary_world_pivot",
        view(anchor),
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":0.027}),
        blue,
    );
    for (i, n) in s.nodes.iter().enumerate() {
        f.add(
            &format!("material_node_{i}"),
            view((*n).into()),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.016}),
            gold,
        );
    }
    if s.nodes.len() == 4 {
        // Draw material coordinates of the two authored triangles at every accepted position.
        // Fixed-size point markers preserve identity while the physical triangles deform.
        for (face_id, vertices) in [[0, 1, 3], [0, 3, 2]].into_iter().enumerate() {
            let [a, b, d] = vertices.map(|i| DVec3::from(s.nodes[i]));
            for i in 0..=5 {
                for j in 0..=5 - i {
                    let point = a + (b - a) * (f64::from(i) / 5.) + (d - a) * (f64::from(j) / 5.);
                    f.add(
                        &format!("cloth_face_{face_id}_{i}_{j}"),
                        view(point),
                        DQuat::IDENTITY,
                        json!({"kind":"sphere","radius":0.0075}),
                        if face_id == 0 {
                            gold
                        } else {
                            [0.56, 0.36, 0.12, 1.]
                        },
                    );
                }
            }
        }
    } else {
        for i in 1..32 {
            let p = DVec3::from(s.nodes[0]).lerp(DVec3::from(s.nodes[1]), f64::from(i) / 32.);
            f.add(
                &format!("material_cable_{i}"),
                view(p),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":0.009}),
                gold,
            );
        }
    }
    // The grey markers show the certified circular centerline around the world axis, at its real radius.
    for i in 0..32 {
        let p = anchor
            + DQuat::from_axis_angle(
                spec.pad.axis.into(),
                std::f64::consts::TAU * f64::from(i) / 32.,
            ) * (DVec3::from(spec.pad.link.position) - anchor);
        f.add(
            &format!("native_revolute_orbit_{i}"),
            view(p),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.004}),
            [0.33, 0.39, 0.46, 1.],
        );
    }
    let center = DVec3::from(r.head_links[0].position);
    let inset = |p: DVec3| DVec3::new(0., 0.76, 0.58) + basis * (p - center) * 3.8;
    let row = &r.contact.loads[0];
    let vertices: Vec<_> = row
        .feature
        .iter()
        .map(|&i| DVec3::from(r.head_particles[i].0))
        .collect();
    let on_material: DVec3 = vertices.iter().zip(row.weights).map(|(v, w)| *v * w).sum();
    let tangent = (vertices[1] - vertices[0]).normalize();
    let span = 0.076;
    f.add(
        "event_pad",
        inset(center),
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":radius*3.8}),
        c,
    );
    if vertices.len() == 3 {
        let other = (vertices[2] - vertices[0])
            .cross(tangent)
            .normalize()
            .cross(tangent);
        // The face around this witness is interior by more than the depicted 6 cm patch.
        for i in -2..=2 {
            for j in -2..=2 {
                f.add(
                    &format!("local_cloth_patch_{i}_{j}"),
                    inset(on_material + 0.018 * (tangent * f64::from(i) + other * f64::from(j))),
                    DQuat::IDENTITY,
                    json!({"kind":"sphere","radius":0.009}),
                    gold,
                );
            }
        }
    } else {
        f.edge(
            "event_cable_feature",
            inset(on_material - tangent * span),
            inset(on_material + tangent * span),
            span * 2. * 3.8,
            0.022,
            gold,
        );
    }
    let point = inset(DVec3::from(row.point));
    f.add(
        "measured_contact",
        point,
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":0.018}),
        coral,
    );
    let n = basis * DVec3::from(row.on_link).normalize();
    let tail = point + DVec3::Z * 0.23;
    f.edge(
        "impulse_on_rotor",
        tail,
        tail + 0.29 * n,
        0.29,
        0.024,
        coral,
    );
    f.add(
        "signed_impulse_tip",
        tail + 0.29 * n,
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":0.026}),
        coral,
    );
    f
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut a = env::args_os().skip(1);
    let dir = PathBuf::from(a.next().ok_or("output directory")?);
    let exe = PathBuf::from(a.next().ok_or("managed worker executable")?);
    fs::create_dir_all(&dir)?;
    let xml = match a.next().as_deref() {
        None => include_str!("../../../examples/distributed_rotating_pad_cable.sdf"),
        Some(flag) if flag == "--cloth" => {
            include_str!("../../../examples/distributed_rotating_pad_cloth.sdf")
        }
        _ => return Err("expected optional --cloth".into()),
    };
    let r = distributed_rotor_material_contact_spec(xml, "rotating_pad.sdf", "default")?;
    let spec = AutomaticRotorMaterialSpec {
        pad: r.pad,
        material_name: r.material_name,
        step_ns: r.step.as_nanos() as u64,
        restitution: r.restitution,
    };
    let mut p = RotorMaterialPool::create(
        &exe,
        &dir.join("session"),
        3,
        spec.clone(),
        r.material.clone(),
    )?;
    let mut out = fs::File::create(dir.join("trajectory.jsonl"))?;
    let mut states = vec![];
    let mut event = None;
    let interruptions = if r.material.particles().len() == 4 {
        [10, 14, 15, 70]
    } else {
        [12, 25, 26, 70]
    };
    for i in 0..=100u64 {
        if i > 0 {
            if interruptions.contains(&i) {
                p.transfer((p.owner() + 1) % p.width())?;
                assert!(
                    Command::new("kill")
                        .args([
                            "-KILL",
                            &p.worker_pid(p.owner()).ok_or("worker absent")?.to_string()
                        ])
                        .status()?
                        .success()
                );
            }
            p.advance()?;
            if interruptions.contains(&i) {
                drop(p);
                p = RotorMaterialPool::resume(&exe, &dir.join("session"), r.material.clone())?;
            }
        }
        if event.is_none()
            && let Some(e) = p.last_impact()
        {
            states.push(Sample {
                ns: e.input_tick * spec.step_ns + e.head_ns,
                pad: e.head_links[0],
                nodes: e.head_particles.iter().map(|n| n.0).collect(),
            });
            event = Some(e.clone());
        }
        states.push(Sample {
            ns: p.tick() * spec.step_ns,
            pad: p.observation().links[0],
            nodes: p.particles().iter().map(|n| n.position).collect(),
        });
        writeln!(
            out,
            "{}",
            json!({"tick":i,"time_ns":i*spec.step_ns,"pad":p.observation().links[0],"q":p.observation().joints[0].q,"qdot":p.observation().joints[0].qdot,"particles":p.particles().iter().map(|n|(n.position,n.velocity)).collect::<Vec<_>>(),"owner":p.owner(),"generation":p.generation()})
        )?;
    }
    let e = event.ok_or("missing actual angular sweep impact")?;
    fs::write(dir.join("true-head.json"), serde_json::to_vec_pretty(&e)?)?;
    let frames: Vec<_> = states
        .iter()
        .enumerate()
        .map(|(i, s)| frame(s, &e, &spec).finish(Duration::from_nanos(s.ns), i + 1))
        .collect();
    gz_sim_cli::export_archive(&dir.join("replay"), BTreeMap::new(), frames)?;
    println!(
        "verified {} accepted and private geometric frames, four native worker deaths, four warm transfers and four disk reopens; absolute head {} ns",
        states.len(),
        e.input_tick * spec.step_ns + e.head_ns
    );
    Ok(())
}
