// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay the real managed material timeline and a magnified inset at its measured contact head.
#[path = "../../gz-sim-cli/examples/soft_replay/mod.rs"]
mod soft_replay;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics_distributed::{
    articulation::ArticulationLinkState,
    floating_tree::{
        AutomaticImpactReceipt, AutomaticMaterialPool, AutomaticMaterialSpec, ContactTreeSpec,
    },
};
use gz_sim_scene::deformable::articulated_impact::distributed_automatic_material_contact_tree_spec;
use serde::Serialize;
use serde_json::json;
use soft_replay::Frame;
use std::{
    collections::BTreeMap, error::Error, fs, path::PathBuf, process::Command, time::Duration,
};
#[derive(Parser)]
struct Args {
    output: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    session: PathBuf,
    /// A four-node frictionless cloth. By default, use the thin, fast cable.
    #[arg(long)]
    cloth: bool,
}
#[derive(Serialize)]
struct Sample {
    nanos: u64,
    private_head: bool,
    links: Vec<ArticulationLinkState>,
    particles: Vec<([f64; 3], [f64; 3])>,
}
impl Sample {
    fn public(p: &AutomaticMaterialPool) -> Self {
        Self {
            nanos: p.tick() * p.spec().step_ns,
            private_head: false,
            links: p.observation().links.clone(),
            particles: p
                .particles()
                .iter()
                .map(|n| (n.position, n.velocity))
                .collect(),
        }
    }
}
const GOLD: [f64; 4] = [0.96, 0.62, 0.12, 1.];
const TEAL: [f64; 4] = [0.07, 0.64, 0.57, 1.];
const CORAL: [f64; 4] = [0.97, 0.25, 0.16, 1.];
fn actual_frame(
    s: &Sample,
    spec: &AutomaticMaterialSpec,
    skin: f64,
    hit: &AutomaticImpactReceipt,
) -> Frame {
    let mut f = Frame::default();
    let turn = DQuat::from_rotation_z(-0.72);
    let view = |p: DVec3| turn * p - DVec3::Y * 0.78;
    for (i, (l, b)) in spec.tree.tree.links.iter().zip(&s.links).enumerate() {
        let q = DQuat::from_array(b.orientation);
        let world = DVec3::from(b.position);
        f.add(
            &format!("moving_link_{}", l.id),
            view(world),
            turn * q,
            json!({"kind":"sphere","radius":l.radius}),
            TEAL,
        );
        let (offset, size) = if i == 0 {
            (DVec3::X * 0.02, [0.15, 0.58, 0.095])
        } else {
            (-DVec3::X * 0.30, [0.60, 0.045, 0.055])
        };
        f.add(
            &format!("moving_shank_{}", l.id),
            view(world + q * offset),
            turn * q,
            json!({"kind":"box","size":size}),
            [0.18, 0.39, 0.55, 1.],
        );
    }
    let pts = s
        .particles
        .iter()
        .map(|(p, _)| DVec3::from(*p))
        .collect::<Vec<_>>();
    let edges = if pts.len() == 4 {
        vec![(0, 1), (0, 2), (1, 3), (2, 3), (0, 3)]
    } else {
        vec![(0, 1)]
    };
    for (i, p) in pts.iter().enumerate() {
        f.add(
            &format!("moving_node_{i}"),
            view(*p),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":skin.max(0.012)}),
            GOLD,
        );
    }
    let stations = if pts.len() == 4 { 20 } else { 32 };
    for (a, b) in edges {
        // Scene identities keep fixed shapes. Uniform mid-edge samples show the actual
        // changing segment length via measured positions at every recorded timestamp.
        for station in 1..stations {
            let point = pts[a].lerp(pts[b], f64::from(station) / f64::from(stations));
            f.add(
                &format!("moving_material_{a}_{b}_{station}"),
                view(point),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":skin.max(0.007)}),
                GOLD,
            );
        }
    }
    // All vertices of this fixed inset lie on the source feature at the solver's actual event time.
    let row = &hit.contact.loads[0];
    let i = spec
        .tree
        .tree
        .links
        .iter()
        .position(|l| l.id == row.link)
        .unwrap();
    let r = spec.tree.tree.links[i].radius;
    let c = DVec3::from(hit.head_links[i].position);
    let zoom = 0.18 / r;
    let inset = |p: DVec3| DVec3::new(0., 0.80, 0.57) + turn * ((p - c) * zoom);
    f.add(
        "actual_head_rigid_receiver",
        inset(c),
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":r*zoom}),
        TEAL,
    );
    let center = row
        .feature
        .iter()
        .zip(row.weights)
        .fold(DVec3::ZERO, |p, (&node, w)| {
            p + DVec3::from(hit.head_particles[node].0) * w
        });
    let verts = row
        .feature
        .iter()
        .map(|&id| {
            let delta = DVec3::from(hit.head_particles[id].0) - center;
            center + delta.clamp_length_max(2.3 * r)
        })
        .collect::<Vec<_>>();
    for (i, p) in verts.iter().enumerate() {
        f.add(
            &format!("actual_head_feature_witness_{i}"),
            inset(*p),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":(skin*zoom).max(0.012)}),
            GOLD,
        );
    }
    for i in 0..if verts.len() == 2 { 1 } else { verts.len() } {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        f.edge(
            &format!("actual_head_feature_{i}"),
            inset(a),
            inset(b),
            a.distance(b) * zoom,
            2. * skin * zoom,
            GOLD,
        );
    }
    let point = inset(DVec3::from(row.point));
    let direction = turn * DVec3::from(row.on_link).normalize();
    f.add(
        "actual_head_surface_contact",
        point,
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":0.021}),
        CORAL,
    );
    // The arrow is displaced sideways from the sphere for legibility. Its direction is the actual signed link impulse.
    let offset = point + turn * DVec3::Y * 0.34;
    f.edge(
        "measured_link_impulse",
        offset,
        offset + direction * 0.22,
        0.22,
        0.023,
        CORAL,
    );
    f.add(
        "measured_link_impulse_tip",
        offset + direction * 0.22,
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":0.030}),
        CORAL,
    );
    f
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let xml = if a.cloth {
        include_str!("../../../examples/articulated_first_impact.sdf")
            .replace("0.001</max_step_size>", "0.01</max_step_size>")
            .replace("friction=\"0.9\"", "friction=\"0\"")
    } else {
        include_str!("../../../examples/articulated_transient_cable.sdf").to_owned()
    };
    let s = distributed_automatic_material_contact_tree_spec(&xml, "automatic.sdf", "default")?;
    let spec = AutomaticMaterialSpec {
        tree: ContactTreeSpec {
            tree: s.tree,
            limits: s.limits,
        },
        step_ns: s.step.as_nanos() as u64,
        material_name: s.material_name,
        links: s.links,
        restitution: s.restitution,
    };
    let mut p =
        AutomaticMaterialPool::create(&a.worker, &a.session, 3, spec.clone(), s.material.clone())?;
    let mut states = vec![Sample::public(&p)];
    let mut hit = None;
    for tick in 0..40 {
        if [0, 10, 16].contains(&tick) {
            let pid = p.worker_pid(p.owner()).unwrap();
            if !Command::new("kill")
                .args(["-KILL", &pid.to_string()])
                .status()?
                .success()
            {
                return Err("native owner did not receive SIGKILL".into());
            }
        }
        p.advance_at(0, tick)?;
        if hit.is_none()
            && let Some(r) = p.last_impact()
        {
            states.push(Sample {
                nanos: r.input_tick * spec.step_ns + r.head_ns,
                private_head: true,
                links: r.head_links.clone(),
                particles: r.head_particles.clone(),
            });
            hit = Some(r.clone());
        }
        states.push(Sample::public(&p));
        if [0, 10, 16].contains(&tick) {
            p.transfer((p.owner() + 1) % 3)?;
            drop(p);
            p = AutomaticMaterialPool::resume(&a.worker, &a.session, s.material.clone())?;
        }
    }
    let hit = hit.ok_or("material event was not detected")?;
    fs::write(
        a.session.join("timeline.json"),
        serde_json::to_vec(&states)?,
    )?;
    fs::write(
        a.session.join("head.json"),
        serde_json::to_vec_pretty(&hit)?,
    )?;
    let skin = s.material.config().particle_radius;
    let frames: Vec<_> = states
        .iter()
        .enumerate()
        .map(|(i, state)| {
            actual_frame(state, &spec, skin, &hit).finish(Duration::from_nanos(state.nanos), i + 1)
        })
        .collect();
    gz_sim_cli::export_archive(&a.output, BTreeMap::new(), frames)?;
    println!(
        "{} actual frames including private event head at {} ns, 3 real worker SIGKILLs, 3 transfers and 3 disk reopens",
        states.len(),
        hit.input_tick * spec.step_ns + hit.head_ns
    );
    Ok(())
}
