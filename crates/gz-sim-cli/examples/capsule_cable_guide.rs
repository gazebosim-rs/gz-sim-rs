// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record an ordinary Bevy cable or cloth carried over a translating, spinning capsule.
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step};
use serde_json::json;
mod soft_replay;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    /// New gz-web-rs archive to write.
    output: PathBuf,
    /// Record the 3-by-5 cloth instead of the crown and endcap cables.
    #[arg(long)]
    cloth: bool,
    /// Use a free finite-mass capsule in zero gravity.
    #[arg(long)]
    dynamic: bool,
    /// Write all measured ticks as JSON lines, including the rigid guide and each particle and reaction.
    #[arg(long)]
    measurements: Option<PathBuf>,
}
fn guide(sim: &Simulation) -> BodyState {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::roller::guide")
        })
        .unwrap();
    sim.world()
        .resource::<PhysicsBackend>()
        .state(e.get::<BodyHandle>().unwrap().id())
        .unwrap()
}
fn sample(
    sim: &Simulation,
    state: BodyState,
    sequence: usize,
    cloth: bool,
    dynamic: bool,
    rest: &BTreeMap<String, Vec<DVec3>>,
) -> (String, String) {
    let mut f = Frame {
        visuals: vec![],
        models: vec![],
    };
    let q = DQuat::from_array(state.orientation);
    let center = DVec3::from(state.position);
    let blue = [0.10, 0.41, 0.76, 1.];
    // The primitive replay format supports spheres and boxes. A closed barrel
    // of tangent facets and two spherical ends represent the measured capsule pose.
    for (i, sign) in [(-1, "negative"), (1, "positive")] {
        f.add(
            &format!("guide_cap_{sign}"),
            center + q * DVec3::Z * (0.52 * i as f64),
            q,
            json!({"kind":"sphere","radius":0.2}),
            blue,
        );
    }
    let panels = 16;
    for i in 0..panels {
        let around = DQuat::from_rotation_z((i as f64) * std::f64::consts::TAU / panels as f64);
        let r = q * around;
        let c = if i == 0 { [0.89, 0.73, 0.20, 1.] } else { blue };
        f.add(&format!("guide_barrel_{i}"),center+r*DVec3::X*0.194,r,json!({"kind":"box","size":[0.012,0.4*(std::f64::consts::PI/panels as f64).tan(),1.04]}),c);
    }
    if !dynamic {
        f.add(
            "floor",
            DVec3::new(0., 0., -0.025),
            DQuat::IDENTITY,
            json!({"kind":"box","size":[1.8,2.2,0.05]}),
            [0.19, 0.23, 0.27, 1.],
        );
    }
    for (name, soft) in bodies(sim) {
        let ps = soft.particles();
        let color = match name {
            "cap" => [0.23, 0.8, 0.53, 1.],
            "crown" => [0.96, 0.44, 0.15, 1.],
            _ => [0.84, 0.27, 0.49, 1.],
        };
        for (i, p) in ps.iter().enumerate() {
            let xyz = DVec3::from(p.position);
            f.add(
                &format!("{name}_node_{i}"),
                xyz,
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":if cloth {0.018}else{0.025}}),
                color,
            );
            if i > 0 && (!cloth || i % 5 != 0) {
                f.edge(
                    &format!("{name}_row_{i}"),
                    ps[i - 1].position.into(),
                    xyz,
                    rest[name][i - 1].distance(rest[name][i]),
                    0.016,
                    color,
                );
            }
            if cloth && i >= 5 {
                f.edge(
                    &format!("{name}_column_{i}"),
                    ps[i - 5].position.into(),
                    xyz,
                    rest[name][i - 5].distance(rest[name][i]),
                    0.012,
                    color,
                );
            }
        }
    }
    f.finish(sim.elapsed(), sequence)
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let (xml, name) = if args.dynamic && args.cloth {
        (
            include_str!("../../../examples/dynamic_capsule_cloth.sdf"),
            "dynamic_capsule_cloth.sdf",
        )
    } else if args.dynamic {
        (
            include_str!("../../../examples/dynamic_capsule_cable.sdf"),
            "dynamic_capsule_cable.sdf",
        )
    } else if args.cloth {
        (
            include_str!("../../../examples/capsule_cloth_guide.sdf"),
            "capsule_cloth_guide.sdf",
        )
    } else {
        (
            include_str!("../../../examples/capsule_cable_guide.sdf"),
            "capsule_cable_guide.sdf",
        )
    };
    let mut sim = load_world_with_sdf_step(
        xml,
        name,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut data = args
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let rest = bodies(&sim)
        .map(|(name, b)| {
            (
                name.to_owned(),
                b.particles()
                    .iter()
                    .map(|p| DVec3::from(p.position))
                    .collect(),
            )
        })
        .collect();
    let mut frames = Vec::new();
    let mut touching = vec![false; bodies(&sim).count()];
    for tick in 0..=500 {
        if tick > 0 {
            sim.step_exact(1)
                .map_err(|e| format!("tick {tick}: {e}; {:?}", sim.fault()))?;
        }
        let state = guide(&sim);
        let mut event = false;
        let mut soft = BTreeMap::new();
        for ((name, body), prior) in bodies(&sim).zip(&mut touching) {
            let contact = body
                .last_boundary_impulses()
                .first()
                .is_some_and(|j| j.linear != [0.; 3]);
            if *prior != contact {
                println!("tick {tick}, {name}: capsule contact {contact}");
                event = true;
            }
            *prior = contact;
            if data.is_some() {
                let reactions: Vec<_> = body
                    .last_boundary_impulses()
                    .iter()
                    .map(|r| json!({"linear":r.linear,"angular":r.angular}))
                    .collect();
                let particles:Vec<_>=body.particles().iter().map(|p|json!({"position":p.position,"velocity":p.velocity,"inverse_mass":p.inverse_mass})).collect();
                let mut sample = json!({"particles":particles,"reactions":reactions});
                if args.dynamic {
                    sample["sweep_refinements"] = json!(body.last_sweep_refinements());
                }
                soft.insert(name, sample);
            }
        }
        let frame = if tick % 5 == 0 || event {
            frames.push(sample(
                &sim,
                state,
                frames.len() + 1,
                args.cloth,
                args.dynamic,
                &rest,
            ));
            Some(frames.len())
        } else {
            None
        };
        if let Some(writer) = &mut data {
            serde_json::to_writer(
                &mut *writer,
                &json!({"tick":tick,"frame":frame,"guide":{"position":state.position,"orientation":state.orientation,"velocity":state.velocity,"angular_velocity":state.angular_velocity},"soft":soft}),
            )?;
            writeln!(writer)?;
        }
    }
    if let Some(w) = &mut data {
        w.flush()?;
    }
    println!("{} measured frames over 1.000 seconds", frames.len());
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    Ok(())
}
