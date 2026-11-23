// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay two independent worlds, the guarded arm and an always-dynamic reference.
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, Name, PhysicsEngine, WorldPose, contact_promotion, load_world_with_sdf_step,
    model_fidelity, set_model_fidelity,
};
use serde_json::{Value, json};
use std::{error::Error, path::PathBuf};

const SOURCE: &str = include_str!("../../../examples/multifidelity_predicted_contact.sdf");
struct Row {
    sim: Simulation,
    arm: u64,
    members: [u64; 3],
}

impl Row {
    fn load(guard: bool) -> Result<Self, Box<dyn Error>> {
        let mut sim = load_world_with_sdf_step(
            SOURCE,
            "multifidelity_predicted_contact.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let lookup = |name: &str| -> Result<u64, Box<dyn Error>> {
            let q = format!("default::{name}");
            Ok(sim
                .world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == q))
                .and_then(|e| e.get::<EntityId>())
                .ok_or("missing fixture identity")?
                .get())
        };
        let arm = lookup("arm")?;
        let members = [
            lookup("arm::base")?,
            lookup("arm::tip")?,
            lookup("ram::head")?,
        ];
        if guard {
            set_model_fidelity(&mut sim, arm, ArticulationTier::Prescribed)?;
            contact_promotion::configure(&mut sim, arm, vec![members[2]])?;
        }
        Ok(Self { sim, arm, members })
    }

    fn poses(&self) -> [WorldPose; 3] {
        self.members.map(|id| {
            *self
                .sim
                .world()
                .get::<WorldPose>(entity_by_id(self.sim.world(), id).unwrap())
                .unwrap()
        })
    }

    fn dynamic(&self) -> Result<bool, Box<dyn Error>> {
        Ok(model_fidelity(&self.sim, self.arm)? == ArticulationTier::Dynamic)
    }
}

fn visual(id: u64, name: &str, p: DVec3, r: DQuat, shape: Value, color: [f64; 3]) -> Value {
    json!({"id":id,"name":name,"translation":p.to_array(),"rotation":r.to_array(),"shape":shape,"color":[color[0],color[1],color[2],1.]})
}

fn frame(rows: &[Row; 2], seq: u64) -> Result<(String, String), Box<dyn Error>> {
    // View both XY planes upright, with one vertical display offset per independent world.
    let view = DQuat::from_xyzw(-0.5, 0.5, 0.5, -0.5);
    let mut visuals = Vec::new();
    let mut models = Vec::new();
    for (i, row) in rows.iter().enumerate() {
        let model = 2 + 8 * i as u64;
        let lane = if i == 0 {
            "predicted"
        } else {
            "always_dynamic"
        };
        let name = format!("default::{lane}");
        let poses = row.poses();
        let offset = DVec3::new(0., 0., if i == 0 { 0.49 } else { -0.70 });
        let centers = poses.map(|p| view * DVec3::from(p.translation) + offset);
        let mut ids = Vec::new();
        let mut add = |part: &str, p, r, shape, color| {
            let id = model + 1 + ids.len() as u64;
            ids.push(id);
            visuals.push(visual(id, &format!("{name}::{part}"), p, r, shape, color));
        };
        let color = if i == 0 {
            [0.04, 0.8, 0.68]
        } else {
            [0.16, 0.54, 0.94]
        };
        // A fixed scale beneath each trajectory also keeps both endpoints in the
        // viewer's initial camera bounds. One mark lies at the positive endpoint.
        let ruler = offset + DVec3::new(0.0, 0.0, -0.20);
        add(
            "display_ruler",
            ruler,
            DQuat::IDENTITY,
            json!({"kind":"box","size":[0.04,3.8,0.018]}),
            [0.20, 0.26, 0.32],
        );
        add(
            "ruler_end",
            ruler + DVec3::new(0., -1.90, 0.),
            DQuat::IDENTITY,
            json!({"kind":"box","size":[0.06,0.018,0.085]}),
            [0.31, 0.38, 0.43],
        );
        let direction = centers[1] - centers[0];
        let q = DQuat::from_rotation_arc(DVec3::Z, direction.normalize());
        add(
            "joint_guide",
            (centers[0] + centers[1]) * 0.5,
            q,
            json!({"kind":"box","size":[0.065,0.065,0.65]}),
            color,
        );
        for (j, part) in ["base", "tip"].iter().enumerate() {
            add(
                part,
                centers[j],
                view * DQuat::from_array(poses[j].rotation),
                json!({"kind":"sphere","radius":0.14}),
                color,
            );
        }
        add(
            "prescribed_ram",
            centers[2],
            view * DQuat::from_array(poses[2].rotation),
            json!({"kind":"box","size":[0.32,0.26,0.32]}),
            [0.94, 0.39, 0.06],
        );
        // A small status lamp above the base marks the response tier by its height.
        let height = if row.dynamic()? { 0.08 } else { 0.22 };
        add(
            "response_lamp",
            centers[0] + DVec3::new(0.12, 0., height),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.045}),
            [1., 0.90, 0.52],
        );
        models.push(json!({"model_id":model,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false}));
    }
    let elapsed = rows[0].sim.elapsed();
    let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":seq,"seconds":elapsed.as_secs(),"nanoseconds":elapsed.subsec_nanos(),"paused":false,"visuals":visuals});
    let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":seq,"models":models,"templates":[]});
    Ok((scene.to_string(), catalog.to_string()))
}

fn main() -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: predicted_contact REPLAY_DIRECTORY")?,
    );
    let mut rows = [Row::load(true)?, Row::load(false)?];
    let mut frames = vec![frame(&rows, 1)?];
    let mut first = None;
    let mut error = 0.0_f64;
    for tick in 1..=1200 {
        for row in &mut rows {
            row.sim.step_exact(1)?;
        }
        let changed = first.is_none() && rows[0].dynamic()?;
        if changed {
            first = Some(tick);
        }
        for (a, b) in rows[0].poses().iter().zip(rows[1].poses()) {
            for (x, y) in a
                .translation
                .into_iter()
                .chain(a.rotation)
                .zip(b.translation.into_iter().chain(b.rotation))
            {
                error = error.max((x - y).abs());
            }
        }
        if tick % 25 == 0 || changed || tick == 540 {
            frames.push(frame(&rows, frames.len() as u64 + 1)?);
        }
    }
    let first = first.ok_or("missing predicted contact")?;
    if error > 1e-10 {
        return Err("reference diverged".into());
    }
    println!(
        "1 ms steps; first dynamic tick {first}; maximum link pose difference {error:.3e} over 1200 ticks; {} committed replay samples",
        frames.len()
    );
    gz_sim_cli::export_archive(&out, Default::default(), frames)?;
    Ok(())
}
