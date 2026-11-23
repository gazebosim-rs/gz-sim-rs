// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Export a measured four-worker contact trace as a native gz-web-rs recording.
#[path = "support/curved_ground_plot.rs"]
mod plot;
use clap::Parser;
use gz_physics_distributed::physics::{Body, ContactImpulse, GroundSupport, Primitive};
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    /// distributed.jsonl from a four-worker contact qualification.
    trace: PathBuf,
    /// New gz-web-rs archive directory.
    output: PathBuf,
    /// Optional SVG of the contact impulses and post-recovery cylinder velocity.
    #[arg(long)]
    impulse_plot: Option<PathBuf>,
    /// Color bodies blue for the 10 ms solve and orange for ten 1 ms solves.
    #[arg(long, conflicts_with = "impulse_plot")]
    color_by_resolution: bool,
}

#[derive(Deserialize)]
struct Sample {
    tick: u64,
    bodies: Vec<Body>,
    owners: Vec<usize>,
    fine_ids: Option<Vec<u64>>,
    impulses: Vec<ContactImpulse>,
    supports: Vec<GroundSupport>,
}

impl Sample {
    fn material(&self, body: &Body, owner: usize, by_resolution: bool) -> usize {
        if by_resolution {
            usize::from(
                self.fine_ids
                    .as_deref()
                    .unwrap()
                    .binary_search(&body.id)
                    .is_ok(),
            )
        } else {
            owner
        }
    }
}

const COLORS: [[f32; 4]; 4] = [
    [0.12, 0.55, 0.98, 1.],
    [1., 0.43, 0.08, 1.],
    [0.16, 0.76, 0.46, 1.],
    [0.66, 0.36, 0.92, 1.],
];

fn route(owner: usize) -> String {
    format!("/assets/{owner}/cylinder.glb")
}

fn body_visuals(body: &Body, name: &str, material: usize, model: u64) -> (Vec<Value>, Value) {
    let mut visuals = Vec::new();
    let mut ids = Vec::new();
    let mut add = |part, position, shape, color| {
        let id = model + 1 + ids.len() as u64;
        ids.push(id);
        visuals.push(json!({"id":id,"name":format!("{name}::{part}"),
            "translation":position,"rotation":body.orientation,"shape":shape,"color":color}));
    };
    let radius = body.size[0] / 2.;
    let sphere = json!({"kind":"sphere","radius":radius});
    match body.primitive {
        Primitive::Box => add(
            "box",
            body.position,
            json!({"kind":"box","size":body.size}),
            COLORS[material],
        ),
        Primitive::Sphere => add("sphere", body.position, sphere, COLORS[material]),
        Primitive::Capsule | Primitive::Cylinder => {
            let uri = format!("{}{}", gz_sim_cli::EXPORT_ASSET_PREFIX, route(material));
            add(
                "barrel",
                body.position,
                json!({"kind":"mesh","uri":uri,
                "scale":[radius,radius,body.size[2]]}),
                [1.; 4],
            );
            if body.primitive == Primitive::Capsule {
                let [x, y, z, w] = body.orientation;
                let axis = [
                    2. * (x * z + w * y),
                    2. * (y * z - w * x),
                    1. - 2. * (x * x + y * y),
                ];
                for (part, sign) in [("cap_a", -0.5), ("cap_b", 0.5)] {
                    let center =
                        std::array::from_fn(|i| body.position[i] + sign * body.size[2] * axis[i]);
                    add(part, center, sphere.clone(), COLORS[material]);
                }
            }
        }
    }
    (
        visuals,
        json!({"model_id":model,"name":name,"visual_ids":ids,
        "can_remove":false,"can_set_state":false}),
    )
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut samples = Vec::new();
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    let mut assets = BTreeMap::new();
    let mut first_ids = None;
    for line in BufReader::new(File::open(args.trace)?).lines() {
        let s: Sample = serde_json::from_str(&line?)?;
        let ids: BTreeSet<_> = s.bodies.iter().map(|b| b.id).collect();
        if s.bodies.is_empty()
            || s.bodies.len() != ids.len()
            || s.bodies.len() != s.owners.len()
            || *first_ids.get_or_insert_with(|| ids.clone()) != ids
            || samples
                .last()
                .is_some_and(|prev: &Sample| prev.tick.checked_add(1) != Some(s.tick))
        {
            return Err("expected contiguous public barriers with an owner per body".into());
        }
        if args.color_by_resolution {
            let fine = s
                .fine_ids
                .as_deref()
                .ok_or("missing island resolution receipt")?;
            if (s.tick == 0 && !fine.is_empty())
                || fine.windows(2).any(|p| p[0] >= p[1])
                || fine.iter().any(|id| !ids.contains(id))
            {
                return Err("invalid island resolution receipt".into());
            }
        }
        for (b, &owner) in s.bodies.iter().zip(&s.owners) {
            b.shape()?;
            if owner >= COLORS.len() {
                return Err("invalid four-worker owner".into());
            }
            let material = s.material(b, owner, args.color_by_resolution);
            if matches!(b.primitive, Primitive::Cylinder | Primitive::Capsule)
                && let std::collections::btree_map::Entry::Vacant(e) = assets.entry(route(material))
            {
                e.insert(gz_assets::mesh::cylinder_visual_glb(COLORS[material])?.into());
            }
            let r = 0.5 * (b.size[0].hypot(b.size[1]).hypot(b.size[2]) + b.size[0]);
            for i in 0..2 {
                lo[i] = lo[i].min(b.position[i] - r);
                hi[i] = hi[i].max(b.position[i] + r);
            }
        }
        samples.push(s);
    }
    if samples.is_empty() {
        return Err("empty distributed trace".into());
    }
    if samples[0].tick != 0 {
        return Err("the replay requires the actual startup frame at tick zero".into());
    }
    if let Some(path) = args.impulse_plot {
        plot::draw(&samples, &path)?;
    }
    let floor = json!({"id":3,"name":"default::ground::surface",
        "translation":[(lo[0]+hi[0])/2.,(lo[1]+hi[1])/2.,-0.025],"rotation":[0.,0.,0.,1.],
        "shape":{"kind":"box","size":[hi[0]-lo[0]+0.5,hi[1]-lo[1]+0.5,0.05]},
        "color":[0.32,0.37,0.43,1.]});
    let floor_model = json!({"model_id":2,"name":"default::ground","visual_ids":[3],
        "can_remove":false,"can_set_state":false});
    let mut identities = BTreeMap::new();
    let mut next_model = 4;
    let count = samples.len();
    let frames = samples.into_iter().enumerate().map(|(i,s)| {
        let mut visuals = vec![floor.clone()];
        let mut models = vec![floor_model.clone()];
        for (b,&owner) in s.bodies.iter().zip(&s.owners) {
            let material = s.material(b, owner, args.color_by_resolution);
            // Materials and names are immutable for a display identity.
            let startup = args.color_by_resolution && s.tick == 0;
            let appearance = (owner, material, startup);
            let entry = identities.entry(b.id).or_insert((None,0));
            if entry.0 != Some(appearance) {
                *entry = (Some(appearance), next_model);
                next_model += 4;
            }
            let mut name = format!("default::{:?}_{}_worker_{owner}", b.primitive, b.id);
            if args.color_by_resolution {
                name.push_str(if startup { "_startup" } else if material == 1 { "_1ms" } else { "_10ms" });
            }
            let (parts, model) = body_visuals(b, &name, material, entry.1);
            visuals.extend(parts);
            models.push(model);
        }
        let sequence = i+1;
        let scene = json!({"version":2,"kind":"scene","world":"default","world_id":1,"epoch":0,
            "seconds":s.tick/100,"nanoseconds":(s.tick%100)*10_000_000,"sequence":sequence,"paused":false,"visuals":visuals});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,
            "scene_sequence":sequence,"models":models,"templates":[]});
        (scene.to_string(),catalog.to_string())
    });
    gz_sim_cli::export_archive(&args.output, assets, frames)?;
    println!(
        "Exported {count} four-worker barriers to {}",
        args.output.display()
    );
    Ok(())
}
