// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Drop the free end of a folded sheet onto its supported lower layer.
use bytes::Bytes;
use clap::Parser;
use glam::DVec3;
use gz_sim_cli::{EXPORT_ASSET_PREFIX, export_archive};
use gz_sim_scene::{PhysicsEngine, deformable::bodies, load_world_with_sdf_step};
use serde_json::json;
use std::{
    collections::BTreeMap,
    error::Error,
    path::PathBuf,
    time::{Duration, Instant},
};
#[path = "support/garment_geometry.rs"]
mod garment_geometry;

#[derive(Parser)]
struct Args {
    /// Optional new gzweb replay archive.
    output: Option<PathBuf>,
    /// Compare a sliding upper layer with and without material self-friction.
    #[arg(long)]
    sliding: bool,
}

const XML: &str = include_str!("../../../examples/folded_cloth.sdf");
const N: usize = 250;
const STRIDE: usize = 5;
type Points = Vec<[f64; 3]>;
type ErrorBox = Box<dyn Error>;
struct Run {
    samples: Vec<Points>,
    gap: f64,
    strain: f64,
    elapsed: Duration,
    loaded_distance: f64,
    loaded_time: f64,
    normal_impulse: f64,
    tangent_impulse: f64,
    friction_work: f64,
}
fn triangles() -> impl Iterator<Item = [usize; 3]> {
    (1..9).flat_map(|r| {
        (1..3).flat_map(move |c| {
            let i = r * 3 + c;
            [[i - 4, i - 3, i], [i - 4, i, i - 1]]
        })
    })
}

/// Independent geometric witness: clip every upper triangle to the lower
/// rectangle's vertical prism and minimize its affine height on that polygon.
fn layer_clearance(points: &Points) -> f64 {
    let mut clearance = f64::INFINITY;
    for triangle in triangles().filter(|t| t.iter().all(|i| *i >= 15)) {
        let mut polygon = triangle.map(|i| DVec3::from(points[i])).to_vec();
        for (axis, limit, sign) in [
            (0, -0.50, 1.),
            (0, 0.50, -1.),
            (1, -0.90, 1.),
            (1, 0.51, -1.),
        ] {
            if polygon.is_empty() {
                break;
            }
            let mut clipped = Vec::new();
            let mut a = *polygon.last().unwrap();
            for &b in &polygon {
                let da = (a[axis] - limit) * sign;
                let db = (b[axis] - limit) * sign;
                if (da < 0.) != (db < 0.) {
                    clipped.push(a.lerp(b, da / (da - db)));
                }
                if db >= 0. {
                    clipped.push(b);
                }
                a = b;
            }
            polygon = clipped;
        }
        for p in polygon {
            clearance = clearance.min(p.z - 0.65);
        }
    }
    clearance
}
fn run(enabled: bool) -> Result<Run, ErrorBox> {
    let source = if enabled {
        XML.to_owned()
    } else {
        XML.replace("self_contact=\"true\"", "self_contact=\"false\"")
    };
    run_scene(&source)
}
fn run_scene(source: &str) -> Result<Run, ErrorBox> {
    let mut sim = load_world_with_sdf_step(
        source,
        "folded_cloth.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let points = |sim: &gz_sim_core::Simulation| -> Points {
        bodies(sim)
            .next()
            .unwrap()
            .1
            .particles()
            .iter()
            .map(|p| p.position)
            .collect()
    };
    let initial = points(&sim);
    sim.set_paused(true);
    sim.tick()?;
    if points(&sim) != initial {
        return Err("paused sheet moved".into());
    }
    sim.set_paused(false);
    let mut samples = vec![initial.clone()];
    let mut gap = f64::INFINITY;
    let mut strain = 0.0_f64;
    let mut elapsed = Duration::ZERO;
    let mut loaded_distance = 0.;
    let mut loaded_time = 0.;
    let mut normal_impulse = 0.;
    let mut tangent_impulse = 0.;
    let mut friction_work = 0.;
    let mut previous = initial.clone();
    for tick in 1..=N {
        let start = Instant::now();
        sim.step_exact(1)
            .map_err(|e| format!("cloth tick {tick}: {e}; {:?}", sim.fault()))?;
        elapsed += start.elapsed();
        let receipt = bodies(&sim).next().unwrap().1.last_self_contact_impulse();
        normal_impulse += receipt.normal;
        tangent_impulse += receipt.tangent;
        friction_work += receipt.friction_work;
        let p = points(&sim);
        gap = gap.min(layer_clearance(&p));
        if p[..15] != initial[..15] {
            return Err("pinned vertices moved".into());
        }
        for i in 0..p.len() {
            for j in [
                i.checked_sub(3),
                i.checked_sub(1).filter(|_| i % 3 > 0),
                i.checked_sub(4).filter(|_| i % 3 > 0),
            ]
            .into_iter()
            .flatten()
            {
                let distance = |q: &Points| DVec3::from(q[i]).distance(q[j].into());
                strain = strain.max((distance(&p) / distance(&initial) - 1.).abs());
            }
        }
        for (&q, &old) in p[15..].iter().zip(&previous[15..]) {
            let loaded = |q: [f64; 3]| {
                q[0].abs() < 0.499
                    && (-0.899..0.509).contains(&q[1])
                    && ((q[2] - 0.65) - 0.03).abs() < 0.0001
            };
            if loaded(q) && loaded(old) {
                loaded_time += 0.002;
                loaded_distance += DVec3::from(q)
                    .truncate()
                    .distance(DVec3::from(old).truncate());
            }
        }
        previous.clone_from(&p);
        if tick % STRIDE == 0 {
            samples.push(p);
        }
    }
    sim.reset()?;
    if points(&sim) != initial {
        return Err("reset did not restore exact vertices".into());
    }
    sim.step_exact(STRIDE as u64)?;
    if points(&sim) != samples[1] {
        return Err("reset replay diverged at first sample".into());
    }
    Ok(Run {
        samples,
        gap,
        strain,
        elapsed,
        loaded_distance,
        loaded_time,
        normal_impulse,
        tangent_impulse,
        friction_work,
    })
}

fn main() -> Result<(), ErrorBox> {
    let args = Args::parse();
    if args.sliding {
        let (friction, smooth) = sliding_trials(2)?;
        if let Some(out) = args.output {
            export_panels(
                &out,
                &[
                    View {
                        samples: &friction.samples,
                        name: "friction_055",
                        offset: -0.87,
                        color: [0.92, 0.35, 0.055],
                    },
                    View {
                        samples: &smooth.samples,
                        name: "frictionless",
                        offset: 0.87,
                        color: [0.08, 0.66, 0.43],
                    },
                ],
            )?;
        }
        return Ok(());
    }
    let cloth = run(true)?;
    let pass = run(false)?;
    for (label, r) in [("swept", &cloth), ("disabled", &pass)] {
        println!(
            "{label}: {N} ticks; minimum upper triangle height over fixed lower patch {:.9} m, max |edge strain| {:.6}, physics {:.3} ms/tick",
            r.gap,
            r.strain,
            r.elapsed.as_secs_f64() * 1000. / N as f64
        );
    }
    if (cloth.gap - 0.03).abs() > 1e-6
        || !cloth.gap.is_finite()
        || pass.gap >= 0.
        || !pass.gap.is_finite()
        || cloth.strain > 0.01
    {
        return Err("crossing control or 30 mm clearance was not observed".into());
    }
    if let Some(output) = args.output {
        export(&output, &cloth.samples)?;
    }
    Ok(())
}

fn sliding_trials(substeps: u32) -> Result<(Run, Run), ErrorBox> {
    let xml = include_str!("../../../examples/sliding_folded_cloth.sdf")
        .replace("substeps=\"2\"", &format!("substeps=\"{substeps}\""));
    let friction = run_scene(&xml)?;
    let smooth = run_scene(&xml.replace("self_friction=\"0.55\"", "self_friction=\"0\""))?;
    for (label, r) in [("mu=0.55", &friction), ("mu=0", &smooth)] {
        println!(
            "{label}, {substeps} substeps: clearance {:.9} m; maximum strain {:.6}; near-contact travel {:.6} m / {:.3} vertex s = {:.6} m/s; normal impulse {:.7} N s; tangential impulse {:.7} N s; tangential dissipation {:.7} J; terminal tip {:?}; {:.3} ms/tick",
            r.gap,
            r.strain,
            r.loaded_distance,
            r.loaded_time,
            r.loaded_distance / r.loaded_time,
            r.normal_impulse,
            r.tangent_impulse,
            r.friction_work,
            r.samples.last().unwrap()[25],
            r.elapsed.as_secs_f64() * 1000. / N as f64
        );
    }
    if [(&friction, 0.55), (&smooth, 0.)]
        .into_iter()
        .any(|(r, mu)| {
            r.loaded_time <= 0.15
                || (r.gap - 0.03).abs() > 1e-6
                || r.strain > 0.06
                || r.tangent_impulse > mu * r.normal_impulse + 1e-12
        })
        || friction.friction_work <= 0.
        || smooth.friction_work != 0.
        || friction.loaded_distance / friction.loaded_time
            >= 0.8 * smooth.loaded_distance / smooth.loaded_time
    {
        return Err(
            "committed folded-layer clearance, Coulomb bound or measured sliding control failed"
                .into(),
        );
    }
    Ok((friction, smooth))
}

struct View<'a> {
    samples: &'a [Points],
    name: &'a str,
    offset: f64,
    color: [f32; 3],
}
fn export(out: &std::path::Path, samples: &[Points]) -> Result<(), ErrorBox> {
    export_panels(
        out,
        &[View {
            samples,
            name: "fold",
            offset: 0.,
            color: [0.92, 0.35, 0.055],
        }],
    )
}

fn export_panels(out: &std::path::Path, views: &[View<'_>]) -> Result<(), ErrorBox> {
    let count = views[0].samples.len();
    let mut assets = BTreeMap::new();
    let mut visuals = Vec::new();
    let mut models = Vec::new();
    let mut sequences = Vec::new();
    let pinned_triangles: Vec<_> = triangles().filter(|t| t.iter().all(|&i| i < 15)).collect();
    let moving_triangles: Vec<_> = triangles()
        .filter(|t| t.iter().any(|&i| i >= 15))
        .map(|t| t.map(|i| i - 12))
        .collect();
    for (g, view) in views.iter().enumerate() {
        let View {
            samples,
            name,
            offset,
            color,
        } = view;
        if samples.len() != count {
            return Err("comparison time grids differ".into());
        }
        let base_id = 10 + g * (count + 7);
        let (base_visual, pinned_visual, first_mesh, base_name) = if views.len() == 1 {
            (3, 9, 10, "default::display::base".to_owned())
        } else {
            (
                base_id + 1,
                base_id + 2,
                base_id + 3,
                format!("default::{name}::display_base"),
            )
        };
        let mesh = |id: usize, label: &str, asset: &str, z: f64| {
            json!({"id":id,"name":format!("default::{name}::{label}"),
            "translation":[offset,0,z],"rotation":[0,0,0,1],"color":[1,1,1,1],
            "shape":{"kind":"mesh","uri":format!("{EXPORT_ASSET_PREFIX}{asset}"),"scale":[1,1,1]}})
        };
        let pinned_asset = format!("/assets/{}/supported-panel.glb", g * (count + 1) + count);
        assets.insert(
            pinned_asset.clone(),
            Bytes::from(garment_geometry::surface_glb(
                "gz-sim-rs folded cloth",
                "supported section",
                &samples[0][..15],
                &pinned_triangles,
                [0.04, 0.40, 0.69],
            )?),
        );
        visuals.push(mesh(pinned_visual, "supported_section", &pinned_asset, 0.));
        visuals.push(json!({"id":base_visual,"name":base_name,"translation":[offset,0,0.07],"rotation":[0,0,0,1],"shape":{"kind":"box","size":[1.6,2.,0.94]},"color":[0.19,0.23,0.27,1.]}));
        let parked_z = -0.94;
        let start = visuals.len();
        for (k, pts) in samples.iter().enumerate() {
            if pts.iter().any(|q| {
                q[0].abs() >= 0.8 || q[1].abs() >= 1. || !(-0.4..0.54).contains(&(q[2] + parked_z))
            }) {
                return Err("hidden mesh exceeds display plinth".into());
            }
            let asset = format!("/assets/{}/sheet.glb", g * (count + 1) + k);
            assets.insert(
                asset.clone(),
                Bytes::from(garment_geometry::surface_glb(
                    "gz-sim-rs folded cloth",
                    "moving section",
                    &pts[12..],
                    &moving_triangles,
                    *color,
                )?),
            );
            visuals.push(mesh(
                first_mesh + k,
                &format!("sample_{k}"),
                &asset,
                parked_z,
            ));
        }
        // A small tracking marker follows the measured free-tip vertex in
        // the comparison. The lower marker identifies its seed footprint.
        let marker = if views.len() > 1 {
            let q = samples[0][25];
            let id = base_id + 3 + count;
            visuals.push(json!({"id":id,"name":format!("default::{name}::seed_footprint"),"translation":[q[0]+offset,q[1],0.663],"rotation":[0,0,0,1],"shape":{"kind":"sphere","radius":0.026},"color":[0.96,0.81,0.16,1.]}));
            let slot = visuals.len();
            visuals.push(json!({"id":id+1,"name":format!("default::{name}::measured_tip"),"translation":[q[0]+offset,q[1],q[2]],"rotation":[0,0,0,1],"shape":{"kind":"sphere","radius":0.031},"color":[1.,0.96,0.88,1.]}));
            Some(slot)
        } else {
            None
        };
        sequences.push((start, *offset, marker, g));
        if views.len() == 1 {
            models.extend([
                json!({"model_id":2,"name":"default::display","visual_ids":[3],"can_remove":false,"can_set_state":false}),
                json!({"model_id":4,"name":"default::fold","visual_ids":(9..10+count).collect::<Vec<_>>(),"can_remove":false,"can_set_state":false}),
            ]);
        } else {
            models.push(json!({"model_id":base_id,"name":format!("default::{name}"),"visual_ids":(base_id+1..base_id+3+count+2*usize::from(marker.is_some())).collect::<Vec<_>>(),"can_remove":false,"can_set_state":false}));
        }
    }
    let mut frames = Vec::new();
    for i in 0..count {
        let mut draw = visuals.clone();
        for &(start, offset, marker, g) in &sequences {
            draw[start + i]["translation"] = json!([offset, 0, 0]);
            if let Some(slot) = marker {
                let q = views[g].samples[i][25];
                draw[slot]["translation"] = json!([q[0] + offset, q[1], q[2]]);
            }
        }
        let ns = i * STRIDE * 2_000_000;
        let scene = json!({"version":2,"kind":"scene","world":"default","world_id":1,"epoch":0,"seconds":ns/1_000_000_000,"nanoseconds":ns%1_000_000_000,"sequence":i+1,"paused":false,"visuals":draw});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":i+1,"templates":[],"models":models});
        frames.push((scene.to_string(), catalog.to_string()));
    }
    export_archive(out, assets, frames)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loaded_friction_reduces_slip_at_refined_timesteps() {
        for n in [1, 2, 4] {
            sliding_trials(n).unwrap();
        }
    }
    #[test]
    fn committed_sheet_stops_at_thickness_and_disabled_control_crosses() {
        let with_contact = run(true).unwrap();
        let control = run(false).unwrap();
        assert_eq!(with_contact.samples.len(), 51);
        assert_eq!(with_contact.samples[0], control.samples[0]);
        assert!((with_contact.gap - 0.03).abs() < 1e-6);
        assert!(control.gap < -0.1);
        assert!(with_contact.strain < 0.01);
    }
}
