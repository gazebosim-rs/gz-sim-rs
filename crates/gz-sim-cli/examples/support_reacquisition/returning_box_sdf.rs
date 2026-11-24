// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Observe geometry and native loads from ordinary SDF/Bevy steps, including declined queries.
use super::*;
use gz_sim_scene::{
    load_world_with_sdf_step,
    primary_contact::{last_refinement, last_returning_box_features},
};
use std::path::Path;
const SOURCE: &str = include_str!("../../../../examples/multifidelity_local_primary.sdf");
const FLAG: &str = "<refresh_returning_box_features>true</refresh_returning_box_features>";
fn source(offset: f64, roll: f64, pitch: f64, speed: f64, option: &str) -> String {
    SOURCE.replace("<pose>0.18 0 -0.06",&format!("<pose>{offset:.17} 0 -0.06"))
        .replace("<pose>0 0 0.20 0 0.06 0",&format!("<pose>0 0 0.20 {roll:.17} {pitch:.17} 0"))
        .replace("<linear_velocity>0 0 0.08",&format!("<linear_velocity>0 0 {speed:.17}"))
        .replace("<solver_iterations>8</solver_iterations>","<solver_iterations>4</solver_iterations><relaxation_iterations>8</relaxation_iterations>")
        .replace("</support></pair>",&format!("</support>{option}</pair>"))
}
fn load(sdf: &str) -> Result<Lane> {
    Lane::from_sim(load_world_with_sdf_step(
        sdf,
        "returning_geometry.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn returned(l: &Lane) -> Result<Option<Value>> {
    let Some(r) = last_returning_box_features(&l.sim).filter(|v| v.sim_time == l.sim.elapsed())
    else {
        return Ok(None);
    };
    let [p] = r.pairs.as_slice() else {
        return Err("wrong authored pair count".into());
    };
    if (p.foot, p.support) != (l.foot, l.pad)
        || p.counts.requery > 1
        || p.counts.stronger_plane != p.counts.requery
    {
        return Err("invalid native return receipt".into());
    }
    let scope = last_refinement(&l.sim)
        .filter(|v| v.iteration == r.iteration)
        .ok_or("geometry receipt outside a nominated landing")?;
    if scope.interval.native_steps != 8 || scope.requested_pairs.len() != 1 {
        return Err("geometry changed the fixed grid".into());
    }
    let pair = scope.requested_pairs[0];
    let c = p.counts;
    Ok(Some(
        json!({"tick":r.iteration,"tests":c.unloaded_tests,"queries":c.requery,"normal_ns":pair.normal_impulse,"first_ns":pair.first_positive_end.map(|x|x.as_nanos() as u64),"first_witness":c.first_witness.map(|w|json!({"other_origin_in_first":w.other_origin_in_first,"other_rotation_in_first":w.other_rotation_in_first,"half_extents":w.half_extents,"cached_axis":w.cached_axis,"stronger_axis":w.stronger_axis,"gap_m":w.gap_m,"tolerance_m":w.tolerance_m,"unloaded_points":w.unloaded_points}))}),
    ))
}
fn frame(l: &Lane, tick: u64) -> Result<Value> {
    let s = l.sample()?;
    let step = last_refinement(&l.sim).filter(|r| r.iteration == tick);
    let n = step.map_or(s.normal_impulse_ns, |r| r.requested_pairs[0].normal_impulse);
    Ok(
        json!({"tick":tick,"time_us":tick*500,"native_steps":if tick==0{0}else{step.map_or(1,|r|r.interval.native_steps)},"foot":s.foot,"upper":s.upper,"pad":s.pad,"normal_impulse_ns":n}),
    )
}
fn save(p: &Path, v: &Value) -> Result<()> {
    fs::write(p, serde_json::to_vec(v)?)?;
    Ok(())
}
pub(super) fn run(dir: &Path) -> Result<()> {
    fs::create_dir_all(dir)?;
    let mut cases = vec![];
    let mut selected = None;
    // A deterministic field of finite footprints, initial attitudes and two support entry speeds.
    for (oi, offset) in [0.13, 0.16, 0.19, 0.22, 0.25, 0.28].into_iter().enumerate() {
        for (ri, roll) in [-0.08, 0., 0.08].into_iter().enumerate() {
            for (pi, pitch) in [-0.14, -0.04, 0.04, 0.14].into_iter().enumerate() {
                for (si, speed) in [0.08, 0.45].into_iter().enumerate() {
                    let index = ((oi * 3 + ri) * 4 + pi) * 2 + si;
                    let sdf = source(offset, roll, pitch, speed, FLAG);
                    let mut lane = load(&sdf)?;
                    let mut events = vec![];
                    for _ in 0..660 {
                        lane.sim.step_exact(1)?;
                        if let Some(e) = returned(&lane)? {
                            events.push(e);
                        }
                    }
                    if selected.is_none() && let Some(hit) = events.iter().find(|e| {
                            e["queries"] == 1 && e["normal_ns"].as_f64().is_some_and(|n| n > 0.)
                        }) {
                        selected = Some((index, sdf, hit["tick"].as_u64().unwrap()));
                    }
                    cases.push(json!({"source":index,"offset":offset,"roll":roll,"pitch":pitch,"speed":speed,"events":events}));
                }
            }
        }
        eprintln!("checked {} authored returning-box fixtures", cases.len());
    }
    save(
        &dir.join("authored-cases.json"),
        &json!({"cases":cases,"selected":selected.as_ref().map(|r|r.0)}),
    )?;
    let Some((_, sdf, hit)) = selected else {
        return Ok(());
    };
    fs::create_dir_all(dir.join("traces"))?;
    let mut baseline = load(&sdf.replace(FLAG, ""))?;
    let mut candidate = load(&sdf)?;
    let mut a = vec![];
    let mut b = vec![];
    for tick in 0..=hit + 20 {
        if tick > 0 {
            baseline.sim.step_exact(1)?;
            candidate.sim.step_exact(1)?;
        }
        let x = frame(&baseline, tick)?;
        let y = frame(&candidate, tick)?;
        if tick < hit && x != y {
            return Err("source choice diverged before first registered requery".into());
        }
        if tick + 4 >= hit {
            a.push(x);
            b.push(y);
        }
    }
    fs::write(dir.join("scene.sdf"), &sdf)?;
    let mut lanes = vec![];
    for (id, title, frames) in [
        ("ordinary", "Ordinary native geometry", a),
        (
            "registered",
            "One independently disproved unloaded plane",
            b,
        ),
    ] {
        let file = format!("traces/{id}.json");
        save(
            &dir.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"label":id,"name":title,"file":file}));
    }
    save(
        &dir.join("comparison.json"),
        &json!({"title":"Rechecking an unloaded contact plane in an authored world","pair_totals_only":true,"lanes":lanes,"capture_ticks":{"first_proved_refresh":hit}}),
    )?;
    Ok(())
}
