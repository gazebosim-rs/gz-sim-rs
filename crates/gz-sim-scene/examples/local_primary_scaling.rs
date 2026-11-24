// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sparse-contact receipt cost and authored detector cost in large, mostly static local worlds.
use gz_physics::{BodyId, BodySpec, DynamicBackend, RotationSpec, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, primary_contact::last_refinement,
};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    hint::black_box,
    path::PathBuf,
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const STEP: Duration = Duration::from_micros(500);
const SCENE: &str = include_str!("../../../examples/multifidelity_local_primary.sdf");
fn digest(hash: &mut u64, v: u64) {
    *hash ^= v;
    *hash = hash.wrapping_mul(0x100000001b3);
}
fn stats(ns: &[u64]) -> Value {
    let mut s = ns.to_vec();
    s.sort();
    json!({"p50_ns":s[s.len()/2],"p95_ns":s[(s.len()*95).div_ceil(100)-1],"samples_ns":ns})
}
fn spawn(
    s: &mut MultiFidelityBackend,
    center: [f64; 3],
    mass: Option<f64>,
    half: [f64; 3],
) -> Result<BodyId> {
    Ok(s.spawn_link(
        BodySpec {
            shape: Shape::Box { half_extents: half },
            position: center,
            velocity: [0.; 3],
            mass,
        },
        RotationSpec::default(),
        None,
    )?)
}
fn native(n: usize, reuse: bool, calls: u32) -> Result<Value> {
    let mut s = MultiFidelityBackend::new([0., 0., -9.81])?;
    let old = spawn(&mut s, [0., 5., -1.], None, [0.1; 3])?;
    for i in 0..n {
        spawn(
            &mut s,
            [1.5 * (i % 64) as f64, 10. + 1.5 * (i / 64) as f64, 0.],
            None,
            [0.1; 3],
        )?;
    }
    if reuse {
        assert!(s.remove(old));
    }
    let foot = spawn(&mut s, [0., 0., 0.051], Some(1.), [0.1, 0.1, 0.05])?;
    let pad = spawn(&mut s, [0., 0., -0.05], None, [0.5, 0.5, 0.05])?;
    for _ in 0..128 {
        s.step(STEP)?;
    }
    let mut samples = vec![];
    let mut contact = vec![];
    let mut hash = 0xcbf29ce484222325_u64;
    for rep in 0..145 {
        let t = Instant::now();
        let r = s.step_contact_interval(STEP, calls, 4)?;
        let ns = t.elapsed().as_nanos() as u64;
        let pair = r
            .pairs
            .iter()
            .find(|p| p.a == foot.min(pad) && p.b == foot.max(pad))
            .ok_or("missing sparse contact")?;
        if r.native_steps != calls || !pair.active_at_end || pair.normal_impulse <= 0. {
            return Err("invalid sparse native receipt".into());
        }
        for v in [
            pair.normal_impulse.to_bits(),
            pair.first_positive_end.unwrap().as_nanos() as u64,
        ] {
            digest(&mut hash, v);
        }
        let b = s.state(foot).unwrap();
        for v in b
            .position
            .into_iter()
            .chain(b.orientation)
            .chain(b.velocity)
            .chain(b.angular_velocity)
        {
            digest(&mut hash, v.to_bits());
        }
        let t = Instant::now();
        for _ in 0..8 {
            black_box(s.rigid_contact_observations());
        }
        let query_ns = (t.elapsed().as_nanos() / 8) as u64;
        if rep > 16 {
            samples.push(ns);
            contact.push(query_ns);
        }
    }
    Ok(
        json!({"static_extras":n,"recycled_handle":reuse,"native_calls":calls,"native_digest":format!("{hash:016x}"),"tick":stats(&samples),"query":stats(&contact)}),
    )
}
fn sdf(n: usize, extra_pairs: usize, policy: bool) -> String {
    let mut s = SCENE
        .replace("<pose>0 0 0.20 0 0.06 0", "<pose>0 0 0.07 0 0 0")
        .replace("<pose>0.18 0 -0.06", "<pose>0 0 -0.06")
        .replace("<linear_velocity>0 0 0.08", "<linear_velocity>0 0 0");
    let mut bodies = String::new();
    let mut pairs = String::new();
    for i in 0..n {
        bodies.push_str(&format!(r#"<model name="fixed_{i}"><static>true</static><pose>{} {} 0 0 0 0</pose><link name="pad"><collision name="c"><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision></link></model>"#,1.5*(i%64)as f64,10.+1.5*(i/64)as f64));
        if i < extra_pairs {
            pairs.push_str(&format!("<pair><foot>default::rocker::foot</foot><support>default::fixed_{i}::pad</support></pair>"));
        }
    }
    s = s.replace("</world>", &format!("{bodies}</world>"));
    if policy {
        let at = s.find("</pair>").unwrap() + "</pair>".len();
        s.insert_str(at, &pairs);
    } else {
        let start = s
            .find("    <plugin name=\"gz::sim::systems::PrimaryContactRefinement\"")
            .unwrap();
        let end = start + s[start..].find("</plugin>").unwrap() + "</plugin>".len();
        s.replace_range(start..end, "");
    }
    s
}
fn load(s: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        s,
        "scale.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn bevy(n: usize, pairs: usize, policy: bool) -> Result<Value> {
    let mut sim = load(&sdf(n, pairs, policy))?;
    // Keep the active contact but no release during the measured interval.
    sim.step_exact(600)?;
    let last = last_refinement(&sim).map(|r| r.iteration);
    #[allow(deprecated)]
    let foot = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::rocker::foot")
        })
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id();
    let mut samples = vec![];
    let mut hash = 0xcbf29ce484222325_u64;
    for rep in 0..145 {
        let start = Instant::now();
        sim.step_exact(1)?;
        let ns = start.elapsed().as_nanos() as u64;
        let b = sim
            .world()
            .resource::<gz_physics::PhysicsBackend>()
            .state(foot)
            .unwrap();
        for v in b
            .position
            .into_iter()
            .chain(b.orientation)
            .chain(b.velocity)
            .chain(b.angular_velocity)
        {
            digest(&mut hash, v.to_bits());
        }
        if rep > 16 {
            samples.push(ns);
        }
    }
    if last_refinement(&sim).map(|r| r.iteration) != last {
        return Err("settled foot selected an interval in timing window".into());
    }
    Ok(
        json!({"static_extras":n,"authored_pairs":if policy{pairs+1}else{0},"native_digest":format!("{hash:016x}"),"tick":stats(&samples)}),
    )
}
fn main() -> Result<()> {
    if std::env::args_os().nth(1).as_deref() == Some(std::ffi::OsStr::new("--viewer")) {
        let path = std::env::args_os()
            .nth(2)
            .ok_or("local_primary_scaling --viewer OUTPUT_SDF")?;
        let s=sdf(32,32,true).replace("</collision></link></model>",r#"</collision><visual name="v"><geometry><box><size>0.2 0.2 0.2</size></box></geometry><material><diffuse>0.3 0.5 0.6 1</diffuse></material></visual></link></model>"#);
        fs::write(path, s)?;
        return Ok(());
    }
    let path = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: local_primary_scaling RESULTS_JSON")?,
    );
    let mut sparse = vec![];
    let mut gated = vec![];
    for n in [0, 32, 256, 1024] {
        for calls in [1, 8] {
            for reuse in [false, true] {
                sparse.push(native(n, reuse, calls)?);
            }
        }
        gated.push(bevy(n, 0, false)?);
        gated.push(bevy(n, 0, true)?);
        if n > 0 {
            gated.push(bevy(n, n.min(128), true)?);
        }
    }
    fs::write(
        path,
        serde_json::to_vec(&json!({"native":sparse,"bevy":gated}))?,
    )?;
    Ok(())
}
