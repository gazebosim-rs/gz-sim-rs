// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify deterministic native world staging in a 2,048-environment Bevy physics schedule.
#[allow(dead_code)]
#[path = "support/sphere_pair/reference.rs"]
mod pair;
#[allow(dead_code)]
#[path = "support/sphere_plane/reference.rs"]
mod plane;
use gz_physics_gpu::{
    BodySlot, CentralImpulse, Environment, EnvironmentState as State, MixedBatch,
    PairSolver::{NativeRapier, SpecializedGpu},
    STEP,
    simulation::{BatchTiming, install_mixed},
};
use gz_sim_core::Simulation;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const TOTAL: usize = 2048;
const NATIVE: usize = 96;
const CAPS: [usize; 3] = [1, 2, 4];
fn fixture() -> (Vec<Environment>, Vec<u64>) {
    let spheres = plane::fixture(TOTAL);
    let pairs = pair::fixture(TOTAL);
    let envs = (0..TOTAL)
        .map(|i| Environment {
            id: i as u64,
            state: if i % 2 == 0 {
                State::SpherePlane(spheres[i])
            } else {
                State::SpherePair(pairs[i])
            },
        })
        .collect::<Vec<_>>();
    let ids = (0..TOTAL)
        .map(|i| (i * 787 + 19) % TOTAL)
        .filter(|&i| envs[i].validate_native_pair().is_ok())
        .take(NATIVE)
        .map(|i| i as u64)
        .collect();
    (envs, ids)
}
fn new(
    adapter: &str,
    mut all: Vec<Environment>,
    ids: &[u64],
    cap: usize,
    overlap: bool,
) -> Result<Simulation> {
    if cap == 2 {
        all.reverse();
    } else if cap == 4 {
        all.rotate_left(89);
    }
    let marked = ids
        .iter()
        .map(|&id| (id, SpecializedGpu))
        .collect::<Vec<_>>();
    let batch = MixedBatch::from_fixed_scene(adapter, all)?.with_guarded_native_pairs(&marked)?;
    let batch = if overlap {
        batch.with_overlapped_native_staging(cap)?
    } else {
        batch.with_native_staging(cap)?
    };
    assert_eq!(batch.overlaps_native_gpu(), overlap);
    assert_eq!(batch.native_staging_workers(), cap);
    let mut sim = Simulation::new(STEP)?;
    install_mixed(&mut sim, batch)?;
    Ok(sim)
}
enum Oracle {
    Pair(Box<pair::Native>),
    Plane(Box<plane::Native>),
}
impl Oracle {
    fn new(v: State) -> Self {
        match v {
            State::SpherePlane(s) => Self::Plane(Box::new(plane::Native::with_substeps(s, 16))),
            State::SpherePair(s) => Self::Pair(Box::new(pair::Native::new(s))),
            _ => unreachable!(),
        }
    }
    fn kick(&mut self, j: [f64; 3]) {
        match self {
            Self::Plane(r) => r.impulse_a(j),
            Self::Pair(r) => r.impulse_a(j),
        }
    }
    fn compare(&mut self, state: State) -> [f64; 5] {
        match (self, state) {
            (Self::Plane(r), State::SpherePlane(s)) => {
                r.step();
                let q = r.state;
                let n = plane::V::from(s.plane_normal);
                let mut d = body(
                    (s.position, s.velocity, s.angular_velocity, s.orientation),
                    (q.position, q.velocity, q.angular_velocity, q.orientation),
                );
                d[4] = (n * (s.normal_impulse - q.normal_impulse)
                    + plane::V::from(s.tangential_impulse)
                    - plane::V::from(q.tangential_impulse))
                .length();
                d
            }
            (Self::Pair(r), State::SpherePair(s)) => {
                r.step();
                let mut d = [0f64; 5];
                for (a, b) in [(s.a, r.state.a), (s.b, r.state.b)] {
                    let values = body(
                        (a.position, a.velocity, a.angular_velocity, a.orientation),
                        (b.position, b.velocity, b.angular_velocity, b.orientation),
                    );
                    for (v, x) in d.iter_mut().zip(values) {
                        *v = v.max(x);
                    }
                }
                d[4] = (pair::V::from(s.contact_normal) * s.normal_impulse
                    + pair::V::from(s.tangential_impulse)
                    - r.impulse_b)
                    .length();
                d
            }
            _ => unreachable!(),
        }
    }
}
type Frame = ([f64; 3], [f64; 3], [f64; 3], [f64; 4]);
fn body((p, v, w, q): Frame, (pr, vr, wr, qr): Frame) -> [f64; 5] {
    let distance = |a: [f64; 3], b: [f64; 3]| plane::V::from(a).distance(plane::V::from(b));
    [
        distance(p, pr),
        distance(v, vr),
        distance(w, wr),
        q.into_iter()
            .zip(qr)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt(),
        0.,
    ]
}
fn qualify(worlds: &mut [Simulation], ids: &[u64]) -> Result<()> {
    let mut controls = ids
        .iter()
        .map(|&id| {
            Oracle::new(
                worlds[0].world().resource::<MixedBatch>().environments()[id as usize].state,
            )
        })
        .collect::<Vec<_>>();
    for sim in worlds.iter_mut() {
        assert_eq!(
            sim.world_mut()
                .resource_mut::<MixedBatch>()
                .set_pair_solver(0, ids, NativeRapier)?
                .moved,
            ids.len()
        );
    }
    let mut peak = [0f64; 5];
    let (mut plane_contact, mut pair_contact, mut inputs) = (0, 0, 0);
    for t in 0..360 {
        if t % 47 == 17 {
            let actions = ids
                .iter()
                .enumerate()
                .filter(|(i, _)| (i + t as usize).is_multiple_of(9))
                .map(|(i, &id)| {
                    let j = [0.002, -0.003, 0.004];
                    controls[i].kick(j);
                    CentralImpulse {
                        id,
                        body: BodySlot::A,
                        impulse: j,
                    }
                })
                .collect::<Vec<_>>();
            inputs += actions.len();
            for sim in worlds.iter_mut() {
                sim.world_mut()
                    .resource_mut::<MixedBatch>()
                    .queue_impulses(t, &actions)?;
            }
        }
        for sim in worlds.iter_mut() {
            sim.step_exact(1)?;
        }
        let serial = worlds[0].world().resource::<MixedBatch>();
        for (i, &id) in ids.iter().enumerate() {
            let state = serial.environments()[id as usize].state;
            for (k, x) in controls[i].compare(state).into_iter().enumerate() {
                peak[k] = peak[k].max(x);
            }
            if let Some(c) = serial.native_contact(id)?
                && c.first_impulse_interval.is_some()
            {
                if matches!(state, State::SpherePlane(_)) {
                    plane_contact += 1;
                } else {
                    pair_contact += 1;
                }
            }
        }
        for parallel in &worlds[1..] {
            let actual = parallel.world().resource::<MixedBatch>();
            for (a, b) in actual.environments().iter().zip(serial.environments()) {
                assert_eq!(a.id, b.id);
                assert!(
                    a.state.values().iter().map(|x| x.to_bits()).eq(b
                        .state
                        .values()
                        .iter()
                        .map(|x| x.to_bits())),
                    "id={} tick={} workers={}",
                    a.id,
                    t + 1,
                    actual.native_staging_workers()
                );
            }
            for &id in ids {
                assert_eq!(actual.native_contact(id)?, serial.native_contact(id)?);
            }
        }
    }
    assert!(plane_contact > 1000 && pair_contact > 10 && inputs > 30);
    assert!(
        peak.iter()
            .zip([1e-8, 1e-8, 1e-7, 1e-8, 1e-8])
            .all(|(v, b)| *v < b),
        "raw Rapier comparison {peak:?}"
    );
    println!(
        "bit-identical records across 1, 2 and 4 workers, all 2048 rows × 360 epochs; {inputs} inputs; {plane_contact} supported and {pair_contact} free-pair impulse ticks; 96 independent raw Rapier reference peaks [m,m/s,rad/s,quaternion,Ns]={peak:?}"
    );
    Ok(())
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("provide ADAPTER NEW_DIR")?;
    let dir = args.next().ok_or("provide new artifact directory")?;
    let mode = args.next();
    let overlapped = mode.as_deref() == Some("--overlap-oracle");
    if mode.is_some() && !overlapped || args.next().is_some() {
        return Err("unexpected argument".into());
    }
    std::fs::create_dir(&dir)?;
    let dir = Path::new(&dir);
    let (all, ids) = fixture();
    let mut worlds = vec![];
    for n in CAPS {
        let t = Instant::now();
        worlds.push(new(&adapter, all.clone(), &ids, n, overlapped && n > 1)?);
        println!(
            "workers={n} construction_ms={:.3}",
            t.elapsed().as_secs_f64() * 1000.
        );
    }
    if overlapped {
        worlds.push(new(&adapter, all, &ids, 1, true)?);
    }
    qualify(&mut worlds, &ids)?;
    if overlapped {
        println!(
            "CPU-first serial and GPU-first native staging with 1/2/4 workers matched every committed record."
        );
        return Ok(());
    }
    let mut csv = BufWriter::new(File::create_new(dir.join("steps.csv"))?);
    writeln!(
        csv,
        "rep,native,workers,tick,wall_ns,native_stage_ns,gpu_dispatch_ns,upload_B,read_B"
    )?;
    let mut summary = BufWriter::new(File::create_new(dir.join("timing.csv"))?);
    writeln!(
        summary,
        "rep,native,workers,p50_us,p95_us,p99_us,mean_us,env_steps_s"
    )?;
    for rep in 0..3 {
        let order = match rep {
            0 => [0, 4, 24, 96],
            1 => [96, 24, 4, 0],
            _ => [24, 0, 96, 4],
        };
        for (case, n) in order.into_iter().enumerate() {
            for j in 0..3 {
                let k = (j + rep + case) % 3;
                let cap = CAPS[k];
                let sim = &mut worlds[k];
                sim.reset()?;
                {
                    let mut batch = sim.world_mut().resource_mut::<MixedBatch>();
                    assert_eq!(batch.native_staging_workers(), cap);
                    batch.set_pair_solver(0, &ids[..n], NativeRapier)?;
                }
                let mut times = vec![];
                for t in 1..=210 {
                    let start = Instant::now();
                    sim.step_exact(1)?;
                    let elapsed = start.elapsed().as_nanos();
                    let m = sim.world().resource::<BatchTiming>().last.unwrap();
                    writeln!(
                        csv,
                        "{rep},{n},{cap},{t},{elapsed},{},{},{},{}",
                        m.cpu_solve.as_nanos(),
                        m.gpu_dispatch.map_or(0, |v| v.as_nanos()),
                        m.uploaded_bytes,
                        m.readback_bytes
                    )?;
                    if t > 30 {
                        times.push(elapsed as f64 * 0.001);
                    }
                }
                times.sort_by(f64::total_cmp);
                let mean = times.iter().sum::<f64>() / times.len() as f64;
                let rate = TOTAL as f64 * 1e6 / mean;
                writeln!(
                    summary,
                    "{rep},{n},{cap},{},{},{},{mean},{rate}",
                    times[89], times[170], times[178]
                )?;
                println!(
                    "rep={rep} native={n} workers={cap} whole p50/p95/p99_us={:.1}/{:.1}/{:.1} env_steps/s={rate:.0}",
                    times[89], times[170], times[178]
                );
            }
        }
    }
    Ok(())
}
