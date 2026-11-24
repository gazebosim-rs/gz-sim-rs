// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cross fractional landing times with support geometry, friction and insertion order.
#[path = "carry.rs"]
pub(super) mod carry;
use super::*;
use gz_physics_multifidelity::{ContactSegment, ContactWindow, NormalApproach};
use std::collections::BTreeMap;
use std::io::BufReader;
const PHASES: [u64; 11] = [
    0, 1, 15_625, 62_499, 62_501, 138_197, 250_000, 312_501, 437_499, 484_375, 499_999,
];
const N: usize = FIELD * PHASES.len();
const TAG: &str = "native-fractional-touchdown-v1";

#[derive(Clone, Copy)]
enum Policy {
    Uniform(u32, usize),
    Near {
        margin: u64,
        spacing: u64,
        relax: usize,
    },
}
const MODES: [(&str, Policy); 8] = [
    ("ordinary", Policy::Uniform(1, 1)),
    ("uniform8_r1", Policy::Uniform(8, 1)),
    ("uniform8_r8", Policy::Uniform(8, 8)),
    ("uniform8_r32", Policy::Uniform(8, 32)),
    (
        "near8_r32",
        Policy::Near {
            margin: 8_000,
            spacing: 4_000,
            relax: 32,
        },
    ),
    (
        "near32_r32",
        Policy::Near {
            margin: 32_000,
            spacing: 8_000,
            relax: 32,
        },
    ),
    (
        "near64_r8",
        Policy::Near {
            margin: 64_000,
            spacing: 16_000,
            relax: 8,
        },
    ),
    ("uniform256_r32", Policy::Uniform(256, 32)),
];
fn budget(r: usize) -> ContactSolverIterations {
    ContactSolverIterations {
        temporal: 4,
        pgs: None,
        relaxation: Some(r),
    }
}
impl Policy {
    fn schedule(self, foot: BodyState, pad: BodyState) -> Result<Vec<ContactSegment>> {
        match self {
            Self::Uniform(n, r) => Ok(vec![ContactSegment {
                duration: DT,
                native_steps: n,
                iterations: budget(r),
            }]),
            Self::Near {
                margin,
                spacing,
                relax,
            } => {
                let gap = foot.position[2] - FOOT[2] - pad.position[2] - PAD[2];
                let approach = NormalApproach {
                    gap,
                    closing_speed: pad.velocity[2] - foot.velocity[2],
                    closing_acceleration: 0.,
                };
                let touch = approach.touchdown(DT)?;
                let plan = ContactWindow {
                    before: Duration::from_nanos(margin),
                    after: Duration::from_nanos(margin),
                    max_native_dt: Duration::from_nanos(spacing),
                    ordinary: budget(1),
                    refined: budget(relax),
                };
                Ok(plan.schedule(DT, touch.as_slice())?)
            }
        }
    }
}
fn fixture(index: usize) -> (Case, Duration) {
    // Permute phase within each geometry row so short pilots include every phase.
    let geometry = index / PHASES.len();
    let phase = (index % PHASES.len() + geometry % PHASES.len()) % PHASES.len();
    (Case::at(geometry), Duration::from_nanos(PHASES[phase]))
}
#[derive(Clone, Copy, Serialize, Deserialize)]
struct Outcome {
    impulse: f64,
    spin: f64,
    com: f64,
    first_ns: u64,
    calls: u32,
}
fn run(c: Case, touch: Duration, policy: Policy, capture: bool) -> Result<(Outcome, Vec<Value>)> {
    let Rig {
        mut native,
        foot,
        upper,
        pad,
        velocity: incoming,
    } = Rig::arriving(c, Model::Locked, touch)?;
    let lower = native.state(foot).unwrap();
    let support = native.state(pad).unwrap();
    let plan = policy.schedule(lower, support)?;
    let target = c.analytic()?;
    let mut frames = vec![];
    let mut outcome = None;
    let mut frame = |tick: u64, b: [BodyState; 2], support: BodyState, normal: f64, calls: u32| {
        frames.push(json!({"tick":tick,"time_us":tick*500,"foot":as_json(b[0]),"upper":as_json(b[1]),"pad":as_json(support),"native_steps":calls,"solver_iterations":4,"normal_impulse_ns":normal}));
    };
    let b = links(lower, upper.and_then(|k| native.state(k)));
    let com0 = (DVec3::from(b[0].position) + DVec3::from(b[1].position)) * 0.5;
    if capture {
        frame(0, b, support, 0., 0);
    }
    for tick in 1..=if capture { 16 } else { 1 } {
        let ordinary;
        let work = if tick == 1 {
            plan.as_slice()
        } else {
            ordinary = [ContactSegment {
                duration: DT,
                native_steps: 1,
                iterations: budget(1),
            }];
            &ordinary
        };
        let (r, _) = native.step_contact_schedule(DT, work, &[])?;
        let b = links(
            native.state(foot).unwrap(),
            upper.and_then(|k| native.state(k)),
        );
        let pair = r
            .pairs
            .iter()
            .find(|p| (p.a == foot && p.b == pad) || (p.b == foot && p.a == pad));
        let normal = pair.map_or(0., |p| p.normal_impulse);
        if tick == 1 {
            let j = 2. * (velocity(b) - incoming);
            let target_j = DVec3::new(target.impulse[0], 0., target.impulse[1]);
            let com = (DVec3::from(b[0].position) + DVec3::from(b[1].position)) * 0.5;
            let exact_com =
                com0 + incoming * DT.as_secs_f64() + 0.5 * target_j * (DT - touch).as_secs_f64();
            outcome = Some(Outcome {
                impulse: j.distance(target_j),
                spin: DVec3::from(b[0].angular_velocity).distance(DVec3::Y * target.pitch_rate),
                com: com.distance(exact_com),
                first_ns: pair
                    .and_then(|p| p.first_positive_end)
                    .map_or(u64::MAX, |t| t.as_nanos() as u64),
                calls: r.native_steps,
            });
        }
        if capture {
            frame(tick, b, native.state(pad).unwrap(), normal, r.native_steps);
        }
    }
    Ok((outcome.unwrap(), frames))
}
#[derive(Serialize, Deserialize)]
struct Batch {
    format: String,
    first: usize,
    end: usize,
    rows: Vec<[Outcome; MODES.len()]>,
}
pub(super) fn campaign(out: &Path, first: usize, count: usize) -> Result<()> {
    let end = first
        .checked_add(count)
        .filter(|v| *v <= N && count > 0)
        .ok_or("invalid fractional census range")?;
    let dir = out.join("batches");
    fs::create_dir_all(&dir)?;
    for i in (first..end).step_by(64) {
        let stop = (i + 64).min(end);
        let f = dir.join(format!("batch-{i:06}-{stop:06}.json"));
        if f.exists() {
            let b: Batch = serde_json::from_reader(BufReader::new(fs::File::open(f)?))?;
            if b.format != TAG || b.first != i || b.end != stop || b.rows.len() != stop - i {
                return Err("invalid saved fractional batch".into());
            }
            continue;
        }
        let mut rows = vec![];
        for k in i..stop {
            let (c, t) = fixture(k);
            let mut samples = vec![];
            for (_, p) in MODES {
                samples.push(run(c, t, p, false)?.0);
            }
            rows.push(samples.try_into().map_err(|_| "profile count")?);
        }
        let tmp = f.with_extension(format!("partial-{}", std::process::id()));
        let mut w = BufWriter::new(fs::File::create(&tmp)?);
        serde_json::to_writer(
            &mut w,
            &Batch {
                format: TAG.into(),
                first: i,
                end: stop,
                rows,
            },
        )?;
        w.flush()?;
        drop(w);
        fs::rename(tmp, f)?;
        if (i - first).is_multiple_of(1024) {
            eprintln!("fractional fixtures through {stop} of {end}");
        }
    }
    Ok(())
}

pub(super) fn replay(out: &Path, index: usize) -> Result<()> {
    if index >= N {
        return Err("fractional case outside lattice".into());
    }
    let (c, t) = fixture(index);
    let dir = out.join(format!("case-{index:06}"));
    fs::create_dir_all(dir.join("traces"))?;
    let mut lanes = vec![];
    let mut outcomes = vec![];
    for (label, p) in MODES {
        let keep = ["ordinary", "near8_r32", "uniform256_r32"].contains(&label);
        let (o, frames) = run(c, t, p, keep)?;
        outcomes.push(json!({"label":label,"actual":o}));
        if keep {
            write_json(
                &dir.join("traces").join(format!("{label}.json")),
                &json!({"geometry":{"foot_half_extents":FOOT,"upper_half_extents":TOP,"pad_half_extents":PAD},"frames":frames}),
            )?;
            lanes.push(json!({"name":match label{"ordinary"=>"Ordinary full tick","near8_r32"=>"Predicted ±8 µs",_=>"Uniform 256 calls"},"label":label,"file":format!("traces/{label}.json")}));
        }
    }
    write_json(
        &dir.join("comparison.json"),
        &json!({"title":"Landing between actuator updates","pair_totals_only":true,"index":index,"fixture":c,"touch_ns":t.as_nanos() as u64,"target":describe(c)?,"outcomes":outcomes,"lanes":lanes}),
    )
}
#[derive(Default)]
struct Stats {
    j: Vec<(f64, usize)>,
    w: Vec<(f64, usize)>,
    r: Vec<(f64, usize)>,
    lag: Vec<(f64, usize)>,
    calls: u64,
    miss: usize,
    early: usize,
}
impl Stats {
    fn push(&mut self, o: Outcome, k: usize, t: u64) {
        self.j.push((o.impulse, k));
        self.w.push((o.spin, k));
        self.r.push((o.com, k));
        self.calls += u64::from(o.calls);
        if o.first_ns == u64::MAX {
            self.miss += 1;
        } else {
            self.early += usize::from(o.first_ns < t);
            self.lag.push(((o.first_ns.abs_diff(t)) as f64, k));
        }
    }
    fn report(mut self) -> Value {
        fn stats(v: &mut [(f64, usize)]) -> Value {
            if v.is_empty() {
                return Value::Null;
            }
            v.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            let n = v.len();
            json!({"median":v[n.div_ceil(2)-1].0,"p95":v[(95*n).div_ceil(100)-1].0,"max":v[n-1].0,"max_case":v[n-1].1})
        }
        json!({"n":self.j.len(),"first_positive_before_geometric_touch":self.early,"no_positive_on_touch_tick":self.miss,"mean_native_calls":self.calls as f64/self.j.len() as f64,"impulse_ns":stats(&mut self.j),"spin_rad_s":stats(&mut self.w),"com_m":stats(&mut self.r),"first_load_absolute_time_difference_ns":stats(&mut self.lag)})
    }
}
pub(super) fn summary(out: &Path) -> Result<()> {
    let mut seen = vec![false; N];
    let mut groups: BTreeMap<(String, usize), Stats> = BTreeMap::new();
    for f in fs::read_dir(out.join("batches"))? {
        let f = f?.path();
        if f.extension().is_none_or(|p| p != "json") {
            continue;
        }
        let b: Batch = serde_json::from_reader(BufReader::new(fs::File::open(f)?))?;
        if b.format != TAG || b.end > N || b.first >= b.end || b.end - b.first != b.rows.len() {
            return Err("invalid fractional batch".into());
        }
        for (k, row) in (b.first..b.end).zip(b.rows) {
            if std::mem::replace(&mut seen[k], true) {
                return Err("overlapping fractional batches".into());
            }
            let (c, t) = fixture(k);
            let mode = if c.analytic()?.bearing == Bearing::Face {
                "face"
            } else {
                "edge"
            };
            for (i, o) in row.into_iter().enumerate() {
                for group in [
                    "all".to_string(),
                    mode.into(),
                    format!("phase_{}ns", t.as_nanos()),
                ] {
                    groups
                        .entry((group, i))
                        .or_default()
                        .push(o, k, t.as_nanos() as u64);
                }
            }
        }
    }
    let count = seen.iter().filter(|&&v| v).count();
    let rows = groups
        .into_iter()
        .map(|((s, i), v)| json!({"group":s,"profile":MODES[i].0,"statistics":v.report()}))
        .collect::<Vec<_>>();
    write_json(
        &out.join("summary.json"),
        &json!({"format":TAG,"cases":count,"total":N,"complete":count==N,"rows":rows}),
    )
}

/// Time only the predictor and native interval on a fresh cold world. The
/// independently checked physical lattice supplies each input; construction
/// and post-solve readback stay outside the timer.
pub(super) fn cost(out: &Path) -> Result<()> {
    const CASES: usize = 1024;
    let mut times = vec![vec![]; MODES.len()];
    let mut checks = vec![0.; MODES.len()];
    let mut totals = vec![0_u64; MODES.len()];
    for i in 0..CASES + 64 {
        let k = (i * 109_913) % N;
        let (c, t) = fixture(k);
        for j in 0..MODES.len() {
            let m = (j * 5 + i) % MODES.len();
            let mut rig = Rig::arriving(c, Model::Locked, t)?;
            let start = std::time::Instant::now();
            let plan = MODES[m].1.schedule(
                rig.native.state(rig.foot).unwrap(),
                rig.native.state(rig.pad).unwrap(),
            )?;
            let (r, _) = rig.native.step_contact_schedule(DT, &plan, &[])?;
            let ns = start.elapsed().as_nanos() as u64;
            if i >= 64 {
                times[m].push(ns);
                totals[m] += u64::from(r.native_steps);
                checks[m] += r.pairs.iter().map(|p| p.normal_impulse).sum::<f64>();
            }
        }
    }
    let rows=times.into_iter().enumerate().map(|(i,mut v)|{
        v.sort_unstable();
        let quantile=|n:usize|v[(n*v.len()).div_ceil(100)-1];
        json!({"profile":MODES[i].0,"samples":v.len(),"median_ns":quantile(50),"p95_ns":quantile(95),"p99_ns":quantile(99),"max_ns":v.last(),"total_native_calls":totals[i],"normal_sum_for_repeat_ns":checks[i]})
    }).collect::<Vec<_>>();
    write_json(
        &out.join("cost.json"),
        &json!({"scope":"cold arrival tick, live state queries + predictor + solver interval receipt","warmup_per_profile":64,"rows":rows}),
    )
}
