// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Standalone oracle for the frictional relaxation census, both stored policies and actual plot cells.
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::Path,
};
type Key = (usize, usize, bool, usize); // offset, speed, previous holds, stroke
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
#[path = "shared/continuation.rs"]
mod continuation;
#[path = "shared/signatures.rs"]
mod signatures;
#[path = "shared/svg_cells.rs"]
mod svg_cells;
type Policy = (u64, u64, bool);
const R1: Policy = (8, 1, false);
const R8: Policy = (8, 8, false);
const ALT: Policy = (4096, 1, false);
const FINE: Policy = (4096, 8, false);
const CANDS: [Policy; 3] = [R8, (8, 16, false), (8, 8, true)];
const SENS: [(Policy, Policy); 4] = [
    ((256, 8, false), (1024, 8, false)),
    ((1024, 8, false), FINE),
    ((1024, 8, false), (1024, 16, false)),
    (ALT, FINE),
];
const END: [u64; 3] = [1, 10, 26];
const NAMES: [&str; 3] = ["primary", "at_5ms", "at_13ms"];
fn val(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn arr(v: &Value) -> Result<&[Value]> {
    v.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing array".into())
}
fn u(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "missing integer".into())
}
fn d(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "missing measurement".into())
}
fn vec3(v: &Value) -> Result<[f64; 3]> {
    Ok([d(&v[0])?, d(&v[1])?, d(&v[2])?])
}
fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2])
}
fn eq(lhs: f64, rhs: f64) -> Result<()> {
    if (lhs - rhs).abs() > 1e-10 {
        return Err(format!("expected {lhs} got {rhs}").into());
    }
    Ok(())
}
fn pol(p: &Value) -> Result<Policy> {
    Ok((
        u(&p["calls"])?,
        u(&p["relaxation"])?,
        p["onset_only"].as_bool().ok_or("onset")?,
    ))
}
fn name((c, r, o): Policy) -> String {
    format!("{c}x4_r{r}{}", if o { "_onset" } else { "" })
}
fn q(a: &[f64], pct: usize) -> f64 {
    let mut v = a.to_vec();
    v.sort_unstable_by(f64::total_cmp);
    v[(v.len() * pct).div_ceil(100) - 1]
}
#[derive(Default)]
struct Aggregate {
    data: BTreeMap<String, Vec<f64>>,
}
impl Aggregate {
    fn push(&mut self, k: &str, v: f64) {
        self.data.entry(k.to_owned()).or_default().push(v);
    }
    fn check(&self, s: &Value) -> Result<usize> {
        let mut n = 0;
        for (k, values) in &self.data {
            let stat = &s[k];
            assert_eq!(u(&stat["n"])? as usize, values.len());
            assert_eq!(
                u(&stat["negative"])? as usize,
                values.iter().filter(|&&v| v < 0.).count()
            );
            assert_eq!(
                u(&stat["positive"])? as usize,
                values.iter().filter(|&&v| v > 0.).count()
            );
            eq(q(values, 50), d(&stat["p50"])?)?;
            eq(q(values, 95), d(&stat["p95"])?)?;
            eq(
                values.iter().sum::<f64>() / values.len() as f64,
                d(&stat["mean"])?,
            )?;
            n += values.len();
        }
        Ok(n)
    }
}
#[derive(Clone, Copy)]
struct E {
    pos: [f64; 3],
    jn: f64,
    total: f64,
    tangent: [f64; 3],
    res: Option<f64>,
}
#[derive(Clone, Copy, Debug)]
enum V {
    Pending,
    Disjoint,
    Num(f64),
}
type Scores = [[V; 8]; 2];
fn main() -> Result<()> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if !(2..=3).contains(&args.len()) {
        return Err("readback RELAXATION_DIR VIEW [ORIGINAL_SURVEY]".into());
    }
    let dir = Path::new(&args[0]);
    let out = Path::new(&args[1]);
    let s = val(&out.join("relaxation-summary.json"))?;
    let study = s["study"].as_bool().ok_or("study type")?;
    assert_eq!(study, args.len() == 2);
    let mut dataset = vec![];
    let mut joined = BTreeSet::new();
    for n in arr(&s["original_grid_matched_indices"])? {
        joined.insert(u(n)?);
    }
    if study {
        let raw = val(&dir.join("study.json"))?;
        for case in arr(&raw["cases"])? {
            dataset.push(case.clone());
        }
    }
    let mut values = BTreeMap::<(Policy, Policy, usize), Aggregate>::new();
    let mut effects = BTreeMap::<(Policy, Policy, usize), Vec<f64>>::new();
    let mut signed = BTreeMap::<(Policy, usize), Vec<(f64, f64)>>::new();
    let mut known = BTreeSet::new();
    let mut lattice = BTreeMap::<Key, Scores>::new();
    let mut ids = BTreeMap::new();
    let mut historical_values = BTreeMap::new();
    let mut outlier = None;
    let mut field_amounts = [vec![], vec![]];
    let mut signature_data = BTreeMap::new();
    let mut postcontact = BTreeMap::new();
    let check_continuation = !study && out.join("relaxation-continuation.json").exists();
    let mut case_ids = vec![];
    for id in arr(&s["completed_indices"])? {
        case_ids.push(u(id)?);
    }
    let mut cut_count = 0usize;
    let mut same_cuts = 0usize;
    let mut rounded_states = 0usize;
    let mut common_loads = 0;
    let mut cumulative_metrics = 0;
    for id in &case_ids {
        let (case, original) = if study {
            (dataset[*id as usize].clone(), None)
        } else {
            let f = format!("case-{id:04}.json");
            let c = val(&dir.join("cases").join(&f))?;
            let o = if joined.contains(id) {
                Some(val(&Path::new(&args[2]).join("cases").join(f))?["result"].clone())
            } else {
                None
            };
            (c["result"].clone(), o)
        };
        let f = &case["fixture"];
        let mm = (d(&f["offset_m"])? * 1000.).round() as usize;
        let speed = (d(&f["speed_scale"])? * 100.).round() as usize;
        let holds = f["switched"].as_bool().ok_or("holds")?;
        known.insert((mm, speed, holds));
        if let Some(o) = &original {
            assert_eq!(o["fixture"], case["fixture"]);
            assert_eq!(o["first_disjoint_tick"], case["first_disjoint_tick"]);
        }
        for t in arr(&case["trials"])? {
            let stroke = u(&t["stroke"])? as usize;
            let key = (mm, speed, holds, stroke);
            let cut = u(&t["cut"]["tick"])?;
            cut_count += 1;
            let mut g = BTreeMap::<Policy, (Option<u64>, [E; 3])>::new();
            let mut add = |p: Policy, lost: Option<u64>, samples: &Value| -> Result<()> {
                let e = |w| -> Result<E> {
                    let a = arr(samples)?
                        .iter()
                        .find(|s| s["i"] == END[w])
                        .ok_or("missing clock")?;
                    Ok(E {
                        pos: vec3(&a["com_m"])?,
                        jn: d(&a["normal_ns"])?,
                        total: d(&a["cumulative_normal_ns"])?,
                        tangent: vec3(&a["cumulative_tangent_ns"])?,
                        res: a["impulse_weighted_abs_post_normal_m_s"].as_f64(),
                    })
                };
                assert!(g.insert(p, (lost, [e(0)?, e(1)?, e(2)?])).is_none());
                Ok(())
            };
            for p in arr(&t["paths"])? {
                add(
                    pol(&p["profile"])?,
                    p["first_disjoint_tick"].as_u64(),
                    &p["samples"],
                )?;
            }
            if let Some(o) = &original {
                let t2 = arr(&o["trials"])?
                    .iter()
                    .find(|x| x["stroke"] == stroke)
                    .ok_or("unmatched donor")?;
                assert_eq!(t2["cut"], t["cut"]);
                let shared = arr(&t["paths"])?
                    .iter()
                    .find(|p| pol(&p["profile"]).is_ok_and(|p| p == R1))
                    .ok_or("missing native eight-solve driver")?;
                let previous = arr(&t2["paths"])?
                    .iter()
                    .find(|p| p["method"] == "onset" && p["calls"] == 8 && p["iterations"] == 4)
                    .ok_or("missing earlier eight-solve driver")?;
                let common_time = |x: &Value| x.as_u64().filter(|t| *t <= 500_000);
                let first_time = common_time(&shared["first"]["end_ns"]);
                assert_eq!(
                    first_time,
                    common_time(&previous["record"]["first_loaded_native_end_ns"])
                );
                common_loads += usize::from(first_time.is_some());
                let endpoint = |p: &Value| -> Result<Value> {
                    Ok(arr(p)?
                        .iter()
                        .find(|x| x["i"] == 1)
                        .ok_or("onset sample")?
                        .clone())
                };
                let first = endpoint(&shared["samples"])?;
                let prior = endpoint(&previous["record"]["samples"])?;
                let mut exact = true;
                for field in ["com_m", "velocity_m_s", "spin_rad_s", "orientation_xyzw"] {
                    let a = arr(&first[field])?;
                    let b = arr(&prior[field])?;
                    assert_eq!(a.len(), b.len());
                    for (x, y) in a.iter().zip(b) {
                        let x = x.as_f64().ok_or("numeric native state")?;
                        let y = y.as_f64().ok_or("numeric native state")?;
                        assert!((x - y).abs() <= 8. * f64::EPSILON * x.abs().max(y.abs()).max(1.));
                        exact &= x == y;
                    }
                }
                rounded_states += usize::from(!exact);
                for field in [
                    "normal_ns",
                    "cumulative_normal_ns",
                    "cumulative_tangent_ns",
                    "cumulative_corners_ns",
                ] {
                    assert_eq!(first[field], prior[field]);
                }

                let path = arr(&t2["paths"])?
                    .iter()
                    .find(|p| {
                        p["method"] == "uniform" && p["calls"] == 4096 && p["iterations"] == 4
                    })
                    .ok_or("no default grid")?;
                add(ALT, None, &path["record"]["samples"])?;
                same_cuts += 1;
            } else if study {
                assert!(g.contains_key(&ALT));
                same_cuts += 1;
            }
            let mut sig = signatures::raw(&t["cut"], d(&f["offset_m"])?)?;
            let mut continuation = if check_continuation {
                Some(continuation::receipt(t, cut)?)
            } else {
                None
            };
            let mut scores = [[V::Pending; 8]; 2];
            for (w, tick) in END.into_iter().enumerate() {
                let get = |p: Policy| -> std::result::Result<E, V> {
                    let (lost, ends) = g.get(&p).ok_or(V::Pending)?;
                    if lost.is_some_and(|k| k <= cut + tick) {
                        return Err(V::Disjoint);
                    }
                    Ok(ends[w])
                };
                let distance = |a, b| Ok::<f64, V>(1000. * dist(get(a)?.pos, get(b)?.pos));
                let change = |a, r| Ok::<f64, V>(distance(a, r)? - distance(R1, r)?);
                for &p in g.keys().filter(|&&p| p != ALT) {
                    for reference in [FINE, ALT] {
                        let (Ok(a), Ok(b)) = (get(p), get(reference)) else {
                            continue;
                        };
                        if study && w > 0 {
                            historical_values
                                .insert((w, key, p, reference), dist(a.pos, b.pos) * 1000.);
                        }
                        let a2 = values.entry((p, reference, w)).or_default();
                        for (k, v) in [
                            ("com_mm", dist(a.pos, b.pos) * 1000.),
                            ("cumulative_normal_delta_ns", a.total - b.total),
                            ("cumulative_normal_abs_delta_ns", (a.total - b.total).abs()),
                            ("cumulative_tangent_distance_ns", dist(a.tangent, b.tangent)),
                            ("endpoint_normal_delta_ns", a.jn - b.jn),
                            ("endpoint_normal_ns", a.jn),
                            ("cumulative_normal_ns", a.total),
                        ] {
                            a2.push(k, v);
                        }
                        if let Some(v) = a.res {
                            a2.push("endpoint_weighted_abs_post_normal_m_s", v);
                        }
                        if let Some((a, b)) = a.res.zip(b.res) {
                            a2.push("endpoint_weighted_residual_delta_m_s", a - b);
                        }
                    }
                }
                for p in CANDS {
                    for r in [FINE, ALT] {
                        if let Ok(delta) = change(p, r) {
                            effects.entry((p, r, w)).or_default().push(delta);
                        }
                    }
                    if let (Ok(a), Ok(b)) = (change(p, FINE), change(p, ALT)) {
                        signed.entry((p, w)).or_default().push((a, b));
                    }
                }
                if w > 0 {
                    for (ri, refer) in [FINE, ALT].into_iter().enumerate() {
                        if let Some(ref mut c) = continuation {
                            c.errors[w - 1][ri] =
                                match (distance((8, 8, true), refer), distance(R8, refer)) {
                                    (Ok(a), Ok(b)) => Ok([a, b]),
                                    (Err(V::Disjoint), _) | (_, Err(V::Disjoint)) => {
                                        Err(V::Disjoint)
                                    }
                                    _ => Err(V::Pending),
                                };
                        }
                        sig.errors[w - 1][ri] = match (distance(R1, refer), distance(R8, refer)) {
                            (Ok(a), Ok(b)) => Ok([a, b]),
                            (Err(V::Disjoint), _) | (_, Err(V::Disjoint)) => Err(V::Disjoint),
                            _ => Err(V::Pending),
                        };
                    }
                    let list = [
                        change(R8, FINE),
                        change(R8, ALT),
                        change((8, 16, false), FINE),
                        change((8, 8, true), FINE),
                        distance(SENS[0].0, SENS[0].1),
                        distance(SENS[1].0, SENS[1].1),
                        distance(SENS[2].0, SENS[2].1),
                        distance(SENS[3].0, SENS[3].1),
                    ];
                    for (j, v) in list.into_iter().enumerate() {
                        scores[w - 1][j] = v.map_or_else(
                            |e| e,
                            |x| {
                                field_amounts[usize::from(j >= 4)].push(x.abs());
                                V::Num(x)
                            },
                        );
                    }
                }
                if w == 2
                    && let Ok(v) = change(R8, FINE)
                {
                    let candidate = (v, *id, stroke);
                    if outlier.is_none_or(|old: (f64, u64, usize)| {
                        v > old.0 || v == old.0 && (*id, stroke) < (old.1, old.2)
                    }) {
                        outlier = Some(candidate);
                    }
                }
            }
            if let Some(c) = continuation {
                assert!(postcontact.insert(key, c).is_none());
            }
            signature_data.insert(key, sig);
            lattice.insert(key, scores);
            ids.insert(key, (*id, cut));
        }
    }
    assert_eq!(u(&s["available_landings"])? as usize, cut_count);
    assert_eq!(
        u(&s["unavailable_landings_in_completed_fixtures"])? as usize,
        case_ids.len() * 3 - cut_count
    );
    if let Some((score, id, stroke)) = outlier {
        assert_eq!(
            (id, stroke),
            (
                u(&s["outlier"]["site"]["index"])?,
                u(&s["outlier"]["site"]["stroke"])? as usize
            )
        );
        eq(score, d(&s["outlier"]["against_4096_r8"]["change_mm"])?)?;
    }
    for ((p, r, w), v) in &values {
        let refer = if *r == FINE {
            "against_4096_r8"
        } else {
            "against_4096_r1"
        };
        cumulative_metrics += v.check(&s["profiles"][name(*p)][refer][NAMES[*w]])?;
    }
    for ((p, r, w), v) in effects {
        let refer = if r == FINE {
            "against_4096_r8_mm"
        } else {
            "against_4096_r1_mm"
        };
        let stat = &s["signed_com_change_vs_8x4_r1"][name(p)][NAMES[w]][refer];
        eq(q(&v, 95), d(&stat["p95"])?)?;
        assert_eq!(
            u(&stat["negative"])? as usize,
            v.iter().filter(|&&x| x < 0.).count()
        );
    }
    for ((p, w), v) in signed {
        let s = &s["signed_com_change_vs_8x4_r1"][name(p)][NAMES[w]];
        assert_eq!(
            u(&s["opposite_nonzero_sign"])? as usize,
            v.iter().filter(|(a, b)| a * b < 0.).count()
        );
    }
    let cell_file = fs::read_to_string(out.join("relaxation-cells.jsonl"))?;
    let mut seen = BTreeSet::new();
    for line in cell_file.lines() {
        let c: Value = serde_json::from_str(line)?;
        let x = &c["site"];
        let key = (
            u(&x["offset_mm"])? as usize,
            u(&x["drive_percent"])? as usize,
            x["holds"].as_bool().unwrap(),
            u(&x["stroke"])? as usize,
        );
        assert!(seen.insert(key));
        signatures::check_features(
            signature_data.get_mut(&key).ok_or("missing cut")?,
            &c["incoming_kinematics"],
        )?;
        assert_eq!(ids[&key], (u(&x["index"])?, u(&x["cut_tick"])?));
        let scores = lattice[&key];
        for (w, slot) in scores.into_iter().enumerate() {
            for (j, score) in slot.into_iter().enumerate() {
                let x = if j < 4 {
                    let p = match j {
                        0 | 1 => R8,
                        2 => (8, 16, false),
                        _ => (8, 8, true),
                    };
                    let r = if j == 1 {
                        "against_4096_r1_mm"
                    } else {
                        "against_4096_r8_mm"
                    };
                    &c["signed_com_change_mm"][name(p)][r][w + 1]
                } else {
                    &c["grid_and_relaxation_distances"][j - 4]["metrics"][NAMES[w + 1]]["com_mm"]
                };
                match score {
                    V::Num(v) => eq(v, d(x)?)?,
                    _ => assert!(x.is_null()),
                }
            }
        }
    }
    assert_eq!(seen.len(), lattice.len());
    let mut squares = 0;
    if !study {
        let names = [
            "effect-8x4_r8-ref-4096x4_r8",
            "effect-8x4_r8-ref-4096x4_r1",
            "effect-8x4_r16-ref-4096x4_r8",
            "effect-8x4_r8_onset-ref-4096x4_r8",
            "distance-256x4_r8-1024x4_r8",
            "distance-1024x4_r8-4096x4_r8",
            "distance-1024x4_r8-1024x4_r16",
            "distance-4096x4_r1-4096x4_r8",
        ];
        for (j, label) in names.into_iter().enumerate() {
            let vals = &field_amounts[usize::from(j >= 4)];
            let limit = if vals.is_empty() {
                1e-9
            } else {
                q(vals, 95).max(1e-9)
            };
            for (w, t) in [5, 13].into_iter().enumerate() {
                svg_cells::check(
                    &out.join(format!("{label}-{t}ms.svg")),
                    |key @ (mm, s, h, _)| match lattice.get(&key).map(|o| o[w][j]) {
                        Some(V::Num(v)) => svg_cells::color(v, limit, j < 4),
                        Some(V::Disjoint) => "#6B2F7C".into(),
                        Some(V::Pending) => "#8E7D5F".into(),
                        None if known.contains(&(mm, s, h)) => "#4F4E53".into(),
                        None => "#D9DBDF".into(),
                    },
                )?;
                squares += 3750;
            }
        }
    }
    assert_eq!(
        u(&s["identical_native_primary_outcomes_across_drivers"])? as usize,
        if study { 0 } else { same_cuts - rounded_states }
    );
    let mut historical_tiles = 0;
    if study {
        let policies = [
            R1,
            (8, 4, false),
            R8,
            (8, 16, false),
            (8, 8, true),
            (16, 1, false),
            (16, 8, false),
            (16, 8, true),
            (64, 8, false),
            (256, 8, false),
            (1024, 8, false),
            FINE,
            (256, 16, false),
            (1024, 16, false),
        ];
        let mut coords = ids.keys().copied().collect::<Vec<_>>();
        coords.sort_by_key(|k| (k.2, k.1, k.3));
        for (w, t) in [(1, 5), (2, 13)] {
            let vals = historical_values
                .iter()
                .filter(|((i, _, _, _), _)| *i == w)
                .map(|(_, v)| *v)
                .collect::<Vec<_>>();
            let limit = q(&vals, 95).max(1e-9);
            let svg = fs::read_to_string(out.join(format!("historical-{t}ms.svg")))?;
            let mut tiles = vec![];
            let mut ys = BTreeSet::new();
            let mut xs = BTreeSet::new();
            let lines = svg.lines().collect::<Vec<_>>();
            for (i, line) in lines.iter().enumerate().filter(|(_, l)| {
                l.starts_with("<rect ") && l.contains("width=\"86\"") && l.contains("height=\"23\"")
            }) {
                let x = svg_cells::attr(line, "x")?.parse::<u32>()?;
                let y = svg_cells::attr(line, "y")?.parse::<u32>()?;
                let color = svg_cells::attr(line, "fill")?;
                assert!(lines[i + 1].starts_with("<text "));
                tiles.push((x, y, color, lines[i + 2]));
                xs.insert(x);
                ys.insert(y);
            }
            let xs = xs.into_iter().collect::<Vec<_>>();
            let ys = ys.into_iter().collect::<Vec<_>>();
            assert_eq!(
                (xs.len(), ys.len(), tiles.len()),
                (coords.len(), 28, coords.len() * 28)
            );
            for (x, y, color, text) in tiles {
                let j = ys.binary_search(&y).unwrap();
                let reference = if j < 14 { FINE } else { ALT };
                let p = policies[j % 14];
                let k = coords[xs.binary_search(&x).unwrap()];
                if let Some(v) = historical_values.get(&(w, k, p, reference)) {
                    assert_eq!(text, format!("{v:.3}"));
                    assert_eq!(color, svg_cells::color(*v, limit, false));
                } else {
                    assert_eq!(text, "Disjoint");
                }
                historical_tiles += 1;
            }
        }
    }
    if !study {
        assert_eq!(
            common_loads as u64,
            u(&s["checked_shared_first_ticks_with_load"])?
        );
        assert_eq!(
            (same_cuts - common_loads) as u64,
            u(&s["checked_shared_first_ticks_without_load"])?
        );
    }
    let kinematics = if !study {
        Some(signatures::verify(out, &signature_data)?)
    } else {
        None
    };
    let continuation = if check_continuation {
        Some(continuation::verify(out, &postcontact, &ids)?)
    } else {
        None
    };
    if let Some(other) = s.get("nonbitwise_pose_or_twist") {
        assert_eq!(arr(other)?.len(), rounded_states);
    }
    let report = json!({"transform_twist_roundoff_pairs":rounded_states,"fixtures":case_ids.len(),"continuation":continuation,"kinematics":kinematics,"matched_landings":cut_count,"matched_default_cuts":same_cuts,"physics_values_used_in_summary_checks":cumulative_metrics,"independent_source_tolerance":1e-10,
    "physically_addressed_factorial_svg_cells_checked":squares,"historical_svg_cells_checked":historical_tiles,"outlier_checked":outlier.is_some()});
    fs::write(
        out.join("relaxation-readback.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    Ok(())
}
