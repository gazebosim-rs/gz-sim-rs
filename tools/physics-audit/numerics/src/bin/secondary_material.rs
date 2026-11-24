// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent material laws and first-paid native responses at finite support returns.
use glam::{DQuat, DVec3};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs,
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn chk(b: bool, e: &str) -> Result<()> {
    if b { Ok(()) } else { Err(e.into()) }
}
fn n(x: &Value) -> Result<f64> {
    x.as_f64()
        .filter(|v| v.is_finite())
        .ok_or_else(|| "missing finite measurement".into())
}
fn v(x: &Value) -> Result<DVec3> {
    let r = DVec3::from_array(serde_json::from_value(x.clone())?);
    chk(r.is_finite(), "nonfinite vector")?;
    Ok(r)
}
fn q(x: &Value) -> Result<DQuat> {
    let r = DQuat::from_array(serde_json::from_value(x.clone())?);
    chk((r.length() - 1.).abs() < 1e-11, "unnormalized rotation")?;
    Ok(r.normalize())
}
fn a(x: &Value) -> Result<&[Value]> {
    x.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing samples".into())
}
fn dist(mut x: Vec<f64>) -> Value {
    x.sort_by(f64::total_cmp);
    let k = x.len();
    let rank = |p: f64| {
        if k > 0 {
            Some(x[(p * k as f64).ceil() as usize - 1])
        } else {
            None
        }
    };
    json!({"n":k,"median":rank(0.5),"p95":rank(0.95),"max":x.last()})
}

#[derive(Clone, Copy)]
struct Cuboid {
    p: DVec3,
    r: DQuat,
    h: DVec3,
}
impl Cuboid {
    fn vertices(self) -> [DVec3; 8] {
        std::array::from_fn(|i| {
            self.p
                + self.r
                    * (self.h
                        * DVec3::from_array(std::array::from_fn(|j| {
                            if i & (1 << j) == 0 { -1. } else { 1. }
                        })))
        })
    }
    fn surface(self, p: DVec3) -> bool {
        let local = (self.r.inverse() * (p - self.p)).abs() - self.h;
        local.max_element().abs() < 3e-10
    }
}
fn gaps(f: Cuboid, p: Cuboid) -> Vec<(f64, DVec3)> {
    let (fv, pv) = (f.vertices(), p.vertices());
    let dirs = [DVec3::X, DVec3::Y, DVec3::Z];
    let a = dirs.map(|v| f.r * v);
    let b = dirs.map(|v| p.r * v);
    a.into_iter()
        .chain(b)
        .chain(a.into_iter().flat_map(|u| b.map(|v| u.cross(v))))
        .filter_map(|d| {
            if d.length() < 1e-10 {
                return None;
            }
            let mut n = d.normalize();
            if n.dot(f.p - p.p) < 0. {
                n = -n
            }
            let one = fv.iter().map(|p| p.dot(n)).fold(f64::INFINITY, f64::min);
            let two = pv
                .iter()
                .map(|p| p.dot(n))
                .fold(f64::NEG_INFINITY, f64::max);
            Some((one - two, n))
        })
        .collect()
}
fn maximum(x: &[(f64, DVec3)]) -> f64 {
    x.iter().map(|r| r.0).fold(f64::NEG_INFINITY, f64::max)
}
fn law(s: &Value) -> Result<Value> {
    let material = &s["material"];
    chk(
        material["status"] == "unique",
        "this panel expects a uniquely qualified event; preserve and audit any further censored input",
    )?;
    let t = n(&material["time_s"])?;
    let links = a(&s["incoming"]["motion"]["links"])?;
    let [foot, upper, pad]: [&Value; 3] = [&links[0], &links[1], &links[2]];
    let initially = v(&foot["position"])?;
    let center = (initially + v(&upper["position"])?) * 0.5;
    let drive = v(&s["pad_command_m_s"])?;
    let speed = (v(&foot["velocity"])? + v(&upper["velocity"])?) * 0.5 - drive;
    let w = v(&foot["angular_velocity"])?;
    let gravity = v(&s["incoming"]["motion"]["gravity"])?;
    let rot = q(&foot["orientation"])?;
    let deck = Cuboid {
        p: v(&pad["position"])?,
        r: q(&pad["orientation"])?,
        h: DVec3::new(0.33, 0.18, 0.06),
    };
    let c = |t: f64| center + speed * t + gravity * (0.5 * t * t);
    let body = |t: f64| {
        let dq = DQuat::from_scaled_axis(w * t);
        Cuboid {
            p: c(t) + dq * (initially - center),
            r: dq * rot,
            h: DVec3::new(0.18, 0.1, 0.06),
        }
    };
    let interval = a(&material["integer_bracket_ns"])?;
    let l = n(&interval[0])? * 1e-9;
    let r = n(&interval[1])? * 1e-9;
    chk(
        t >= l - 1e-16 && t <= r + 1e-16 && (r - l) < 1.00001e-9,
        "invalid crossing clock",
    )?;
    chk(
        maximum(&gaps(body(l), deck)) >= -2e-13 && maximum(&gaps(body(r), deck)) <= 2e-13,
        "vertex separation does not support crossing",
    )?;
    let site = &material["site"];
    let f = body(t);
    let j = v(&material["response"]["impulse_ns"])?;
    let nrm = v(&site["normal"])?;
    let pt = v(&site["point"])?;
    let candidates = gaps(f, deck);
    chk(
        maximum(&candidates).abs() < 3e-13
            && candidates
                .iter()
                .any(|(gap, n)| gap.abs() < 3e-13 && n.distance(nrm) < 1e-9),
        "data chose a non-material normal",
    )?;
    chk(
        f.surface(pt) && deck.surface(pt) && pt.distance(v(&site["support_point"])?) < 4e-12,
        "impulse site is not common to both finite surfaces",
    )?;
    let r = pt - c(t);
    let before = speed + gravity * t + w.cross(r);
    let delta = j / 2. + r.cross(j).cross(r) / 0.05;
    let after = before + delta;
    let normal = j.dot(nrm);
    let tangent = j - normal * nrm;
    chk(
        normal > 0.
            && before.dot(nrm) < 0.
            && after.dot(nrm).abs() < 2e-11
            && before.distance(v(&material["response"]["before_relative_point_m_s"])?) < 3e-12
            && after.distance(v(&material["response"]["after_relative_point_m_s"])?) < 3e-12,
        "normal restitution or incoming velocity mismatch",
    )?;
    if material["response"]["sticking"] == true {
        chk(
            after.length() < 3e-11 && tangent.length() <= 0.5 * normal + 1e-12,
            "invalid sticking law",
        )?;
    } else {
        let slip = after - nrm * after.dot(nrm);
        chk(
            slip.length() > 1e-10 && (tangent + 0.5 * normal * slip.normalize()).length() < 3e-11,
            "invalid sliding direction or magnitude",
        )?;
    }
    let work = -0.5 * j.dot(before + after);
    let loss = n(&material["response"]["loss_j"])?;
    chk(
        work > 0.
            && (work - loss).abs() < 3e-11
            && r.cross(j)
                .distance(v(&material["response"]["angular_impulse_nms"])?)
                < 3e-12,
        "incorrect balance at point impact",
    )?;
    // The other box starts far above the pad. Bound its motion from translational and angular speed over the whole 500 µs flight.
    let upper_box = Cuboid {
        p: 2. * c(t) - f.p,
        r: f.r,
        h: DVec3::new(0.05, 0.06, 0.06),
    };
    let upper_gap = maximum(&gaps(upper_box, deck));
    chk(
        (upper_gap - n(&site["upper_separation_m"])?).abs() < 1e-11 && upper_gap > 0.05,
        "unqualified upper link clearance",
    )?;
    let mut end_checks = 0;
    let mut peak_work_error: f64 = 0.;
    for r in a(&s["targets"])? {
        if r["qualified"] != true {
            continue;
        }
        let end = &r["model"]["end"];
        let et = n(&r["time_ns"])? * 1e-9;
        let impulse = v(&end["total_impulse_ns"])?;
        let moment = v(&end["angular_impulse_nms"])?;
        let error = 2. * (v(&end["velocity_m_s"])? - drive - speed - gravity * et) - impulse;
        let angular = 0.05 * (v(&end["spin_rad_s"])? - w) - moment;
        chk(
            error.length() < 1e-8
                && angular.length() < 1e-9
                && (error.length() - n(&end["linear_residual_ns"])?).abs() < 1e-11,
            "whole-body momentum balance did not close",
        )?;
        let relative_velocity = v(&end["velocity_m_s"])? - drive;
        let spin = v(&end["spin_rad_s"])?;
        let delta_kinetic = relative_velocity.length_squared() - speed.length_squared()
            + 0.025 * (spin.length_squared() - w.length_squared());
        let gravity_work = 2. * gravity.dot(v(&end["com_m"])? - drive * et - center);
        let work_error = (delta_kinetic - gravity_work - n(&end["contact_work_j"])?).abs();
        chk(
            work_error < 1e-10,
            "support-frame energy balance did not close",
        )?;
        peak_work_error = peak_work_error.max(work_error);
        let mesh = a(&r["mesh_difference"])?;
        chk(
            n(&mesh[0])? < 1e-6 && n(&mesh[1])? < 1e-8 && n(&mesh[2])? < 1e-6,
            "inadmissible mesh error",
        )?;
        if et < t {
            chk(
                impulse.length() == 0. && r["model"]["status"] == "flight",
                "early mechanical reaction",
            )?;
        }
        end_checks += 1;
    }
    Ok(
        json!({"onset_s":t,"feature":material["feature"],"sticking":material["response"]["sticking"],"mode":material["response"]["outgoing_mode"],"normal":nrm.to_array(),"impulse_ns":j.to_array(),"energy_j":work,"independent_upper_gap_m":upper_gap,"qualified_native_or_common_times":end_checks,"peak_support_frame_energy_residual_j":peak_work_error}),
    )
}
fn score(s: &Value, t: &Value, method: &str) -> Result<Value> {
    let c = t["calls"].as_u64().ok_or("missing grid")?;
    let pass = t["passes"].as_u64().ok_or("missing relaxation")?;
    let start = n(&s["material"]["time_s"])? * 1e9;
    let point = v(&s["material"]["response"]["impulse_ns"])?;
    let endpoint = |where_ns: u64| {
        a(&s["targets"])?
            .iter()
            .find(|r| r["time_ns"] == where_ns)
            .ok_or_else(|| Box::<dyn Error>::from("missing reference sample"))
    };
    let total = v(&t["world_impulse_ns"])?;
    let target = endpoint(500_000)?;
    let mut r = json!({"calls":c,"passes":pass,"graph":t["graph"],"method":method,"public_model":target["model"]["status"],"public_impulse_ns":total.to_array()});
    if target["qualified"] == true {
        let z = &target["model"]["end"];
        r["public_vector_error_ns"] = json!(total.distance(v(&z["total_impulse_ns"])?));
        r["public_com_error_um"] = json!(v(&t["end_com_m"])?.distance(v(&z["com_m"])?) * 1e6);
        r["public_spin_error_rad_s"] =
            json!(v(&t["end_spin_rad_s"])?.distance(v(&z["spin_rad_s"])?));
    }
    let first = &t["first_positive"];
    if first.is_object() {
        let ns = first["elapsed_ns"]
            .as_u64()
            .ok_or("missing first paid end")?;
        let dt = first["native_dt_ns"]
            .as_u64()
            .ok_or("missing first paid duration")?;
        chk(t["first_ns"] == ns && ns >= dt, "invalid first time")?;
        let z = &first["integrals"];
        let j = v(&z["normal_vector_ns"])? + v(&z["tangent_vector_ns"])?;
        let dp = |state: &Value| -> Result<DVec3> {
            Ok(v(&state[0]["velocity"])? + v(&state[1]["velocity"])?)
        };
        let gravity = v(&s["incoming"]["motion"]["gravity"])?;
        let native_residual =
            (dp(&first["after"])? - dp(&first["before"])? - 2. * gravity * (dt as f64 * 1e-9) - j)
                .length();
        let measured_rows =
            a(&first["paid_points"])?
                .iter()
                .try_fold(DVec3::ZERO, |sum, p| -> Result<_> {
                    Ok(sum + v(&p["normal_vector_ns"])? + v(&p["tangent_vector_ns"])?)
                })?;
        chk(
            measured_rows.distance(j) < 2e-12
                && native_residual <= n(&t["peak_native_linear_residual_ns"])? + 3e-12,
            "first native receipt or reported momentum residual is inconsistent",
        )?;
        r["first_native_momentum_residual_ns"] = json!(native_residual);
        let physical = endpoint(ns)?;
        r["paid_end_ns"] = json!(ns);
        r["paid_dt_ns"] = json!(dt);
        r["lag_ns"] = json!(ns as f64 - start);
        r["where"] = json!(if (ns as f64) < start {
            "before"
        } else if ((ns - dt) as f64) > start {
            "after"
        } else {
            "contains"
        });
        r["first_model"] = physical["model"]["status"].clone();
        r["first_vector_ns"] = json!(j.to_array());
        r["instantaneous_difference_ns"] = json!(j.distance(point));
        if physical["qualified"] == true {
            let end = &physical["model"]["end"];
            r["first_vector_error_ns"] = json!(j.distance(v(&end["total_impulse_ns"])?));
            r["real_reaction_after_instantaneous_ns"] =
                json!(v(&end["total_impulse_ns"])?.distance(point));
        }
    }
    Ok(r)
}
fn row(path: &Path, studies: &Path) -> Result<Value> {
    let s = read(path)?;
    let name = path.file_name().unwrap();
    let read_field = |folder: &str| read(&studies.join(folder).join("cases").join(name));
    let fixed = read_field("multifidelity-secondary-return-features")?;
    let natural = read_field("multifidelity-secondary-natural")?;
    let travel = read_field("multifidelity-secondary-travel-features")?;
    for field in [&fixed, &natural, &travel] {
        for k in ["case", "slot", "held", "cut_tick", "incoming"] {
            chk(s[k] == field[k], "native and mechanical cohorts disagree")?;
        }
    }
    if s["material"]["status"] != "unique" {
        return Ok(
            json!({"case":s["case"],"slot":s["slot"],"held":s["held"],"cut_tick":s["cut_tick"],"material":s["material"],"boundary":"unresolved_first_event","qualified":false,"native":[]}),
        );
    }
    let proof = law(&s)?;
    let mut values = vec![];
    let mut keys = BTreeSet::new();
    for (method, data) in [
        ("natural", &natural["treatments"]),
        ("fixed", &fixed["treatments"]),
        ("travel", &travel["treatments"]),
    ] {
        for t in a(data)? {
            if ![8, 64, 4096].iter().any(|n| t["calls"] == *n)
                || ![1, 8, 32].iter().any(|n| t["passes"] == *n)
            {
                continue;
            }
            let kind = t["refresh"]["method"].as_str().unwrap_or("ordinary");
            if method == "travel" && kind != "first_travel" {
                continue;
            }
            let t = if kind == "ordinary" && t["calls"] == 8 && t["passes"] == 8 {
                a(&fixed["ordinary_onset"])?
                    .iter()
                    .find(|r| r["graph"] == t["graph"])
                    .ok_or("missing ordinary material onset")?
            } else {
                t
            };
            chk(
                keys.insert((
                    t["calls"].as_u64().unwrap(),
                    t["passes"].as_u64().unwrap(),
                    t["graph"].as_str().unwrap(),
                    kind,
                )),
                "duplicate native coordinate",
            )?;
            values.push(score(&s, t, kind)?);
        }
    }
    let end = a(&s["targets"])?
        .iter()
        .find(|x| x["time_ns"] == 500000)
        .unwrap();
    Ok(
        json!({"case":s["case"],"slot":s["slot"],"held":s["held"],"cut_tick":s["cut_tick"],"material":proof,"boundary":end["model"]["status"],"qualified":end["qualified"],"boundary_events":end["model"]["events"],"boundary_model":end["model"]["end"],"native":values}),
    )
}
fn main() -> Result<()> {
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();
    chk(args.len() == 2, "secondary_material STUDY REPORT")?;
    let dir = Path::new(&args[0]);
    let out = Path::new(&args[1]);
    fs::create_dir_all(out)?;
    let studies = dir.parent().ok_or("missing sibling studies")?;
    let mut rows = vec![];
    let mut slots = BTreeSet::new();
    for e in fs::read_dir(dir.join("cases"))? {
        let path = e?.path();
        if path.extension().is_some_and(|s| s == "json") {
            let r = row(&path, studies)?;
            chk(
                slots.insert(r["slot"].as_u64().unwrap()),
                "duplicate source slot",
            )?;
            rows.push(r);
        }
    }
    rows.sort_by_key(|r| r["slot"].as_u64());
    let mut output = json!({"joined":rows.len(),"pending_slots":(0..50).filter(|i|!slots.contains(i)).collect::<Vec<_>>()});
    for (label, held) in [("calibration", false), ("holdout", true)] {
        let source = rows
            .iter()
            .filter(|r| r["held"] == held)
            .collect::<Vec<_>>();
        let mut map = BTreeMap::<(u64, u64, String, String), Vec<&Value>>::new();
        let mut modes = BTreeMap::new();
        let mut geometries = BTreeMap::new();
        let mut terminal = BTreeMap::new();
        for r in &source {
            *modes
                .entry(r["material"]["mode"].as_str().unwrap_or("unresolved"))
                .or_insert(0) += 1;
            *geometries
                .entry(r["material"]["feature"].as_str().unwrap_or("unresolved"))
                .or_insert(0) += 1;
            *terminal.entry(r["boundary"].as_str().unwrap()).or_insert(0) += 1;
            for v in a(&r["native"])? {
                map.entry((
                    v["calls"].as_u64().unwrap(),
                    v["passes"].as_u64().unwrap(),
                    v["method"].as_str().unwrap().into(),
                    v["graph"].as_str().unwrap().into(),
                ))
                .or_default()
                .push(v);
            }
        }
        let mut cells = vec![];
        for ((grid, passes, method, graph), x) in map {
            let mut c = json!({"calls":grid,"passes":passes,"method":method,"graph":graph});
            for key in [
                "public_vector_error_ns",
                "public_com_error_um",
                "public_spin_error_rad_s",
                "first_vector_error_ns",
                "lag_ns",
                "instantaneous_difference_ns",
                "real_reaction_after_instantaneous_ns",
            ] {
                c[key] = dist(x.iter().filter_map(|r| r[key].as_f64()).collect());
            }
            let counts = ["before", "contains", "after"]
                .map(|k| x.iter().filter(|v| v["where"] == k).count());
            c["first_interval_before_contains_after"] = json!(counts);
            let mut changes = vec![];
            for r in &source {
                let native = a(&r["native"])?;
                let find = |m: &str| {
                    native
                        .iter()
                        .find(|t| {
                            t["calls"] == grid
                                && t["passes"] == passes
                                && t["method"] == m
                                && t["graph"] == graph
                        })
                        .and_then(|t| t["public_vector_error_ns"].as_f64())
                };
                if let Some((candidate, ordinary)) = find(&method).zip(find("ordinary")) {
                    changes.push(candidate - ordinary);
                }
            }
            c["signed_error_change_vs_ordinary_ns"] = dist(changes.clone());
            c["improved_equal_worse_1e-10_ns"] = json!([
                changes.iter().filter(|d| **d < -1e-10).count(),
                changes.iter().filter(|d| d.abs() <= 1e-10).count(),
                changes.iter().filter(|d| **d > 1e-10).count()
            ]);
            cells.push(c);
        }
        output[label] = json!({"sources":source.len(),"material_modes":modes,"material_features":geometries,"boundary_reasons":terminal,"cells":cells});
    }
    let mut nominated = None;
    for r in rows
        .iter()
        .filter(|r| r["held"] == true && r["qualified"] == true)
    {
        let cell = |id: &str| {
            a(&r["native"])
                .unwrap()
                .iter()
                .find(|v| {
                    v["calls"] == 8
                        && v["passes"] == 8
                        && v["graph"] == "retained"
                        && v["method"] == id
                })
                .unwrap()
        };
        let old = n(&cell("ordinary")["public_vector_error_ns"])?;
        let new = n(&cell("first_stale")["public_vector_error_ns"])?;
        let id = r["case"].as_u64().unwrap();
        if nominated.is_none_or(|(d, i, _, _): (f64, u64, f64, f64)| {
            (new - old).abs() > d || ((new - old).abs() == d && id < i)
        }) {
            nominated = Some(((new - old).abs(), id, old, new));
        }
    }
    output["selection"] = json!(nominated.map(
        |(d, id, a, b)| json!({"case":id,"change_ns":d,"ordinary_error_ns":a,"fixed_error_ns":b})
    ));
    plot(out, &output)?;
    output["cases"] = json!(rows);
    fs::write(
        out.join("material.json"),
        serde_json::to_vec_pretty(&output)?,
    )?;
    Ok(())
}
fn plot(out: &Path, s: &Value) -> Result<()> {
    let path = out.join("material-reaction.svg");
    let root = SVGBackend::new(&path, (1180, 655)).into_drawing_area();
    root.fill(&WHITE)?;
    root.draw(&Text::new(
        "Returning contact compared at the same physical time",
        (25, 38),
        ("sans-serif", 28).into_font(),
    ))?;
    root.draw(&Text::new("Held-out retained graph  ·  p95 signed impulse-vector error in N·s  ·  qualified piecewise rigid mechanics",(26,69),("sans-serif",16).into_font()))?;
    let cells = a(&s["holdout"]["cells"])?;
    for (row, (label, c, p, m)) in [
        ("8 calls, 8 passes · ordinary", 8, 8, "ordinary"),
        ("8 calls, 8 passes · fixed witness", 8, 8, "first_stale"),
        ("8 calls, 8 passes · travel witness", 8, 8, "first_travel"),
        ("8 calls, 8 passes · every native query", 8, 8, "every"),
        ("4,096 calls, 1 pass · ordinary", 4096, 1, "ordinary"),
        ("4,096 calls, 32 passes · ordinary", 4096, 32, "ordinary"),
    ]
    .into_iter()
    .enumerate()
    {
        let Some(cell) = cells.iter().find(|v| {
            v["calls"] == c && v["passes"] == p && v["graph"] == "retained" && v["method"] == m
        }) else {
            continue;
        };
        let y = 112 + row as i32 * 82;
        root.draw(&Text::new(
            label,
            (25, y + 20),
            ("sans-serif", 16).into_font(),
        ))?;
        for (j, (key, color)) in [
            ("first_vector_error_ns", RGBColor(38, 134, 127)),
            ("public_vector_error_ns", RGBColor(168, 100, 43)),
        ]
        .into_iter()
        .enumerate()
        {
            let d = &cell[key];
            let a = y + j as i32 * 23;
            let value = d["p95"].as_f64().unwrap_or(0.);
            let x = 350 + (value * 240.).round() as i32;
            root.draw(&Rectangle::new([(350, a), (x, a + 13)], color.filled()))?;
            root.draw(&Text::new(
                format!("{value:.5}   n={}", d["n"]),
                (x + 6, a + 12),
                ("sans-serif", 14).into_font(),
            ))?;
        }
    }
    root.draw(&Text::new("Green ends when the first native solve pays. Brown ends at the 500 µs public clock. Denominators depend on material qualification.",(25,629),("sans-serif",15).into_font()))?;
    root.present()?;
    Ok(())
}
