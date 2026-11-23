// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the second-landing continuation grid and incoming history as crossed factors.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{env, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const TAGS: [&str; 3] = ["8r1_then512", "finite8_then512", "continuous4096r1"];
const NAMES: [&str; 3] = [
    "Ordinary first landing",
    "Geometric first landing",
    "Continuous fine incoming",
];
const OUTPUTS: [&str; 5] = [
    "uninterrupted",
    "cold512",
    "cold4096",
    "warm512",
    "warm4096",
];
fn file_label(mode: &str, incoming: u64) -> &str {
    match (mode, incoming) {
        ("warm512", 512) | ("warm4096", 4096) => "uninterrupted",
        ("warm512", _) | ("warm4096", _) => "warm_swapped",
        _ => mode,
    }
}
fn read(f: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(f)?)?)
}
fn save(f: &Path, v: &Value) -> Result<()> {
    Ok(fs::write(f, serde_json::to_vec_pretty(v)?)?)
}
fn array(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "missing rows".into())
}
fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "missing measured number".into())
}
fn tick(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "missing public tick".into())
}
fn vector(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(v.clone())?))
}
#[derive(Clone, Copy)]
struct Sample {
    com: DVec3,
    spin: DVec3,
    velocity: DVec3,
    tangent: DVec3,
    normal: f64,
    first: Option<u64>,
}
fn parse(r: &Value) -> Result<Sample> {
    let p = |body: &str, field: &str| vector(&r[body][field]);
    Ok(Sample {
        com: (p("foot", "position")? + p("upper", "position")?) * 0.5,
        spin: p("foot", "angular_velocity")?,
        velocity: (p("foot", "velocity")? + p("upper", "velocity")?) * 0.5,
        tangent: vector(&r["tangent_on_foot_ns"])?,
        normal: number(&r["normal_impulse_ns"])?,
        first: r["first_local_ns"].as_u64(),
    })
}
fn checks(a: bool, message: &str) -> Result<()> {
    if !a { Err(message.into()) } else { Ok(()) }
}
fn main() -> Result<()> {
    let args = env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 1 {
        return Err("secondary_restart .../multifidelity-secondary-restart/case-010".into());
    }
    let dir = Path::new(&args[0]);
    let d = read(&dir.join("restart.json"))?;
    let crossed = d["format"] == "matched-secondary-restart-v2";
    let modes = &OUTPUTS[..if crossed { 5 } else { 3 }];
    checks(
        crossed || d["format"] == "matched-secondary-restart-v1",
        "incompatible restart archive",
    )?;
    let near = tick(&d["near_cut"])?;
    let wide = tick(&d["wide_cut"])?;
    let end = tick(&d["last_tick"])?;
    checks(
        end == near + 180 && wide < near && tick(&d["second_tick"])? == near + 1,
        "clock mismatch",
    )?;
    let mut conclusions = vec![];
    let mut snapshots = vec![];
    let mut cuts = vec![wide, near];
    if crossed {
        cuts.insert(0, tick(&d["far_cut"])?);
    }
    for cut in cuts {
        let mut sources = vec![];
        let mut per_source = vec![];
        for (i, tag) in TAGS.iter().enumerate() {
            let stage = dir.join(tag).join(cut.to_string());
            let audit = read(&stage.join("audit.json"))?;
            let budget = if i == 2 { 4096 } else { 512 };
            let snap = &audit["snapshot"];
            checks(
                audit["profile"] == *tag
                    && audit["original_suffix"] == budget
                    && audit["warm_twin"]["exact_differences"] == 0,
                "invalid donor or warm twin",
            )?;
            let controls = array(&d["controls"])?;
            checks(
                controls.iter().filter(|v| *v == &audit).count() == 1,
                "standalone audit not in source manifest",
            )?;
            checks(
                number(&snap["source_gap_m"])? > 0.
                    && (cut == near || number(&snap["source_gap_m"])? >= 0.002),
                "source is not clear",
            )?;
            let m = &snap["source"];
            checks(
                m["motor_enabled"] == false
                    && m["pending_hinge_torque_nm"].is_null()
                    && number(&m["hinge_q"])? == 0.
                    && number(&m["hinge_qdot"])? == 0.
                    && vector(&m["gravity"])? == DVec3::new(0., 0., -9.81),
                "active force or displaced hinge",
            )?;
            for v in array(&m["user_forces_n"])?
                .iter()
                .chain(array(&m["user_torques_nm"])?)
            {
                checks(vector(v)? == DVec3::ZERO, "unexpected applied wrench")?;
            }
            for h in array(&snap["source_cache"])?
                .iter()
                .chain(array(&snap["cold_cache"])?)
            {
                checks(
                    !h["discard_pending"].as_bool().ok_or("cache flag")?
                        && [
                            "absolute_normal_ns",
                            "absolute_tangent_ns",
                            "absolute_torsion_nms",
                        ]
                        .iter()
                        .all(|k| h[k].as_f64() == Some(0.)),
                    "stale seed in cut",
                )?;
            }
            for h in array(&snap["cold_cache"])? {
                checks(
                    h["pair_present"] == false
                        && h["native_points"] == 0
                        && h["solver_points"] == 0,
                    "cold restart retained pair geometry",
                )?;
            }
            let duration = tick(&audit["unloaded_public_intervals"])?;
            checks(
                tick(&audit["last_positive_receipt"]["tick"])? + duration == cut
                    && duration > 0
                    && number(&audit["last_positive_receipt"]["normal_impulse_ns"])? > 0.,
                "unloading count inconsistent",
            )?;
            let schedule = array(&audit["public_vz_schedule"])?;
            checks(
                schedule.len() as u64 == end - cut,
                "wrong actuator schedule length",
            )?;
            let mut native = vec![];
            let mut raw = vec![];
            for (j, label) in modes.iter().enumerate() {
                let file = file_label(label, budget);
                let tr = read(&stage.join(format!("traces/{file}.json")))?;
                let frames = array(&tr["frames"])?;
                checks(frames.len() as u64 == end - cut + 1, "truncated trajectory")?;
                let mut path = vec![];
                for (k, r) in frames.iter().enumerate() {
                    let n = cut + k as u64;
                    checks(
                        tick(&r["tick"])? == n
                            && r["time_us"] == n * 500
                            && r["other_pair_normal_ns"] == 0.,
                        "clock or extra load",
                    )?;
                    if k == 0 {
                        checks(
                            r["foot"] == m["links"][0]
                                && r["upper"] == m["links"][1]
                                && r["pad"] == m["links"][2],
                            "cut links changed",
                        )?;
                    } else {
                        let cmd = &schedule[k - 1];
                        checks(
                            cmd[0] == n
                                && vector(&r["pad"]["velocity"])? == DVec3::Z * number(&cmd[1])?
                                && vector(&r["pad"]["angular_velocity"])? == DVec3::ZERO,
                            "held pad velocity differs",
                        )?;
                        checks(
                            r["native_steps"] == [budget, 512, 4096, 512, 4096][j]
                                && r["solver_iterations"] == 4,
                            "native budget mismatch",
                        )?;
                    }
                    path.push(parse(r)?);
                }
                native.push(path);
                raw.push(frames.clone());
            }
            let same = if i == 2 { 2 } else { 1 };
            let counts = raw[0]
                .iter()
                .zip(&raw[same])
                .filter(|(a, b)| a != b)
                .count();
            checks(
                audit["cold_same_grid"]["exact_differences"] == counts,
                "cold twin readback count mismatch",
            )?;
            let first = raw[0]
                .iter()
                .zip(&raw[same])
                .find(|(a, b)| a != b)
                .map(|(r, _)| r["tick"].clone());
            checks(
                audit["cold_same_grid"]["first_difference_tick"] == json!(first),
                "first cold mismatch changed",
            )?;
            per_source.push(json!({"source":tag,"unloaded_intervals":duration,"incoming_gap_m":snap["source_gap_m"],"source_cache":snap["source_cache"],
      "exact_twin_samples":raw[0].len()-counts,"cold_same_grid":audit["cold_same_grid"],"first_loaded_end_ns":native.iter().map(|path|path.iter().enumerate().skip(1).find_map(|(k,t)|t.first.map(|q|(cut+k as u64-1)*500000+q))).collect::<Vec<_>>() }));
            sources.push(native);
        }
        let mut curve = vec![];
        let mut details = vec![];
        let mut normal_sum = [[0.; 5]; 3];
        let mut tangent_sum = [[DVec3::ZERO; 5]; 3];
        for (k, time) in (cut..=end).enumerate() {
            let s = |source: usize, grid: usize| sources[source][grid][k];
            if time > near {
                for (i, loads) in tangent_sum.iter_mut().enumerate() {
                    for (g, load) in loads.iter_mut().enumerate().take(modes.len()) {
                        *load += s(i, g).tangent;
                        normal_sum[i][g] += s(i, g).normal;
                    }
                }
            }
            let rel = |g: usize| {
                1000. * (s(1, g).com.distance(s(2, g).com) - s(0, g).com.distance(s(2, g).com))
            };
            let shifts: Vec<_> = (0..modes.len()).map(rel).collect();
            let (coarse, fine) = if crossed { (3, 4) } else { (1, 2) };
            let grid_sep: [f64; 3] =
                std::array::from_fn(|i| 1000. * s(i, coarse).com.distance(s(i, fine).com));
            let cold_sep: [f64; 3] = std::array::from_fn(|i| {
                1e6 * s(i, 0).com.distance(s(i, if i == 2 { 2 } else { 1 }).com)
            });
            if time >= near {
                let mut entry = vec![(time - near) as f64 * 0.5];
                entry.extend_from_slice(&shifts);
                entry.extend_from_slice(&grid_sep);
                entry.extend_from_slice(&cold_sep);
                curve.push(entry);
            }
            if [0, 1, 2, 10, 26, 94, 148, 180].contains(&time.saturating_sub(near)) && time >= near
            {
                let mut readings = vec![];
                for (g, mode) in modes.iter().enumerate() {
                    readings.push(json!({"mode":mode,"com_to_fine_incoming_mm":[1000.*s(0,g).com.distance(s(2,g).com),1000.*s(1,g).com.distance(s(2,g).com)],
            "spin_to_fine_incoming_rad_s":[s(0,g).spin.distance(s(2,g).spin),s(1,g).spin.distance(s(2,g).spin)],
            "velocity_to_fine_incoming_m_s":[s(0,g).velocity.distance(s(2,g).velocity),s(1,g).velocity.distance(s(2,g).velocity)],
            "public_normal_ns":(0..3).map(|i|s(i,g).normal).collect::<Vec<_>>(),
            "public_tangent_ns":(0..3).map(|i|s(i,g).tangent.to_array()).collect::<Vec<_>>(),
            "cumulative_normal_ns":normal_sum.map(|a|a[g]),
            "cumulative_tangent_ns":tangent_sum.map(|a|a[g].to_array()),
            "public_impulse_from_momentum_ns":if time>near {Some((0..3).map(|i|(2.*(s(i,g).velocity-sources[i][g][k-1].velocity)+DVec3::Z*2.*9.81*0.0005).to_array()).collect::<Vec<_>>())}else{None} }));
                }
                details.push(json!({"tick":time,"from_near_ms":(time-near)as f64*0.5,"geometric_minus_ordinary_error_mm":shifts,
          "grid_sensitivity_mm":grid_sep,"cold_same_resolution_um":cold_sep,"modes":readings}));
            }
        }
        let thresholds=modes.iter().enumerate().map(|(g,mode)|{
      let first=|cutoff:f64,span:usize|curve.windows(span).position(|w|w.iter().all(|x|x[g+1]>=cutoff)).map(|idx|curve[idx][0]);
      json!({"mode":mode,"minimum_signed_mm":curve.iter().map(|p|p[g+1]).fold(f64::INFINITY,f64::min),
      "maximum_signed_mm":curve.iter().map(|p|p[g+1]).fold(f64::NEG_INFINITY,f64::max),
      "first_three_over_10um_ms":first(0.01,3),"first_ten_over_1mm_ms":first(1.,10)})
   }).collect::<Vec<_>>();
        snapshots.push(json!({"cut_tick":cut,"donors":per_source,"endpoint_comparison":details,"relative_error_crossings":thresholds}));
        conclusions.push(curve);
    }
    save(
        &dir.join("readback.json"),
        &json!({"case":d["case"],"source_order":TAGS,"mode_order":modes,"second_tick":d["second_tick"],"cuts":snapshots}),
    )?;
    let data = conclusions.last().ok_or("missing near cut")?;
    let svg = dir.join("restart-comparison.svg");
    let root = SVGBackend::new(&svg, (1190, 1120)).into_drawing_area();
    root.fill(&WHITE)?;
    root.draw(&Text::new(
        format!(
            "Case {}  ·  Three incoming states at the second lift landing",
            d["case"]
        ),
        (26, 34),
        ("sans-serif", 25).into_font(),
    ))?;
    root.draw(&Text::new("One 500 µs actuator clock. Each cold branch starts with identical motion to its uninterrupted source.",(26,61),("sans-serif",17).into_font()))?;
    let rows = root.margin(79, 12, 22, 20).split_evenly((3, 1));
    let colors = [
        RGBColor(80, 80, 90),
        RGBColor(219, 95, 35),
        RGBColor(43, 113, 172),
    ];
    for (i, panel) in rows.iter().enumerate() {
        let panels = [
            [1, if crossed { 4 } else { 2 }, if crossed { 5 } else { 3 }],
            [modes.len() + 1, modes.len() + 2, modes.len() + 3],
            [modes.len() + 4, modes.len() + 5, modes.len() + 6],
        ];
        let columns = panels[i];
        let labels = if i == 0 {
            [
                "Original native histories",
                "All 512 after cut",
                "All 4,096 after cut",
            ]
        } else {
            NAMES
        };
        let mut mn = 0.0_f64;
        let mut mx = 0.0_f64;
        for row in data {
            for j in columns {
                mn = mn.min(row[j]);
                mx = mx.max(row[j]);
            }
        }
        let margin = ((mx - mn) * 0.06).max(0.001);
        let title = if i == 0 {
            "Change in COM error from the geometric first landing; positive means farther from the fine incoming donor"
        } else if i == 1 {
            "COM separation from changing 512 to 4,096 calls with the incoming state held fixed"
        } else {
            "COM change from rebuilding collision graphs at the original solver resolution"
        };
        let mut chart = ChartBuilder::on(panel)
            .caption(title, ("sans-serif", 17))
            .margin(8)
            .x_label_area_size(38)
            .y_label_area_size(72)
            .build_cartesian_2d(0.0f64..90.0f64, mn - margin..mx + margin)?;
        chart
            .configure_mesh()
            .x_desc("Time after incoming cut (ms)")
            .y_desc(if i == 2 {
                "COM distance (µm)"
            } else {
                "COM distance (mm)"
            })
            .light_line_style(RGBColor(226, 231, 235))
            .draw()?;
        chart.draw_series(LineSeries::new([(0., 0.), (90., 0.)], BLACK.mix(0.4)))?;
        for l in 0..3 {
            let c = colors[l];
            chart
                .draw_series(LineSeries::new(
                    data.iter().map(|p| (p[0], p[columns[l]])),
                    c.stroke_width(2),
                ))?
                .label(labels[l])
                .legend(move |(x, y)| PathElement::new([(x, y), (x + 22, y)], c.stroke_width(3)));
        }
        chart
            .configure_series_labels()
            .background_style(WHITE.mix(0.90))
            .border_style(RGBColor(180, 185, 190))
            .label_font(("sans-serif", 13))
            .draw()?;
    }
    root.present()?;
    for (n, tag) in [
        (512, if crossed { "warm512" } else { "cold512" }),
        (4096, if crossed { "warm4096" } else { "cold4096" }),
    ] {
        let out = dir.join(format!("common-{n}"));
        fs::create_dir_all(&out)?;
        let lanes=TAGS.iter().zip(NAMES).enumerate().map(|(i,(tag_in,name))|{
            let file=file_label(tag, if i==2{4096}else{512});
            json!({"label":tag_in,"name":name,"file":format!("../{tag_in}/{near}/traces/{file}.json")})
        }).collect::<Vec<_>>();
        save(
            &out.join("comparison.json"),
            &json!({"independent_histories":true,"pair_totals_only":true,
      "title":format!("Three incoming histories at the second landing, {n} calls on every subsequent tick"),
      "capture_ticks":{"second_impact":near+1,"at_47_ms":near+94,"at_74_ms":near+148,"at_90_ms":end},"lanes":lanes}),
        )?;
    }
    Ok(())
}
