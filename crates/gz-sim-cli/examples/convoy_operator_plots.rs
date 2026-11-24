// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Regenerate continuation and healthy acknowledgement plots from convoy qualification records.
use plotters::{coord::Shift, prelude::*};
use serde::{Deserialize, de::DeserializeOwned};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
type Panel<'a> = DrawingArea<SVGBackend<'a>, Shift>;
const INK: RGBColor = RGBColor(32, 49, 63);
const MUTED: RGBColor = RGBColor(84, 99, 110);
const TEAL: RGBColor = RGBColor(5, 115, 119);
const ORANGE: RGBColor = RGBColor(183, 92, 28);
const BLUE: RGBColor = RGBColor(44, 79, 150);

fn read<T: DeserializeOwned>(path: &Path) -> Result<T> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn heading(panel: &Panel, title: &str, subtitle: &str) -> Result<()> {
    panel.draw(&Text::new(
        title,
        (30, 32),
        ("sans-serif", 25)
            .into_font()
            .style(FontStyle::Bold)
            .color(&INK),
    ))?;
    panel.draw(&Text::new(
        subtitle,
        (30, 58),
        ("sans-serif", 16).into_font().color(&MUTED),
    ))?;
    Ok(())
}

#[derive(Deserialize, PartialEq)]
struct Observation {
    tick: i32,
    sha256: String,
    next_effort_nm: f64,
    angular_velocity_rad_s: f64,
    angle_rad: f64,
    sphere_impulse_ns: f64,
}
#[derive(Deserialize)]
struct Drive {
    samples: Vec<Observation>,
    exact_native_archives: usize,
    ordinary_observations: usize,
}
fn continuation(dir: &Path) -> Result<()> {
    let [held, trace, network] =
        ["held", "trace", "network"].map(|name| read::<Drive>(&dir.join(format!("{name}.json"))));
    let (held, trace, network) = (held?, trace?, network?);
    for drive in [&held, &trace, &network] {
        if drive.samples.len() < 2
            || drive.samples.len() != drive.exact_native_archives
            || drive.samples.len() != drive.ordinary_observations
            || drive.samples.windows(2).any(|w| w[1].tick != w[0].tick + 1)
        {
            return Err("incomplete continuation cut sequence".into());
        }
    }
    if network.samples != trace.samples
        || !held
            .samples
            .iter()
            .map(|s| s.tick)
            .eq(trace.samples.iter().map(|s| s.tick))
    {
        return Err("drive clocks or network and pipe trace references differ".into());
    }
    let first = held.samples[0].tick;
    let last = held.samples.last().unwrap().tick;
    let path = dir.join("drives.svg");
    let root = SVGBackend::new(&path, (1040, 656)).into_drawing_area();
    root.fill(&WHITE)?;
    let (header, rest) = root.split_vertically(100);
    heading(
        &header,
        "Hinge drive and response after recovery",
        "Each point is a committed 1 ms cut. Torque at a cut drives the following interval.",
    )?;
    for (x, c, t) in [
        (30, ORANGE, "Held program · pipe"),
        (285, BLUE, "Trace · pipe"),
        (490, TEAL, "Trace · loopback Zenoh"),
    ] {
        if c == TEAL {
            header.draw(&Circle::new((x + 12, 82), 4, c.stroke_width(2)))?;
        } else {
            header.draw(&PathElement::new(
                vec![(x, 82), (x + 24, 82)],
                c.stroke_width(3),
            ))?;
        }
        header.draw(&Text::new(
            t,
            (x + 34, 87),
            ("sans-serif", 16).into_font().color(&INK),
        ))?;
    }
    let (panels, footer) = rest.split_vertically(516);
    let panels = panels.split_evenly((2, 1));
    for (i, panel) in panels.iter().enumerate() {
        let value = |o: &Observation| {
            if i == 0 {
                o.next_effort_nm * 1000.0
            } else {
                o.angular_velocity_rad_s * 1000.0
            }
        };
        let values = held.samples.iter().chain(&trace.samples).map(value);
        let (min, max) = values.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
            (lo.min(v), hi.max(v))
        });
        let pad = (max - min).max(1.0) * 0.08;
        let mut chart = ChartBuilder::on(panel)
            .margin_left(20)
            .margin_right(30)
            .margin_top(8)
            .x_label_area_size(39)
            .y_label_area_size(74)
            .build_cartesian_2d(first..last, min - pad..max + pad)?;
        chart
            .configure_mesh()
            .x_desc("Committed tick")
            .x_labels(10)
            .y_desc(if i == 0 {
                "Torque (mN·m)"
            } else {
                "Hinge speed (mrad/s)"
            })
            .axis_desc_style(("sans-serif", 15))
            .label_style(("sans-serif", 14))
            .max_light_lines(1)
            .light_line_style(RGBColor(235, 239, 242))
            .draw()?;
        for (program, color) in [(&held, ORANGE), (&trace, BLUE)] {
            let mut line = Vec::new();
            for (index, s) in program.samples.iter().enumerate() {
                if i == 0 && index > 0 {
                    line.push((s.tick, value(&program.samples[index - 1])));
                }
                line.push((s.tick, value(s)));
            }
            chart.draw_series(LineSeries::new(line, color.stroke_width(3)))?;
        }
        chart.draw_series(
            network
                .samples
                .iter()
                .map(|s| Circle::new((s.tick, value(s)), 4, TEAL.stroke_width(2))),
        )?;
    }
    footer.draw(&Text::new(format!("Pipe and loopback Zenoh produced identical trace observations and native archive bytes at all {} cuts.",trace.samples.len()),
            (30,26),("sans-serif",16).into_font().color(&INK)))?;
    root.present()?;
    Ok(())
}

const ARMS: [(&str, &str, &str); 3] = [
    (
        "before",
        "Before change",
        "Managed recovery enabled on the baseline server with default TCP coalescing",
    ),
    (
        "managed",
        "Managed",
        "Optimized server with TCP_NODELAY on the operator connection and managed recovery enabled",
    ),
    (
        "plain",
        "Plain",
        "Optimized server with TCP_NODELAY on the operator connection and managed recovery disabled",
    ),
];
const PERCENTILES: [usize; 3] = [50, 95, 99];
#[derive(Deserialize)]
struct Timing {
    latency_ms: Vec<f64>,
    initial_tick: u64,
    final_tick: u64,
    sample_count: usize,
    warmup: usize,
    p50_ms: f64,
    p95_ms: f64,
    p99_ms: f64,
}
fn median(mut values: Vec<f64>) -> f64 {
    values.sort_unstable_by(f64::total_cmp);
    let mid = values.len() / 2;
    (values[(values.len() - 1) / 2] + values[mid]) * 0.5
}
fn archive_equality(dir: Option<&Path>) -> Result<Value> {
    let caveat = "Final committed archive only. The timing records do not contain an archive digest for each timed interval.";
    let Some(dir) = dir else {
        return Ok(json!({"archives_read":0,"all_equal":null,"scope":caveat}));
    };
    let mut hash = None;
    let mut n = 0;
    for trial in 1..=6 {
        for (key, _, _) in ARMS {
            let path = dir.join(format!("after-{trial}-{key}/session/barrier.gzmultisphere"));
            let digest = format!("{:x}", Sha256::digest(fs::read(path)?));
            if hash.as_ref().is_some_and(|before| before != &digest) {
                return Err("final archive bytes differed between latency runs".into());
            }
            hash = Some(digest);
            n += 1;
        }
    }
    Ok(json!({"archives_read":n,"all_equal":true,"sha256":hash,"scope":caveat}))
}
fn latency(dir: &Path, archive_root: Option<&Path>) -> Result<()> {
    let mut trials = Vec::new();
    let mut summaries = Vec::new();
    let mut medians = Vec::new();
    let mut samples = 0;
    let mut warmup = 0;
    for (key, _, definition) in ARMS {
        let mut arm = Vec::new();
        let mut arm_samples = 0;
        let mut arm_warmup = 0;
        for i in 1..=6 {
            let mut t: Timing = read(&dir.join("paired").join(format!("{i}-{key}.json")))?;
            if t.sample_count != 64
                || t.latency_ms.len() != 64
                || t.warmup != 8
                || t.initial_tick != 150
                || t.final_tick != 222
                || t.latency_ms.iter().any(|v| !v.is_finite() || *v < 0.0)
            {
                return Err("invalid paired trial data".into());
            }
            t.latency_ms.sort_unstable_by(f64::total_cmp);
            let ranks = PERCENTILES.map(|p| t.latency_ms[(p * t.sample_count).div_ceil(100) - 1]);
            if [t.p50_ms, t.p95_ms, t.p99_ms] != ranks {
                return Err("raw samples disagree with recorded quantiles".into());
            }
            arm_samples += t.sample_count;
            arm_warmup += t.warmup;
            arm.push(ranks);
        }
        let mid = [0, 1, 2].map(|p| median(arm.iter().map(|q| q[p]).collect()));
        summaries.push(json!({"arm":key,"definition":definition,"trials":arm.len(),"measured_samples":arm_samples,"warmup_requests":arm_warmup,
          "median_of_trial_percentiles_ms":{"p50":mid[0],"p95":mid[1],"p99":mid[2]}}));
        samples += arm_samples;
        warmup += arm_warmup;
        trials.push(arm);
        medians.push(mid);
    }
    let report = json!({"metric":"Elapsed time from operator_step request to acknowledgement on a persistent negotiated loopback WebSocket",
       "method":"Six counterbalanced trials per arm. Nearest rank within each trial, followed by the median of the six trial percentiles. Warmups excluded.",
       "initial_tick":150,"final_tick":222,"total_trials":18,"total_measured_samples":samples,"total_warmup_requests":warmup,"arms":summaries,
       "managed_p95_reduction_vs_before_pct":(1.0-medians[1][1]/medians[0][1])*100.0,
       "managed_p95_increase_vs_plain_pct":(medians[1][1]/medians[2][1]-1.0)*100.0,
       "final_archive_equality":archive_equality(archive_root)?});
    let path = dir.join("paired.svg");
    let root = SVGBackend::new(&path, (1120, 460)).into_drawing_area();
    root.fill(&WHITE)?;
    let (header, rest) = root.split_vertically(83);
    heading(
        &header,
        "Healthy paused-step acknowledgement latency",
        "Six counterbalanced trials per arm · 64 measured steps per trial · lower is faster",
    )?;
    let (panels, footer) = rest.split_vertically(316);
    let ceiling = (trials
        .iter()
        .flatten()
        .flatten()
        .copied()
        .fold(0f64, f64::max)
        / 5.0)
        .ceil()
        * 5.0
        + 3.0;
    for (p, panel) in panels.split_evenly((1, 3)).iter().enumerate() {
        let mut chart = ChartBuilder::on(panel)
            .caption(format!("p{}", PERCENTILES[p]), ("sans-serif", 20))
            .margin_left(8)
            .margin_right(16)
            .x_label_area_size(36)
            .y_label_area_size(45)
            .build_cartesian_2d(-0.35f64..2.35f64, 0f64..ceiling)?;
        chart
            .configure_mesh()
            .disable_x_mesh()
            .x_labels(3)
            .x_label_formatter(&|v| {
                if (v - v.round()).abs() < 0.001 && *v >= 0.0 && *v <= 2.0 {
                    ARMS[v.round() as usize].1.to_owned()
                } else {
                    String::new()
                }
            })
            .y_desc("ms")
            .y_labels(6)
            .axis_desc_style(("sans-serif", 15))
            .label_style(("sans-serif", 14))
            .max_light_lines(1)
            .light_line_style(RGBColor(235, 239, 242))
            .draw()?;
        for ((before, managed), plain) in trials[0].iter().zip(&trials[1]).zip(&trials[2]) {
            chart.draw_series(LineSeries::new(
                [(0., before[p]), (1., managed[p]), (2., plain[p])],
                RGBColor(188, 198, 204).stroke_width(2),
            ))?;
        }
        chart.draw_series(LineSeries::new(
            (0..3).map(|a| (a as f64, medians[a][p])),
            TEAL.stroke_width(3),
        ))?;
        for (a, m) in medians.iter().enumerate() {
            let point = (a as f64, m[p]);
            chart.draw_series(std::iter::once(
                EmptyElement::at(point)
                    + Circle::new((0, 0), 5, TEAL.filled())
                    + Text::new(
                        format!("{:.1}", m[p]),
                        (-14, -11),
                        ("sans-serif", 14)
                            .into_font()
                            .style(FontStyle::Bold)
                            .color(&INK),
                    ),
            ))?;
        }
    }
    footer.draw(&Text::new("Gray lines connect paired trials. Teal labels show the median across six trials in each arm, in milliseconds.",
        (30,20),("sans-serif",16).into_font().color(&INK)))?;
    footer.draw(&Text::new("Managed recovery was enabled for Before change and Managed. Managed and Plain use TCP_NODELAY on the operator socket.",
        (30,45),("sans-serif",15).into_font().color(&MUTED)))?;
    root.present()?;
    let mut summary = serde_json::to_string_pretty(&report)?;
    summary.push('\n');
    fs::write(dir.join("summary.json"), summary)?;
    Ok(())
}
fn main() -> Result<()> {
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();
    if !(1..=2).contains(&args.len()) {
        return Err("expected OPERATOR_EVIDENCE_DIR [FINAL_ARCHIVE_RUN_ROOT]".into());
    }
    let dir = Path::new(&args[0]);
    continuation(&dir.join("continuation"))?;
    latency(&dir.join("latency"), args.get(1).map(Path::new))
}
