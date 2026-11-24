// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Preserve native time across edits and paused ticks when displaying the pre-solve policy.
#[path = "support_loss_promotion_view/boundary.rs"]
mod boundary;
#[path = "two_support_foot_view/plots.rs"]
mod plots;
mod soft_replay;
#[path = "two_support_foot_view/trace.rs"]
mod trace;
use clap::Parser;
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Duration,
};
use trace::{Events, Geometry, Lane, Sample, Trace};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    measurements: PathBuf,
    output: PathBuf,
}
#[derive(Deserialize)]
struct Receipt {
    label: String,
    time_s: f64,
    iteration: u64,
    paused: bool,
    tier: String,
    reference_tier: String,
    support_b_present: bool,
    sample: Lane,
    reference: Lane,
}
#[derive(Deserialize)]
struct Policy {
    boundary: Vec<Receipt>,
    before_prepare: Receipt,
    prepare: Receipt,
    commit: Receipt,
}
#[derive(Deserialize)]
struct Measure {
    #[serde(flatten)]
    events: Events,
    policy: Policy,
}
impl Policy {
    fn get(&self, key: &str) -> Result<&Receipt> {
        match key {
            "prepare" => Ok(&self.prepare),
            "before_prepare" => Ok(&self.before_prepare),
            "commit" => Ok(&self.commit),
            name => self
                .boundary
                .iter()
                .find(|s| s.label == name)
                .ok_or_else(|| format!("missing measured boundary {name}").into()),
        }
    }
}
#[derive(Serialize)]
struct Shot<'a> {
    tick: u64,
    label: &'a str,
    running: &'a str,
    paused: &'a str,
}
fn read<T: serde::de::DeserializeOwned>(p: &Path) -> Result<T> {
    Ok(serde_json::from_reader(std::io::BufReader::new(
        fs::File::open(p)?,
    ))?)
}
fn box_shape(half: [f64; 3]) -> Value {
    json!({"kind":"box","size":half.map(|x|2.*x)})
}
#[derive(Clone, Copy)]
struct State<'a> {
    body: &'a Lane,
    control: &'a Lane,
    tier: &'a str,
    paused: bool,
    present: bool,
    iteration: Option<u64>,
}
fn state<'a>(source: &str, sample: &'a Sample, policy: &'a Policy, dt: f64) -> Result<State<'a>> {
    if source == "step" {
        return Ok(State {
            body: &sample.switched,
            control: &sample.reference,
            tier: if sample.prescribed {
                "prescribed"
            } else {
                "dynamic"
            },
            paused: false,
            present: sample.support_b_present,
            iteration: None,
        });
    }
    let r = policy.get(source)?;
    if (r.time_s - sample.tick as f64 * dt).abs() > 1e-12
        || r.reference_tier != "dynamic"
        || !matches!(r.tier.as_str(), "prescribed" | "dynamic")
    {
        return Err(format!("unexpected native receipt clock or tier for {source}").into());
    }
    Ok(State {
        body: &r.sample,
        control: &r.reference,
        tier: &r.tier,
        paused: r.paused,
        present: r.support_b_present,
        iteration: Some(r.iteration),
    })
}
// Optional pause markers appear while the removable supports still exist.
// Assign IDs by name so geometry and catalog references survive their independent lifetimes.
fn keep_ids(f: &mut Frame, ids: &mut BTreeMap<String, u64>) -> Result<()> {
    for (model, visual) in f.models.iter_mut().zip(&mut f.visuals) {
        let name = model["name"].as_str().ok_or("missing model name")?;
        let next = 2 + 2 * ids.len() as u64;
        let id = *ids.entry(name.to_owned()).or_insert(next);
        model["model_id"] = json!(id);
        model["visual_ids"] = json!([id + 1]);
        visual["id"] = json!(id + 1);
    }
    Ok(())
}
struct Layout {
    width: f64,
    cx: f64,
    z: f64,
    max_load: f64,
}
impl Layout {
    fn measure(t: &Trace, p: &Trace) -> Result<Self> {
        let g = &t.geometry;
        let (ac, bc) = (
            DVec3::from(g.support_a_center),
            DVec3::from(g.support_b_center),
        );
        let (ae, be) = (
            DVec3::from(g.support_a_half_extents),
            DVec3::from(g.support_b_half_extents),
        );
        let mut lo = (ac - ae).min(bc - be);
        let mut hi = (ac + ae).max(bc + be);
        let mut max_load = 0.0_f64;
        for (i, (a, b)) in t.frames.iter().zip(&p.frames).enumerate() {
            if a.tick != i as u64
                || a.tick != b.tick
                || a.time_s != b.time_s
                || (a.time_s - a.tick as f64 * t.dt_s).abs() > 1e-12
                || a.reference != b.reference
            {
                return Err(format!("unmatched advancing references at frame {i}").into());
            }
            for lane in [&a.switched, &b.switched, &a.reference] {
                max_load = max_load.max(lane.normal_impulses_ns[0]);
                for (body, half) in [
                    (&lane.foot, g.foot_half_extents),
                    (&lane.upper, g.upper_half_extents),
                ] {
                    let pos = DVec3::from(body.position);
                    let q = DQuat::from_array(body.orientation);
                    let size = (q * DVec3::X).abs() * half[0]
                        + (q * DVec3::Y).abs() * half[1]
                        + (q * DVec3::Z).abs() * half[2];
                    lo = lo.min(pos - size);
                    hi = hi.max(pos + size);
                }
            }
        }
        let width = hi.x - lo.x;
        if max_load <= 0. || width <= 0. {
            return Err("expected finite loaded supports".into());
        }
        Ok(Self {
            width,
            cx: (hi.x + lo.x) / 2.,
            z: lo.z - width * 0.17,
            max_load,
        })
    }
    fn lane(&self, f: &mut Frame, name: &str, state: State<'_>, g: &Geometry) -> Result<()> {
        let State {
            body,
            tier: mode,
            paused,
            present,
            ..
        } = state;
        let v = DQuat::from_rotation_z(-std::f64::consts::FRAC_PI_2);
        let (shift, foot) = match name {
            "running" => (1., [0.13, 0.86, 0.7, 1.]),
            "paused_edit" => (0., [0.80, 0.49, 0.94, 1.]),
            "always_dynamic" => (-1., [0.43, 0.71, 1., 1.]),
            _ => return Err("unknown lane".into()),
        };
        let point =
            |p: DVec3| v * (p - DVec3::X * self.cx) + DVec3::Y * (shift * self.width * 1.22);
        for (key, c, h, color, visible) in [
            (
                "a",
                g.support_a_center,
                g.support_a_half_extents,
                [0.96, 0.66, 0.18, 1.],
                true,
            ),
            (
                "b",
                g.support_b_center,
                g.support_b_half_extents,
                [0.79, 0.43, 0.61, 1.],
                present,
            ),
        ] {
            if visible {
                f.add(
                    &format!("{name}_support_{key}"),
                    point(c.into()),
                    v,
                    box_shape(h),
                    color,
                );
            }
        }
        for (key, pose, shape, color) in [
            ("foot", &body.foot, g.foot_half_extents, foot),
            (
                "upper",
                &body.upper,
                g.upper_half_extents,
                [0.81, 0.85, 0.89, 1.],
            ),
        ] {
            f.add(
                &format!("{name}_{key}"),
                point(pose.position.into()),
                v * DQuat::from_array(pose.orientation),
                box_shape(shape),
                color,
            );
        }
        let gauge = |u: f64, dz: f64| {
            point(DVec3::new(
                self.cx + (u - 0.5) * self.width * 0.76,
                0.,
                self.z + dz * self.width,
            ))
        };
        let rule = [0.46, 0.55, 0.63, 1.];
        for (label, frac, dz, col) in [
            ("tier", if mode == "dynamic" { 1. } else { 0. }, 0., foot),
            (
                "a_reaction",
                body.normal_impulses_ns[0] / self.max_load,
                -0.17,
                [0.96, 0.66, 0.18, 1.],
            ),
        ] {
            f.edge(
                &format!("{name}_{label}_rail"),
                gauge(0., dz),
                gauge(1., dz),
                self.width * 0.76,
                self.width * 0.014,
                rule,
            );
            for (end, x) in [("left", 0.), ("right", 1.)] {
                f.edge(
                    &format!("{name}_{label}_{end}"),
                    gauge(x, dz - 0.03),
                    gauge(x, dz + 0.03),
                    self.width * 0.06,
                    self.width * 0.012,
                    rule,
                );
            }
            f.add(
                &format!("{name}_{label}"),
                gauge(frac, dz + 0.018),
                v,
                box_shape([self.width * 0.018; 3]),
                col,
            );
        }
        if paused {
            let top =
                DVec3::from(body.upper.position).z + g.upper_half_extents[2] + 0.08 * self.width;
            for (i, offset) in [-0.023, 0.023].into_iter().enumerate() {
                f.add(
                    &format!("{name}_pause_{i}"),
                    point(DVec3::new(self.cx + offset * self.width, 0., top)),
                    v,
                    box_shape([0.013 * self.width, 0.018 * self.width, 0.043 * self.width]),
                    [1., 0.83, 0.22, 1.],
                );
            }
        }
        Ok(())
    }
}
fn main() -> Result<()> {
    let args = Args::parse();
    let dir = &args.measurements;
    let running: Trace = read(&dir.join("trajectory.json"))?;
    let paused: Trace = read(&dir.join("paused/trajectory.json"))?;
    let rs: Measure = read(&dir.join("summary.json"))?;
    let ps: Measure = read(&dir.join("paused/summary.json"))?;
    if running.dt_s != paused.dt_s
        || running.geometry != paused.geometry
        || running.frames.len() != paused.frames.len()
        || rs.events != ps.events
    {
        return Err("inconsistent independent experiment definitions".into());
    }
    let layout = Layout::measure(&running, &paused)?;
    let promoted = rs.events.promotion_tick;
    let edit = rs
        .events
        .loss_tick
        .checked_sub(1)
        .ok_or("loss at startup")?;
    if promoted != edit + 1 {
        return Err("expected automatic promotion on first advancing solve".into());
    }
    let dt_ns = u64::try_from(Duration::try_from_secs_f64(running.dt_s)?.as_nanos())?;
    let mut schedule = Vec::new();
    let stride = running.frames.len().div_ceil(53).max(1);
    for (i, s) in running.frames.iter().enumerate() {
        if i % stride != 0
            && i + 1 != running.frames.len()
            && ![rs.events.demotion_tick, edit, promoted]
                .iter()
                .any(|t| t.abs_diff(s.tick) <= 2)
        {
            continue;
        }
        schedule.push(Shot {
            tick: s.tick,
            label: if s.tick == promoted {
                "first_solve"
            } else {
                "step"
            },
            running: if s.tick == promoted { "commit" } else { "step" },
            paused: if s.tick == promoted { "commit" } else { "step" },
        });
        if s.tick == edit {
            let mut rsrc = "before_edit";
            for snapshot in &ps.policy.boundary {
                let label = snapshot.label.as_str();
                if rs.policy.boundary.iter().any(|r| r.label == label) {
                    rsrc = label;
                }
                schedule.push(Shot {
                    tick: edit,
                    label,
                    running: rsrc,
                    paused: label,
                });
            }
            for label in ["before_prepare", "prepare"] {
                schedule.push(Shot {
                    tick: edit,
                    label,
                    running: label,
                    paused: label,
                });
            }
        }
    }
    let mut frames = Vec::new();
    let mut index = Vec::new();
    let mut ids = BTreeMap::new();
    let mut captures = BTreeMap::new();
    for shot in schedule {
        let tick = shot.tick as usize;
        let a = state(
            shot.running,
            &running.frames[tick],
            &rs.policy,
            running.dt_s,
        )?;
        let b = state(shot.paused, &paused.frames[tick], &ps.policy, running.dt_s)?;
        if a.present != b.present || a.control != b.control {
            return Err(format!(
                "mismatched pad or independent reference snapshot for {}",
                shot.label
            )
            .into());
        }
        let mut f = Frame::default();
        layout.lane(&mut f, "running", a, &running.geometry)?;
        layout.lane(&mut f, "paused_edit", b, &running.geometry)?;
        layout.lane(
            &mut f,
            "always_dynamic",
            State {
                body: a.control,
                tier: "dynamic",
                paused: false,
                ..a
            },
            &running.geometry,
        )?;
        keep_ids(&mut f, &mut ids)?;
        let ordinal = frames.len() + 1;
        if shot.label != "step" {
            captures.insert(shot.label, ordinal - 1);
        }
        index.push(json!({"tick":tick,"ordinal":ordinal,"label":shot.label,"running":shot.running,"paused":shot.paused,"time_s":shot.tick as f64*running.dt_s,
            "running_iteration":a.iteration,"paused_iteration":b.iteration,"tiers":[a.tier,b.tier,"dynamic"],"pad_b_present":a.present,"paused_flag":b.paused,"normal_a_ns":[a.body.normal_impulses_ns[0],b.body.normal_impulses_ns[0],a.control.normal_impulses_ns[0]]}));
        frames.push(f.finish(Duration::from_nanos(shot.tick * dt_ns), ordinal));
    }
    fs::create_dir_all(&args.output)?;
    gz_sim_cli::export_archive(&args.output.join("replay"), Default::default(), frames)?;
    fs::write(
        args.output.join("frames.json"),
        serde_json::to_vec_pretty(
            &json!({"kind":"support-loss-promotion-v1","dt_ns":dt_ns,"frames":index,"captures":captures,
       "width":layout.width,"cx":layout.cx,"gauge_z":layout.z,"gauge_max_ns":layout.max_load,"all_advancing_switched_samples_equal":running.frames.iter().zip(&paused.frames).all(|(a,b)|a.switched==b.switched)}),
        )?,
    )?;
    boundary::draw(&rs.policy, &ps.policy, &args.output.join("boundary.svg"))?;
    plots::draw(&running, &rs.events, &args.output.join("response.svg"))
}
