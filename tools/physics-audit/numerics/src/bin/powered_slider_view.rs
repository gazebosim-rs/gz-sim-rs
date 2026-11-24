// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check native replay geometry against independent encoders and sphere impulse balance.
//! Input is the canonical planar 200 g sphere and force-driven wrist/finger fixture.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::Path,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const HUB: DVec3 = DVec3::new(0., 0., 0.1);
const RAIL: f64 = 0.10;
const MASS: f64 = 0.20;
const SPEED: f64 = 0.75;
const RELEASE_CLEARANCE: f64 = 1e-6;

fn num(value: &Value) -> Result<f64> {
    value
        .as_f64()
        .ok_or_else(|| "expected numerical observation".into())
}
fn samples(path: &Path) -> Result<impl Iterator<Item = Result<Value>>> {
    Ok(BufReader::new(File::open(path)?)
        .lines()
        .map(|row| Ok(serde_json::from_str(&row?)?)))
}
fn visual<'a>(scene: &'a Value, name: &str) -> Result<&'a Value> {
    scene["visuals"]
        .as_array()
        .and_then(|vs| vs.iter().find(|v| v["name"] == name))
        .ok_or_else(|| format!("missing fixture visual {name}").into())
}
fn center(visual: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(
        visual["translation"].clone(),
    )?))
}

#[derive(Clone, Copy)]
struct Sample {
    angle: f64,
    slide: f64,
    sphere: DVec3,
}

fn check(trace: &Path, archive: &Path, output: &Path) -> Result<()> {
    let mut frames = Vec::new();
    for record in samples(&archive.join("frames.jsonl"))? {
        let r = record?;
        if r["kind"] == "frame" {
            let s =
                serde_json::from_str::<Value>(r["scene"].as_str().ok_or("missing native scene")?)?;
            let time = s["seconds"].as_u64().ok_or("scene seconds")? * 1_000_000_000
                + s["nanoseconds"].as_u64().ok_or("scene nanoseconds")?;
            frames.push((time, s));
        }
    }
    let (0, initial) = frames.first().ok_or("empty scene")? else {
        return Err("missing initial frame".into());
    };
    let sphere_name = "default::parcel::body::sphere";
    let mut sphere = center(visual(initial, sphere_name)?)?;
    let approach = -sphere.x.signum();
    let expected_start = DVec3::new(-approach * 0.02405, 0.12, 0.1);
    if sphere.distance(expected_start) > 1e-12 {
        return Err("unexpected incoming sphere pose".into());
    }
    let crossing_ns = (sphere.x.abs() - 0.009 - 0.015) / SPEED * 1e9;
    let mut velocity = DVec3::X * (approach * SPEED);
    let mut states = BTreeMap::new();
    states.insert(
        0,
        Sample {
            angle: 0.,
            slide: 0.,
            sphere,
        },
    );
    let mut previous_time = 0;
    let mut previous_angle = 0.;
    let mut force = None;
    let mut first_load = None;
    let mut impulse = 0.;
    let mut max_travel = 0.;
    let mut max_travel_time = 0;
    let mut released = None;
    let mut reseated = None;
    let mut rows = samples(trace)?;
    let profile = rows.next().ok_or("missing execution profile")??;
    let dt_ns = profile["step_ns"]
        .as_u64()
        .ok_or("missing authored timestep")?;
    for row in rows {
        let row = row?;
        let time = row["time_ns"].as_u64().ok_or("missing public time")?;
        if time == previous_time {
            continue;
        }
        if time.checked_sub(previous_time) != Some(dt_ns) {
            return Err("nonconsecutive advancing observations".into());
        }
        let held = row["held_joint_forces"]
            .as_array()
            .ok_or("missing persistent force")?;
        let [held] = held.as_slice() else {
            return Err("expected one powered slider".into());
        };
        let held_force = num(&held[1])?;
        if held_force.abs() != 2. || force.replace(held_force).is_some_and(|f| f != held_force) {
            return Err("slider force changed during the trial".into());
        }
        let joints = row["scalar_joints"]
            .as_array()
            .ok_or("missing joint encoders")?
            .iter();
        let encoder = |slide| -> Result<f64> {
            joints
                .clone()
                .find(|j| (j["id"] == held[0]) == slide)
                .map(|j| num(&j["position"]))
                .ok_or("missing encoder")?
        };
        let angle = encoder(false)?;
        let slide = encoder(true)?;
        let reaction = num(&row["delivered_normal_ns"])?;
        // A frictionless sphere receives only the impulse opposing its approach.
        // Use the pre-step face angle and apply the impulse at step end. The
        // resulting one-step position uncertainty is bounded below during readback.
        let dt = dt_ns as f64 * 1e-9;
        sphere += velocity * dt;
        velocity -=
            DQuat::from_rotation_z(previous_angle) * DVec3::X * (approach * reaction / MASS);
        if reaction > 0. {
            if first_load.replace(time).is_some() {
                return Err("the fixture has more than one loaded tick".into());
            }
            impulse = reaction;
        }
        if slide.abs() > RELEASE_CLEARANCE {
            released.get_or_insert(time);
        } else if released.is_some() {
            reseated.get_or_insert(time);
        }
        if slide.abs() > max_travel {
            max_travel = slide.abs();
            max_travel_time = time
        }
        states.insert(
            time,
            Sample {
                angle,
                slide,
                sphere,
            },
        );
        previous_angle = angle;
        previous_time = time;
    }
    if frames.last().unwrap().0 != previous_time {
        return Err("recording ended before the final observed physics step".into());
    }
    let hit = first_load.ok_or("the native solver delivered no impact")?;
    if (hit as f64) < crossing_ns || (hit - dt_ns) as f64 > crossing_ns {
        return Err("first loaded tick does not bracket the incoming face crossing".into());
    }
    let mut rig_error = 0f64;
    let mut orientation_error = 0f64;
    let mut sphere_error = 0f64;
    for (time, scene) in &frames {
        let s = states
            .get(time)
            .ok_or("recorded frame does not have an encoder observation")?;
        let turn = DQuat::from_rotation_z(s.angle);
        for (name, local) in [
            ("default::wrist_rig::wrist::hub", DVec3::ZERO),
            (
                "default::wrist_rig::wrist::guide",
                DVec3::new(0., 0.052, 0.008),
            ),
            (
                "default::wrist_rig::finger::finger",
                DVec3::new(s.slide, RAIL, 0.),
            ),
        ] {
            let v = visual(scene, name)?;
            let orientation = DQuat::from_array(serde_json::from_value(v["rotation"].clone())?);
            rig_error = rig_error.max(center(v)?.distance(HUB + turn * local));
            // Antipodal unit quaternions represent the same rotation.
            orientation_error = orientation_error.max(
                (turn - orientation)
                    .length()
                    .min((turn + orientation).length()),
            );
        }
        sphere_error = sphere_error.max(center(visual(scene, sphere_name)?)?.distance(s.sphere));
    }
    let sphere_bound = dt_ns as f64 * 1e-9 * (impulse / MASS + SPEED);
    if rig_error > 1e-9 || orientation_error > 1e-9 || sphere_error > sphere_bound {
        return Err(format!("native replay errors: rig {rig_error:e} m, quaternion {orientation_error:e}, sphere {sphere_error:e} m (bound {sphere_bound:e})").into());
    }
    let hit_frame = frames
        .iter()
        .position(|(t, _)| *t == hit)
        .ok_or("frame cadence missed the first impulse")?;
    let peak_frame = frames
        .iter()
        .enumerate()
        .min_by_key(|(_, (t, _))| t.abs_diff(max_travel_time))
        .unwrap()
        .0;
    let force = force.unwrap();
    let holds = approach * force > 0.;
    if (holds && max_travel > 1e-5) || (!holds && max_travel < 0.001) {
        return Err("held/release motion disagrees with impact side".into());
    }
    let mut captures = json!({"incoming":0,"first_contact":hit_frame,"end":frames.len()-1});
    if !holds {
        captures["largest_slide"] = peak_frame.into();
        if let Some(time) = reseated {
            captures["reseated"] = frames.iter().position(|(t, _)| *t >= time).unwrap().into();
        }
    }
    fs::create_dir_all(output)?;
    let index = json!({"kind":"authored-archive-v1","camera":"top","dt_ns":1,"captures":captures});
    let report = json!({"force_n":force,"approach_x_m_s":approach*SPEED,"first_contact_ns":hit,"geometric_crossing_ns":crossing_ns,"end_ns":previous_time,
        "positive_impulse_ns":impulse,"max_slide_m":max_travel,"max_slide_time_ns":max_travel_time,
        "release_clearance_m":RELEASE_CLEARANCE,"first_clearance_ns":released,"reseated_ns":reseated,
        "end_slide_m":states[&previous_time].slide,"end_angle_rad":previous_angle,"frames":frames.len(),
        "max_encoder_pose_error_m":rig_error,"max_encoder_quaternion_chord_error":orientation_error,
        "max_sphere_impulse_path_error_m":sphere_error,"sphere_impulse_path_bound_m":sphere_bound});
    fs::write(
        output.join("frames.json"),
        serde_json::to_vec_pretty(&index)?,
    )?;
    fs::write(
        output.join("measurements.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    Ok(())
}
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let [trace, archive, out] = args.as_slice() else {
        return Err("expected TRACE REPLAY_DIRECTORY OUTPUT_DIRECTORY".into());
    };
    check(trace.as_ref(), archive.as_ref(), out.as_ref())
}

#[cfg(test)]
#[path = "../powered_slider_view_tests.rs"]
mod tests;
