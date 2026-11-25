// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run the stock wheel-drive journal, measure link poses and encoders, and exercise two traction controls.
use glam::{DQuat, EulerRot};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader, BufWriter, Write},
    path::{Path, PathBuf},
    process::Command,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORLD: &str = "examples/worlds/upstream_diff_drive.sdf";
const JOURNAL: &str = "examples/replay/upstream_diff_drive.json";
const HEADER: &str = "tick,chassis_x,chassis_y,chassis_z,chassis_roll,chassis_pitch,chassis_yaw,left_angle,right_angle,left_rate,right_rate,axle_x,axle_y,axle_z";

fn lines(path: &Path) -> Result<impl Iterator<Item = Result<Value>>> {
    Ok(BufReader::new(File::open(path)?)
        .lines()
        .map(|line| Ok(serde_json::from_str(&line?)?)))
}

fn sample<'a>(row: &'a Value, key: &str, id: u64) -> Result<&'a Value> {
    row[key]
        .as_array()
        .ok_or("missing observations")?
        .iter()
        .find(|item| item["id"] == id)
        .ok_or_else(|| format!("missing {key} ID {id}").into())
}

fn measure(trace: &Path, csv: &Path) -> Result<Value> {
    let mut records = lines(trace)?;
    let profile = records.next().ok_or("missing profile")??;
    let ids = &profile["observed_entities"];
    let id = |name: &str| {
        ids[format!("diff_drive::{name}")]
            .as_u64()
            .ok_or("missing observed name")
    };
    let ground = id("ground_plane::link")?;
    let chassis = id("vehicle::chassis")?;
    let wheels = [id("vehicle::left_wheel")?, id("vehicle::right_wheel")?];
    let joints = [
        id("vehicle::left_wheel_joint")?,
        id("vehicle::right_wheel_joint")?,
    ];
    let mut output = BufWriter::new(File::create_new(csv)?);
    writeln!(output, "{HEADER}")?;
    let mut count = 0;
    let mut next_tick = 0;
    let mut checkpoints = BTreeMap::new();
    let mut wheel_ground_impulses = [0.; 2];
    let mut minimum_clearance = f64::INFINITY;
    for row in records {
        let row = row?;
        let tick = row["step"].as_u64().ok_or("missing tick")?;
        if row["epoch"] != 0 || tick < next_tick {
            continue;
        }
        if tick != next_tick {
            return Err(format!("missing tick {next_tick}").into());
        }
        next_tick += 1;
        let body = sample(&row, "observed_links", chassis)?;
        let ground = &sample(&row, "observed_links", ground)?["native_body_id"];
        let mut axle = [0.; 3];
        for (side, wheel) in wheels.iter().enumerate() {
            let wheel = sample(&row, "observed_links", *wheel)?;
            let pos: [f64; 3] = serde_json::from_value(wheel["translation_m"].clone())?;
            for (sum, value) in axle.iter_mut().zip(pos) {
                *sum += value * 0.5;
            }
            minimum_clearance = minimum_clearance.min(pos[2] - 0.3);
            for pair in row["pair_reactions_ns"]
                .as_array()
                .ok_or("missing reactions")?
            {
                if (&pair[0] == ground && pair[1] == wheel["native_body_id"])
                    || (&pair[1] == ground && pair[0] == wheel["native_body_id"])
                {
                    wheel_ground_impulses[side] += pair[2].as_f64().ok_or("invalid impulse")?;
                }
            }
        }
        if !tick.is_multiple_of(10) {
            continue;
        }
        let pos: [f64; 3] = serde_json::from_value(body["translation_m"].clone())?;
        let orientation = DQuat::from_array(serde_json::from_value(body["rotation_xyzw"].clone())?);
        let (yaw, pitch, roll) = orientation.to_euler(EulerRot::ZYX);
        let left = sample(&row, "scalar_joints", joints[0])?;
        let right = sample(&row, "scalar_joints", joints[1])?;
        let encoder = |joint: &Value, key: &str| joint[key].as_f64().ok_or("invalid encoder");
        let angles = [encoder(left, "position")?, encoder(right, "position")?];
        let rates = [encoder(left, "velocity")?, encoder(right, "velocity")?];
        write!(output, "{tick}")?;
        for v in pos
            .into_iter()
            .chain([roll, pitch, yaw])
            .chain(angles)
            .chain(rates)
            .chain(axle)
        {
            if !v.is_finite() {
                return Err("nonfinite sample".into());
            }
            write!(output, ",{v:.16e}")?;
        }
        writeln!(output)?;
        if [0, 500, 2500, 4500, 5500].contains(&tick) {
            checkpoints.insert(tick, json!({"chassis_m":pos,"yaw_rad":yaw,"wheel_angle_rad":angles,"wheel_rate_rad_s":rates,"axle_m":axle}));
        }
        count += 1;
    }
    if next_tick != 5501 || count != 551 {
        return Err("incomplete drive trace".into());
    }
    output.flush()?;
    let straight_distance = |a: &Value, b: &Value| -> Result<f64> {
        let xy = |v: &Value, i: usize| v["chassis_m"][i].as_f64().ok_or("missing chassis position");
        Ok((xy(a, 0)? - xy(b, 0)?).hypot(xy(a, 1)? - xy(b, 1)?))
    };
    Ok(
        json!({"samples":count,"checkpoints":checkpoints,"straight_chassis_travel_m":straight_distance(&checkpoints[&500],&checkpoints[&2500])?,
        "wheel_ground_normal_impulse_ns":wheel_ground_impulses,"minimum_wheel_bottom_height_m":minimum_clearance}),
    )
}

fn replay(bin: &Path, world: &Path, journal: &Path, flags: &[&str], output: &Path) -> Result<()> {
    let status = Command::new(bin)
        .arg("--sdf-default-friction")
        .arg(world)
        .arg(journal)
        .args(flags)
        .arg(output)
        .status()?;
    if !status.success() {
        return Err(format!("replay failed ({status}): {}", world.display()).into());
    }
    Ok(())
}

fn check_reset(ground: &Path, reset: &Path) -> Result<usize> {
    let mut baseline = lines(ground)?.skip(1);
    let mut checked = 0;
    for row in lines(reset)? {
        let mut row = row?;
        if row["epoch"] != 1 {
            continue;
        }
        let mut original = baseline.next().ok_or("extra reset observation")??;
        for value in [&mut original, &mut row] {
            let object = value.as_object_mut().ok_or("invalid trace observation")?;
            object.remove("epoch");
            object.remove("command");
        }
        if row != original {
            return Err(format!("reset diverged at observation {checked}").into());
        }
        checked += 1;
    }
    if baseline.next().is_some() || checked == 0 {
        return Err("missing reset observations".into());
    }
    Ok(checked)
}

fn run(bin: &Path, out: &Path) -> Result<()> {
    fs::create_dir(out)?;
    let world = Path::new(WORLD);
    let journal = Path::new(JOURNAL);
    let xml = fs::read_to_string(world)?;
    let source = serde_json::from_reader::<_, Value>(File::open(journal)?)?;
    let ground = out.join("ground.jsonl");
    let record = out.join("record");
    replay(
        bin,
        world,
        journal,
        &[
            "--threads",
            "1",
            "--record-every",
            "50",
            "--viewer-lighting",
            "--record",
            record.to_str().ok_or("non-UTF8 path")?,
            "--trace",
        ],
        &ground,
    )?;
    replay(
        bin,
        world,
        journal,
        &["--threads", "8", "--verify"],
        &ground,
    )?;
    let ground_metrics = measure(&ground, &out.join("ground.csv"))?;
    let mut reset = source.clone();
    let commands = source["commands"].as_array().ok_or("invalid journal")?;
    let mut both = commands[..5].to_vec(); // Reset with the arc command still held.
    both.push(json!({"op":"reset"}));
    both.extend_from_slice(commands);
    reset["commands"] = both.into();
    let reset_journal = out.join("reset.json");
    fs::write(&reset_journal, serde_json::to_vec_pretty(&reset)?)?;
    let reset_path = out.join("reset.jsonl");
    replay(bin, world, &reset_journal, &["--trace"], &reset_path)?;
    let repeated = check_reset(&ground, &reset_path)?;
    // Give both sides the same isotropic Coulomb coefficient.
    let with_friction = |mu: f64| {
        xml.replace("</collision>", &format!(
        "<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>"
    ))
    };
    // Raise the original vehicle 20 cm and support the chassis belly on a narrow static stand.
    let lifted = xml.replace("<pose>0 0 0.325 0 -0 0</pose>","<pose>0 0 0.525 0 -0 0</pose>").replace("</world>",r#"<model name="stand"><static>true</static><pose>-0.15 0 0.2075 0 0 0</pose>
      <link name="link"><collision name="block"><geometry><box><size>1.1 0.45 0.415</size></box></geometry></collision></link></model></world>"#);
    let mut result = json!({"ground":ground_metrics,"exact_reset_observations":repeated,"exact_thread_counts":[1,8]});
    for (name, xml) in [
        ("low_friction", with_friction(0.001)),
        ("zero_friction", with_friction(0.0)),
        ("lifted", lifted),
    ] {
        let variant = out.join(format!("{name}.sdf"));
        let trace = out.join(format!("{name}.jsonl"));
        fs::write(&variant, xml)?;
        replay(bin, &variant, journal, &["--trace"], &trace)?;
        result[name] = measure(&trace, &out.join(format!("{name}.csv")))?;
    }
    fs::write(
        out.join("controls.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    for case in ["low_friction", "zero_friction", "lifted"] {
        let m = &result[case];
        if m["straight_chassis_travel_m"]
            .as_f64()
            .ok_or("missing travel")?
            > 0.05
            || m["checkpoints"]["2500"]["wheel_angle_rad"][0]
                .as_f64()
                .ok_or("missing encoder")?
                < 2.5
            || m["checkpoints"]["2500"]["wheel_angle_rad"][1]
                .as_f64()
                .ok_or("missing encoder")?
                < 2.5
        {
            return Err(format!("{case} did not isolate wheel spin from chassis motion").into());
        }
    }
    if result["ground"]["straight_chassis_travel_m"]
        .as_f64()
        .ok_or("missing travel")?
        < 0.75
        || result["lifted"]["minimum_wheel_bottom_height_m"]
            .as_f64()
            .ok_or("missing height")?
            < 0.1
        || result["lifted"]["wheel_ground_normal_impulse_ns"] != json!([0., 0.])
    {
        return Err("ground/lifted qualification failed".into());
    }
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<PathBuf> = std::env::args_os().skip(1).map(PathBuf::from).collect();
    match args.as_slice() {
        [bin, out] => run(bin, out),
        _ => Err(
            "usage: wheel_drive REPLAY_BINARY NEW_OUTPUT_DIRECTORY (from simulator root)".into(),
        ),
    }
}
