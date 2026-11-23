// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify public rigid body spin energy, sign symmetry, ideal yaw braking and native pressure work.
#[path = "support/torsional_energy_plot.rs"]
mod torsional_energy_plot;
use clap::Parser;
use gz_physics_distributed::physics::{Body, ContactImpulse};
use rapier3d_f64::math::{Rotation, Vector};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
    process::Command,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    qualified: PathBuf,
    /// Native qualification executable with corrected contact substep indices.
    #[arg(long)]
    runner: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    /// Reuse retained native frames after checking their physical digest and controls.
    #[arg(long)]
    retained: Option<PathBuf>,
    /// A new directory for any missing native traces, raw work samples and comparison results.
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Point {
    microstep: u8,
    normal_n_s: f64,
    radius_m: f64,
}
#[derive(Deserialize)]
struct Frame {
    tick: u64,
    bodies: [Body; 2],
    reference_bodies: [Body; 2],
    pairs: Vec<ContactImpulse>,
    substeps: u8,
    native_points: Vec<Point>,
    events: Vec<Value>,
    omega_before: f64,
    angular_reaction_z_nms: f64,
    predicted_twist_limit_nms: f64,
}
#[derive(Clone, Copy)]
struct State {
    w: Vector,
    h: Vector,
    energy: f64,
}
fn inertia(b: &Body) -> Vector {
    let [x, y, z] = b.size;
    Vector::new(y * y + z * z, x * x + z * z, x * x + y * y) * (b.mass / 12.)
}
fn state(b: &Body, kick: f64) -> State {
    let r = Rotation::from_array(b.orientation);
    let iz = inertia(b);
    let w = Vector::from(b.angular);
    let h = r * (iz * (r.inverse() * w)) + Vector::Z * kick;
    let w = if kick == 0. {
        w
    } else {
        w + r * ((r.inverse() * (Vector::Z * kick)) / iz)
    };
    State {
        w,
        h,
        energy: 0.5 * w.dot(h),
    }
}
#[derive(Serialize, Deserialize)]
struct Sample {
    tick: u64,
    energy_j: f64,
    ideal_j: f64,
    omega_z: f64,
    interval_work_vector_j: f64,
    interval_work_yaw_j: f64,
    interval_bound_work_j: f64,
    work_closure_j: f64,
}
fn read(path: &Path) -> Result<Value> {
    Ok(serde_json::from_reader(File::open(path)?)?)
}
fn analyse(dir: &Path, qualified: &Value) -> Result<(Value, Vec<Sample>)> {
    let summary = read(&dir.join("summary.json"))?;
    if summary["trajectory_sha256"] != qualified["trajectory_sha256"] {
        return Err("repeated run differed physically".into());
    }
    let mut body: Body = serde_json::from_value(summary["box"].clone())?;
    let ref_kick = summary["angular_impulse_nms"].as_f64().unwrap();
    let radius = 0.5 * body.size[0].hypot(body.size[1]);
    let torque = body.friction * body.mass * 9.81 * radius;
    let izz = inertia(&body).z;
    let mut energy_at_kick = 0.;
    let mut pre_drive = 0.;
    let mut speed_at_kick = 0.;
    let mut total_vec = 0.;
    let mut total_yaw = 0.;
    let mut sum_sat = 0.;
    let mut bound_sat = 0.;
    let mut saturated = 0;
    let mut closure = 0_f64;
    let mut yaw_closure = 0_f64;
    let mut max_ideal = 0_f64;
    let mut per_step = 0_f64;
    let mut correction_sum = 0.;
    let mut samples = vec![];
    let mut digest = Sha256::new();
    for (i, line) in BufReader::new(File::open(dir.join("frames.jsonl"))?)
        .lines()
        .enumerate()
    {
        let f: Frame = serde_json::from_str(&line?)?;
        let t = i as u64 + 1;
        if t != f.tick || f.bodies != f.reference_bodies || f.substeps == 0 {
            return Err("unqualified distributed pose".into());
        }
        digest.update(bincode::serialize(&(
            t,
            f.bodies.iter().collect::<Vec<_>>(),
            &f.pairs,
        ))?);
        let mut kick = 0.;
        for e in f.events.iter().filter(|e| e["kind"] == "angular impulse") {
            kick += e["angular_nms"][2].as_f64().unwrap();
        }
        if (t == 31 && kick != ref_kick) || (t != 31 && kick != 0.) {
            return Err("unmatched angular kick".into());
        }
        let before = state(&body, kick);
        let after = state(&f.bodies[1], 0.);
        if (f.omega_before - before.w.z).abs() > 2e-8 {
            return Err(format!("pre-step angular-velocity reconstruction differs at {t}").into());
        }
        let dh = after.h - before.h;
        if (dh.z - f.angular_reaction_z_nms).abs() > 2e-9 {
            return Err("rotated inertia misses the public angular impulse".into());
        }
        let avg = (before.w + after.w) * 0.5;
        let work = avg.dot(dh);
        let yaw = avg.z * dh.z;
        // The endpoint trapezoid residual has an exact algebraic expression for a rotating inertia.
        let correction = 0.5 * (after.w.dot(before.h) - before.w.dot(after.h));
        if ((after.energy - before.energy) - work - correction).abs() > 2e-13 {
            return Err("rotating-inertia work identity failed".into());
        }
        let delta_ke = after.energy - before.energy;
        let mut ideal = 0.;
        let mut predicted_work = 0.;
        if t >= 31 {
            if t == 31 {
                pre_drive = state(&body, 0.).energy;
                energy_at_kick = before.energy;
                speed_at_kick = before.w.z.abs();
            }
            total_vec += work;
            total_yaw += yaw;
            correction_sum += correction;
            closure = closure.max((after.energy - energy_at_kick - total_vec).abs());
            yaw_closure = yaw_closure.max((after.energy - energy_at_kick - total_yaw).abs());
            per_step = per_step.max((delta_ke - work).abs());
            let elapsed = (t - 30) as f64 * 0.01;
            ideal = (ref_kick.abs() - torque * elapsed).max(0.).powi(2) / (2. * izz);
            max_ideal = max_ideal.max((after.energy - ideal).abs());
            let mut counts = vec![0; usize::from(f.substeps)];
            for p in &f.native_points {
                *counts
                    .get_mut(usize::from(p.microstep))
                    .ok_or("bad source microstep")? += 1;
            }
            let cap = body.friction
                * f.native_points
                    .iter()
                    .map(|p| p.normal_n_s * p.radius_m)
                    .sum::<f64>();
            if (cap - f.predicted_twist_limit_nms).abs() > 1e-10 {
                return Err("point pressure moment bound differs".into());
            }
            predicted_work = cap * avg.z.abs();
            if counts.iter().all(|&n| n == 4)
                && before.w.z.abs() > 0.2
                && after.w.z.abs() > 0.2
                && cap > 1e-9
            {
                saturated += 1;
                sum_sat -= yaw;
                bound_sat += predicted_work;
            }
        }
        samples.push(Sample {
            tick: t,
            energy_j: after.energy,
            ideal_j: ideal,
            omega_z: after.w.z,
            interval_work_vector_j: work,
            interval_work_yaw_j: yaw,
            interval_bound_work_j: predicted_work,
            work_closure_j: if t >= 31 {
                after.energy - energy_at_kick - total_vec
            } else {
                0.
            },
        });
        body = f.bodies[1].clone();
    }
    if samples.len() != 120
        || summary["trajectory_sha256"] != format!("{:x}", digest.finalize())
        || saturated != summary["saturated_intervals"].as_u64().unwrap()
    {
        return Err("incomplete native or saturation trace".into());
    }
    let terminal = samples.last().unwrap().energy_j;
    let sat_ratio = (bound_sat > 0.).then(|| ((sum_sat - bound_sat) / bound_sat).abs());
    let smooth = torque == 0.;
    let closure_ok = closure <= 0.001 * energy_at_kick;
    let terminal_ok = if smooth {
        (terminal - energy_at_kick).abs() <= 0.001 * energy_at_kick
    } else {
        terminal <= 0.0001 * energy_at_kick
    };
    let ideal_ok = max_ideal <= 0.02 * energy_at_kick;
    let sat_ok = sat_ratio.is_none_or(|d| d <= 0.01);
    let result = json!({"trajectory_sha256":summary["trajectory_sha256"],"peak_after_kick_speed_rad_s":speed_at_kick,"pre_kick_energy_j":pre_drive,"post_kick_energy_j":energy_at_kick,"terminal_energy_j":terminal,"trapezoidal_vector_work_j":total_vec,"trapezoidal_yaw_work_j":total_yaw,"sum_rotated_inertia_correction_j":correction_sum,"max_cumulative_full_work_residual_j":closure,"max_cumulative_yaw_only_residual_j":yaw_closure,"max_single_step_full_work_residual_j":per_step,"maximum_ideal_coulomb_energy_error_j":max_ideal,"saturated_intervals":saturated,"observed_resisting_yaw_work_j":sum_sat,"native_pressure_predicted_work_j":bound_sat,"relative_saturation_work_error":sat_ratio,"pass":{"full_rotational_work":closure_ok,"terminal_energy":terminal_ok,"ideal_energy_history":ideal_ok,"saturated_work":sat_ok},"pass_all":closure_ok&&terminal_ok&&ideal_ok&&sat_ok});
    Ok((result, samples))
}
fn main() -> Result<()> {
    let a = Args::parse();
    let runner = a.runner.canonicalize()?;
    let worker = a.worker.canonicalize()?;
    fs::create_dir(&a.directory)?;
    let generated = a.directory.join("observations");
    fs::create_dir(&generated)?;
    let records = a.directory.join("energy");
    fs::create_dir(&records)?;
    let mut cases = BTreeMap::new();
    let mut signed_pairs = BTreeMap::new();
    let mut rows = 0;
    let mut generated_runs = 0;
    let mut pass_all = true;
    for shape in ["rectangle", "square"] {
        for speed in ["moving", "stationary"] {
            for material in ["rough", "smooth"] {
                for resolution in ["fine", "coarse"] {
                    let mut signs = Vec::new();
                    for sign in ["positive", "negative"] {
                        let name = format!("{shape}-{sign}-{speed}-{material}-{resolution}");
                        let src = a.qualified.join(format!("{name}-packed"));
                        let baseline = read(&src.join("summary.json"))?;
                        let json_reference =
                            read(&a.qualified.join(format!("{name}-json/summary.json")))?;
                        if baseline["trajectory_sha256"] != json_reference["trajectory_sha256"] {
                            return Err("prior physical encodings differ".into());
                        }
                        let src = if src.join("frames.jsonl").is_file() {
                            src
                        } else if let Some(old) = a
                            .retained
                            .as_ref()
                            .map(|parent| parent.join(&name))
                            .filter(|p| p.join("frames.jsonl").is_file())
                        {
                            old
                        } else {
                            let out = generated.join(&name);
                            let mut cmd = Command::new(&runner);
                            cmd.arg("--worker")
                                .arg(&worker)
                                .args(["--frames", "--packed"]);
                            for (on, flag) in [
                                (shape == "rectangle", "--rectangular"),
                                (sign == "negative", "--reverse"),
                                (speed == "moving", "--moving"),
                                (material == "smooth", "--frictionless"),
                                (resolution == "coarse", "--coarse"),
                            ] {
                                if on {
                                    cmd.arg(flag);
                                }
                            }
                            let work = cmd.arg(&out).output()?;
                            if !work.status.success() {
                                return Err(format!(
                                    "{name} failed original qualification: {}",
                                    String::from_utf8_lossy(&work.stderr)
                                )
                                .into());
                            }
                            generated_runs += 1;
                            out
                        };
                        let (out, series) = analyse(&src, &baseline)?;
                        pass_all &= out["pass_all"] == true;
                        serde_json::to_writer(
                            File::create(records.join(format!("{name}.json")))?,
                            &series,
                        )?;
                        rows += series.len();
                        signs.push((
                            series,
                            out["post_kick_energy_j"].as_f64().unwrap(),
                            out["peak_after_kick_speed_rad_s"].as_f64().unwrap(),
                        ));
                        cases.insert(name, out);
                    }
                    let [(left, el, wl), (right, er, wr)] = &signs[..] else {
                        unreachable!()
                    };
                    let mut de = 0_f64;
                    let mut dw = 0_f64;
                    for (pos, neg) in left.iter().zip(right).skip(30) {
                        de = de.max((pos.energy_j - neg.energy_j).abs());
                        dw = dw.max((pos.omega_z + neg.omega_z).abs());
                    }
                    let erel = de / el.max(*er);
                    let wrel = dw / wl.max(*wr);
                    let ok = erel <= 0.01 && wrel <= 0.01;
                    pass_all &= ok;
                    signed_pairs.insert(format!("{shape}-{speed}-{material}-{resolution}"),json!({"max_energy_j":de,"max_signed_omega_sum_rad_s":dw,"relative_energy":erel,"relative_yaw_speed":wrel,"pass":ok}));
                }
            }
        }
    }
    torsional_energy_plot::draw(&a.directory)?;
    let report = json!({"physical_cases":cases.len(),"qualified_json_packed_cases":2*cases.len(),"fresh_repeated_runs":generated_runs,"native_barriers":rows,"predeclared_checks_all_pass":pass_all,"cases":cases,"positive_negative_pairs":signed_pairs});
    serde_json::to_writer_pretty(File::create(a.directory.join("summary.json"))?, &report)?;
    if !pass_all {
        return Err("one or more declared physical limits failed; see case-level summary".into());
    }
    println!(
        "All 32 six-DOF spin-energy, independent ideal braking, native point pressure work and 16 sign-pair bounds passed at {rows} native barriers; {generated_runs} fresh distributed runs."
    );
    Ok(())
}
