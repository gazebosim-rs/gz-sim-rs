// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the accepted and accurate range of a loaded two-link physical pendulum.
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, queue_link_world_wrench,
};
use serde::Serialize;
use serde_json::json;
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const DURATION_US: u32 = 350_000;
const PIVOT: DVec3 = DVec3::new(0., 0., 0.70);
const GRAVITY: f64 = 9.81;

#[derive(Parser)]
struct Args {
    /// Write the raw sweep, boundary brackets and a native replay to a new directory.
    output: PathBuf,
}

#[derive(Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Sweep {
    Absolute,
    ChildInertia,
}

#[derive(Clone, Copy, Serialize)]
struct Parameters {
    sweep: Sweep,
    log10_scale: f64,
    dt_us: u32,
    root_mass_kg: f64,
    child_mass_kg: f64,
    root_y_kg_m2: f64,
    child_y_kg_m2: f64,
}
impl Parameters {
    fn new(sweep: Sweep, log10_scale: f64, dt_us: u32) -> Self {
        let scale = 10_f64.powf(log10_scale);
        let base = match sweep {
            Sweep::Absolute => scale,
            Sweep::ChildInertia => 1.0,
        };
        Self {
            sweep,
            log10_scale,
            dt_us,
            root_mass_kg: 0.8 * base,
            child_mass_kg: 0.4 * base,
            root_y_kg_m2: 0.012 * base,
            child_y_kg_m2: 0.004 * scale,
        }
    }
    fn pivot_inertia(self) -> f64 {
        self.root_y_kg_m2
            + self.child_y_kg_m2
            + self.root_mass_kg * 0.10_f64.powi(2)
            + self.child_mass_kg * 0.28_f64.powi(2)
    }
    fn torque(self) -> f64 {
        5. * self.pivot_inertia()
    }
    fn gravity_acceleration(self) -> f64 {
        GRAVITY * (self.root_mass_kg * 0.10 + self.child_mass_kg * 0.28) / self.pivot_inertia()
    }
    fn sdf(self) -> String {
        let link = |name: &str,
                    x: f64,
                    com: f64,
                    mass: f64,
                    moments: [f64; 3],
                    tilt: f64,
                    size: &str,
                    color: &str| {
            let [xx, yy, zz] = moments;
            format!(
                r#"<link name="{name}"><pose>{x} 0 0 0 0 0</pose>
        <inertial><pose>{com} 0 0 0 {tilt} 0</pose><mass>{mass:e}</mass>
          <inertia><ixx>{xx:e}</ixx><iyy>{yy:e}</iyy><izz>{zz:e}</izz></inertia></inertial>
        <collision name="body"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="body"><geometry><box><size>{size}</size></box></geometry><material><diffuse>{color}</diffuse></material></visual>
      </link>"#
            )
        };
        let root = link(
            "root",
            0.08,
            0.02,
            self.root_mass_kg,
            [
                0.75 * self.root_y_kg_m2,
                self.root_y_kg_m2,
                1.25 * self.root_y_kg_m2,
            ],
            0.38,
            "0.16 0.08 0.07",
            "0.12 0.72 0.72 1",
        );
        let child = link(
            "payload",
            0.24,
            0.04,
            self.child_mass_kg,
            [
                0.75 * self.child_y_kg_m2,
                self.child_y_kg_m2,
                1.25 * self.child_y_kg_m2,
            ],
            -0.27,
            "0.15 0.07 0.065",
            "0.95 0.52 0.16 1",
        );
        let dt = f64::from(self.dt_us) * 1e-6;
        format!(
            r#"<?xml version="1.0"?>
<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity>
    <physics name="local" type="ignored"><max_step_size>{dt}</max_step_size></physics>
    <model name="arm"><pose>0 0 0.70 0 0 0</pose>
      {root}
      {child}
      <joint name="pivot" type="ball"><parent>world</parent><child>root</child><pose>-0.08 0 0 0 0 0</pose></joint>
      <joint name="weld" type="fixed"><parent>root</parent><child>payload</child><pose>-0.08 0 0 0 0 0</pose></joint>
    </model>
  </world></sdf>
"#
        )
    }
}

#[derive(Clone, Copy, Default)]
struct Reference {
    angle: f64,
    rate: f64,
}
impl Reference {
    fn advance(&mut self, dt_us: u32, gravity_acceleration: f64) {
        // theta'' = drive torque / I_pivot + g * sum(m*x0) / I_pivot * cos(theta)
        let acc = |q: f64| 5. + gravity_acceleration * q.cos();
        let substeps = dt_us.div_ceil(25);
        let h = 1e-6 * f64::from(dt_us) / f64::from(substeps);
        for _ in 0..substeps {
            let (q, w) = (self.angle, self.rate);
            let (k1q, k1w) = (w, acc(q));
            let (k2q, k2w) = (w + 0.5 * h * k1w, acc(q + 0.5 * h * k1q));
            let (k3q, k3w) = (w + 0.5 * h * k2w, acc(q + 0.5 * h * k2q));
            let (k4q, k4w) = (w + h * k3w, acc(q + h * k3q));
            self.angle += h * (k1q + 2. * k2q + 2. * k3q + k4q) / 6.;
            self.rate += h * (k1w + 2. * k2w + 2. * k3w + k4w) / 6.;
        }
    }
}

#[derive(Default, Serialize)]
struct Errors {
    anchor_gap_m: f64,
    orientation_rad: f64,
    angular_velocity_rad_s: f64,
    com_position_m: f64,
}
impl Errors {
    fn largest_fraction_of_tolerance(&self) -> f64 {
        (self.anchor_gap_m / 0.001)
            .max(self.orientation_rad / 0.01)
            .max(self.angular_velocity_rad_s / 0.10)
            .max(self.com_position_m / 0.005)
    }
    fn observe(&mut self, states: [BodyState; 2], exact: Reference) -> Result<()> {
        let pose = |s: BodyState, x: f64| {
            DVec3::from(s.position) + DQuat::from_array(s.orientation) * (x * DVec3::X)
        };
        let ideal = DQuat::from_rotation_y(exact.angle);
        for (body, offset, x) in [(states[0], 0.02, 0.10), (states[1], 0.04, 0.28)] {
            let q = DQuat::from_array(body.orientation);
            if !q.is_finite()
                || (q.length_squared() - 1.).abs() > 1e-9
                || !DVec3::from(body.position).is_finite()
                || !DVec3::from(body.velocity).is_finite()
                || !DVec3::from(body.angular_velocity).is_finite()
            {
                return Err("invalid body state at advancing boundary".into());
            }
            self.com_position_m = self
                .com_position_m
                .max(pose(body, offset).distance(PIVOT + ideal * (x * DVec3::X)));
            self.orientation_rad = self.orientation_rad.max(q.angle_between(ideal).abs());
            self.angular_velocity_rad_s = self
                .angular_velocity_rad_s
                .max(DVec3::from(body.angular_velocity).distance(DVec3::Y * exact.rate));
        }
        self.anchor_gap_m = self
            .anchor_gap_m
            .max(pose(states[0], -0.08).distance(PIVOT))
            .max(pose(states[0], 0.08).distance(pose(states[1], -0.08)));
        Ok(())
    }
}

#[derive(Serialize)]
struct Case {
    #[serde(flatten)]
    params: Parameters,
    status: &'static str,
    detail: Option<String>,
    completed_us: u32,
    max_errors: Errors,
    p95_step_ns: Option<u64>,
}
impl Case {
    fn passes(&self) -> bool {
        self.status == "pass"
    }
}

#[allow(deprecated)]
fn links(sim: &Simulation) -> Result<[(u64, gz_physics::BodyId); 2]> {
    let lookup = |name: &str| -> Result<(u64, gz_physics::BodyId)> {
        sim.world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|n| n.0 == name))
            .and_then(|entity| {
                Some((
                    entity.get::<EntityId>()?.get(),
                    entity.get::<BodyHandle>()?.id(),
                ))
            })
            .ok_or_else(|| format!("missing {name}").into())
    };
    Ok([
        lookup("default::arm::root")?,
        lookup("default::arm::payload")?,
    ])
}

fn run(params: Parameters, replay: Option<&Path>) -> Result<Case> {
    let xml = params.sdf();
    let mut case = Case {
        params,
        status: "load_refused",
        detail: None,
        completed_us: 0,
        max_errors: Errors::default(),
        p95_step_ns: None,
    };
    let mut sim = match load_world_with_sdf_step(
        &xml,
        "loaded-inertia.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    ) {
        Ok(sim) => sim,
        Err(e) => {
            case.detail = Some(e.to_string());
            return Ok(case);
        }
    };
    let links = links(&sim)?;
    let mut exact = Reference::default();
    let n = DURATION_US / params.dt_us;
    let mut times = Vec::with_capacity(n as usize);
    let sdf = replay
        .map(|_| sdformat::load_str(&xml, "loaded-inertia.sdf", &sdformat::NoResources))
        .transpose()?;
    let mut frames = Vec::new();
    if let Some(sdf) = &sdf {
        frames.push(gz_sim_cli::primitive_frame(&sim, sdf, "default", 1)?);
    }
    for tick in 1..=n {
        if let Err(e) =
            queue_link_world_wrench(&mut sim, links[1].0, [0.; 3], [0., params.torque(), 0.])
        {
            case.status = "load_fault";
            case.detail = Some(e.to_string());
            return Ok(case);
        }
        let start = Instant::now();
        if let Err(e) = sim.step_exact(1) {
            case.status = "step_fault";
            case.detail = Some(e.to_string());
            return Ok(case);
        }
        times.push(start.elapsed().as_nanos() as u64);
        case.completed_us = tick * params.dt_us;
        exact.advance(params.dt_us, params.gravity_acceleration());
        let physical = sim.world().resource::<PhysicsBackend>();
        let state = |index: usize| {
            physical
                .state(links[index].1)
                .ok_or("native body went missing")
        };
        if let Err(e) = case.max_errors.observe([state(0)?, state(1)?], exact) {
            case.status = "invalid_state";
            case.detail = Some(e.to_string());
            return Ok(case);
        }
        if let Some(sdf) = &sdf
            && (case.completed_us.is_multiple_of(5_000) || tick == n)
        {
            frames.push(gz_sim_cli::primitive_frame(
                &sim,
                sdf,
                "default",
                frames.len() as u64 + 1,
            )?);
        }
    }
    times.sort_unstable();
    case.p95_step_ns = Some(times[(times.len() * 95).div_ceil(100) - 1]);
    case.status = if case.max_errors.largest_fraction_of_tolerance() <= 1.0 {
        "pass"
    } else {
        "outside_tolerance"
    };
    if let Some(output) = replay {
        fs::write(output.join("representative.sdf"), &xml)?;
        let last = frames.len() - 1;
        gz_sim_cli::export_archive(&output.join("replay"), Default::default(), frames)?;
        write(
            output.join("frames.json"),
            &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"captures":{"midpoint":last/2,"end":last}}),
        )?;
    }
    Ok(case)
}
fn write(path: PathBuf, data: &impl Serialize) -> Result<()> {
    fs::write(path, serde_json::to_vec_pretty(data)?)?;
    Ok(())
}
/// Plot measured samples, using the same four tolerances as the pass decision.
fn plot_cases(path: &Path, cases: &[Case]) -> Result<()> {
    use plotters::prelude::*;
    let figure = SVGBackend::new(path, (830, 720)).into_drawing_area();
    figure.fill(&WHITE)?;
    let rows = figure.split_evenly((2, 1));
    for (sweep, title, panel) in [
        (
            Sweep::Absolute,
            "Scale both masses, both tensors and torque together",
            &rows[0],
        ),
        (
            Sweep::ChildInertia,
            "Scale payload inertia with fixed masses and root tensor",
            &rows[1],
        ),
    ] {
        let mut samples: Vec<_> = cases
            .iter()
            .filter(|case| case.params.sweep == sweep && case.completed_us == DURATION_US)
            .map(|case| {
                (
                    case.params.dt_us,
                    case.params.log10_scale,
                    case.max_errors.largest_fraction_of_tolerance().log10(),
                )
            })
            .collect();
        samples.sort_by(|a, b| a.1.total_cmp(&b.1));
        let ymin = samples.iter().map(|v| v.2).fold(0.0, f64::min).floor() as i32 - 1;
        let ymax = samples.iter().map(|v| v.2).fold(0.0, f64::max).ceil() as i32 + 1;
        let mut chart = ChartBuilder::on(panel)
            .margin(14)
            .caption(title, ("sans-serif", 19))
            .x_label_area_size(40)
            .y_label_area_size(80)
            .build_cartesian_2d(-31.0..31.0, f64::from(ymin)..f64::from(ymax))?;
        let scientific = |power: &f64| format!("10^{power:.0}");
        chart
            .configure_mesh()
            .x_desc("Scale relative to nominal")
            .y_desc("Largest error / tolerance")
            .x_label_formatter(&scientific)
            .y_label_formatter(&scientific)
            .light_line_style(RGBColor(234, 237, 238))
            .draw()?;
        chart
            .draw_series(LineSeries::new([(-31., 0.), (31., 0.)], BLACK.mix(0.65)))?
            .label("Acceptance limit")
            .legend(|(x, y)| PathElement::new([(x, y), (x + 18, y)], BLACK));
        for (dt, color) in [
            (500, RGBColor(0, 117, 113)),
            (1000, RGBColor(34, 69, 135)),
            (2000, RGBColor(185, 78, 4)),
        ] {
            let points: Vec<_> = samples
                .iter()
                .filter(|row| row.0 == dt)
                .map(|row| (row.1, row.2))
                .collect();
            chart
                .draw_series(LineSeries::new(points.iter().copied(), color))?
                .label(format!("{:.1} ms", f64::from(dt) / 1000.))
                .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], color));
            chart.draw_series(
                points
                    .into_iter()
                    .map(|p| Circle::new(p, 2, color.filled())),
            )?;
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperMiddle)
            .background_style(WHITE.mix(0.93))
            .border_style(RGBColor(190, 198, 201))
            .draw()?;
    }
    figure.present()?;
    Ok(())
}
fn campaign(output: &Path) -> Result<()> {
    fs::create_dir_all(output)?;
    if output.join("replay").exists() {
        return Err("choose a fresh output directory".into());
    }
    let mut cases = Vec::new();
    let mut boundaries = Vec::new();
    for grid in [500, 1000, 2000] {
        for sweep in [Sweep::Absolute, Sweep::ChildInertia] {
            let nominal = run(Parameters::new(sweep, 0., grid), None)?;
            if !nominal.passes() {
                return Err(
                    format!("nominal case failed: {}", serde_json::to_string(&nominal)?).into(),
                );
            }
            cases.push(nominal);
            for sign in [-1., 1.] {
                let mut pass = 0.;
                let mut first_fail = None;
                for decade in 1..=30 {
                    if decade > 12 && first_fail.is_some() {
                        break;
                    }
                    let power = sign * f64::from(decade);
                    let case = run(Parameters::new(sweep, power, grid), None)?;
                    if first_fail.is_none() {
                        if case.passes() {
                            pass = power
                        } else {
                            first_fail = Some(power)
                        }
                    }
                    cases.push(case);
                }
                if let Some(mut fail) = first_fail {
                    while f64::abs(fail - pass) > 0.10 {
                        let mid = (pass + fail) * 0.5;
                        let case = run(Parameters::new(sweep, mid, grid), None)?;
                        if case.passes() {
                            pass = mid
                        } else {
                            fail = mid
                        }
                        cases.push(case);
                    }
                    first_fail = Some(fail);
                }
                boundaries.push(json!({"sweep":sweep,"dt_us":grid,"direction":sign,"passing":Parameters::new(sweep,pass,grid),"nearest_failure":first_fail.map(|p|Parameters::new(sweep,p,grid))}));
            }
        }
    }
    let sample = run(Parameters::new(Sweep::Absolute, 0., 1000), Some(output))?;
    write(output.join("nominal.json"), &sample)?;
    plot_cases(&output.join("accuracy.svg"), &cases)?;
    write(output.join("cases.json"), &cases)?;
    write(output.join("boundaries.json"), &boundaries)?;
    println!(
        "{} of {} loaded cases met tolerance. Results in {}",
        cases.iter().filter(|case| case.passes()).count(),
        cases.len(),
        output.display()
    );
    Ok(())
}
fn main() -> Result<()> {
    campaign(&Args::parse().output)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reference_integrates_constant_drive_from_rest() {
        let mut reference = Reference::default();
        for _ in 0..350 {
            reference.advance(1000, 0.0);
        }
        assert!((reference.angle - 0.5 * 5.0 * 0.35_f64.powi(2)).abs() < 1e-11);
        assert!((reference.rate - 5.0 * 0.35).abs() < 1e-11);
    }
    #[test]
    fn loaded_pendulum_matches_independent_reference_at_both_grid_ends() -> Result<()> {
        for grid in [500, 2000] {
            for (sweep, power) in [
                (Sweep::Absolute, 0.),
                (Sweep::Absolute, -18.),
                (Sweep::Absolute, 18.),
                (Sweep::ChildInertia, -6.),
                (Sweep::ChildInertia, 18.),
            ] {
                let case = run(Parameters::new(sweep, power, grid), None)?;
                assert!(case.passes(), "{}", serde_json::to_string(&case)?);
                assert_eq!(case.completed_us, DURATION_US);
            }
        }
        Ok(())
    }
}
