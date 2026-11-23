// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Pinch and lift a free XPBD cable with the articulated four-drive arm.
use super::*;
use crate::{arm::Controller, gripper::ramp};
use glam::DQuat;
use gz_sim_scene::deformable::{DeformableSample, bodies};
use serde::Serialize;
use std::fmt::Write;

const COUNT: usize = 11;
const SPACING: f64 = 0.022;
const RADIUS: f64 = 0.025;
const NODE_MASS: f64 = 0.002;
const BENCH_Z: f64 = 0.12;
const DT: f64 = 0.001;
const WEIGHT: f64 = NODE_MASS * COUNT as f64 * 9.81;
const PAD_COMPLIANCE: f64 = 0.001;
const STRETCH: f64 = 0.00001;
const CHORD: f64 = 0.003;
const LIFT_TRAVEL: f64 = 0.15;
const MU: f64 = 0.7;
const END: usize = 3400;
const STRIDE: usize = 25;
const HOLD: std::ops::RangeInclusive<usize> = 1651..=1950;
const RELEASED_TICKS: usize = 200;

fn sphere_name(i: usize) -> String {
    format!("default::source::body::cable_{i}")
}

// Use the robot's original links, joints, inertias, drive limits and solver settings.
// The wider workbench covers the cable footprint during each pickup and release.
fn scene(robot_sdf: &str, substeps: usize, reverse_jaws: bool) -> Result<String> {
    let (arm, rest) = robot_sdf
        .split_once("<model name=\"parcel\">")
        .ok_or("missing robot parcel")?;
    let (_, rest) = rest
        .split_once("<model name=\"source\">")
        .ok_or("missing source stand")?;
    let (source, rest) = rest.split_once("</link>").ok_or("missing source link")?;
    let mut xml = format!(
        "{arm}<model name=\"source\">{}",
        source.replace("0.2 0.18 0.12", "0.4 0.6 0.12")
    );
    let mut particles = String::new();
    let mut visuals = String::new();
    for i in 0..COUNT {
        let y = (i as f64 - (COUNT - 1) as f64 / 2.) * SPACING;
        let z = BENCH_Z + RADIUS;
        writeln!(
            particles,
            "<particle position=\"0.32 {y} {z}\" mass=\"{NODE_MASS}\"/>"
        )
        .expect("formatting into a String is infallible");
        writeln!(
            xml,
            "<visual name=\"cable_{i}\"><pose>0 {y} {} 0 0 0</pose><geometry><sphere><radius>{RADIUS}</radius></sphere></geometry><material><diffuse>0.92 0.45 0.09 1</diffuse></material></visual>",
            z - 0.06
        ).expect("formatting into a String is infallible");
        writeln!(
            visuals,
            "<particle_visual material=\"cable\" particle=\"{i}\" visual=\"{}\"/>",
            sphere_name(i)
        )
        .expect("formatting into a String is infallible");
    }
    let [first, second] = if reverse_jaws {
        ["right", "left"]
    } else {
        ["left", "right"]
    };
    let (world, _) = rest.split_once("</world>").ok_or("missing world")?;
    write!(
        xml,
        "</link>{world}<plugin name=\"gz::sim::systems::Deformables\" filename=\"gz-sim-deformables-system\">\n\
        <box link=\"default::arm::{first}\" coupling=\"articulated_wrench\" normal_compliance=\"{PAD_COMPLIANCE}\"/>\n\
        <box link=\"default::arm::{second}\" coupling=\"articulated_wrench\" normal_compliance=\"{PAD_COMPLIANCE}\"/>\n\
        <plane point=\"0 0 {BENCH_Z}\" normal=\"0 0 1\"/>\n\
        <cable name=\"cable\" substeps=\"{substeps}\" iterations=\"8\" radius=\"{RADIUS}\" self_contact=\"false\" friction=\"{MU}\" stretch=\"{STRETCH}\" bend=\"{CHORD}\">{particles}</cable>{visuals}</plugin></world></sdf>"
    ).expect("formatting into a String is infallible");
    Ok(xml)
}

fn sample(sim: &Simulation) -> Result<&DeformableSample> {
    bodies(sim)
        .find(|(name, _)| *name == "cable")
        .map(|(_, s)| s)
        .ok_or_else(|| "no measured cable".into())
}

fn drive(c: &Controller, sim: &mut Simulation, t: f64, closed: bool) -> Result<()> {
    // Lower the jaws before closing so a 5 mm clearance remains above the workbench.
    let (approach, va) = ramp(t, 0., 0.25);
    let (close, vc) = ramp(t, 0.10, 0.55);
    let (raise, vu) = ramp(t, 0.85, 0.55);
    let (lower, vd) = ramp(t, 2.0, 0.55);
    let (open, vo) = ramp(t, 2.65, 0.35);
    let stroke = if closed { 0.075 } else { 0. };
    let jaw = (stroke * (close - open), stroke * (vc - vo), 0.);
    c.actuate(
        sim,
        [
            (0., 0., 0.),
            (
                LIFT_TRAVEL * (raise - lower) - 0.010 * approach,
                LIFT_TRAVEL * (vu - vd) - 0.010 * va,
                0.96 * 9.81,
            ),
            jaw,
            jaw,
        ],
        true,
    )?;
    Ok(())
}
#[derive(Debug, Serialize, Default)]
struct Metrics {
    substeps: usize,
    closed: bool,
    mean_hold_lift_m: f64,
    mean_hold_tip_sag_m: [f64; 2],
    hold_middle_downward_drift_m: f64,
    mean_hold_jaw_up_n: [f64; 2],
    mean_hold_grip_n: [f64; 2],
    mean_hold_bench_up_n: f64,
    mean_released_bench_up_n: f64,
    mean_released_jaw_force_n: f64,
    max_edge_strain: f64,
    max_pad_compression_m: f64,
    min_bench_footprint_margin_m: f64,
    max_linear_impulse_residual_ns: f64,
    min_hold_floor_gap_m: f64,
    final_cm_lift_m: f64,
    step_p95_ns: u64,
}

impl Metrics {
    fn qualify(&self) -> Result<()> {
        let weight_tolerance = 0.02 * WEIGHT;
        let released = (self.mean_released_bench_up_n - WEIGHT).abs() < weight_tolerance
            && self.mean_released_jaw_force_n < 1e-4
            && self.final_cm_lift_m.abs() < 0.002;
        let within_fixture = self.min_bench_footprint_margin_m > 0.
            && self.max_pad_compression_m < 0.1 * RADIUS
            && self.max_edge_strain < 0.01
            && self.max_linear_impulse_residual_ns < 1e-7;
        let expected = if self.closed {
            let jaw_up: f64 = self.mean_hold_jaw_up_n.iter().sum();
            self.mean_hold_lift_m > 0.10
                && self.hold_middle_downward_drift_m.abs() < 0.001
                && self.min_hold_floor_gap_m > 0.025
                && self
                    .mean_hold_tip_sag_m
                    .iter()
                    .all(|&v| (0.001..0.112).contains(&v))
                && self.mean_hold_bench_up_n.abs() < 1e-4
                && (jaw_up - WEIGHT).abs() < weight_tolerance
                && self
                    .mean_hold_jaw_up_n
                    .iter()
                    .zip(self.mean_hold_grip_n)
                    .all(|(&up, grip)| {
                        (up - WEIGHT / 2.).abs() < 0.02 * WEIGHT / 2.
                            && up <= MU * grip + weight_tolerance
                    })
        } else {
            self.mean_hold_lift_m.abs() < 1e-6
                && (self.mean_hold_bench_up_n - WEIGHT).abs() < weight_tolerance
                && self.mean_hold_jaw_up_n == [0.; 2]
                && self.mean_hold_grip_n == [0.; 2]
        };
        if released && within_fixture && expected {
            Ok(())
        } else {
            Err(format!("cable grasp qualification failed: {self:?}").into())
        }
    }
}

#[derive(Serialize)]
struct ParticleFrame {
    time_ns: u128,
    positions: Vec<[f64; 3]>,
}

fn capture(r: &mut Recording, frames: &mut Vec<ParticleFrame>) -> Result<()> {
    let cable = sample(&r.sim)?;
    frames.push(ParticleFrame {
        time_ns: cable.sim_time().as_nanos(),
        positions: cable.particles().iter().map(|p| p.position).collect(),
    });
    r.capture()
}

// The scene adapter already requires centered collision boxes on these links.
fn half_size(sdf: &sdformat::Sdf, model: &str, link: &str) -> Result<DVec3> {
    let collision = sdf
        .worlds
        .iter()
        .flat_map(|w| &w.models)
        .find(|m| m.name == model)
        .and_then(|m| m.links.iter().find(|l| l.name == link))
        .and_then(|l| l.collisions.first());
    if let Some(sdformat::Geometry::Box { size }) = collision.map(|c| &c.geometry) {
        Ok(DVec3::from(*size) * 0.5)
    } else {
        Err(format!("missing box geometry on {model}::{link}").into())
    }
}

fn cycle(substeps: usize, closed: bool, reverse_jaws: bool, out: Option<&Path>) -> Result<Metrics> {
    cycle_with_scene(
        &scene(arm::SCENE, substeps, reverse_jaws)?,
        substeps,
        closed,
        reverse_jaws,
        out,
    )
}

fn cycle_with_scene(
    xml: &str,
    substeps: usize,
    closed: bool,
    reverse_jaws: bool,
    out: Option<&Path>,
) -> Result<Metrics> {
    let mut r = Recording::new(xml, "arm_cable.sdf", "default")?;
    let c = Controller::new(&r.sim)?;
    let jaws = [
        entity_id(&r.sim, "default::arm::left")?,
        entity_id(&r.sim, "default::arm::right")?,
    ];
    let pads = [
        half_size(&r.sdf, "arm", "left")?,
        half_size(&r.sdf, "arm", "right")?,
    ];
    let bench_half = half_size(&r.sdf, "source", "body")?;
    let bench_center = state(&r.sim, entity_id(&r.sim, "default::source::body")?)?.position;
    let hold_ticks = HOLD.clone().count() as f64;
    let mut m = Metrics {
        substeps,
        closed,
        min_hold_floor_gap_m: f64::INFINITY,
        min_bench_footprint_margin_m: f64::INFINITY,
        ..Metrics::default()
    };
    let mut frames = vec![];
    if out.is_some() {
        capture(&mut r, &mut frames)?;
    }
    let mut prior_momentum = DVec3::ZERO;
    let mut initial_middle_relative_z = 0.;
    for tick in 1..=END {
        drive(&c, &mut r.sim, (tick - 1) as f64 * DT, closed)?;
        r.advance().map_err(|error| {
            format!(
                "cable tick {tick} ({substeps} substeps): {error} {:?}",
                r.sim.fault()
            )
        })?;
        let cable = sample(&r.sim)?;
        let points = cable.particles();
        let impulses = cable.last_boundary_impulses();
        let [first, second, bench] = impulses else {
            return Err("missing jaw or bench reaction".into());
        };
        let (left, right) = if reverse_jaws {
            (second, first)
        } else {
            (first, second)
        };
        let jaw_j = [DVec3::from(left.linear), DVec3::from(right.linear)];
        let momentum = points
            .iter()
            .map(|p| NODE_MASS * DVec3::from(p.velocity))
            .sum::<DVec3>();
        let balance = momentum - prior_momentum
            + WEIGHT * DT * DVec3::Z
            + jaw_j[0]
            + jaw_j[1]
            + DVec3::from(bench.linear);
        m.max_linear_impulse_residual_ns = m.max_linear_impulse_residual_ns.max(balance.length());
        prior_momentum = momentum;
        for pair in points.windows(2) {
            let length = DVec3::from(pair[0].position).distance(pair[1].position.into());
            m.max_edge_strain = m.max_edge_strain.max((length / SPACING - 1.).abs());
        }
        let cm_z = points.iter().map(|p| p.position[2]).sum::<f64>() / COUNT as f64;
        let lift = cm_z - BENCH_Z - RADIUS;
        // The soft plane is infinite. Its supported footprint must remain on the real bench.
        for p in points {
            let margin = (bench_half.x - (p.position[0] - bench_center[0]).abs())
                .min(bench_half.y - (p.position[1] - bench_center[1]).abs())
                - RADIUS;
            m.min_bench_footprint_margin_m = m.min_bench_footprint_margin_m.min(margin);
        }
        let mut jaw_height = 0.;
        for (i, id) in jaws.iter().enumerate() {
            let jaw = state(&r.sim, *id)?;
            jaw_height += 0.5 * jaw.position[2];
            let rot = DQuat::from_array(jaw.orientation);
            for p in points {
                let local = rot.conjugate() * (DVec3::from(p.position) - DVec3::from(jaw.position));
                let q = local.abs() - pads[i];
                let distance = q.max(DVec3::ZERO).length() + q.max_element().min(0.);
                m.max_pad_compression_m = m.max_pad_compression_m.max(RADIUS - distance);
            }
            if HOLD.contains(&tick) {
                let outward = (rot * DVec3::X) * if i == 0 { -1. } else { 1. };
                m.mean_hold_grip_n[i] += jaw_j[i].dot(outward) / (DT * hold_ticks);
                m.mean_hold_jaw_up_n[i] += -jaw_j[i].z / (DT * hold_ticks);
            }
        }
        if HOLD.contains(&tick) {
            m.mean_hold_lift_m += lift / hold_ticks;
            let z = points[COUNT / 2].position[2];
            let relative = z - jaw_height;
            if tick == *HOLD.start() {
                initial_middle_relative_z = relative;
            }
            m.hold_middle_downward_drift_m = initial_middle_relative_z - relative;
            for (i, index) in [0, COUNT - 1].into_iter().enumerate() {
                m.mean_hold_tip_sag_m[i] += (z - points[index].position[2]) / hold_ticks;
            }
            let lowest = points
                .iter()
                .map(|p| p.position[2])
                .fold(f64::INFINITY, f64::min);
            m.min_hold_floor_gap_m = m.min_hold_floor_gap_m.min(lowest - BENCH_Z - RADIUS);
            m.mean_hold_bench_up_n += -bench.linear[2] / (DT * hold_ticks);
        }
        if tick > END - RELEASED_TICKS {
            m.mean_released_bench_up_n += -bench.linear[2] / (DT * RELEASED_TICKS as f64);
            m.mean_released_jaw_force_n +=
                (jaw_j[0].length() + jaw_j[1].length()) / (DT * RELEASED_TICKS as f64);
        }
        m.final_cm_lift_m = lift;
        if out.is_some() && tick % STRIDE == 0 {
            capture(&mut r, &mut frames)?;
        }
    }
    r.costs_ns.sort_unstable();
    m.step_p95_ns = r.costs_ns[(r.costs_ns.len() * 95).div_ceil(100) - 1];
    if let Some(out) = out {
        r.save(out)?;
        write_json(
            &out.join("particle-frames.json"),
            &serde_json::to_value(frames)?,
        )?;
        write_json(
            &out.join("frames.json"),
            &json!({"kind":"authored-archive-v1","camera":"front","dt_ns":1,"particle_visuals":(0..COUNT).map(sphere_name).collect::<Vec<_>>(),"captures":{"pickup":26,"rising":45,"hold":72,"release":112,"settled":136}}),
        )?;
    }
    Ok(m)
}

pub(super) fn run(out: &Path) -> Result<()> {
    let mut rows = vec![];
    for substeps in [1, 2, 4] {
        for closed in [true, false] {
            let dir = (substeps == 2).then(|| out.join(if closed { "held" } else { "open" }));
            let metrics = cycle(substeps, closed, false, dir.as_deref())?;
            metrics.qualify()?;
            rows.push(metrics);
        }
    }
    let reversed = cycle(2, true, true, None)?;
    reversed.qualify()?;
    write_json(&out.join("comparison.json"), &comparison(&rows, &reversed)?)
}

fn comparison(rows: &[Metrics], reversed: &Metrics) -> Result<Value> {
    // Each adjacent pair shares a material clock and differs only by its grasp command.
    let [coarse, _, normal, _, fine, _] = rows else {
        return Err("expected the three paired material clocks".into());
    };
    let mut max_lift_delta = 0_f64;
    let mut max_tip_delta = 0_f64;
    for held in [coarse, normal] {
        max_lift_delta = max_lift_delta.max((held.mean_hold_lift_m - fine.mean_hold_lift_m).abs());
        for (tip, reference) in held
            .mean_hold_tip_sag_m
            .iter()
            .zip(fine.mean_hold_tip_sag_m)
        {
            max_tip_delta = max_tip_delta.max((tip - reference).abs());
        }
    }
    if max_lift_delta > 0.002 || max_tip_delta > 0.004 {
        return Err(format!("changing material clock changed lift by {max_lift_delta} m or tip sag by {max_tip_delta} m").into());
    }
    let order_difference = (normal.mean_hold_jaw_up_n[0] - reversed.mean_hold_jaw_up_n[0])
        .abs()
        .max((normal.mean_hold_jaw_up_n[1] - reversed.mean_hold_jaw_up_n[1]).abs());
    let order_lift = (normal.mean_hold_lift_m - reversed.mean_hold_lift_m).abs();
    if order_difference > 1e-6 || order_lift > 1e-7 {
        return Err(format!(
            "jaw order changed mean support by {order_difference} N or lift by {order_lift} m"
        )
        .into());
    }
    Ok(
        json!({"weight_n":WEIGHT,"particle_count":COUNT,"node_mass_kg":NODE_MASS,
            "particle_radius_m":RADIUS,"material_length_m":SPACING*(COUNT-1) as f64,
            "pad_normal_compliance_m_per_n":PAD_COMPLIANCE, "friction":MU,
            "stretch_compliance_m_per_n":STRETCH,"chord_bend_compliance_m_per_n":CHORD, "commanded_lift_m":LIFT_TRAVEL,
            "hold_time_s":[1.65,1.95],"released_time_s":[3.2,3.4],
            "max_lift_difference_from_four_substeps_m":max_lift_delta,
            "max_tip_sag_difference_from_four_substeps_m":max_tip_delta,
            "trials":rows, "reversed_jaw_order_control":reversed, "max_order_change_in_jaw_support_n":order_difference, "order_change_in_lift_m":order_lift}),
    )
}

#[cfg(test)]
#[path = "arm_cable_tests.rs"]
mod tests;
