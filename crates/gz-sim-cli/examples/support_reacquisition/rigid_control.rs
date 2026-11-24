// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Remove the joint while preserving the mass, centre of mass, inertia and sole geometry.
use super::*;
use gz_physics::{BodySpec, DynamicBackend, Inertia, RotationSpec, Shape};
use std::path::Path;

const G: f64 = 9.81;
const MASS: f64 = 2.;
fn ideal(stroke: &Stroke) -> (f64, f64) {
    let down = stroke.down as f64 * 0.0005;
    let c = 1.25 * down + stroke.up * (down + 0.012);
    let t = 2. * c / (stroke.up + (stroke.up.powi(2) + 2. * G * c).sqrt());
    (t, MASS * (stroke.up + G * t))
}

struct Reading {
    impulse: f64,
    gap: f64,
    vz: f64,
    upper_gap: f64,
}

fn run(step_us: u64) -> Result<Value> {
    let dt = std::time::Duration::from_micros(step_us);
    let mut sim = Simulation::new(dt)?;
    let mut physics = MultiFidelityBackend::new([0., 0., -G])?;
    let cube = |half_extents, z, mass| BodySpec {
        shape: Shape::Box { half_extents },
        position: [0., 0., z],
        velocity: [0.; 3],
        mass,
    };
    // Origin at sole centre. The parallel-axis sum about the aggregate COM
    // gives 2*(0.0201+0.07^2)=0.05 for X/Y and 2*0.025=0.05 for Z.
    let body = physics.spawn_link(
        cube(FOOT_HALF, 0.1, Some(MASS)),
        RotationSpec::default(),
        Some(Inertia {
            center: [0., 0., 0.07],
            orientation: [0., 0., 0., 1.],
            tensor: [0.05, 0.05, 0.05, 0., 0., 0.],
        }),
    )?;
    let pad = physics.spawn_kinematic_link(
        cube(PAD_HALF, -0.06, Some(1.)),
        RotationSpec::default(),
        None,
    )?;
    gz_physics::install(&mut sim, physics)?;
    sim.step_exact(1_200_000 / step_us)?;
    let read = |sim: &Simulation| -> Result<Reading> {
        let physics = sim.world().resource::<PhysicsBackend>();
        let b = physics.state(body).ok_or("body unavailable")?;
        let p = physics.state(pad).ok_or("lift unavailable")?;
        let q = DQuat::from_array(b.orientation);
        let gap = b.position[2]
            - (q.conjugate() * DVec3::Z).abs().dot(FOOT_HALF.into())
            - p.position[2]
            - PAD_HALF[2];
        let solver = physics
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local solver")?;
        let impulse = solver
            .rigid_contact_observations()
            .iter()
            .filter(|c| (c.a, c.b) == (body, pad) || (c.a, c.b) == (pad, body))
            .map(|c| c.normal_impulse)
            .sum();
        Ok(Reading {
            impulse,
            gap,
            vz: b.velocity[2],
            upper_gap: b.position[2] + (q * (DVec3::Z * 0.14)).z
                - (q.conjugate() * DVec3::Z)
                    .abs()
                    .dot(DVec3::new(0.05, 0.06, 0.06))
                - p.position[2]
                - PAD_HALF[2],
        })
    };
    let mut frames = vec![read(&sim)?];
    let mut last = 0.;
    for tick in 0..sweep::END_US / step_us {
        let (_, v) = sweep::command(tick * step_us, 0);
        if v != last {
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .set_world_velocity(pad, [0., 0., v], [0.; 3])?;
            last = v;
        }
        sim.step_exact(1)?;
        frames.push(read(&sim)?);
    }
    let upper_clearance = frames
        .iter()
        .map(|f| f.upper_gap)
        .fold(f64::INFINITY, f64::min);
    if upper_clearance <= 0.0 {
        return Err("upper shape would collide in single-body control".into());
    }
    let win = (5000 / step_us) as usize;
    let mut cycles = Vec::new();
    for (i, departure) in sweep::DEPART_US.into_iter().enumerate() {
        let begin = (departure / step_us + 1) as usize;
        let end = sweep::DEPART_US
            .get(i + 1)
            .map_or(frames.len(), |next| (next / step_us + 1) as usize);
        let mut airborne = false;
        let mut landings = Vec::new();
        let mut peak_gap = 0.0_f64;
        for k in begin..end {
            let r = &frames[k];
            peak_gap = peak_gap.max(r.gap);
            if r.gap > 1e-5 && r.impulse == 0. {
                airborne = true;
            }
            if airborne && r.impulse > 0. {
                let window = frames
                    .get(k..k + win)
                    .ok_or("incomplete rigid impact window")?;
                let impulse: f64 = window.iter().map(|f| f.impulse).sum();
                let momentum = MASS * (window[win - 1].vz - frames[k - 1].vz + G * 0.005);
                landings.push(json!({"tick":k,"time_s":(k as u64*step_us) as f64*1e-6,
                    "first_ns":r.impulse,"window_ns":impulse,"window_vertical_momentum_ns":momentum}));
                airborne = false;
            }
        }
        if landings.is_empty() {
            return Err(format!("single rigid body missed stroke {}", i + 1).into());
        }
        let (impact, impulse) = ideal(&STROKES[i]);
        cycles.push(
            json!({"cycle":i+1,"ideal_impact_s":departure as f64*1e-6+impact,
        "ideal_first_ns":impulse,"ideal_5ms_ns":impulse+MASS*G*0.005,
        "landings":landings,"peak_gap_m":peak_gap}),
        );
    }
    Ok(
        json!({"dt_us":step_us,"cycles":cycles,"initial_sole_gap_m":frames[0].gap,"min_upper_clearance_m":upper_clearance}),
    )
}

pub(super) fn write(output: &Path) -> Result<()> {
    let runs = [500, 250, 125]
        .map(run)
        .into_iter()
        .collect::<Result<Vec<_>>>()?;
    fs::create_dir_all(output)?;
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(&json!({
            "mass_kg":MASS,"inertia_kg_m2":[0.05,0.05,0.05],"runs":runs,
            "ideal_assumptions":"Each departure starts level at rest, vertical free fall, perfectly inelastic central catch and sustained contact during the next 5 ms."
        }))?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn compound_mass_control_balances_each_landing_window_on_all_three_grids() -> Result<()> {
        for step in [500, 250, 125] {
            let result = run(step)?;
            assert!(result["min_upper_clearance_m"].as_f64().unwrap() > 0.13);
            for cycle in result["cycles"].as_array().unwrap() {
                let hits = cycle["landings"].as_array().unwrap();
                assert!(!hits.is_empty() && cycle["peak_gap_m"].as_f64().unwrap() > 0.025);
                for hit in hits {
                    let measured = hit["window_ns"].as_f64().unwrap();
                    let balance = hit["window_vertical_momentum_ns"].as_f64().unwrap();
                    assert!((measured - balance).abs() < 1e-9);
                }
            }
        }
        Ok(())
    }

    #[test]
    fn ideal_catch_crosses_the_rising_plateau_and_closes_momentum_balance() {
        for stroke in &STROKES {
            let (t, impulse) = ideal(stroke);
            let down = stroke.down as f64 * 0.0005;
            let rise = down + 0.012;
            assert!(t > rise && t + 0.005 < rise + stroke.plateau as f64 * 0.0005);
            assert!((0.5 * G * t * t - (1.25 * down - stroke.up * (t - rise))).abs() < 1e-12);
            assert!((-G * t + impulse / MASS - stroke.up).abs() < 1e-12);
        }
    }
}
