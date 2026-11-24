// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::queue_joint_effort;
use std::fmt::Write;

fn inertia_probe(scene: &str, us: u32) -> Result<Value> {
    let xml = scene.replace("0 0 -9.81", "0 0 0").replace(
        "<max_step_size>0.001</max_step_size>",
        &format!("<max_step_size>{}</max_step_size>", f64::from(us) * 1e-6),
    );
    let mut r = Recording::new(&xml, "arm-inertia.sdf", "default")?;
    let shoulder = entity_id(&r.sim, "default::arm::shoulder")?;
    let drive = Controller::new(&r.sim)?;
    // Moment of four boxes about the vertical pin using the parallel-axis theorem.
    // The parcel is on its stand, clear of the open jaws. Radial jaw translation
    // has zero coupling to infinitesimal shoulder rotation at rest.
    let links = [
        (0.85, 0.16, 0.38, 0.09),
        (0.6, 0.32, 0.26, 0.095),
        (0.18, 0.21, 0.024, 0.085),
        (0.18, 0.43, 0.024, 0.085),
    ];
    let moment: f64 = links
        .into_iter()
        .map(|(m, r, x, y)| m * ((x * x + y * y) / 12. + r * r))
        .sum();
    let torque = 0.2;
    queue_joint_effort(&mut r.sim, shoulder, torque)?;
    r.advance()?;
    let expected = torque / moment * r.sim.step_duration().as_secs_f64();
    let first = drive.sample(&r.sim, 0)?.velocity;
    let ids = ["rotor", "palm", "left", "right"]
        .map(|n| entity_id(&r.sim, &format!("default::arm::{n}")))
        .into_iter()
        .collect::<Result<Vec<_>>>()?;
    let lz = |sim: &Simulation| -> Result<f64> {
        let mut l = 0.;
        for (&id, (mass, _, x, y)) in ids.iter().zip(links) {
            let body = state(sim, id)?;
            l += mass * (body.position[0] * body.velocity[1] - body.position[1] * body.velocity[0])
                + mass * (x * x + y * y) / 12. * body.angular_velocity[2];
        }
        Ok(l)
    };
    let impulse = torque * r.sim.step_duration().as_secs_f64();
    let actual = lz(&r.sim)?;
    // Include all link spins throughout the transient; a good first shoulder
    // sample alone can mask alternating overshoot in the iterative constraint solve.
    let mut worst = 0_f64;
    for tick in 0..=120 {
        for id in &ids {
            worst = worst.max((state(&r.sim, *id)?.angular_velocity[2] / expected - 1.).abs());
        }
        if tick < 120 {
            r.advance()?;
        }
    }
    let relaxed = drive.sample(&r.sim, 0)?.velocity;
    let relative = (relaxed - expected).abs() / expected;
    let qualified = worst <= 0.012 && relative <= 0.012 && (actual / impulse - 1.).abs() <= 0.002;
    Ok(
        json!({"qualified":qualified,"dt_us":us,"moment_kg_m2":moment,"predicted_angular_speed_rad_s":expected,"first_angular_speed_rad_s":first,"settled_angular_speed_rad_s":relaxed,
        "peak_link_speed_relative_error":worst,"angular_momentum_after_pulse":actual,"applied_angular_impulse":impulse,"first_speed_relative_error":(first/expected-1.).abs(),"settled_speed_relative_error":relative}),
    )
}

fn chain(links: usize, load: bool) -> String {
    let mut xml = String::from(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity><physics name="local" type="ignored"><max_step_size>0.001</max_step_size></physics><model name="chain">"#,
    );
    for i in 0..links {
        let parent = if i == 0 {
            "world".to_owned()
        } else {
            format!("l{}", i - 1)
        };
        write!(xml,r#"<link name="l{i}"><pose>{} 0 0.50 0 0 0</pose><inertial><mass>0.08</mass><inertia><ixx>0.0000053333333333</ixx><iyy>0.0000193333333333</iyy><izz>0.0000193333333333</izz></inertia></inertial><collision name="body"><geometry><box><size>0.05 0.02 0.02</size></box></geometry></collision></link><joint name="j{i}" type="revolute"><parent>{parent}</parent><child>l{i}</child><pose>-0.025 0 0 0 0 0</pose><axis><xyz>0 1 0</xyz><limit><lower>-2.8</lower><upper>2.8</upper></limit></axis></joint>"#,(i as f64+0.5)*0.05).unwrap();
    }
    xml.push_str("</model>");
    if load {
        write!(xml,r#"<model name="support"><static>true</static><link name="top"><pose>{} 0 0.4648 0 0 0</pose><collision name="pad"><geometry><box><size>{} 0.08 0.05</size></box></geometry></collision></link></model>"#,links as f64*0.025,links as f64*0.05+0.05).unwrap();
    }
    xml.push_str("</world></sdf>");
    xml
}
fn scale(xml: &str, links: usize, load: bool) -> Result<Value> {
    let mut r = Recording::new(xml, "chain.sdf", "default")?;
    let body = entity_id(&r.sim, &format!("default::chain::l{}", links - 1))?;
    let support = if load {
        Some(handle(&r.sim, entity_id(&r.sim, "default::support::top")?)?)
    } else {
        None
    };
    let mut loaded = 0;
    let mut peak = 0_f64;
    let mut tip_before = 0.;
    let mut cost = vec![];
    for tick in 0..220 {
        // Discard cold startup; report stepping alone. External contact is measured
        // after the timer, independently of internal articulation constraints.
        let start = Instant::now();
        r.sim.step_exact(1)?;
        let ns = start.elapsed().as_nanos() as u64;
        let obs = r
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing solver")?
            .rigid_contact_observations();
        let impulse: f64 = obs
            .iter()
            .filter(|c| Some(c.a) == support || Some(c.b) == support)
            .map(|c| c.normal_impulse)
            .sum();
        if tick >= 20 {
            cost.push(ns);
            loaded += usize::from(impulse > 0.);
            peak = peak.max(impulse);
        }
        if tick == 19 {
            tip_before = state(&r.sim, body)?.position[2];
        }
    }
    let tip = state(&r.sim, body)?.position[2];
    if !tip.is_finite() || (load && loaded < 190) || (!load && (tip - tip_before).abs() < 0.005) {
        return Err(format!(
            "invalid chain workload links={links} contact={load} loaded={loaded} tip={tip}"
        )
        .into());
    }
    cost.sort_unstable();
    Ok(
        json!({"links":links,"table_contact":load,"contact_ticks_of_200":loaded,"peak_support_impulse_ns":peak,"final_tip_z_m":tip,"p50_ns":cost[99],"p95_ns":cost[189]}),
    )
}
fn probes(scene: &str) -> Result<Vec<Value>> {
    [500, 1000, 2000]
        .into_iter()
        .map(|us| inertia_probe(scene, us))
        .collect()
}

fn iteration_sweep(arm: &str) -> Result<Vec<Value>> {
    let mut result = vec![];
    // Densely sample the first passing neighborhood; retain the failed controls.
    for passes in [
        0, 3, 7, 15, 31, 39, 40, 41, 42, 43, 45, 47, 63, 79, 91, 95, 127,
    ] {
        let scene = arm.replace(
            "<additional_pgs_iterations>41</additional_pgs_iterations>",
            &format!("<additional_pgs_iterations>{passes}</additional_pgs_iterations>"),
        );
        let inertia = probes(&scene)?;
        let qualified = inertia.iter().all(|v| v["qualified"] == true);
        let mut cycle_p95 = vec![];
        if [0, 41, 95].contains(&passes) {
            for _ in 0..3 {
                let mut r = Recording::new(&scene, "arm-iterations.sdf", "default")?;
                metrics(&mut r, false, false)?;
                r.costs_ns.sort_unstable();
                cycle_p95.push(r.costs_ns[(r.costs_ns.len() * 95).div_ceil(100) - 1]);
            }
        }
        result.push(json!({"additional_pgs_iterations":passes,"inertia_probes":inertia,
                            "qualified_at_all_three_steps":qualified,"clear_cycle_step_p95_ns":cycle_p95}));
    }
    Ok(result)
}

fn qualified_inertia(scene: &str) -> Result<Vec<Value>> {
    let inertia = probes(scene)?;
    if inertia.iter().any(|v| v["qualified"] != true) {
        return Err(format!("arm angular impulse outside accuracy budget: {inertia:?}").into());
    }
    Ok(inertia)
}

pub(super) fn run(output: &Path) -> Result<()> {
    let mut rows = vec![];
    for repeat in 0..3 {
        for n in [4, 8, 16, 32] {
            for j in 0..2 {
                let loaded = (j + repeat) % 2 == 0;
                rows.push(scale(&chain(n, loaded), n, loaded)?);
            }
        }
    }
    let inertia = qualified_inertia(SCENE)?;
    write_json(
        &output.join("iteration-sweep.json"),
        &json!(iteration_sweep(SCENE)?),
    )?;
    write_json(
        &output.join("scaling.json"),
        &json!({"inertia_probes":inertia,"link_benchmarks":rows}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn joint_reactions_propagate_the_measured_pulse_to_every_link() -> Result<()> {
        assert!(probes(SCENE)?.iter().all(|v| v["qualified"] == true));
        let bare = SCENE.replace(
            "<additional_pgs_iterations>41</additional_pgs_iterations>",
            "",
        );
        assert!(probes(&bare)?.iter().all(|v| v["qualified"] == false));
        Ok(())
    }
}

#[cfg(test)]
#[path = "scaling_failure_tests.rs"]
mod scaling_failure_tests;
