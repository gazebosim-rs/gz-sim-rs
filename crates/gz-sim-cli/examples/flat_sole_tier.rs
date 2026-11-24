// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare an opt-in exact elastic tier to the ordinary 3D coating solver.
#[allow(dead_code)]
mod soft_replay;
use glam::{DQuat, DVec3};
use gz_sim_scene::{
    PhysicsEngine, damped_sole::last_step, load_world_with_sdf_step, queue_link_world_wrench,
};
use serde_json::{Value, json};
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    path::PathBuf,
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const H: f64 = 0.001;
const G: f64 = 9.81;
const M: f64 = 2.;
const K: f64 = 8000.;
#[derive(Clone, Copy)]
struct Fixture {
    name: &'static str,
    gap: f64,
    rise: f64,
    alpha: f64,
    shove: bool,
}
impl Fixture {
    fn sdf(self, fast: bool, steps: u32) -> String {
        format!(
            r#"<sdf version="1.12"><world name="default"><physics name="local" type="ignored"><max_step_size>0.001</max_step_size></physics><gravity>0 0 -9.81</gravity>
<plugin name="gz::sim::systems::DampedSole" filename="gz-sim-damped-sole-system"><foot>default::foot::body</foot><support>default::deck::body</support><foot_stiffness>200000</foot_stiffness><deck_stiffness>200000</deck_stiffness><damping>{}</damping><friction>0.4</friction><slip_speed>0.01</slip_speed><substeps>{steps}</substeps><quadrature_depth>0</quadrature_depth><flat_elastic_fast_path>{fast}</flat_elastic_fast_path><flat_damped_fast_path>{fast}</flat_damped_fast_path></plugin>
<model name="foot"><pose>0 0 {} 0 0 0</pose><plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system"><linear_velocity>0 0 {}</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin><link name="body"><inertial><mass>2</mass><inertia><ixx>0.03</ixx><iyy>0.05</iyy><izz>0.05</izz></inertia></inertial><collision name="sole"><geometry><box><size>0.4 0.2 0.09</size></box></geometry></collision></link></model>
<model name="deck"><pose>0 0 -0.04 0 0 0</pose><plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>0 0 {}</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin><link name="body"><collision name="plate"><geometry><box><size>0.8 0.5 0.08</size></box></geometry></collision></link></model></world></sdf>"#,
            self.alpha,
            0.045 + self.gap,
            self.rise,
            self.rise
        )
    }
}
fn p95(v: &mut [u64]) -> u64 {
    v.sort_unstable();
    v[(v.len() * 95).div_ceil(100) - 1]
}
fn run(f: Fixture, steps: u32, out: &std::path::Path, frames: &mut [Frame]) -> Result<Value> {
    let configs = [f.sdf(true, steps), f.sdf(false, steps)];
    fs::write(out.join(format!("{}.sdf", f.name)), &configs[0])?;
    let mut worlds = configs
        .iter()
        .map(|s| {
            load_world_with_sdf_step(
                s,
                "flat-tier.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )
        })
        .collect::<std::result::Result<Vec<_>, _>>()?;
    let mut costs = [vec![], vec![]];
    let mut active_costs = [vec![], vec![]];
    let (mut exact_ticks, mut active, mut flights, mut landings) = (0, 0, 0, 0);
    let (mut position, mut speed, mut impulse, mut energy_error, mut momentum_error) =
        (0_f64, 0_f64, 0_f64, 0_f64, 0_f64);
    let mut scalar_ticks = 0;
    let mut heat = [0.; 2];
    let mut j = [0.; 2];
    let mut was_loaded = false;
    let mut points = vec![];
    let mut recorder = gz_sim_scene::sole_episodes::SoleEpisodes::new(0.5, 0.1).unwrap();
    let mut events = vec![];
    let duration = 600;
    for tick in 1..=duration {
        for index in [tick % 2, 1 - tick % 2] {
            let sim = &mut worlds[index];
            if f.shove && tick == 151 {
                let foot = last_step(sim).unwrap().foot;
                queue_link_world_wrench(sim, foot, [1., 0., 0.], [0., 0.15, 0.])?;
            }
            let began = Instant::now();
            sim.step_exact(1)?;
            let cost = began.elapsed().as_nanos() as u64;
            let row = last_step(sim).ok_or("missing committed foot")?;
            if tick > 5 {
                costs[index].push(cost);
                if row.step.normal_impulse > 0. {
                    active_costs[index].push(cost)
                }
            }
            j[index] += row.step.normal_impulse;
            heat[index] += row.step.dissipated_energy;
        }
        for e in recorder.observe(&worlds[0])? {
            events.push(json!({"support":e.support,
                "earliest_s":e.earliest.as_secs_f64(),
                "latest_s":e.latest.as_secs_f64(),
                "change":format!("{:?}",e.change),
                "episode_impulse_ns":e.totals.normal_impulse_ns,
                "episode_peak_n":e.totals.peak_force_n}));
        }
        let a = last_step(&worlds[0]).unwrap();
        let b = last_step(&worlds[1]).unwrap();
        let t = tick as f64 * H;
        if a.time != b.time || a.time.sim_time != Duration::from_millis(tick as u64) {
            return Err("clock divergence".into());
        }
        if a.flat_elastic {
            exact_ticks += 1
        }
        if a.flat_damped {
            scalar_ticks += 1
        }
        if a.flat_damped != (f.alpha > 0.) || (heat[0] - heat[1]).abs() > 1e-9 {
            return Err("damped tier or heat mismatch".into());
        }
        if a.flat_elastic != (f.alpha == 0. && (!f.shove || tick < 151)) {
            return Err(format!("{} at {tick}: unexpected solver ownership", f.name).into());
        }
        position = position.max(a.state.center.distance(b.state.center));
        speed = speed.max(a.state.velocity.distance(b.state.velocity));
        impulse = impulse.max((j[0] - j[1]).abs());
        momentum_error =
            momentum_error.max((j[0] - M * (a.state.velocity.z - f.rise + G * t)).abs());
        if !f.shove && f.alpha == 0. {
            let initial = M * G * (0.045 + f.gap) + 0.5 * K * f.gap.min(0.).powi(2);
            let current = 0.5 * M * (a.state.velocity.z - f.rise).powi(2)
                + M * G * (a.state.center.z - t * f.rise)
                + a.step.contact.strain_energy;
            energy_error = energy_error.max((current - initial).abs());
        }
        let loaded = a.step.normal_impulse > 0.;
        if loaded {
            active += 1;
            if !was_loaded {
                landings += 1
            }
        } else {
            flights += 1
        }
        was_loaded = loaded;
        if tick % 5 == 0 {
            let frame = &mut frames[tick / 5 - 1];
            for (index, r) in [a, b].into_iter().enumerate() {
                let lane = match f.name {
                    "bounces" => -2.2,
                    "stance" => -1.1,
                    "lift" => 0.,
                    "shove" => 1.1,
                    _ => 2.2,
                };
                let offset = DVec3::new(lane, if index == 0 { -0.32 } else { 0.32 }, 0.1);
                let name = format!("{}_{}", f.name, if index == 0 { "fast" } else { "fine" });
                frame.add(
                    &format!("{name}_deck"),
                    offset + r.deck.center,
                    r.deck.orientation,
                    json!({"kind":"box","size":[0.8,0.5,0.08]}),
                    [0.22, 0.30, 0.38, 1.],
                );
                frame.add(
                    &format!("{name}_foot"),
                    offset + r.state.center,
                    r.state.orientation,
                    json!({"kind":"box","size":[0.4,0.2,0.09]}),
                    if index == 0 {
                        [0.1, 0.7, 0.92, 1.]
                    } else {
                        [0.96, 0.61, 0.2, 1.]
                    },
                );
                frame.add(
                    &format!("{name}_reaction"),
                    offset
                        + DVec3::new(
                            -0.31,
                            0.,
                            0.07 + r.deck.center.z + 0.003 * r.step.contact.normal_force,
                        ),
                    DQuat::IDENTITY,
                    json!({"kind":"sphere","radius":0.025}),
                    [0.5, 0.88, 0.38, 1.],
                );
            }
            points.push(json!({"tick":tick,
                "center_m":a.state.center.to_array(),
                "reference_m":b.state.center.to_array(),
                "fast_path":a.flat_elastic,
                "normal_n":a.step.contact.normal_force,
                "reference_n":b.step.contact.normal_force}));
        }
    }
    if position > 4e-6
        || speed > 0.0004
        || impulse > 0.001
        || energy_error > 1e-9
        || momentum_error > 1e-8
    {
        return Err(format!("{}: position {position}, speed {speed}, impulse {impulse}, energy {energy_error}, momentum {momentum_error}",f.name).into());
    }
    Ok(json!({
        "fixture": f.name,
        "exact_ticks": exact_ticks,
        "damped_scalar_ticks": scalar_ticks,
        "normal_heat_j": heat,
        "ticks": duration,
        "loaded_ticks": active,
        "free_ticks": flights,
        "load_starts": landings,
        "repeat_grid": steps,
        "max_center_error_m": position,
        "max_speed_error_m_s": speed,
        "max_summed_impulse_error_ns": impulse,
        "max_vertical_momentum_residual_ns": momentum_error,
        "max_elastic_energy_residual_j": if f.alpha == 0. && !f.shove {Some(energy_error)} else {None},
        "p95_ns": {
            "fast": p95(&mut costs[0]),
            "fine": p95(&mut costs[1]),
            "loaded_fast": p95(&mut active_costs[0]),
            "loaded_fine": p95(&mut active_costs[1])
        },
        "samples": points,
        "contact_events": events
    }))
}
fn main() -> Result<()> {
    let out = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("provide a fresh output directory")?,
    );
    fs::create_dir_all(&out)?;
    let standard = Fixture {
        name: "bounces",
        gap: 0.03,
        rise: 0.,
        alpha: 0.,
        shove: false,
    };
    let mut frames: Vec<_> = (0..120).map(|_| Frame::default()).collect();
    let mut rows = vec![];
    for f in [
        standard,
        Fixture {
            name: "stance",
            gap: -M * G / K,
            ..standard
        },
        Fixture {
            name: "lift",
            rise: 0.35,
            ..standard
        },
        Fixture {
            name: "shove",
            shove: true,
            ..standard
        },
        Fixture {
            name: "damped",
            alpha: 0.8,
            ..standard
        },
    ] {
        let row = run(f, 128, &out, &mut frames)?;
        println!(
            "{}: {} elastic and {} damped reduced ticks, {} m maximum difference, p95 {} / {} ns",
            row["fixture"],
            row["exact_ticks"],
            row["damped_scalar_ticks"],
            row["max_center_error_m"],
            row["p95_ns"]["fast"],
            row["p95_ns"]["fine"]
        );
        rows.push(row);
    }
    fs::write(out.join("measurements.json"), serde_json::to_vec(&rows)?)?;
    gz_sim_cli::export_archive_at(
        &out.join("replay"),
        BTreeMap::new(),
        0,
        Duration::from_millis(5),
        frames
            .into_iter()
            .enumerate()
            .map(|(i, f)| f.finish(Duration::from_millis((i as u64 + 1) * 5), i + 1)),
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn damped() -> Fixture {
        Fixture {
            name: "control",
            gap: -0.001,
            rise: 0.,
            alpha: 0.8,
            shove: false,
        }
    }
    fn load(xml: &str) -> gz_sim_core::Simulation {
        load_world_with_sdf_step(
            xml,
            "flat.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap()
    }
    #[test]
    fn adaptive_request_keeps_its_error_estimates_and_both_paths_emit_one_step() {
        let xml = damped().sdf(true, 2);
        let control = "<max_substeps>64</max_substeps><max_quadrature_depth>1</max_quadrature_depth><position_tolerance>0.1</position_tolerance><rotation_tolerance>0.1</rotation_tolerance><impulse_tolerance>0.1</impulse_tolerance><angular_impulse_tolerance>0.1</angular_impulse_tolerance><heat_tolerance>0.1</heat_tolerance>";
        let mut sim = load(&xml.replacen(
            "</flat_damped_fast_path>",
            &format!("</flat_damped_fast_path>{control}"),
            1,
        ));
        sim.step_exact(1).unwrap();
        let row = last_step(&sim).unwrap();
        assert!(!row.flat_damped && row.adaptive.is_some());
        assert_eq!(row.time.iteration, 1);
    }
    #[test]
    fn command_and_reset_move_the_damped_sole_in_and_out_of_scalar_motion() {
        let mut sim = load(&damped().sdf(true, 32));
        sim.step_exact(1).unwrap();
        let first = *last_step(&sim).unwrap();
        assert!(first.flat_damped);
        sim.set_paused(true);
        queue_link_world_wrench(&mut sim, first.foot, [0., 1., 0.], [0.; 3]).unwrap();
        sim.tick().unwrap();
        assert_eq!(last_step(&sim).unwrap().time, first.time);
        sim.step_exact(1).unwrap();
        let disturbed = last_step(&sim).unwrap();
        assert!(!disturbed.flat_damped && disturbed.state.velocity.y > 0.);
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        let restored = last_step(&sim).unwrap();
        assert!(restored.flat_damped);
        assert_eq!(first.step.impulse, restored.step.impulse);
    }
}
