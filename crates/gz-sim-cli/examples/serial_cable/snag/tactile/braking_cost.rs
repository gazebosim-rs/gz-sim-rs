// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::time::Instant;

fn route(xml: &str, sdf: &sdformat::Sdf, active: bool) -> Result<Vec<u64>> {
    let mut sim = load_world_with_sdf_step(
        xml,
        "brake-cost.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let robot = Robot::load(&sim, sdf)?;
    let mut ctl = Controller::new();
    ctl.active_braking = active;
    let mut observer = Observer::default();
    let mut guard = SupportLoss::default();
    let mut ns = Vec::with_capacity(14000);
    for tick in 1..=MAX_TICKS {
        let start = Instant::now();
        ctl.drive(&robot, &mut sim, tick)?;
        sim.step_exact(1)?;
        let jaws = JawReadings::read(&sim, &robot)?;
        let (mut feedback, signal) = observer.update(&jaws);
        if active {
            feedback.snag |= guard.observe(ctl.stage, &jaws, &signal) >= UNLOAD_TRAVEL_M;
        }
        ctl.apply(tick, jaws.rail_y_m, &feedback)?;
        ns.push(start.elapsed().as_nanos() as u64);
        if ctl.done(tick) {
            break;
        }
    }
    if !ctl.done(ns.len()) {
        return Err("uncompleted clear route in timing run".into());
    }
    Ok(ns)
}
pub(super) fn measure() -> Result<Value> {
    let xml = Fixture::Clear.xml();
    let sdf = sdformat::load_str(&xml, "brake-cost.sdf", &sdformat::NoResources)?;
    let warm = route(&xml, &sdf, false)?.len();
    let mut costs = [vec![], vec![]];
    let mut pairs = vec![];
    for active in [false, true, true, false, true, false, false, true] {
        let mut ns = route(&xml, &sdf, active)?;
        if ns.len() != warm {
            return Err("cost runs finished at different physical times".into());
        }
        let wall_ms = ns.iter().sum::<u64>() as f64 / 1e6;
        ns.sort_unstable();
        let p95 = ns[(ns.len() * 95).div_ceil(100) - 1];
        pairs.push(json!({"active_braking":active,"tick_count":ns.len(),"p95_ns":p95,"total_measured_ms":wall_ms}));
        costs[usize::from(active)].extend(ns);
    }
    let pooled=costs.map(|mut ns:Vec<u64>|{
        ns.sort_unstable();
        let percentile = |k:usize|ns[(ns.len()*k).div_ceil(100)-1];
        json!({"ticks":ns.len(),"p50_ns":percentile(50usize),"p95_ns":percentile(95),"p99_ns":percentile(99)})
    });
    Ok(
        json!({"physical_ticks_per_route":warm,"runs":pairs,"normal_stop":pooled[0],"active_brake":pooled[1]}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_benchmark_must_reach_a_physical_setdown() -> Result<()> {
        let xml = Fixture::Clear
            .xml()
            .replace("0 0.46 0.574", "0.85 0.46 0.574");
        let sdf = sdformat::load_str(&xml, "missed.sdf", &sdformat::NoResources)?;
        assert!(route("invalid missing SDF", &sdf, true).is_err());
        assert!(
            route(&xml, &sdf, true)
                .unwrap_err()
                .to_string()
                .contains("uncompleted clear route")
        );
        Ok(())
    }
}
