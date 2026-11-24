// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::deformable::{finite_landing_step, set_material_resolution};
const MS: u64 = 1_000_000;
const NATIVE: u64 = 62_500;

#[derive(clap::ValueEnum, Clone, Copy, PartialEq, Eq, Debug, Serialize)]
pub(crate) enum Clock {
    Coarse,
    Adaptive,
    Fine,
    MatchedFine,
}

#[derive(Default, Serialize, PartialEq)]
struct Events {
    requests_ns: Vec<u64>,
    fast_landings_ns: Vec<u64>,
    guarded_landings_ns: Vec<u64>,
    refined_steps: usize,
    window_steps: usize,
    window_impulse_ns: f64,
    rebound_m: f64,
}

struct Episode {
    rows: Vec<Reading>,
    events: Events,
    captures: Vec<arm_sheet::Frame>,
    window_costs: Vec<u64>,
    baseline_costs: Vec<u64>,
}

fn measure(
    r: &mut Recording,
    mode: Mode,
    clock: Clock,
    target_ns: u64,
    draw: bool,
) -> Result<Episode> {
    let substeps = NATIVE.div_ceil(target_ns) as u32;
    if clock == Clock::Fine {
        set_material_resolution(&mut r.sim, "sheet", substeps, 24)?;
    }
    let deck = entity_id(&r.sim, "default::source::body")?;
    let half = arm_cable::half_size(&r.sdf, "source", "body")?;
    let mut events = Events::default();
    let mut captures = Vec::new();
    if draw {
        arm_sheet::capture(r, &mut captures)?;
    }
    let (mut free_ns, mut vz) = (0, 0.);
    let mut hit = false;
    let (mut window_costs, mut baseline_costs) = (Vec::new(), Vec::new());
    let rows = episode_native(r, mode, &mut None, |r, _, elapsed| {
        let time = r.sim.committed_update_info()?.sim_time.as_nanos() as u64;
        let cloth = arm_sheet::cloth(&r.sim)?;
        let [l, right, table] = cloth.last_boundary_impulses() else {
            return Err("missing gripper contacts".into());
        };
        let table: f64 = -table.linear[2];
        let held = [l, right]
            .iter()
            .any(|p| DVec3::from(p.linear).length() > 1e-12);
        let stats = finite_landing_step(&r.sim).unwrap_or_default();
        if stats.triggered {
            events.requests_ns.push(time);
        }
        if stats.refined {
            events.refined_steps += 1;
        }
        // Independent truth uses the first actual upward impulse after a free
        // millisecond and the previously observed downward COM speed.
        if table > 1e-12 && free_ns >= MS && vz < -0.1 {
            events.fast_landings_ns.push(time);
            if stats.refined {
                events.guarded_landings_ns.push(time);
            }
        }
        free_ns = if table <= 1e-12 && !held {
            free_ns + NATIVE
        } else {
            0
        };
        let deck_state = state(&r.sim, deck)?;
        let ps = cloth.particles();
        vz = ps
            .iter()
            .map(|p| p.velocity[2] / p.inverse_mass)
            .sum::<f64>()
            / ps.iter().map(|p| 1. / p.inverse_mass).sum::<f64>()
            - deck_state.velocity[2];
        let in_window = elapsed.is_some_and(|t| t > 500_000 && t <= 80 * MS + 500_000);
        let cost = *r.costs_ns.last().expect("just committed native step");
        if in_window {
            events.window_steps += 1;
            events.window_impulse_ns += table;
            hit |= table > 1e-12;
            if hit {
                let gap = ps
                    .iter()
                    .map(|p| p.position[2] - arm_sheet::RADIUS - deck_state.position[2] - half.z)
                    .fold(f64::INFINITY, f64::min);
                events.rebound_m = events.rebound_m.max(gap);
            }
            window_costs.push(cost);
        } else {
            baseline_costs.push(cost);
        }
        if draw && (time.is_multiple_of(25 * MS) || in_window && time.is_multiple_of(MS)) {
            arm_sheet::capture(r, &mut captures)?;
        }
        if clock == Clock::MatchedFine
            && let Some(t) = elapsed
            && (t == 500_000 || t == 80 * MS + 500_000)
        {
            set_material_resolution(
                &mut r.sim,
                "sheet",
                if t == 500_000 { substeps } else { 1 },
                24,
            )?;
        }
        Ok(())
    })?;
    Ok(Episode {
        rows,
        events,
        captures,
        window_costs,
        baseline_costs,
    })
}
fn costs(xs: &mut [u64]) -> Value {
    if xs.is_empty() {
        return Value::Null;
    }
    xs.sort_unstable();
    json!({"n":xs.len(),"total_ns":xs.iter().sum::<u64>(),"p50_ns":xs[(50*xs.len()).div_ceil(100)-1],"p95_ns":xs[(95*xs.len()).div_ceil(100)-1],"p99_ns":xs[(99*xs.len()).div_ceil(100)-1]})
}
pub(crate) fn run(
    out: &Path,
    mode: Mode,
    clock: Clock,
    target_ns: u64,
    verify_reset: bool,
) -> Result<()> {
    if !matches!(mode, Mode::DropEarly | Mode::DropLate | Mode::Finite)
        || target_ns == 0
        || target_ns > NATIVE
        || NATIVE.div_ceil(target_ns) > 1024
    {
        return Err("expected a supported finite receiver and a resolvable target".into());
    }
    let mut xml = world(16, true, false)?;
    if clock == Clock::Adaptive {
        xml = xml.replace(
            "filename=\"gz-sim-deformables-system\"",
            &format!("filename=\"gz-sim-deformables-system\" landing_step_ns=\"{target_ns}\""),
        );
    }
    let mut r = Recording::new(&xml, "sheet-impact-clock.sdf", "default")?;
    let mut a = measure(&mut r, mode, clock, target_ns, true)?;
    if verify_reset {
        gz_sim_scene::reset_loaded_world(&mut r.sim, |_| ())?;
        let repeat = measure(&mut r, mode, clock, target_ns, false)?;
        if a.rows != repeat.rows || a.events != repeat.events {
            return Err("impact resolution changed across reset".into());
        }
        r.costs_ns.truncate(a.rows.len() * 16);
    }
    let physical = ObservationWindow::new(mode, &a.rows, r.sim.step_duration().as_secs_f64())?;
    let receipt = json!({"mode":mode,"clock":clock,"target_ns":target_ns,"reset":verify_reset,"physical":physical,"events":a.events,"window_native_cost":costs(&mut a.window_costs),"other_native_cost":costs(&mut a.baseline_costs)});
    write_json(&out.join("comparison.json"), &receipt)?;
    write_json(&out.join("ticks.json"), &serde_json::to_value(&a.rows)?)?;
    fs::write(out.join("world.sdf"), xml)?;
    let last = a.captures.len() - 1;
    let first = physical.receiver_drop.map_or(130, |d| d.first_tick / 25);
    arm_sheet::save_archive_with_camera(
        r,
        a.captures,
        out,
        json!({"held":60,"pre_withdrawal":first,"arrival":(first+5).min(last),"rebound":(first+50).min(last),"final":last}),
        "right",
    )?;
    println!("{receipt}");
    Ok(())
}
