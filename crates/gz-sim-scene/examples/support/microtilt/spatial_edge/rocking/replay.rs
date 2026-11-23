// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn march(
    path: Arc,
    mut current: Motion,
    duration: f64,
    areas: &mut [f64; 4],
) -> Result<(Motion, u32)> {
    let mut left = duration;
    let mut count = 0;
    while left > 1e-15 {
        let dt = left.min(0.0000625);
        let (m, loads) = path.advance_loaded(current, dt)?;
        if loads.iter().any(|n| *n < -1e-11) {
            return Err("reference crossed a negative constrained reaction".into());
        }
        match path.sites {
            Sites::One(k) => areas[k] += loads[0],
            Sites::Two([a, b]) => {
                areas[a] += loads[0];
                areas[b] += loads[1];
            }
        }
        current = m;
        left -= dt;
        count += 1;
    }
    Ok((current, count))
}
pub(super) fn bodies(i: Input, t: f64, s: State) -> (BodyState, BodyState, BodyState) {
    let pad_v = DVec3::from(i.seed.pad.velocity);
    let pos = DVec3::from(i.seed.pad.position) + pad_v * t;
    let center = pos + PAD + s.c;
    let p = BodyState {
        position: pos.to_array(),
        ..i.seed.pad
    };
    let body = |sign: f64| {
        let r = s.motion.q * (sign * ARM);
        BodyState {
            position: (center + r).to_array(),
            orientation: s.motion.q.to_array(),
            velocity: (pad_v + s.v + s.motion.w.cross(r)).to_array(),
            angular_velocity: s.motion.w.to_array(),
        }
    };
    (body(-1.), body(1.), p)
}
pub(super) fn record(out: &Path, i: Input, path: Arc, event: Arrival, jump: Jump) -> Result<()> {
    if jump.new >= 4 {
        return Err("replay requires both reaction sites on sole".into());
    }
    let second = Arc {
        shape: path.shape,
        start: jump.after,
        sites: jump.new_sites,
    };
    let first_tick = (event.time / DT).ceil() as u64;
    let next = second.next(0.0000625, 0.006)?;
    let safe = if matches!(next.end, End::Horizon) {
        first_tick + 8
    } else {
        ((event.time + next.time) / DT).floor() as u64
    };
    let last = (first_tick + 8).min(safe);
    if last < first_tick {
        return Err("new event inside same published actuator tick".into());
    }
    let mut public = vec![];
    let mut grids = vec![];
    for (label, calls) in [("ordinary", 1), ("refined", 8)] {
        let (profile, rows) = native::native(i, path, calls, last, true, None)?;
        public.push(
            json!({"label":label,"at_event_tick":rows[first_tick as usize],"events":profile}),
        );
        grids.push((label, rows));
    }
    let mut rows = vec![];
    let mut state = path.start;
    let mut m = state.motion;
    let mut previous = 0.;
    let mut applied = false;
    let mut max_balance = 0f64;
    for tick in 0..=last {
        let time = tick as f64 * DT;
        let mut loads = [0.; 4];
        let mut steps = 0;
        if tick > 0 {
            let initial = state;
            if !applied {
                let end = time.min(event.time);
                let (b, n) = march(path, m, end - previous, &mut loads)?;
                steps += n;
                m = b;
                state = path.state(end, m);
                previous = end;
                if end == event.time {
                    state = jump.after;
                    m = state.motion;
                    loads[jump.old] += jump.normal[0];
                    loads[jump.new] += jump.normal[1];
                    applied = true;
                }
            }
            if applied && time > previous {
                let (b, n) = march(second, m, time - previous, &mut loads)?;
                steps += n;
                m = b;
                state = second.state(time - event.time, m);
                previous = time;
            }
            let momentum = path.shape.mass * (state.v - initial.v - path.shape.gravity * DT);
            max_balance = max_balance.max(momentum.distance(DVec3::Z * loads.iter().sum::<f64>()));
        }
        let (f, u, p) = bodies(i, time, state);
        let gap = native::contact_gaps(f, p)
            .into_iter()
            .fold(f64::INFINITY, f64::min);
        rows.push(json!({"tick":tick,"time_us":tick*500,"native_steps":0,"reference_steps":steps,"solver_iterations":0,"selected":false,"weak_start":false,"secondary_start":false,"normal_impulse_ns":loads.iter().sum::<f64>(),"corner_ns":loads,"foot":wire(f),"upper":wire(u),"pad":wire(p),"gap_m":gap}));
    }
    if max_balance > 1e-9 {
        return Err(format!(
            "reference public reaction differs from normal momentum by {max_balance:e}"
        )
        .into());
    }
    public.push(json!({"label":"reference","max_public_momentum_residual_ns":max_balance,"event_s":event.time,"old_site":jump.old,"new_site":jump.new,"instant_jump_ns":jump.normal,"next_horizon_end":kind(next.end),"first_public_tick":first_tick,"at_event_tick":rows[first_tick as usize]}));
    grids.push(("reference", rows));
    for (label, rows) in grids {
        fs::write(
            out.join(format!("traces/{label}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
            )?,
        )?;
    }
    let post = Seed {
        state: PoseState {
            v: path.start.v,
            w: path.start.motion.w,
            ..i.seed.state
        },
        ..i.seed
    };
    fs::write(out.join("rocking.sdf"), scene(post, 0., false, "ordinary"))?;
    let lanes = [
        ("ordinary", "One native call", false),
        ("refined", "Eight native calls", false),
        ("reference", "Independent constrained rigid body", true),
    ]
    .map(|(l, n, a)| json!({"file":format!("traces/{l}.json"),"label":l,"name":n,"analytical":a}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"A released foot vertex reaches the support again","capture_ticks":{"second_contact":first_tick},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&public)?)?;
    Ok(())
}
