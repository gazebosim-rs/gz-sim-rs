// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare native load onset from the same physical post-impact state.
use super::*;
#[derive(Clone, Copy, Default)]
struct Loaded {
    site: usize,
    time_ns: u64,
    j: f64,
    gap: f64,
}
impl Loaded {
    fn wire(self) -> Value {
        json!({"site":self.site,"time_ns":self.time_ns,"normal_ns":self.j,"geometric_gap_m":self.gap})
    }
}
pub(super) fn contact_gaps(foot: BodyState, pad: BodyState) -> [f64; 4] {
    let q = DQuat::from_array(foot.orientation);
    local_arms().map(|arm| foot.position[2] + (q * (arm + ARM)).z - pad.position[2] - PAD.z)
}
pub(super) fn native(
    i: Input,
    path: Arc,
    calls: u32,
    ticks: u64,
    record: bool,
    cut_ns: Option<u64>,
) -> Result<(Value, Vec<Value>)> {
    let s0 = path.start;
    let post = Seed {
        state: PoseState {
            v: s0.v,
            w: s0.motion.w,
            q: s0.motion.q,
            c: s0.c,
        },
        ..i.seed
    };
    let src = scene(post, 0., false, "ordinary");
    let mut sim = load(&src, true)?;
    let r = ids(&sim)?;
    let mut owner = sim.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    let opened = path.sites.first();
    let initial = departure::ingest(n, &r);
    let mut clear = contact_gaps(initial.0, initial.2).map(|g| g > 1e-6);
    let mut first = None;
    let mut after_clear = None;
    let mut traces = vec![];
    let mut relative_state = None;
    let mut max_early = 0f64;
    let mut donor = None;
    let mut observed = 0usize;
    for tick in 0..=ticks {
        let mut corner = [0.; 4];
        let mut err = None;
        if tick > 0 {
            let start = (tick - 1) * 500000;
            n.observe_contact_interval(Duration::from_micros(500), calls, 4, |s| {
                let shoe = s.state(r.foot).unwrap();
                let pad = s.state(r.pad).unwrap();
                let q = DQuat::from_array(shoe.orientation);
                let p = match moments(&s.contact_patches(), &r, q, false) {
                    Ok(p) => p.2,
                    Err(e) => {
                        err = Some(e);
                        return;
                    }
                };
                let now = start + s.elapsed.as_nanos() as u64;
                if cut_ns.is_some_and(|cut| now <= cut) {
                    let b = Seed::read((shoe, Some(s.state(r.upper.unwrap()).unwrap()), pad)).state;
                    donor = Some((now, b));
                }
                let gaps = contact_gaps(shoe, pad);
                observed += 1;
                for k in 0..4 {
                    corner[k] += p[k];
                    if k == opened {
                        continue;
                    }
                    if p[k] > 1e-8 {
                        let load = Loaded {
                            site: k,
                            time_ns: start + s.elapsed.as_nanos() as u64,
                            j: p[k],
                            gap: gaps[k],
                        };
                        first.get_or_insert(load);
                        if clear[k] && after_clear.is_none() {
                            after_clear = Some(load);
                            relative_state = Some(
                                Seed::read((shoe, Some(s.state(r.upper.unwrap()).unwrap()), pad))
                                    .state
                                    .wire(),
                            );
                        } else if !clear[k] {
                            max_early = max_early.max(p[k]);
                        }
                    }
                    clear[k] |= gaps[k] > 1e-6;
                }
            })?;
            if let Some(e) = err {
                return Err(e);
            }
        }
        if record {
            let (f, u, p) = departure::ingest(n, &r);
            let min = contact_gaps(f, p).into_iter().fold(f64::INFINITY, f64::min);
            traces.push(json!({"tick":tick,"time_us":tick*500,"native_steps":if tick==0{0}else{calls},"solver_iterations":4,"selected":calls>1&&tick>0,"weak_start":false,"secondary_start":false,"normal_impulse_ns":corner.iter().sum::<f64>(),"corner_ns":corner,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"gap_m":min}));
        } else if after_clear.is_some() {
            break;
        }
    }
    let matched = match donor {
        Some((stamp, b)) => {
            let m = Motion { q: b.q, w: b.w };
            let sample = State {
                motion: m,
                c: b.c,
                v: b.v,
            };
            let given = Arc {
                start: sample,
                ..path
            };
            // Preserve measured orientation, angular speed and the unconstrained
            // horizontal COM components. Measure both normal manifold corrections.
            let start = given.state(0., m);
            let shadow = Arc { start, ..path };
            let next = shadow.next(0.00003125, 0.010)?;
            Some(
                json!({"native_time_ns":stamp,"native_state":state_wire(sample),"position_correction_m":start.c.z-b.c.z,"speed_correction_m_s":start.v.z-b.v.z,"start":state_wire(start),"end":kind(next.end),"next_event_ns":stamp as f64+next.time*1e9}),
            )
        }
        None => None,
    };
    Ok((
        json!({"native_calls_per_tick":calls,"matched_bearing_shadow":matched,"first_any":first.map(Loaded::wire),"first_after_clearance":after_clear.map(Loaded::wire),"state_at_first_after_clearance":relative_state,"max_impulse_before_observed_clearance_ns":max_early,"native_calls_observed":observed}),
        traces,
    ))
}
