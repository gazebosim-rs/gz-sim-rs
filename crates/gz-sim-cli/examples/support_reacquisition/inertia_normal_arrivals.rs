// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Shift the real public update boundary relative to the independently reconstructed lower strike.
use super::*;
pub(super) fn survey(
    i: &Impact,
    g: &Gate,
    upper: Crossing,
    lower: Crossing,
    terminal: Option<&terminal::Terminal>,
    dt: u64,
) -> crate::Result<Vec<Value>> {
    let approach = lower.at - upper.at;
    if approach <= 0. {
        return Err("second lower must follow the first roof".into());
    }
    let mut results = vec![];
    // Include a pre-roof control. Changing both materials then changes that
    // earlier roof reaction, so no later matched trajectory is claimed for it.
    let mut variants = vec![("prior_roof", approach + 16e-9)];
    for ns in [0.125, 0.75, 1.5, 6., 16., 64., 256., 2048.] {
        let delta = ns * 1e-9;
        if delta < approach {
            variants.push(("lower_first", delta));
        }
    }
    for (order, delta) in variants {
        let t = lower.at - delta;
        let x = if order == "prior_roof" {
            from_primary(&i.r, t)
        } else {
            advance(&i.r, upper.after(), t - upper.at)
        };
        let span = dt + ((delta * 1e9).ceil() as u64);
        let end = span as f64 * 1e-9 - delta;
        let files = [
            authored(i, g, t, x, span, false),
            authored(i, g, t, x, span, true),
        ];
        let mut lanes = vec![];
        for (k, raw) in files.iter().enumerate() {
            lanes.push(run_raw(raw, span, k == 1, true)?);
        }
        let legacy = (format!("{:.12}", span as f64 * 1e-9).parse::<f64>()? * 1e9) as u64;
        let matched_legacy = if legacy != span {
            let mut old = vec![];
            for on in [false, true] {
                old.push(run_raw(
                    &authored(i, g, t, x, legacy, on),
                    legacy,
                    on,
                    true,
                )?);
            }
            Some(old)
        } else {
            None
        };
        let reference = if order == "lower_first" {
            terminal.map(|s|{
            assert!(end>s.offset+s.gap_time && end<s.offset+s.free);
            let mut j=s.bin(0.,end);j[0]+=lower.law.impulse;
            json!({"loss_j":loss(i,g,lower,s),"loads":[j[0].dot(i.r.n),j[1].dot(-i.r.n)],"radii":s.radius,"after":body_view(native::at_foot(&i.r,s.state(&i.r,lower,end))),"bounds":s.bounds(&i.r,end-s.offset),"floor_at_s":delta,"limit_after_s":s.offset+delta})
        })
        } else {
            None
        };
        results.push(json!({"previous_clock_ns":legacy,"matched_previous_clock":matched_legacy,"order":order,"start_to_lower_s":delta,"start_to_prior_s":delta-approach,"dt_ns":span,"incoming":body_view(native::at_foot(&i.r,x)),"oracle":reference,"runs":lanes}));
    }
    Ok(results)
}

/// Time advancing Bevy updates after a paused schedule warmup. Geometry, XML
/// loading, diagnostics, independent physics and serialization are outside it.
pub(super) fn costs(
    i: &Impact,
    g: &Gate,
    u: Crossing,
    l: Crossing,
    dt: u64,
) -> crate::Result<Option<Value>> {
    let shift = 64e-9;
    if l.at - u.at <= shift {
        return Ok(None);
    }
    let start = l.at - shift;
    let x = advance(&i.r, u.after(), start - u.at);
    let files = [
        authored(i, g, start, x, dt + 64, false),
        authored(i, g, start, x, dt + 64, true),
    ];
    let mut samples: [Vec<[u64; 8]>; 2] = [vec![], vec![]];
    let mut later = [[0.; 2]; 2];
    let mut accepted = None;
    for rep in 0..=8 {
        for k in [rep % 2, 1 - rep % 2] {
            let (mut sim, ids) = load(&files[k], false)?;
            let roof = id_named(&sim, "default::opponent::gate")?;
            sim.set_paused(true);
            sim.tick()?;
            sim.set_paused(false);
            assert!(sim.elapsed().is_zero());
            let mut times = [0; 8];
            let mut j = [0.; 2];
            for (n, t) in times.iter_mut().enumerate() {
                let stamp = std::time::Instant::now();
                sim.step_exact(1)?;
                *t = stamp.elapsed().as_nanos() as u64;
                let interval = gz_sim_scene::primary_contact::last_opposing_corner(&sim)
                    .filter(|x| x.sim_time == sim.elapsed());
                if n == 0 {
                    let hit =
                        interval.is_some_and(|x| !x.interval.opposing_normal_series.is_empty());
                    if k == 1 {
                        if let Some(a) = accepted {
                            assert_eq!(a, hit);
                        }
                        accepted = Some(hit);
                    } else {
                        assert!(!hit);
                    }
                } else {
                    for (site, support) in [ids.pad, roof].into_iter().enumerate() {
                        j[site] += if let Some(x) = interval {
                            x.interval
                                .pairs
                                .iter()
                                .filter(|p| {
                                    (p.a, p.b) == (ids.foot.min(support), ids.foot.max(support))
                                })
                                .map(|p| p.normal_impulse)
                                .sum::<f64>()
                        } else {
                            backend(&sim)
                                .rigid_contact_patches()
                                .into_iter()
                                .filter(|p| {
                                    (p.a.min(p.b), p.a.max(p.b))
                                        == (ids.foot.min(support), ids.foot.max(support))
                                })
                                .map(|p| p.points.iter().map(|p| p.normal_impulse).sum::<f64>())
                                .sum()
                        };
                    }
                }
            }
            if rep == 0 {
                later[k] = j;
            } else {
                assert_eq!(later[k], j);
                samples[k].push(times);
            }
        }
    }
    Ok(Some(
        json!({"public_dt_ns":dt+64,"start_to_floor_ns":64,"accepted":accepted,"late_loads":later,"each_update_ns":samples}),
    ))
}
