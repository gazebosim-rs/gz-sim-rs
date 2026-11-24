// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Paired ordinary and opted-in Rapier steps from measured lower-impact cuts.
#[path = "inertia_normal_arrivals.rs"]
mod arrivals;
#[path = "inertia_normal_grid.rs"]
mod grids;
#[path = "inertia_normal_period.rs"]
mod long_period;
#[path = "inertia_native_normal_view.rs"]
mod replay;
use super::super::views::initial_sdf;
use super::*;
use gz_physics_multifidelity::{BoxContactActions, ContactSolverIterations, OpposingNormalReceipt};
fn xml(i: &Impact, g: &Gate, l: Crossing, dt: u64, flag: bool) -> String {
    authored(i, g, l.at, l.before, dt, flag)
}
fn authored(i: &Impact, g: &Gate, at: f64, x: Flight, dt: u64, flag: bool) -> String {
    let x = projected::enable(&initial_sdf(i, g, 0., at, x, dt, true));
    let mat = format!("<mu>{}</mu><mu2>{}</mu2>", i.mu, i.mu);
    assert!(x.contains(&mat));
    let x = x.replace(&mat, "<mu>0</mu><mu2>0</mu2>").replace(
        "<native_steps>8</native_steps>",
        "<native_steps>1</native_steps>",
    );
    if flag {
        x.replace(
            "</project_opponent_corner>",
            "</project_opponent_corner><project_opposing_normals>true</project_opposing_normals>",
        )
    } else {
        x
    }
}
fn data(r: &OpposingNormalReceipt) -> Value {
    let b = r.first_lower.incoming;
    let hit = r.first_lower.point.impact;
    let arm =
        DVec3::from(hit.point) - DVec3::from(b.position) - DQuat::from_array(b.orientation) * ARM;
    json!({"first_lower_loss_j":hit.dissipated_energy,"first_lower":r.first_lower.point.after_cut_s,"lower_time_error_s":r.first_lower.timing_error_s,"lower_lever":arm.to_array(),"lower_normal":r.first_lower.point.impact.impulse_on_foot,"supports":r.supports.map(|j|j.raw()),"corners":r.corners,"normals":r.normals,"finite":r.finite.iter().map(|h|json!({"site":h.site,"at_s":h.time_s,"error_s":h.time_error_s,"normal_ns":h.impulse,"dissipated_j":h.dissipated_energy_j,"point":h.world_point,"lever":h.lever_from_com})).collect::<Vec<_>>(),"normal_sum":r.normal_impulse,"radius":r.impulse_radius,"terminal_ns":r.terminal_normal,"terminal_levers":r.terminal_levers,"time_bound_s":r.terminal_time_s,"acceleration_floor":r.separation_acceleration,"footprint_margin_m":r.min_face_margin_m,"other_gap_m":r.min_other_gap_m,"com_error_m":r.com_error_m,"spin_error_rad_s":r.spin_error_rad_s,"energy":{"support_work_j":r.support_work_j,"gravity_work_j":r.gravity_work_j,"kinetic_change_j":r.kinetic_energy_change_j,"dissipated_j":r.dissipated_energy_j,"radius_j":r.energy_error_j},"end":body_view(r.end)})
}
fn run(i: &Impact, g: &Gate, l: Crossing, dt: u64, on: bool, public: bool) -> crate::Result<Value> {
    let raw = xml(i, g, l, dt, on);
    run_raw(&raw, dt, on, public)
}
fn run_raw(raw: &str, dt: u64, on: bool, public: bool) -> crate::Result<Value> {
    let (mut sim, ids) = load(raw, false)?;
    let roof = id_named(&sim, "default::opponent::gate")?;
    let out = if public {
        sim.step_exact(1)?;
        gz_sim_scene::primary_contact::last_opposing_corner(&sim)
            .filter(|r| r.sim_time == sim.elapsed())
            .map(|r| r.interval.clone())
    } else {
        let mut phy = sim.world_mut().resource_mut::<PhysicsBackend>();
        let native = phy.downcast_mut::<MultiFidelityBackend>().unwrap();
        let pair = [(ids.foot, ids.pad)];
        let triple = [(ids.foot, ids.pad, roof)];
        let mut observations = vec![];
        let got = native.advance_box_contacts(
            Duration::from_nanos(dt),
            1,
            ContactSolverIterations {
                temporal: 4,
                pgs: Some(1),
                relaxation: Some(16),
            },
            BoxContactActions {
                guard_separated_flight: &pair,
                integrate_free_rotation: &pair,
                integrate_ballistic_translation: &pair,
                opposing_box_guards: &triple,
                project_opposing_corners: &triple,
                project_opposing_normals: if on { &triple } else { &[] },
                ..Default::default()
            },
            Some(&mut |s| observations.push(s.opposing_normal_series.map(data))),
        )?;
        assert_eq!(observations.len(), 1);
        assert_eq!(
            observations[0],
            got.interval.opposing_normal_series.first().map(data)
        );
        Some(got.interval)
    };
    let measured_ns = if public {
        sim.elapsed().as_nanos() as u64
    } else {
        dt
    };
    assert_eq!(dt, measured_ns);
    let e = backend(&sim);
    let foot = e.state(ids.foot).unwrap();
    let js = out.as_ref().map(|p| {
        [ids.pad, roof].map(|id| {
            p.pairs
                .iter()
                .filter(|r| (r.a, r.b) == (id.min(ids.foot), id.max(ids.foot)))
                .map(|r| r.normal_impulse)
                .sum::<f64>()
        })
    });
    Ok(
        json!({"elapsed_ns":measured_ns,"failures":out.as_ref().map(|i|i.opposing_normal_declines.iter().map(|x|json!({"stage":format!("{:?}",x.stage),"limit":x.limit.map(|l|format!("{l:?}")),"returns":x.finite_returns})).collect::<Vec<_>>()),"native":out.as_ref().and_then(|p|p.opposing_normal_series.first()).map(data),"loads":js,"after":body_view(foot),"calls":out.as_ref().map(|p|p.native_steps),"public":public,"enabled":on}),
    )
}
fn loss(i: &Impact, g: &Gate, l: Crossing, t: &terminal::Terminal) -> f64 {
    let finite = l.law.energy_loss + t.finite.iter().map(|(_, h)| h.law.energy_loss).sum::<f64>();
    let before = t.finite.last().unwrap().1.after();
    let kinetic = |x: Flight| 0.5 * (M * x.v.length_squared() + x.l.dot(i.r.body.omega(x.q, x.l)));
    finite + t.impulses[0].dot(i.r.pad_v) + t.impulses[1].dot(g.velocity) + kinetic(before)
        - kinetic(t.cut)
}
fn sample(xml: &str) -> crate::Result<(u64, bool)> {
    let (mut sim, _) = load(xml, false)?;
    let stamp = std::time::Instant::now();
    sim.step_exact(1)?;
    let ns = stamp.elapsed().as_nanos() as u64;
    let accepted = gz_sim_scene::primary_contact::last_opposing_corner(&sim)
        .is_some_and(|r| !r.interval.opposing_normal_series.is_empty());
    Ok((ns, accepted))
}
pub(crate) fn write(
    path: &Path,
    heldout: bool,
    cost: bool,
    phase: bool,
    period: bool,
) -> crate::Result<()> {
    fs::create_dir_all(path)?;
    let mut groups = vec![];
    let mut feature = false;
    let donors = if heldout {
        compact::heldout_inputs()
    } else {
        inputs()
    };
    let delays: &[u64] = if heldout {
        &[37000, 430000, 910000, 1350000]
    } else {
        &[20000, 750000, 1100000]
    };
    for i in donors {
        let mut cases = vec![];
        for &delay in delays {
            let g = gate(&i, delay as f64 * 1e-9);
            let planes = faces(&i.r, &g);
            for gm in if heldout {
                [0.1, 0.45, 0.75]
            } else {
                [0., 0.35, 0.8]
            } {
                let original_mu = [i.mu, i.mu.min(gm)];
                let Ok(u) = upper(&i, &g, original_mu[1]) else {
                    continue;
                };
                let Some(mut l) = follow(&i, planes, original_mu, u, WINDOW)
                    .next
                    .filter(|h| h.surface == 0)
                else {
                    continue;
                };
                l.mu = 0.;
                l.law = coulomb(i.r.body, l.before, l.arm, l.plane, 0.).unwrap();
                let s = sequence(&i, &g, u, l, [0.; 2]);
                let term = terminal::Terminal::new(&i, l, &s);
                let dt = term.as_ref().map_or(128, |t| {
                    (t.offset * 1e9).ceil() as u64 + ((0.5 * t.free * 1e9).min(32.) as u64)
                });
                if period && cost {
                    if let Some(t) = term.as_ref() {
                        let feature = (i.r.id == 1374
                            && MODES[i.mode] == "near_stick"
                            && gm == 0.35
                            && delay == 750000)
                            .then_some(path);
                        cases.push(json!({"delay_ns":delay,"gate_mu":gm,"world":grids::survey(&i,&g,l,t,feature)?}));
                    }
                    continue;
                }
                if period {
                    if let Some(t) = term.as_ref() {
                        let world = long_period::survey(&i, &g, l, t)?;
                        if i.r.id == 2681 && delay == 430000 && gm == 0.1 {
                            long_period::replay(path, &i, &g, l, &world)?;
                        }
                        cases.push(json!({"delay_ns":delay,"gate_mu":gm,"world":world}));
                    }
                    continue;
                }
                if phase && cost {
                    if let Some(s) = arrivals::costs(&i, &g, u, l, dt)? {
                        cases.push(json!({"delay_ns":delay,"gate_mu":gm,"world":s}));
                    }
                    continue;
                }
                if phase {
                    let trials = arrivals::survey(&i, &g, u, l, term.as_ref(), dt)?;
                    if i.r.id == 2853 && delay == 910000 && gm == 0.75 {
                        let shift = 64e-9;
                        let cut = advance(&i.r, u.after(), l.at - shift - u.at);
                        let span = dt + 64;
                        let hit = trials
                            .iter()
                            .find(|v| {
                                v["start_to_lower_s"]
                                    .as_f64()
                                    .is_some_and(|x| (x - shift).abs() < 1e-18)
                            })
                            .unwrap();
                        assert!(hit["runs"][1]["native"].is_object());
                        let dir = path.join("shifted");
                        fs::create_dir_all(&dir)?;
                        write_json(&dir.join("featured.json"), hit)?;
                        replay::write(
                            &dir,
                            &i,
                            &g,
                            l,
                            span,
                            term.as_ref().unwrap(),
                            Some((shift, cut)),
                        )?;
                    }
                    cases.push(json!({"delay_ns":delay,"gate_mu":gm,"source_lower":l.data(),"roof_velocity":g.velocity.to_array(),"reference_kind":s.limit["kind"],"trials":trials}));
                    continue;
                }
                if cost {
                    let files = [xml(&i, &g, l, dt, false), xml(&i, &g, l, dt, true)];
                    let mut nanos: [Vec<u64>; 2] = [vec![], vec![]];
                    let mut admitted = false;
                    for pass in 0..=8 {
                        for side in [pass % 2, 1 - pass % 2] {
                            let (ns, hit) = sample(&files[side])?;
                            if side == 1 {
                                if pass > 0 {
                                    assert_eq!(hit, admitted);
                                }
                                admitted = hit;
                            } else {
                                assert!(!hit);
                            }
                            if pass > 0 {
                                nanos[side].push(ns);
                            }
                        }
                    }
                    cases.push(json!({"delay_ns":delay,"gate_mu":gm,"dt_ns":dt,"admitted":admitted,"wall_ns":nanos}));
                    continue;
                }
                let mut lanes = vec![];
                for on in [false, true] {
                    lanes.push(run(&i, &g, l, dt, on, false)?);
                }
                let oracle=term.as_ref().map(|t|{
                    let h=dt as f64*1e-9;let j=t.bin(0.,h);let totals=[j[0]+l.law.impulse,j[1]];let end=t.state(&i.r,l,h);
                    json!({"loss_j":loss(&i,&g,l,t),"loads":[totals[0].dot(i.r.n),totals[1].dot(-i.r.n)],"radius":t.radius,"after":body_view(native::at_foot(&i.r,end)),"bounds":t.bounds(&i.r,h-t.offset),"offset_s":t.offset,"finite":t.finite.len(),"corners":t.corner.map(|k|k%4),"gap_cap_s":t.gap_time})
                });
                {
                    for on in [false, true] {
                        lanes.push(run(&i, &g, l, dt, on, true)?);
                    }
                    if !heldout
                        && !feature
                        && i.r.id == 17
                        && MODES[i.mode] == "stick"
                        && delay == 750000
                        && gm == 0.35
                        && oracle.is_some()
                        && lanes[3]["native"].is_object()
                    {
                        replay::write(path, &i, &g, l, dt, term.as_ref().unwrap(), None)?;
                        fs::write(path.join("ordinary.sdf"), xml(&i, &g, l, dt, false))?;
                        fs::write(path.join("projected.sdf"), xml(&i, &g, l, dt, true))?;
                        write_json(
                            &path.join("featured.json"),
                            &json!({"id":i.r.id,"mode":MODES[i.mode],"delay_ns":delay,"gate_mu":gm,"dt_ns":dt,"oracle":oracle,"runs":lanes}),
                        )?;
                        feature = true;
                    }
                }
                cases.push(json!({"delay_ns":delay,"gate_mu":gm,"source_lower":l.data(),"roof_velocity":g.velocity.to_array(),"reference_kind":s.limit["kind"],"dt_ns":dt,"oracle":oracle,"runs":lanes}));
            }
        }
        if !cases.is_empty() {
            groups.push(json!({"id":i.r.id,"mode":MODES[i.mode],"tensor":i.r.body.tensor.to_cols_array(),"gravity":i.r.gravity.to_array(),"pad_velocity":i.r.pad_v.to_array(),"cases":cases}));
        }
    }
    write_json(
        &path.join(if period && cost {
            if heldout {
                "grid_heldout.json"
            } else {
                "grid.json"
            }
        } else if period {
            if heldout {
                "period_heldout.json"
            } else {
                "period.json"
            }
        } else if phase && cost {
            "arrival_cost.json"
        } else if phase {
            if heldout {
                "phase_heldout.json"
            } else {
                "phase.json"
            }
        } else if cost {
            "timings.json"
        } else if heldout {
            "heldout.json"
        } else {
            "native.json"
        }),
        &json!({"donors":groups,"featured":feature}),
    )
}
