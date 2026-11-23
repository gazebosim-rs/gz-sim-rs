// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time a fixed 500 µs interval on the same measured warm and cold return states.
use super::*;
use gz_physics_multifidelity::{ContactIntervalReceipt, ReturnFeatureMode};
use std::time::Instant;

#[derive(Clone, Copy)]
enum Action {
    Ordinary,
    Hook(LandingRowOrder),
    Fresh,
    Continuation { directed: bool, retime: bool },
    Witness(ReturnFeatureMode),
    Material { certified: bool, follow: bool },
}
fn thread_cpu_ns() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        let t = rustix::time::clock_gettime(rustix::time::ClockId::ThreadCPUTime);
        Some(t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64)
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}
#[derive(Clone, Copy)]
struct Treatment {
    cold: bool,
    calls: u32,
    passes: usize,
    hook: Action,
}
impl Treatment {
    fn label(self) -> &'static str {
        match self.hook {
            Action::Ordinary => "ordinary",
            Action::Material {
                certified: false,
                follow: false,
            } => "point",
            Action::Material {
                certified: true,
                follow: false,
            } => "certified_point",
            Action::Material {
                certified: false,
                follow: true,
            } => "launch",
            Action::Material {
                certified: true,
                follow: true,
            } => "certified_launch",
            Action::Continuation {
                directed: true,
                retime: true,
            } => "retimed_directed",
            Action::Continuation {
                directed: true,
                retime: false,
            } => "directed",
            Action::Continuation {
                directed: false, ..
            } => "subdivided",
            Action::Fresh => "every",
            Action::Witness(ReturnFeatureMode::FirstUnloaded) => "first_unloaded",
            Action::Witness(ReturnFeatureMode::FirstStale) => "first_stale",
            Action::Witness(ReturnFeatureMode::FirstTravel) => "first_travel",
            Action::Witness(ReturnFeatureMode::WhileStale) => "while_stale",
            Action::Hook(LandingRowOrder::Unchanged) => "no_op",
            Action::Hook(LandingRowOrder::SoleQuadrants) => "ordered",
            Action::Hook(LandingRowOrder::ExchangePositiveX) => unreachable!(),
        }
    }
}
#[derive(PartialEq)]
struct Readback {
    receipt: ContactIntervalReceipt,
    end: [BodyState; 3],
    rows: Option<(u64, u64)>,
    geometry: Option<(u64, u64, u64)>,
}
fn advance(
    cut: &MultiFidelityBackend,
    ids: [BodyId; 3],
    speed: f64,
    t: Treatment,
    observe: bool,
) -> Result<(Readback, u64, Option<u64>)> {
    let [foot, _, pad] = ids;
    let mut world = cut.copy_airborne_landing(foot, pad, t.cold)?;
    if world.state(pad).ok_or("missing pad")?.velocity[2] != speed {
        world.set_world_velocity(pad, [0., 0., speed], [0.; 3])?;
    }
    let iterations = ContactSolverIterations {
        temporal: 4,
        pgs: None,
        relaxation: Some(t.passes),
    };
    let mut geometry = None;
    let cpu0 = thread_cpu_ns();
    let start = Instant::now();
    let (receipt, rows) = if let Action::Hook(order) = t.hook {
        let (r, counts) = if observe {
            world.observe_landing_row_order((foot, pad), DT, t.calls, iterations, order, |_| {})?
        } else {
            world.step_landing_row_order((foot, pad), DT, t.calls, iterations, order)?
        };
        (
            r,
            Some((
                counts.nonempty_manifolds_visited,
                counts.ordered_manifolds_changed,
            )),
        )
    } else if matches!(
        t.hook,
        Action::Material { .. } | Action::Continuation { .. }
    ) {
        let (certified, follow, subdivide, directed, retime) = match t.hook {
            Action::Material { certified, follow } => (certified, follow, false, false, false),
            Action::Continuation { directed, retime } => (true, true, true, directed, retime),
            _ => unreachable!(),
        };
        let pair = [(foot, pad)];
        let after: &[(BodyId, BodyId)] = if follow { &pair } else { &[] };
        let mut probe = |_: ContactSubstep<'_>| {};
        let receipt = world
            .advance_box_contacts(
                DT,
                t.calls,
                iterations,
                BoxContactActions {
                    finite_point_impacts: &pair,
                    certify_face_vertices: if certified { &pair } else { &[] },
                    retime_finite_transfer: if retime { &pair } else { &[] },
                    subdivide_point_bounds: if subdivide { &pair } else { &[] },
                    target_point_bounds: if directed { &pair } else { &[] },
                    sliding_vertices: after,
                    sticking_points: after,
                    point_transfers: after,
                    sliding_skew_edges: after,
                    slip_launches: after,
                    ..Default::default()
                },
                if observe { Some(&mut probe) } else { None },
            )?
            .interval;
        (receipt, None)
    } else if let Action::Witness(mode) = t.hook {
        let mut probe = |_: ContactSubstep<'_>| {};
        let (r, c) = world.step_returning_box_features(
            (foot, pad),
            DT,
            t.calls,
            iterations,
            mode,
            if observe { Some(&mut probe) } else { None },
        )?;
        geometry = Some((c.unloaded_tests, c.stronger_plane, c.requery));
        (r, None)
    } else if let Action::Fresh = t.hook {
        let mut probe = |_: ContactSubstep<'_>| {};
        let out = world.advance_box_contacts(
            DT,
            t.calls,
            iterations,
            BoxContactActions {
                fresh_features: &[(foot, pad)],
                ..Default::default()
            },
            if observe { Some(&mut probe) } else { None },
        )?;
        geometry = Some((0, 0, out.rebuilt_manifolds));
        (out.interval, None)
    } else if observe {
        (
            world.observe_contact_interval(DT, t.calls, iterations, |_| {})?,
            None,
        )
    } else {
        (world.step_contact_interval(DT, t.calls, iterations)?, None)
    };
    let ns = u64::try_from(start.elapsed().as_nanos())?;
    let cpu = thread_cpu_ns().zip(cpu0).map(|(end, start)| end - start);
    Ok((
        Readback {
            receipt,
            end: states(&world, ids)?,
            rows,
            geometry,
        },
        ns,
        cpu,
    ))
}

pub(crate) fn run(
    dir: &Path,
    repeat: u8,
    features: bool,
    travel: bool,
    material: bool,
    events: bool,
) -> Result<()> {
    if !(1..=2).contains(&repeat) {
        return Err("use repeat index 1 or 2".into());
    }
    let study = dir
        .parent()
        .ok_or("missing studies")?
        .join("multifidelity-secondary-order-field");
    let mut treatments = vec![];
    for cold in [false, true] {
        for calls in [8, 64] {
            let passes: &[usize] = if travel || material || events {
                &[8]
            } else if features {
                &[1, 8, 32]
            } else {
                &[1, 8, 16, 32]
            };
            let modes: &[Action] = if events {
                &[
                    Action::Ordinary,
                    Action::Continuation {
                        directed: false,
                        retime: false,
                    },
                    Action::Continuation {
                        directed: true,
                        retime: false,
                    },
                    Action::Continuation {
                        directed: true,
                        retime: true,
                    },
                ]
            } else if material {
                &[
                    Action::Ordinary,
                    Action::Material {
                        certified: false,
                        follow: false,
                    },
                    Action::Material {
                        certified: true,
                        follow: false,
                    },
                    Action::Material {
                        certified: false,
                        follow: true,
                    },
                    Action::Material {
                        certified: true,
                        follow: true,
                    },
                ]
            } else if travel {
                &[
                    Action::Ordinary,
                    Action::Witness(ReturnFeatureMode::FirstUnloaded),
                    Action::Witness(ReturnFeatureMode::FirstStale),
                    Action::Witness(ReturnFeatureMode::FirstTravel),
                ]
            } else if features {
                &[
                    Action::Ordinary,
                    Action::Fresh,
                    Action::Witness(ReturnFeatureMode::FirstUnloaded),
                    Action::Witness(ReturnFeatureMode::FirstStale),
                    Action::Witness(ReturnFeatureMode::WhileStale),
                ]
            } else {
                &[
                    Action::Ordinary,
                    Action::Hook(LandingRowOrder::Unchanged),
                    Action::Hook(LandingRowOrder::SoleQuadrants),
                ]
            };
            for &passes in passes {
                for &hook in modes {
                    treatments.push(Treatment {
                        cold,
                        calls,
                        passes,
                        hook,
                    });
                }
            }
        }
    }
    let out = dir.join(format!("repeat-{repeat}"));
    fs::create_dir_all(&out)?;
    // Every third held-out donor, including both ends of the registered panel.
    for (ci, slot) in (1..50).step_by(6).enumerate() {
        let (index, f, held) = cohort::coordinate(slot)?;
        let path = out.join(format!("case-{index:03}.json"));
        let src = load(&study.join(format!("cases/case-{index:03}.json")))?;
        if !held || src["slot"] != slot || src["status"] != "measured" {
            return Err("timing panel lost a donor".into());
        }
        let cut_tick = src["incoming"]["cut_tick"]
            .as_u64()
            .ok_or("missing incoming cut")?;
        let first = survey::cuts(f)?.0[0].ok_or("missing primary cut")?;
        let mut ran = false;
        observe_path_until(
            f,
            &first,
            PATHS[6],
            ProjectionSpan::Native,
            cut_tick,
            |cut, ids, incoming| {
                if incoming.tick != cut_tick {
                    return Ok(());
                }
                let check = cut.airborne_restart_state(ids[0], ids[2])?;
                if motion(&check) != src["incoming"]["motion"]
                    || cache(&check) != src["incoming"]["cache"]
                {
                    return Err("timed donor changed".into());
                }
                let speed = command(f, incoming.tick);
                let controls = treatments
                    .iter()
                    .map(|&t| advance(cut, ids, speed, t, true).map(|r| r.0))
                    .collect::<Result<Vec<_>>>()?;
                for triplet in controls
                    .chunks_exact(3)
                    .filter(|_| !features && !travel && !material && !events)
                {
                    if triplet[0].receipt != triplet[1].receipt || triplet[0].end != triplet[1].end
                    {
                        return Err(
                            "ordinary and unmodified hook changed physical histories".into()
                        );
                    }
                }
                let mut times = vec![vec![]; treatments.len()];
                let mut cpus = vec![vec![]; treatments.len()];
                for round in 0..18 {
                    for j in 0..treatments.len() {
                        // Relatively-prime stride 17 spreads consecutive treatments across grids.
                        let pos = (ci * 7 + round * 11 + j * 17) % treatments.len();
                        let ti = if repeat == 1 {
                            pos
                        } else {
                            treatments.len() - 1 - pos
                        };
                        let (result, ns, cpu) = advance(cut, ids, speed, treatments[ti], false)?;
                        if result != controls[ti] {
                            return Err(
                                "timed public receipt, body states or hook counts changed".into()
                            );
                        }
                        if round >= 2 {
                            times[ti].push(ns);
                            cpus[ti].push(cpu);
                        }
                    }
                }
                let cells=treatments.iter().zip(controls.iter()).zip(times).zip(cpus).map(|(((&t,control),wall),cpu)|{
                json!({"rebuilt":t.cold,"calls":t.calls,"passes":t.passes,"hook":t.label(),"wall_ns":wall,"thread_cpu_ns":cpu,"requeries":control.geometry.map(|c|c.2),"unloaded_tests":control.geometry.map(|c|c.0),"certified_stale":control.geometry.map(|c|c.1),
                  "visited_rows":control.rows.map(|r|r.0),"changed_rows":control.rows.map(|r|r.1),
                  "normal_ns":control.receipt.pairs.iter().map(|p|p.normal_impulse).sum::<f64>()})
            }).collect::<Vec<_>>();
                write_json(
                    &path,
                    &json!({"case":index,"slot":slot,"repeat":repeat,"cut_tick":cut_tick,"cells":cells}),
                )?;
                ran = true;
                Ok(())
            },
        )?;
        if !ran {
            return Err("cost donor absent".into());
        }
        eprintln!("fixed-time support-return cost repeat {repeat}, case {index}");
    }
    Ok(())
}
