// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Vary contact refresh cadence, solver iteration count and native grid phase from a common primary landing.
use super::*;

const TICKS: u64 = 26;
const REFERENCE: Effort = Effort::Split(32);
const COMMON_FOLLOW: Effort = Effort::Split(16);
const ALIGNED: &[(u32, usize)] = &[
    // Equal nominal work cohorts of 16, 32, 64 and 128 solver iterations per outer step.
    (1, 16),
    (2, 8),
    (4, 4),
    (1, 32),
    (2, 16),
    (4, 8),
    (8, 4),
    (2, 32),
    (4, 16),
    (8, 8),
    (16, 4),
    (4, 32),
    (8, 16),
    (16, 8),
    (32, 4),
];

#[derive(Clone, Copy)]
struct Schedule {
    onset: Effort,
    follow: Effort,
}
fn grid(calls: u32, iterations: usize, phase: GridPhase) -> Effort {
    Effort::Grid {
        calls,
        iterations,
        phase,
    }
}
fn label(e: Effort) -> String {
    match e {
        Effort::Grid {
            calls,
            iterations,
            phase,
        } => match phase {
            GridPhase::Aligned => format!("{calls}x{iterations}"),
            GridPhase::HalfEarly => format!("{calls}x{iterations}_early"),
            GridPhase::HalfLate => format!("{calls}x{iterations}_late"),
        },
        _ => format!("{}x{}", e.native_steps(), e.iterations()),
    }
}
fn schedules() -> Vec<Schedule> {
    let mut schedules = vec![];
    for &(calls, iterations) in ALIGNED {
        let aligned = grid(calls, iterations, GridPhase::Aligned);
        schedules.push(Schedule {
            onset: aligned,
            follow: aligned,
        });
        if (calls, iterations) != (16, 4) {
            schedules.push(Schedule {
                onset: aligned,
                follow: COMMON_FOLLOW,
            });
            schedules.push(Schedule {
                onset: COMMON_FOLLOW,
                follow: aligned,
            });
        }
    }
    for calls in [4, 8, 16] {
        for phase in [GridPhase::HalfEarly, GridPhase::HalfLate] {
            schedules.push(Schedule {
                onset: grid(calls, 4, phase),
                follow: COMMON_FOLLOW,
            });
        }
    }
    schedules
}

fn recorded(f: Fixture, cut: &ReboundFrame, schedule: Schedule) -> Result<Vec<ReboundFrame>> {
    let mut rig = restart(f, ADAPTIVE, cut)?;
    let mut frames = vec![rig.frame()?];
    for offset in 0..TICKS {
        let effort = if offset == 0 {
            schedule.onset
        } else {
            schedule.follow
        };
        rig.set_mode(Mode::Uniform(effort));
        rig.advance_at(command(f, rig.tick))?;
        let frame = rig.frame()?;
        if (frame.native_steps, frame.solver_iterations)
            != (effort.native_steps(), effort.iterations())
        {
            return Err("native work receipt does not match the commanded schedule".into());
        }
        frames.push(frame);
    }
    Ok(frames)
}
fn endpoints(e: Effort) -> Vec<u64> {
    let mut elapsed = Duration::ZERO;
    (1..=e.native_steps())
        .map(|i| {
            elapsed += e.duration_at(Duration::from_micros(STEP_US), i);
            elapsed.as_nanos() as u64
        })
        .collect()
}

fn measurement(r: &ReboundFrame, endpoints: &[u64]) -> Value {
    json!({"tick":r.tick, "normal_ns":r.normal_impulse_ns,"corner_ns":r.corner_ns,"tangent_on_foot_ns":r.tangent_on_foot_ns,
        "com_m":com(r).to_array(),"velocity_m_s":velocity(r).to_array(),"spin_rad_s":r.foot.angular_velocity,"orientation_xyzw":r.foot.orientation,
        "first_loaded_offset_ns":r.first_loaded_substep.map(|i|endpoints[i as usize-1]),"native_calls":r.native_steps,"solver_iterations":r.solver_iterations})
}
fn difference(a: &ReboundFrame, b: &ReboundFrame) -> Value {
    let corners = std::array::from_fn::<_, 4, _>(|i| a.corner_ns[i] - b.corner_ns[i]);
    let tangent = a
        .tangent_on_foot_ns
        .zip(b.tangent_on_foot_ns)
        .map(|(x, y)| DVec3::from(x).distance(y.into()));
    json!({"com_m":distance(a,b),"velocity_m_s":velocity(a).distance(velocity(b)),
        "spin_rad_s":DVec3::from(a.foot.angular_velocity).distance(b.foot.angular_velocity.into()),
        "normal_ns":a.normal_impulse_ns-b.normal_impulse_ns,
        "corner_l2_ns":corners.into_iter().map(|x|x*x).sum::<f64>().sqrt(),
        "tangent_l2_ns":tangent})
}

fn summary(branch: &[ReboundFrame], reference: &[ReboundFrame], schedule: Schedule) -> Value {
    let incoming = branch[0].tick;
    let elapsed_ns = |r: &ReboundFrame| -> Option<u64> {
        let offset = r.first_loaded_substep?;
        let effort = if r.tick == incoming + 1 {
            schedule.onset
        } else {
            schedule.follow
        };
        Some((r.tick - incoming - 1) * STEP_US * 1000 + endpoints(effort)[offset as usize - 1])
    };
    let onset = branch[1..]
        .iter()
        .position(|r| r.first_loaded_substep.is_some())
        .map(|i| i + 1);
    let next = onset
        .and_then(|i| {
            branch[i + 1..]
                .iter()
                .position(|r| r.normal_impulse_ns == 0.)
                .map(|j| i + j + 1)
        })
        .and_then(|i| {
            branch[i + 1..]
                .iter()
                .find(|r| r.first_loaded_substep.is_some())
        });
    let ini = endpoints(schedule.onset);
    let follow = endpoints(schedule.follow);
    let samples = branch
        .iter()
        .skip(1)
        .map(|f| {
            measurement(
                f,
                if f.tick == incoming + 1 {
                    &ini
                } else {
                    &follow
                },
            )
        })
        .collect::<Vec<_>>();
    json!({"onset":label(schedule.onset),"follow":label(schedule.follow),"onset_native_endpoints_ns":ini,"follow_native_endpoints_ns":follow,
        "primary_first_receipt_ns_after_cut":onset.and_then(|i|elapsed_ns(&branch[i])),
        "next_load_after_unloaded_outer_ns_after_cut":next.and_then(elapsed_ns),
        "normal_total_ns":branch[1..].iter().map(|r|r.normal_impulse_ns).sum::<f64>(),
        "onset_vs_32x4":difference(&branch[1],&reference[1]),
        "after_5ms_vs_32x4":difference(&branch[10],&reference[10]),
        "after_13ms_vs_32x4":difference(branch.last().unwrap(),reference.last().unwrap()),"samples":samples})
}

fn study(f: Fixture, out: &Path) -> Result<Value> {
    let (adaptive, _) = run(f, ADAPTIVE)?;
    let mut cuts = vec![];
    for stroke in 0..sweep::DEPART_US.len() {
        let cut = &adaptive[primary(&adaptive, stroke)? - 1];
        let fine = recorded(
            f,
            cut,
            Schedule {
                onset: REFERENCE,
                follow: REFERENCE,
            },
        )?;
        let mut cells = vec![];
        for schedule in schedules() {
            let records = recorded(f, cut, schedule)?;
            if label(schedule.onset) == "32x4" && label(schedule.follow) == "32x4" {
                native_prefix_check(&records, &fine)?;
            }
            if f.name == "outer-fast"
                && stroke == 1
                && schedule.onset == schedule.follow
                && schedule.onset.native_steps() as usize * schedule.onset.iterations() == 64
                && [4, 8, 16].contains(&schedule.onset.native_steps())
            {
                let stem = format!(
                    "refresh{}_iter{}",
                    schedule.onset.native_steps(),
                    schedule.onset.iterations()
                );
                trace(out, &stem, &records)?;
            }
            cells.push(summary(&records, &fine, schedule));
        }
        cuts.push(json!({"stroke":stroke+1,"cut":snapshot(cut),"cut_foot":cut.foot,"cut_upper":cut.upper,"cells":cells}));
    }
    Ok(json!({"fixture":f,"cuts":cuts}))
}

pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let cases = CASES
        .into_iter()
        .filter(|f| f.name.starts_with("outer-"))
        .map(|f| study(f, out))
        .collect::<Result<Vec<_>>>()?;
    write_json(
        &out.join("cadence.json"),
        &json!({"step_us":STEP_US,"ticks":TICKS,"reference":"32x4",
        "common_follow":"16x4","cases":cases}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn half_shift_translates_interior_boundaries_and_preserves_outer_duration() {
        let dt = Duration::from_micros(STEP_US);
        for calls in [4, 8, 16] {
            let period = STEP_US * 1000 / u64::from(calls);
            for (p, offset) in [
                (GridPhase::Aligned, 0),
                (GridPhase::HalfEarly, -1),
                (GridPhase::HalfLate, 1),
            ] {
                let g = grid(calls, 4, p);
                assert!(g.validate_grid(dt).is_ok());
                let ends = endpoints(g);
                assert_eq!(ends.last(), Some(&(STEP_US * 1000)));
                for i in 1..calls {
                    assert_eq!(
                        ends[i as usize - 1] as i64,
                        ((i64::from(i) * 2 + offset) * period as i64) / 2
                    );
                }
            }
        }
        assert!(grid(1, 4, GridPhase::HalfEarly).validate_grid(dt).is_err());
        assert!(grid(4, 0, GridPhase::Aligned).validate_grid(dt).is_err());
    }

    #[test]
    fn explicit_four_iteration_grid_replays_default_split_and_restores_profile() -> Result<()> {
        let f = CASES.into_iter().find(|f| f.name == "outer-fast").unwrap();
        let (a, _) = run(f, ADAPTIVE)?;
        let cut = &a[primary(&a, 1)? - 1];
        let native = recorded(
            f,
            cut,
            Schedule {
                onset: Effort::Split(8),
                follow: Effort::Split(4),
            },
        )?;
        let configured = recorded(
            f,
            cut,
            Schedule {
                onset: grid(8, 4, GridPhase::Aligned),
                follow: Effort::Split(4),
            },
        )?;
        native_prefix_check(&configured, &native)?;
        let changed = recorded(
            f,
            cut,
            Schedule {
                onset: grid(8, 8, GridPhase::Aligned),
                follow: Effort::Split(4),
            },
        )?;
        assert_eq!(changed[1].solver_iterations, 8);
        assert_eq!(changed[2].solver_iterations, 4);
        assert_ne!(changed[1].corner_ns, native[1].corner_ns);
        assert!(native[0].foot == changed[0].foot);
        Ok(())
    }
}
