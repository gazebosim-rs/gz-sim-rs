// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cross primary-impact and subsequent solver effort at identical native boundaries.
use super::*;

fn replay(
    fixture: Fixture,
    cut: &ReboundFrame,
    end: u64,
    initial: Mode,
    switch_after: (u64, Mode),
) -> Result<Vec<ReboundFrame>> {
    let mut rig = restart(fixture, ADAPTIVE, cut)?;
    rig.set_mode(initial);
    let mut frames = vec![rig.frame()?];
    while rig.tick < end {
        if rig.tick == switch_after.0 {
            rig.set_mode(switch_after.1);
        }
        rig.advance_at(command(fixture, rig.tick))?;
        frames.push(rig.frame()?);
    }
    Ok(frames)
}

fn first_load_episode(frames: &[ReboundFrame]) -> Option<(usize, usize)> {
    // The cut's receipt belongs to the preceding solve.
    let start = frames
        .iter()
        .enumerate()
        .skip(1)
        .find(|(_, r)| r.normal_impulse_ns > 0.)?
        .0;
    let len = frames[start..]
        .iter()
        .take_while(|r| r.normal_impulse_ns > 0.)
        .count();
    Some((start, start + len - 1))
}

fn twist_angle(a: &Body, b: &Body) -> f64 {
    let q = DQuat::from_array(a.orientation).conjugate() * DQuat::from_array(b.orientation);
    2. * DVec3::new(q.x, q.y, q.z).length().atan2(q.w.abs())
}

fn delta(a: &ReboundFrame, b: &ReboundFrame) -> Value {
    let pos = com(a) - com(b);
    let velocity = velocity(a) - velocity(b);
    let spin = DVec3::from(a.foot.angular_velocity) - DVec3::from(b.foot.angular_velocity);
    json!({"tick":a.tick,"com_xyz_m":pos.to_array(),"com_distance_m":pos.length(),
        "com_velocity_xyz_m_s":velocity.to_array(),"com_velocity_distance_m_s":velocity.length(),
        "foot_orientation_angle_rad":twist_angle(&a.foot,&b.foot),"foot_spin_xyz_rad_s":spin.to_array(),
        "normal_ns":a.normal_impulse_ns-b.normal_impulse_ns,
        "corner_ns":std::array::from_fn::<_,4,_>(|k|a.corner_ns[k]-b.corner_ns[k])})
}

fn summarize(frames: &[ReboundFrame], fine: &[ReboundFrame], boundary: usize) -> Value {
    let (start, last) = first_load_episode(frames).unzip();
    let peak = frames
        .iter()
        .zip(fine)
        .map(|(a, b)| (com(a).distance(com(b)), a.tick))
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .expect("nonempty matched window");
    let first_delta = |radius: f64| {
        frames
            .iter()
            .zip(fine)
            .find(|(a, b)| com(a).distance(com(b)) > radius)
            .map(|(a, _)| a.tick)
    };
    let earliest_contact_difference = frames
        .iter()
        .skip(1)
        .zip(&fine[1..])
        .find(|(a, b)| {
            a.corner_ns
                .iter()
                .zip(b.corner_ns)
                .any(|(x, y)| (x - y).abs() > 1e-12)
        })
        .map(|(a, _)| a.tick);
    let later_load = frames[boundary + 1..]
        .iter()
        .find(|r| r.normal_impulse_ns > 0.)
        .map(|r| r.tick);
    let impulses = |rows: &[ReboundFrame]| {
        let mut corners = [0.; 4];
        for f in rows {
            for (sum, n) in corners.iter_mut().zip(f.corner_ns) {
                *sum += n;
            }
        }
        corners
    };
    let raw=frames.iter().map(|r|json!({"tick":r.tick,"com_m":com(r).to_array(),"com_v_m_s":velocity(r).to_array(),
        "foot_q_xyzw":r.foot.orientation,"foot_w_rad_s":r.foot.angular_velocity,"normal_ns":r.normal_impulse_ns,
        "corner_ns":r.corner_ns,"native_steps":r.native_steps,"iterations_per_solve":r.solver_iterations,
        "first_positive_native_substep":r.first_loaded_substep})).collect::<Vec<_>>();
    let end = frames.len() - 1;
    json!({"first_positive_tick":start.map(|i|frames[i].tick),"end_of_first_positive_run_tick":last.map(|i|frames[i].tick),
        "first_positive_after_primary_window":later_load,"first_corner_impulse_delta_over_1e_12_ns_tick":earliest_contact_difference,
        "first_com_delta_over_1nm_tick":first_delta(1e-9),"first_com_delta_over_10um_tick":first_delta(1e-5),
        "first_com_delta_over_100um_tick":first_delta(1e-4),"first_com_delta_over_1mm_tick":first_delta(1e-3),
        "max_com_delta_m":peak.0,"max_com_delta_tick":peak.1,"cut_delta_vs_matched_four":delta(&frames[0],&fine[0]),
        "primary_end_delta_vs_matched_four":delta(&frames[boundary],&fine[boundary]),"end_delta_vs_matched_four":delta(&frames[end],&fine[end]),
        "primary_corner_total_ns":impulses(&frames[1..=boundary]),"following_corner_total_ns":impulses(&frames[boundary+1..]),
        "samples":raw})
}

pub(super) fn investigate(
    f: Fixture,
    adaptive: &[ReboundFrame],
    fine: &[ReboundFrame],
) -> Result<Value> {
    let cut = adaptive.first().ok_or("empty intervention segment")?;
    let end = adaptive.last().expect("nonempty continuation").tick;
    let refined = branch(
        f,
        ADAPTIVE,
        cut,
        Mode::Uniform(Effort::Split(8)),
        end as usize,
    )?;
    let sixteen = branch(
        f,
        ADAPTIVE,
        cut,
        Mode::Uniform(Effort::Split(16)),
        end as usize,
    )?;
    let (_, boundary) =
        first_load_episode(adaptive).ok_or("no primary load in adaptive continuation")?;
    // The crossover boundary is labelled retrospectively from the unmodified
    // adaptive branch and remains identical in clock time for both interventions.
    let last = adaptive[boundary].tick;
    let primary_fine = replay(f, cut, end, FINE, (last, ADAPTIVE))?;
    let after_fine = replay(f, cut, end, ADAPTIVE, (last, FINE))?;
    for i in 0..=boundary {
        if !same_state(&primary_fine[i], &fine[i]) || !same_state(&after_fine[i], &adaptive[i]) {
            return Err(
                "intervention altered its native prefix before the crossover boundary".into(),
            );
        }
    }
    let end_index = adaptive.len() - 1;
    let endpoints = [adaptive, &primary_fine, &after_fine, fine]
        .map(|rows| com(rows.last().expect("nonempty continuation")));
    let [aa, fa, af, ff] = endpoints;
    // Endpoint positions form a two-factor experiment in primary and subsequent
    // solver effort. The interaction matters when earlier solve choices change
    // later contact timing or active features.
    Ok(
        json!({"donor":"adaptive","cut_tick":cut.tick,"end_tick":end,"crossover_after_tick":last,
        "crossover_rule":"end of the first contiguous positive-reaction episode in the unmodified adaptive continuation",
        "sixteen_minus_eight":{"primary_end":delta(&sixteen[boundary],&refined[boundary]),"endpoint":delta(&sixteen[end_index],&refined[end_index])},
        "endpoint_effects_m":{"adaptive_minus_fine":(aa-ff).to_array(),
            "adaptive_primary_minus_fine_primary_with_fine_follow":(af-ff).to_array(),
            "adaptive_follow_minus_fine_follow_with_fine_primary":(fa-ff).to_array(),
            "primary_follow_interaction":((aa-af)-(fa-ff)).to_array()},
        "adaptive":summarize(adaptive,fine,boundary),"matched_four":summarize(fine,fine,boundary),
        "matched_eight":summarize(&refined,fine,boundary), "matched_sixteen":summarize(&sixteen,fine,boundary),
        "fine_primary_adaptive_follow":summarize(&primary_fine,fine,boundary),
        "adaptive_primary_fine_follow":summarize(&after_fine,fine,boundary)}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn outer_primary_crossover_preserves_the_common_prefix_and_uses_a_single_fixed_boundary()
    -> Result<()> {
        let f = CASES
            .iter()
            .find(|f| f.name == "outer-fast")
            .copied()
            .ok_or("outer-fast case missing")?;
        let mut r = start(f, ADAPTIVE)?;
        while r.tick < 217 {
            r.advance_at(command(f, r.tick))?;
        }
        let cut = r.frame()?;
        let adaptive = branch(f, ADAPTIVE, &cut, ADAPTIVE, 222)?;
        let fine = branch(f, ADAPTIVE, &cut, FINE, 222)?;
        let report = investigate(f, &adaptive, &fine)?;
        assert_eq!(report["crossover_after_tick"], 218);
        assert_eq!(
            report["matched_four"]["samples"][1],
            report["fine_primary_adaptive_follow"]["samples"][1]
        );
        assert_eq!(
            report["adaptive"]["samples"][1],
            report["adaptive_primary_fine_follow"]["samples"][1]
        );
        assert_eq!(
            report["adaptive_primary_fine_follow"]["samples"][2]["native_steps"],
            4
        );
        Ok(())
    }
}
