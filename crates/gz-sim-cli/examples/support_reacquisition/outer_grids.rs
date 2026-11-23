// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Nested reference grids for a non-smooth impact at a fixed actuator clock.
use super::*;
use gz_physics::NativeStepGrid;

const GRIDS: [u32; 9] = [4, 8, 16, 32, 64, 128, 256, 512, 1024];
const HORIZON: usize = 26;
const OUTER: Duration = Duration::from_micros(STEP_US);
fn elapsed_ns(n: u32, substep: u32) -> Result<u64> {
    let g = NativeStepGrid::new(OUTER, n)?;
    Ok(g.intervals()
        .take(substep as usize)
        .sum::<Duration>()
        .as_nanos() as u64)
}

fn describe(frames: &[ReboundFrame], n: u32) -> Result<Value> {
    let cut = &frames[0];
    let mut first = None;
    let mut reload = None;
    let mut unloaded_after_impact = false;
    let mut total = 0.;
    let mut load_ticks = Vec::new();
    let mut points = vec![];
    for r in &frames[1..] {
        total += r.normal_impulse_ns;
        if let Some(k) = r.first_loaded_substep {
            let native_end = (r.time_us - STEP_US) * 1000 + elapsed_ns(n, k)?;
            first.get_or_insert(native_end);
            if unloaded_after_impact {
                reload.get_or_insert(native_end);
            }
            load_ticks.push(r.tick);
        } else if first.is_some() {
            unloaded_after_impact = true;
        }
        points.push(json!({"tick":r.tick,"com_m":com(r).to_array(),"v_m_s":velocity(r).to_array(),
            "spin_rad_s":r.foot.angular_velocity,"corners_ns":r.corner_ns,"normal_ns":r.normal_impulse_ns,
            "tangent_on_foot_ns":r.tangent_on_foot_ns,"first_native_load":r.first_loaded_substep}));
    }
    let first = first.ok_or("matched primary continuation had no positive reaction")?;
    Ok(
        json!({"n":n,"first_native_end_ns":first,"first_load_after_unloaded_outer_tick_ns":reload,
        "cut_tick":cut.tick,"normal_sum_ns":total,"loaded_ticks":load_ticks,"samples":points}),
    )
}

fn compare(a: &[ReboundFrame], b: &[ReboundFrame]) -> Value {
    let endpoint = a.len() - 1;
    let (r, s) = (&a[endpoint], &b[endpoint]);
    let spin = |r: &ReboundFrame| DVec3::from(r.foot.angular_velocity);
    let jsum =
        |frames: &[ReboundFrame]| frames[1..].iter().map(|r| r.normal_impulse_ns).sum::<f64>();
    let diff = |x: &ReboundFrame, y: &ReboundFrame| {
        std::array::from_fn::<_, 4, _>(|i| x.corner_ns[i] - y.corner_ns[i])
    };
    json!({"end_distance_m":distance(r,s),"peak_distance_m":a.iter().zip(b).map(|(x,y)|distance(x,y)).fold(0.,f64::max),
        "onset_impulse_delta_ns":a[1].normal_impulse_ns-b[1].normal_impulse_ns,
        "onset_corner_delta_ns":diff(&a[1],&b[1]),"onset_spin_distance_rad_s":spin(&a[1]).distance(spin(&b[1])),
        "end_velocity_distance_m_s":velocity(r).distance(velocity(s)),"end_spin_distance_rad_s":spin(r).distance(spin(s)),
        "window_impulse_delta_ns":jsum(a)-jsum(b)})
}

fn study(f: Fixture, out: &Path) -> Result<Value> {
    let (donor, _) = run(f, ADAPTIVE)?;
    let mut strokes = vec![];
    for stroke in 0..3 {
        let cut_tick = primary(&donor, stroke)? - 1;
        let cut = &donor[cut_tick];
        let mut grids = vec![];
        let mut differences = vec![];
        let mut previous: Option<Vec<ReboundFrame>> = None;
        for n in GRIDS {
            let branch = branch(
                f,
                ADAPTIVE,
                cut,
                Mode::Uniform(Effort::Split(n)),
                cut_tick + HORIZON,
            )?;
            if f.name == "outer-fast" && stroke == 1 && [16, 128, 1024].contains(&n) {
                trace(out, &format!("grid_{n}"), &branch)?;
            }
            grids.push(describe(&branch, n)?);
            if let Some(a) = previous {
                differences.push(json!({"coarse":n/2,"fine":n,"difference":compare(&a,&branch)}));
            }
            previous = Some(branch);
        }
        strokes.push(
            json!({"stroke":stroke+1,"cut":snapshot(cut),"grids":grids,"successive":differences}),
        );
    }
    Ok(json!({"fixture":f,"strokes":strokes}))
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let cases = CASES
        .into_iter()
        .filter(|f| f.name.starts_with("outer-"))
        .map(|f| study(f, out))
        .collect::<Result<Vec<_>>>()?;
    write_json(
        &out.join("grids.json"),
        &json!({"outer_us":STEP_US,"ticks_after_cut":HORIZON,"cases":cases}),
    )?;
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Reference grid sensitivity on the second primary landing",
        "lanes":([16u32,128,1024].map(|n|json!({"name":format!("{n} native intervals"),"file":format!("traces/grid_{n}.json"),"label":format!("grid_{n}")})))}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn balanced_contact_grids_keep_the_actuator_and_the_original_landing_fork_in_agreement()
    -> Result<()> {
        let f = CASES
            .iter()
            .find(|f| f.name == "outer-fast")
            .copied()
            .unwrap();
        let (donor, _) = run(f, ADAPTIVE)?;
        let cut = &donor[primary(&donor, 1)? - 1];
        let r = restart(f, ADAPTIVE, cut)?;
        let lane = &r.case.lane;
        let mut detached = r
            .case
            .backend()
            .landing_fork(None, lane.foot, lane.pad, OUTER)?;
        let n = 64;
        let mut previous = Duration::ZERO;
        let mut total = 0.;
        let mut loads = vec![];
        detached.step_outer_balanced(n, Some(command(f, cut.tick)), |elapsed, s| {
            assert!(elapsed > previous);
            previous = elapsed;
            let j = s.normal_impulse();
            if j > 0. {
                loads.push(elapsed);
            }
            total += j;
        })?;
        let live = branch(
            f,
            ADAPTIVE,
            cut,
            Mode::Uniform(Effort::Split(n)),
            cut.tick as usize + 1,
        )?;
        assert_eq!(previous, OUTER);
        let states = detached.states().map(Body::from);
        assert!(states == [live[1].foot, live[1].upper, live[1].pad]);
        assert!((total - live[1].normal_impulse_ns).abs() < 1e-12);
        assert_eq!(
            loads[0].as_nanos(),
            u128::from(elapsed_ns(n, live[1].first_loaded_substep.unwrap())?)
        );
        Ok(())
    }
}
