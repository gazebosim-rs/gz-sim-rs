// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Several separated moving rollers support independent cables under an ordinary Bevy clock.
use super::*;
use std::{fmt::Write as _, time::Instant};
pub(super) fn fixture(rows: &[usize], guides: usize, processes: u8, seed: u64) -> Result<String> {
    if guides < rows.len() || guides > 48 {
        return Err("choose one to 48 native rollers with at least one roller per material".into());
    }
    let original = batch::fixture(rows, seed, 1, processes)?;
    let mut sdf = String::new();
    let mut lane: usize = 0;
    for line in original.lines() {
        if let Some((_, name)) = line.split_once("<cable name='line_") {
            lane = name[..2].parse()?;
        }
        if let Some((prefix, xyz)) = line.split_once("<particle position='") {
            let (x, rest) = xyz.split_once(' ').ok_or("missing material x coordinate")?;
            let (y, rest) = rest
                .split_once(' ')
                .ok_or("missing material y coordinate")?;
            let x = x.parse::<f64>()? + 0.39 * (lane % 6) as f64;
            let y = y.parse::<f64>()? + 0.98 * (lane / 6) as f64;
            writeln!(sdf, "{prefix}<particle position='{x:.17} {y:.17} {rest}")?;
        } else {
            writeln!(sdf, "{line}")?;
        }
    }
    let mut models = String::new();
    let mut surfaces = String::new();
    for i in 1..guides {
        let x = (i % 6) as f64 * 0.39;
        let y = (i / 6) as f64 * 0.98;
        let lift = 0.003 + 0.0004 * (i % 9) as f64;
        let t = 110_000_000 + u64::try_from(7_895_861 * (i % 11))?;
        let color = if i < rows.len() {
            "0.11 0.54 0.72"
        } else {
            "0.21 0.26 0.31"
        };
        writeln!(models,"<model name='roller_{i:02}'><pose>{x} {y} 0.5 1.5707963267948966 0 0</pose><link name='shaft'>
            <collision name='surface'><geometry><capsule><radius>0.08</radius><length>0.6</length></capsule></geometry></collision>
            <visual name='band_{i:02}'><geometry><cylinder><radius>0.08</radius><length>0.6</length></cylinder></geometry><material><diffuse>{color} 1</diffuse></material></visual>
            </link><plugin filename='gz-sim-prescribed-motion-system' name='gz::sim::systems::PrescribedMotion'>
            <trajectory end='repeat'><waypoint time_ns='0' displacement='0 0 0' velocity='0 0 0'/><waypoint time_ns='{t}' displacement='0 0 {lift}' velocity='0 0 0'/>
            <waypoint time_ns='400000000' displacement='0 0 0' velocity='0 0 0'/></trajectory></plugin></model>")?;
        writeln!(
            surfaces,
            "<capsule link='default::roller_{i:02}::shaft' coupling='one_way'/>"
        )?;
    }
    let start = sdf
        .find("<model name='base'>")
        .ok_or("missing static base")?;
    sdf.insert_str(start, &models);
    let start = sdf.find("<plane point=").ok_or("missing floor")?;
    sdf.insert_str(start, &surfaces);
    Ok(sdf)
}
pub(super) fn run(sdf: &Path, worker: &Path, ticks: u32, faults: bool) -> Result<()> {
    if ticks == 0 {
        return Err("need a positive interval".into());
    }
    let text = fs::read_to_string(sdf)?;
    let mut local = setup(&text, None)?;
    let mut managed = setup(&text, Some(worker))?;
    managed
        .world_mut()
        .resource_mut::<TrajectoryProcessPool>()
        .set_profiling(true);
    let mut d = Sha256::new();
    let mut stages = Vec::new();
    let mut counts = Vec::new();
    let mut loads: Vec<Vec<f64>> = Vec::new();
    let mut moves = 0;
    let mut kills = 0;
    let mut unplanned = Vec::new();
    let mut attempts = 0usize;
    let (mut canonical, mut payload, mut packed_groups) = (0_u64, 0_u64, 0_u64);
    for t in 0..ticks {
        let kill = faults && [25, 37, 59, 72].contains(&t);
        let mut victim = None;
        if kill {
            let p = managed.world().resource::<TrajectoryProcessPool>();
            let owners = p.material_owners();
            let g = kills % owners.len();
            terminate_worker(p.group_pid(g, owners[g]).ok_or("missing original child")?)?;
            kills += 1;
            victim = Some(g);
        }
        if faults && [24, 34, 51, 64].contains(&t) {
            let i = managed.committed_update_info()?;
            let owners = managed
                .world()
                .resource::<TrajectoryProcessPool>()
                .material_owners();
            let g = moves % owners.len();
            trajectory_workers::select_material_group(
                &mut managed,
                i.epoch,
                i.iteration,
                g,
                1 - owners[g],
            )?;
            moves += 1;
        }
        let start = Instant::now();
        local.step_exact(1)?;
        let local_ns = start.elapsed().as_nanos();
        managed.step_exact(1)?;
        let r = managed
            .world()
            .resource::<TrajectoryProcessPool>()
            .last()
            .unwrap();
        canonical += r.successor_bytes as u64;
        payload += r.successor_payload_bytes as u64;
        packed_groups += managed
            .world()
            .resource::<TrajectoryProcessPool>()
            .last_material_steps()
            .iter()
            .filter(|s| s.successor_payload_bytes < s.successor_bytes)
            .count() as u64;
        let p = r.profile.ok_or("missing curved worker measurements")?;
        for (group, step) in managed
            .world()
            .resource::<TrajectoryProcessPool>()
            .last_material_steps()
            .iter()
            .enumerate()
        {
            if step.restarted == (victim == Some(group)) {
                continue;
            }
            if step.restarted && step.warm_timeout {
                unplanned.push(json!({"tick":t+1,"group":group,"warm_budget_us":step.warm_deadline.map(|d|d.as_micros()),"pid":step.pid}));
            } else {
                return Err(
                    format!("lost or extraneous recovery at tick {t}, group {group}").into(),
                );
            }
        }
        attempts += p.packing_attempts;
        let (h, _) = batch::digest(&local)?;
        if batch::digest(&managed)?.0 != h {
            return Err(format!("different public field at tick {t}").into());
        }
        d.update(h);
        stages.push([
            local_ns,
            r.wall.as_nanos(),
            p.child_solve.as_nanos(),
            p.child_snapshot.as_nanos(),
            p.capture.as_nanos(),
            p.install.as_nanos(),
        ]);
        for (i, (_, b)) in deformable::bodies(&local).enumerate() {
            if counts.len() <= i {
                counts.push(vec![0u32; b.last_boundary_impulses().len()]);
                loads.push(vec![0.; b.last_boundary_impulses().len()]);
            }
            for (j, imp) in b.last_boundary_impulses().iter().enumerate() {
                if imp.linear != [0.; 3] {
                    counts[i][j] += 1;
                    loads[i][j] -= imp.linear[2];
                }
            }
        }
    }
    let q = (0..6)
        .map(|i| {
            let mut v = stages.iter().map(|r| r[i]).collect::<Vec<_>>();
            v.sort_unstable();
            [50, 95, 99].map(|f| v[(v.len() * f).div_ceil(100) - 1])
        })
        .collect::<Vec<_>>();
    println!(
        "{}",
        json!({"ticks":ticks,"worker_processes":managed.world().resource::<TrajectoryProcessPool>().material_owners().len(),
        "material_counts":deformable::bodies(&local).map(|(_,b)|b.particles().len()).collect::<Vec<_>>(),"surfaces":counts[0].len(),
        "packing_attempts":attempts,"canonical_bytes":canonical,"successor_payload_bytes":payload,"compressed_group_replies":packed_groups,"migrations":moves,"worker_sigkills":kills,"unplanned_budget_retries":unplanned,"stage_order":["ordinary","managed","child_solve","child_snapshot","capture","install"],"p50_p95_p99_ns":q,
        "loaded_ticks_per_boundary":counts,"vertical_impulse_ns_per_boundary":loads,"all_public_sha256":format!("{:x}",d.finalize())})
    );
    Ok(())
}
