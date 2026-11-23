// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare measured floor contact at a driven articulated arm's round tool.
#[path = "support/planar_arm/capture.rs"]
mod capture;
#[allow(dead_code)]
#[path = "support/planar_arm/native.rs"]
mod native;
#[path = "support/planar_tool/native.rs"]
mod native_tool;
#[allow(dead_code)]
#[path = "support/planar_arm/reference.rs"]
mod reference;
#[path = "support/planar_tool/reference.rs"]
mod tool_reference;
use gz_physics_gpu::{PlanarToolBatch, simulation::planar_tool_simulation};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{error::Error, fs, io, path::Path, time::Instant};
use tool_reference::{Observation, fixture, lift};
const LANES: usize = 2048;
const LIFT: usize = 301;
const TICKS: usize = 600;
#[derive(Default)]
struct Track {
    onset: Option<f64>,
    last: Option<f64>,
    first_n: f64,
    first_t: f64,
    first_tick: Option<usize>,
    first_public_n: f64,
    window_n: f64,
    window_t: f64,
    n: f64,
    t: f64,
    prelift_load: f64,
    final_load: f64,
    min_gap: f64,
}
impl Track {
    fn observe(&mut self, step: usize, o: Observation, gap: f64) {
        let start = (step - 1) as f64 * 0.001;
        if let Some(offset) = o.first {
            assert!(o.n > 0.);
            if self.onset.is_none() {
                self.first_n = o.first_n;
                self.first_t = o.first_t;
                self.first_tick = Some(step);
                self.first_public_n = o.n;
            }
            self.onset.get_or_insert(start + offset);
            self.last = Some(start + 0.001);
        }
        // Sum ten public ticks including each solver's own first positively loaded tick.
        if self.first_tick.is_some_and(|first| step - first < 10) {
            self.window_n += o.n;
            self.window_t += o.t;
        }
        self.n += o.n;
        self.t += o.t;
        self.min_gap = self.min_gap.min(gap);
        if (LIFT - 20..LIFT).contains(&step) {
            self.prelift_load += o.n;
        }
        if step > TICKS - 20 {
            self.final_load += o.n;
        }
    }
    fn result(&self) -> serde_json::Value {
        json!({"onset_s":self.onset,"last_loaded_tick_end_s":self.last,"first_loaded_public_tick_normal_ns":self.first_public_n,"first_impact_normal_ns":self.first_n,"first_impact_tangent_ns":self.first_t,
        "first_10_ticks_normal_ns":self.window_n,"first_10_ticks_tangent_ns":self.window_t,"total_normal_ns":self.n,"total_tangent_ns":self.t,
        "prelift_20ms_normal_ns":self.prelift_load,"final_20ms_normal_ns":self.final_load,"minimum_gap_m":self.min_gap})
    }
}
fn command_lift(batch: &mut PlanarToolBatch) -> Result<(), gz_physics_gpu::GpuError> {
    for i in 0..batch.lanes().len() {
        batch.set_target_angles(i, lift(i))?;
    }
    Ok(())
}
fn compare(g: &Track, r: &Track) -> [f64; 4] {
    [
        (g.onset.unwrap() - r.onset.unwrap()).abs(),
        (g.first_n / r.first_n - 1.).abs(),
        (g.window_n / r.window_n - 1.).abs(),
        (g.last.unwrap() - r.last.unwrap()).abs(),
    ]
}
fn run(adapter: &str, out: &Path) -> Result<(), Box<dyn Error>> {
    if out.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, out.display().to_string()).into());
    }
    let mut lanes = (0..LANES).map(fixture).collect::<Vec<_>>();
    lanes[LANES - 1].ground_z -= 2.; // No-contact control under the same drive and gravity.
    let mut sim = planar_tool_simulation(adapter, lanes.clone())?;
    let ids = (0..9).collect::<Vec<_>>();
    let mut refs = ids
        .iter()
        .map(|&i| reference::State::new(&lanes[i].arm))
        .collect::<Vec<_>>();
    let mut native = ids
        .iter()
        .map(|&i| native_tool::NativeTool::new(&lanes[i]))
        .collect::<Vec<_>>();
    let mut trace = vec![];
    let mut capture = capture::Capture::default();
    capture.sample(0, &lanes);
    let mut tracks = ids
        .iter()
        .map(|_| [Track::default(), Track::default(), Track::default()])
        .collect::<Vec<_>>();
    let mut max_error = vec![[0_f64; 2]; ids.len()];
    let mut trajectory = Sha256::new();
    let mut sham_gap = f64::INFINITY;
    for step in 1..=TICKS {
        if step == LIFT {
            let mut batch = sim.world_mut().resource_mut::<PlanarToolBatch>();
            command_lift(&mut batch)?;
            for (i, l) in lanes.iter_mut().enumerate() {
                l.arm.target = lift(i).map(reference::turn);
            }
        }
        sim.step_exact(1)?;
        let batch = sim.world().resource::<PlanarToolBatch>();
        let all = batch.lanes();
        trajectory.update(bytemuck::cast_slice::<_, u8>(all));
        let sham = &all[LANES - 1];
        sham_gap = sham_gap.min(sham.gap());
        if sham.normal_impulse != 0.
            || sham.tangent_impulse != 0.
            || sham.first_contact_offset != -1.
        {
            return Err("lowered floor received an impulse".into());
        }
        for l in all {
            if !(l.normal_impulse >= 0.
                && l.tangent_impulse.abs() <= l.friction * l.normal_impulse + 1e-12)
            {
                return Err("invalid impulse".into());
            }
        }
        capture.sample((step * 1_000_000) as u64, all);
        let mut cases = vec![];
        for (j, &i) in ids.iter().enumerate() {
            let l = &all[i];
            let gpu = Observation {
                n: l.normal_impulse,
                first_n: l.first_normal_impulse,
                first_t: l.first_tangent_impulse,
                t: l.tangent_impulse,
                first: (l.first_contact_offset >= 0.).then_some(l.first_contact_offset),
                tip: l.arm.points()[1],
            };
            let fine = tool_reference::fine_step(&mut refs[j], &lanes[i]);
            let rapier = native[j].step(&lanes[i]);
            for (v, t) in [gpu, fine, rapier].into_iter().zip(&mut tracks[j]) {
                if !v.tip.into_iter().chain([v.n, v.t]).all(f64::is_finite) {
                    return Err("nonfinite reference".into());
                }
                t.observe(step, v, v.tip[1] - l.radius - l.ground_z);
            }
            for (err, o) in max_error[j].iter_mut().zip([fine, rapier]) {
                *err = err.max((gpu.tip[0] - o.tip[0]).hypot(gpu.tip[1] - o.tip[1]));
            }
            let row = |o: Observation| json!([o.tip[0], o.tip[1], o.n, o.t, o.first]);
            cases.push(
                json!({"lane_id":i,"gpu":row(gpu),"cartesian":row(fine),"rapier":row(rapier)}),
            );
        }
        trace.push(json!({"tick":step,"cases":cases}));
    }
    let digest = trajectory.finalize();
    sim.reset()?;
    let mut repeat = Sha256::new();
    let mut timing = vec![];
    let mut dispatch = vec![];
    for tick in 1..=TICKS {
        let start = Instant::now();
        if tick == LIFT {
            command_lift(&mut sim.world_mut().resource_mut::<PlanarToolBatch>())?;
        }
        sim.step_exact(1)?;
        timing.push(start.elapsed().as_nanos());
        if let Some(t) = sim
            .world()
            .resource::<gz_physics_gpu::simulation::BatchTiming>()
            .last
            .as_ref()
            .unwrap()
            .gpu_dispatch
        {
            dispatch.push(t.as_nanos());
        }
        repeat.update(bytemuck::cast_slice::<_, u8>(
            sim.world().resource::<PlanarToolBatch>().lanes(),
        ));
    }
    if digest != repeat.finalize() {
        return Err("reset trajectory changed".into());
    }
    timing.sort_unstable();
    dispatch.sort_unstable();
    if !tracks
        .iter()
        .flatten()
        .all(|t| t.onset.is_some() && t.prelift_load > 0. && t.final_load == 0.)
    {
        return Err("support and release did not qualify".into());
    }
    let mut worst = [[0f64; 4]; 2];
    for [gpu, fine, native] in &tracks {
        for (row, other) in worst.iter_mut().zip([fine, native]) {
            for (m, e) in row.iter_mut().zip(compare(gpu, other)) {
                *m = m.max(e);
            }
        }
    }
    let cases=ids.iter().enumerate().map(|(j,&i)|{
        let [gpu,fine,rapier]=&tracks[j];
        json!({"lane_id":i,"payload_kg":lanes[i].arm.tip_mass,"friction":lanes[i].friction,"gpu":gpu.result(),
        "cartesian":fine.result(),"rapier":rapier.result(),"max_tip_error_m":max_error[j]})
    }).collect::<Vec<_>>();
    let result = json!({"adapter":sim.world().resource::<PlanarToolBatch>().device_info().name,"lanes":LANES,"ticks":TICKS,
        "all_ticks_reset_sha256":format!("{digest:x}"),"lowered_floor_minimum_gap_m":sham_gap,"lowered_floor_lane":LANES-1,
        "max_discrepancy_order":["onset_s","first_normal_fraction","first_10_ticks_normal_fraction","release_s"],"worst_cartesian":worst[0],"worst_rapier":worst[1],
        "gpu_dispatch_p95_ns":(!dispatch.is_empty()).then(||dispatch[(dispatch.len()*95).div_ceil(100)-1] as u64),"lift_before_tick":LIFT,"gpu_internal_dt_s":0.0005,"reference_dt_s":0.0000625,"cases":cases,"dense_bevy_p95_ns":timing[(TICKS*95).div_ceil(100)-1] as u64});
    println!("{}", serde_json::to_string_pretty(&result)?);
    let bounds = [
        [0.000010, 0.015, 0.020, 0.001001],
        [0.00025, 0.10, 0.020, 0.001001],
    ];
    if !worst
        .iter()
        .zip(bounds)
        .all(|(values, limits)| values.iter().zip(limits).all(|(v, limit)| *v <= limit))
        || max_error.iter().flatten().any(|e| *e > 0.001)
    {
        return Err("trajectory or contact regression".into());
    }
    let parent = out
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::tempdir_in(parent)?;
    capture.write(stage.path())?;
    fs::write(stage.path().join("trace.json"), serde_json::to_vec(&trace)?)?;
    fs::write(
        stage.path().join("comparison.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    fs::rename(stage.path(), out)?;
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let [name, out] = args.as_slice() else {
        return Err("expected ADAPTER NEW_OUTPUT_DIRECTORY".into());
    };
    run(name, Path::new(out))
}
