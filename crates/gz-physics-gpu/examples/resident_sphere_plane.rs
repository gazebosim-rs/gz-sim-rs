// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare exact resident 1 ms GPU ticks with dense device observations, a directional CPU solve
//! and refined native f64 Rapier trajectories, then measure dense and block throughput.
//! cargo run --release -p gz-physics-gpu --example resident_sphere_plane -- --adapter NAME --out NEW_DIR
#[path = "support/sphere_plane/reference.rs"]
pub mod reference;
#[path = "support/sphere_plane/block_summary.rs"]
mod summary;
#[path = "support/sphere_plane/block_timing.rs"]
mod timing;
use gz_physics_gpu::{
    MAX_BLOCK_TICKS, SpherePlaneBatch, SpherePlaneBlockBatch, SpherePlaneLane as Lane,
};
use reference::{Native, Q, V, analytical, fixture};
use std::{error::Error, path::PathBuf};
use summary::Summary;

#[derive(Default, Debug)]
struct Delta {
    position: f64,
    velocity: f64,
    angular: f64,
    orientation: f64,
    impulse: f64,
}
impl Delta {
    fn update(&mut self, a: &Lane, b: &Lane) {
        assert!(
            bytemuck::cast_slice::<Lane, f64>(std::slice::from_ref(b))
                .iter()
                .all(|x| x.is_finite())
        );
        self.position = self
            .position
            .max(V::from(a.position).distance(V::from(b.position)));
        self.velocity = self
            .velocity
            .max(V::from(a.velocity).distance(V::from(b.velocity)));
        self.angular = self
            .angular
            .max(V::from(a.angular_velocity).distance(V::from(b.angular_velocity)));
        let (x, y) = (Q::from_array(a.orientation), Q::from_array(b.orientation));
        self.orientation = self.orientation.max((x - y).length().min((x + y).length()));
        self.impulse = self
            .impulse
            .max((a.normal_impulse - b.normal_impulse).abs())
            .max(V::from(a.tangential_impulse).distance(V::from(b.tangential_impulse)));
    }
}
fn near_impact(tick: usize, n: &Native) -> bool {
    match (n.gpu_first, n.native_first) {
        (Some(a), Some(b)) => (a.min(b).saturating_sub(1)..=a.max(b) + 4).contains(&tick),
        (Some(a), None) | (None, Some(a)) => tick >= a.saturating_sub(1),
        _ => false,
    }
}
fn qualify(adapter: &str) -> Result<(), Box<dyn Error>> {
    const N: usize = 2048;
    let initial = fixture(N);
    let mut cpu = initial.clone();
    let mut gpu = SpherePlaneBlockBatch::new(adapter, initial.clone())?;
    let mut dense = SpherePlaneBatch::new(adapter, initial.clone())?;
    let info = gpu.device_info();
    println!(
        "{} {} {} | {} lanes, {} resident bytes, {} workgroups, timestamps {}",
        info.name,
        info.backend,
        info.driver_info,
        N,
        info.state_bytes,
        info.workgroups,
        info.timestamp_queries
    );
    let mut native: Vec<_> = (0..16)
        .map(|i| {
            let index = i * 93;
            (index, Native::new(initial[index]))
        })
        .collect();
    let mut oracle = Delta::default();
    let mut native_raw = Delta::default();
    let mut native_aligned = Delta::default();
    let (mut max_cpu_block, mut max_native_raw, mut max_native_aligned) = (0f64, 0f64, 0f64);
    let mut impact_offset = 0;
    let mut sensitive = 0;
    let mut aligned = 0;
    let mut tick = 0;
    // Short blocks deliberately cut across free flight, impact and established
    // friction; the final block exercises the full admitted device loop length.
    let blocks = [
        1u32,
        4,
        16,
        11,
        23,
        7,
        9,
        5,
        19,
        3,
        13,
        21,
        17,
        1,
        31,
        93,
        203,
        MAX_BLOCK_TICKS,
    ];
    for k in blocks {
        let start = tick;
        let mut dense_sums: Vec<_> = dense.lanes().iter().map(Summary::initial).collect();
        let mut cpu_sums: Vec<_> = cpu.iter().map(Summary::initial).collect();
        let mut native_sums: Vec<_> = native
            .iter()
            .map(|(_, v)| Summary::initial(&v.state))
            .collect();
        for offset in 1..=k {
            tick += 1;
            dense.step()?;
            for (((observed, ds), scalar), cs) in dense
                .lanes()
                .iter()
                .zip(&mut dense_sums)
                .zip(&mut cpu)
                .zip(&mut cpu_sums)
            {
                ds.add(observed, offset);
                analytical(scalar);
                cs.add(scalar, offset);
            }
            for ((i, n), sum) in native.iter_mut().zip(&mut native_sums) {
                n.step();
                sum.add(&n.state, offset);
                if dense.lanes()[*i].normal_impulse > 1e-5 {
                    n.gpu_first.get_or_insert(tick);
                }
                if n.state.normal_impulse > 1e-5 {
                    n.native_first.get_or_insert(tick);
                }
            }
        }
        let metrics = gpu.step_block(k)?;
        assert_eq!(gpu.tick(), tick as u64);
        assert_eq!(
            metrics.readback_bytes,
            gpu.device_info().state_bytes
                + if gpu.device_info().timestamp_queries {
                    if metrics.gpu_copy.is_some() { 32 } else { 16 }
                } else {
                    0
                }
        );
        for (i, r) in gpu.lanes().iter().enumerate() {
            assert_eq!(
                bytemuck::bytes_of(&r.state),
                bytemuck::bytes_of(&dense.lanes()[i]),
                "resident/dense bytes lane {i}, tick {tick}"
            );
            dense_sums[i].check_device(r);
            oracle.update(&r.state, &cpu[i]);
            max_cpu_block = max_cpu_block.max(cpu_sums[i].distance(r));
        }
        for ((i, n), sum) in native.iter().zip(&native_sums) {
            let measured = &gpu.lanes()[*i];
            native_raw.update(&measured.state, &n.state);
            let jump_speed = sum.distance(measured) * measured.state.inverse_mass;
            max_native_raw = max_native_raw.max(jump_speed);
            if near_impact(start, n) || near_impact(tick, n) {
                sensitive += 1;
            } else {
                aligned += 1;
                native_aligned.update(&measured.state, &n.state);
                max_native_aligned = max_native_aligned.max(jump_speed);
            }
            if let (Some(a), Some(b)) = (n.gpu_first, n.native_first) {
                impact_offset = impact_offset.max(a.abs_diff(b));
            }
        }
    }
    println!(
        "bitwise dense GPU endpoint/exact impulse and discrete summaries: {N} lanes, {} varied block boundaries, {tick} physical ticks",
        blocks.len()
    );
    println!(
        "independent CPU endpoint maxima {oracle:?}; max integrated normal/vector tangential impulse error {max_cpu_block:.3e} N s"
    );
    println!(
        "native f64 Rapier 8x {native_raw:?}; synchronized endpoints {native_aligned:?}; {sensitive} impact-adjacent vs {aligned} outside intervals; raw/aligned integrated impulse / mass {:.5}/{:.5} m/s, impact offset {impact_offset} ticks",
        max_native_raw, max_native_aligned
    );
    assert!(
        oracle.position < 2e-8
            && oracle.velocity < 2e-8
            && oracle.angular < 2e-7
            && oracle.orientation < 2e-8
            && oracle.impulse < 2e-8
            && max_cpu_block < 2e-8
    );
    assert!(
        native
            .iter()
            .all(|(_, v)| v.gpu_first.is_some() && v.native_first.is_some())
    );
    // At 1 ms the gravity drift difference from a continuous ballistic
    // trajectory is <= |g| * h * elapsed / 2, plus the observed impact envelope.
    assert!(
        native_raw.position < 0.003 + 11. * 0.001 * (tick as f64 * 0.001) / 2.
            && impact_offset <= 2
    );
    assert!(
        native_aligned.velocity < 0.065
            && native_aligned.orientation < 0.08
            && max_native_aligned < 2. * 0.065
    );
    assert!(sensitive > 0 && aligned > sensitive);
    gpu.reset();
    let _ = gpu.step_block(1)?;
    let mut original = SpherePlaneBatch::new(adapter, initial)?;
    original.step()?;
    for (a, b) in gpu.lanes().iter().zip(original.lanes()) {
        assert_eq!(bytemuck::bytes_of(&a.state), bytemuck::bytes_of(b));
    }
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let mut adapter = None;
    let mut out = None;
    while let Some(k) = args.next() {
        let x = args.next().ok_or("missing option value")?;
        match k.as_str() {
            "--adapter" if adapter.is_none() => adapter = Some(x),
            "--out" if out.is_none() => out = Some(PathBuf::from(x)),
            _ => return Err("expected --adapter NAME --out NEW_DIRECTORY".into()),
        }
    }
    let name = adapter.ok_or("missing --adapter NAME")?;
    let out = out.ok_or("missing --out NEW_DIRECTORY")?;
    std::fs::create_dir(&out)?;
    qualify(&name)?;
    let measured = timing::run(&name)?;
    timing::write(&out, &measured)?;
    Ok(())
}
