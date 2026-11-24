// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the production resident GPU process path against an uninterrupted local device.
#[allow(dead_code)]
#[path = "support/sphere_pair/reference.rs"]
mod reference;
use gz_physics_gpu::{
    SpherePairBlockBatch as Batch, SpherePairBlockCheckpoint as Cut, SphereRotation,
    pair_process::{PairProcessPool, PairReplyFormat},
};
use serde_json::json;
use std::{error::Error, path::PathBuf, time::Instant};
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("adapter")?;
    let path = PathBuf::from(args.next().ok_or("worker path")?);
    let n = args.next().ok_or("lanes")?.parse::<usize>()?;
    let width = args.next().ok_or("workers")?.parse::<usize>()?;
    let count = args
        .next()
        .ok_or("observation intervals")?
        .parse::<usize>()?;
    let seed = args.next().ok_or("seed")?.parse::<u64>()?;
    let encoding = args.next().ok_or("portable or observations")?;
    let format = match encoding.as_str() {
        "portable" => PairReplyFormat::Portable,
        "observations" => PairReplyFormat::Observations,
        _ => return Err("unknown reply format".into()),
    };
    if n < width || width == 0 || count < 2 || args.next().is_some() {
        return Err("invalid worker, population or interval count".into());
    }
    let mut lanes = reference::fixture(n);
    let mut rng = seed;
    for lane in &mut lanes {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        let u = (rng >> 11) as f64 / 9007199254740992.;
        for body in [&mut lane.a, &mut lane.b] {
            body.position[2] += u;
            body.velocity[1] += u - 0.5;
        }
    }
    let mut warm = Batch::with_rotation(&adapter, lanes.clone(), SphereRotation::FourthOrder)?;
    let mut control = Batch::with_rotation(&adapter, lanes, SphereRotation::FourthOrder)?;
    let mut pool = PairProcessPool::with_format(&path, &adapter, width, format)?;
    let mut times: [Vec<u64>; 8] = Default::default();
    let mut bytes = [0u64; 4];
    let mut warm_ranges = 0;
    let mut hit = 0;
    for i in 0..count {
        let ordinary = Instant::now();
        control.step_block(8)?;
        let local = ordinary.elapsed();
        let m = pool.advance(&mut warm, 8)?;
        let reference = Cut::capture(&control)?;
        if Cut::capture(&warm)?.to_bytes()? != reference.to_bytes()? {
            return Err(format!("different canonical bytes at {} ms", warm.tick()).into());
        }
        hit += reference
            .observations()
            .iter()
            .map(|r| r.contact_ticks() as u64)
            .sum::<u64>();
        if i > 0 {
            let residual = m
                .latency
                .saturating_sub(m.prepare + m.commit + m.wait + m.decode);
            for (t, v) in times.iter_mut().zip([
                m.latency,
                local,
                m.prepare,
                m.wait,
                m.decode,
                m.commit,
                residual,
                m.dispatch_sum,
            ]) {
                t.push(v.as_nanos() as u64);
            }
            for (t, b) in bytes.iter_mut().zip([
                m.request_bytes,
                m.response_bytes,
                m.upload_bytes,
                m.readback_bytes,
            ]) {
                *t += b;
            }
            warm_ranges += m.reused;
        }
    }
    let keys = [
        "process",
        "ordinary_device",
        "prepare",
        "wait",
        "decode",
        "commit",
        "remaining_coordinator",
        "dispatch_sum",
    ];
    let mut distributions = json!({});
    for (key, mut v) in keys.into_iter().zip(times) {
        v.sort_unstable();
        let quant = |p: usize| v[(v.len() * p).div_ceil(100) - 1];
        distributions[key] = json!({"p50_ns":quant(50usize),"p95_ns":quant(95),"p99_ns":quant(99),"sum_ns":v.iter().sum::<u64>()});
    }
    println!(
        "{}",
        json!({"lanes":n,"workers":width,"intervals":count-1,"seed":seed,"encoding":encoding,"request_bytes":bytes[0],"response_bytes":bytes[1],"upload_bytes":bytes[2],"readback_bytes":bytes[3],"reuses":warm_ranges,"loaded_lane_ticks":hit,"verified_cuts":count,"timing":distributions})
    );
    Ok(())
}
