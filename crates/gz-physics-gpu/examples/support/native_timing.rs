// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::Result;
use gz_physics_gpu::{MixedBatch, PairSolver::NativeRapier, simulation::BatchTiming};
use gz_sim_core::Simulation;
use std::{
    fs::File,
    io::{BufWriter, Write},
    time::Instant,
};
pub fn timing(sim: &mut Simulation, ids: &[u64], file: &mut BufWriter<File>) -> Result<()> {
    for rep in 0..3 {
        let order = match rep {
            0 => [0, 16, 48, 96],
            1 => [96, 48, 16, 0],
            _ => [48, 0, 96, 16],
        };
        for n in order {
            sim.reset()?;
            let start = Instant::now();
            let transfer = sim
                .world_mut()
                .resource_mut::<MixedBatch>()
                .set_pair_solver(0, &ids[..n], NativeRapier)?;
            let switch = start.elapsed();
            assert_eq!(transfer.moved, n);
            let mut samples = vec![];
            let mut bytes = 0;
            for t in 1..=210 {
                let start = Instant::now();
                sim.step_exact(1)?;
                let ns = start.elapsed().as_nanos();
                let d = sim.world().resource::<BatchTiming>().last.unwrap();
                bytes = d.readback_bytes;
                writeln!(
                    file,
                    "{rep},{n},{t},{},{ns},{},{},{},{}",
                    if t == 1 { switch.as_nanos() } else { 0 },
                    d.cpu_solve.as_nanos(),
                    d.gpu_dispatch.map_or(0, |v| v.as_nanos()),
                    d.uploaded_bytes,
                    bytes
                )?;
                if t > 30 {
                    samples.push(ns);
                }
            }
            samples.sort_unstable();
            println!(
                "repeat={rep} native={n} gpu={} transfer_us={:.2} p50_us={:.2} p99_us={:.2} read_B={bytes} env_steps_s={:.0}",
                2048 - n,
                switch.as_secs_f64() * 1e6,
                samples[89] as f64 * 1e-3,
                samples[178] as f64 * 1e-3,
                2048. * 180e9 / samples.iter().sum::<u128>() as f64
            );
        }
    }
    Ok(())
}
