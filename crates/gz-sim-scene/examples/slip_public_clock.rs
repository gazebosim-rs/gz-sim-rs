// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/inclined_sheet.rs"]
mod sheet;
#[allow(dead_code)]
#[path = "support/slip_transition.rs"]
mod transition;
use glam::DVec3;
use sheet::Incline;
use std::{error::Error, time::Duration};
use transition::Run;
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let cap = args.first().map_or(Ok(2000), |s| s.parse())?;
    let mass = args.get(1).map_or(Ok(0.), |s| s.parse())?;
    let s = Run {
        speed: -0.42,
        mu: 0.15,
        width: 250000,
        transition: (cap > 0).then_some(cap),
        yaw: 0.,
        extra_mass: mass,
    }
    .sdf();
    let mut reference = None::<Vec<(DVec3, DVec3, DVec3)>>;
    let mut ref_cuts = None::<Vec<(u64, u64)>>;
    for width in [1000000, 250000, 500000, 2000000, 4000000] {
        let xml = s.replace(
            "<max_step_size>0.001</max_step_size>",
            &format!("<max_step_size>{}</max_step_size>", width as f64 * 1e-9),
        );
        let mut g = Incline::new(&xml)?;
        let initial = g.centroid().0;
        let mut receipts = 0;
        let mut cuts = vec![];
        let mut triggers = 0;
        let mut samples = vec![];
        let mut on_native = DVec3::ZERO;
        while g.sim.elapsed() < Duration::from_millis(300) {
            g.step()?;
            for r in g.material().support_this_step() {
                cuts.push((r.start.as_nanos() as u64, r.duration.as_nanos() as u64));
                triggers += u64::from(r.slip_transition_refinements);
                for a in r.beginning.iter().chain(&r.end) {
                    for l in &a.material {
                        receipts += 1;
                        on_native += DVec3::from(l.on_rigid);
                    }
                }
            }
            if g.sim.elapsed().as_nanos().is_multiple_of(4_000_000) {
                let (x, v, _) = g.centroid();
                samples.push((x - initial, v, on_native));
            }
        }
        let other = reference.get_or_insert_with(|| samples.clone());
        let mut dx = 0f64;
        let mut dv = 0f64;
        let mut dj = 0f64;
        for ((x, v, j), (r, u, k)) in samples.iter().zip(other.iter()) {
            dx = dx.max(x.distance(*r));
            dv = dv.max(v.distance(*u));
            dj = dj.max(j.distance(*k));
        }
        let base = ref_cuts.get_or_insert_with(|| cuts.clone());
        let first = cuts.iter().zip(base.iter()).position(|(a, b)| a != b);
        println!(
            "public_ns={width} transition_ns={cap} extra_mass={mass} cuts={} receipts={receipts} rejected={triggers} common_samples={} first_unequal_cut={first:?} equal_cut_history={} max_CM_position_diff_m={dx:e} max_CM_velocity_diff_m_s={dv:e} max_native_impulse_diff_Ns={dj:e}",
            cuts.len(),
            samples.len(),
            cuts == *base
        );
    }
    Ok(())
}
