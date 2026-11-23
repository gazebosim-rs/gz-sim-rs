// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/rotating_pad.rs"]
mod rotor;
use glam::DVec3;
use rotor::Rotor;
use std::error::Error;
#[allow(dead_code)]
#[path = "support/folded.rs"]
mod fold;
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let mu = args.first().map_or(Ok(0.), |s| s.parse::<f64>())?;
    let width = args.get(1).map_or(Ok(250000), |s| s.parse::<u64>())?;
    let steps = args.get(2).map_or(Ok(600), |s| s.parse::<u64>())?;
    let src = fold::sdf(mu, width, args.get(3).is_some_and(|s| s == "spin"));
    let mut g = Rotor::new(&src)?;
    let mut simultaneous = 0;
    for t in 1..=steps {
        g.advance(1)?;
        let mut normal = [0.; 2];
        let mut traction = [DVec3::ZERO; 2];
        let mut single = 0;
        let mut features = Vec::new();
        for r in g.material().support_this_step() {
            let count = r.source_features.len();
            if count == 2 {
                simultaneous += 1;
            } else {
                single += 1;
            }
            features = r.source_features.clone();
            for a in r.beginning.iter().chain(&r.end) {
                for l in &a.material {
                    let j = -DVec3::from(l.on_rigid);
                    let n = DVec3::from(l.site.normal);
                    let i = usize::from(n.y > 0.);
                    normal[i] += j.dot(n);
                    traction[i] += j - j.dot(n) * n;
                }
            }
        }
        if t <= 3 || t % 50 == 0 || t == steps {
            println!(
                "t {t} ms, double intervals {simultaneous}, current singles {single}, last {features:?}, per-face N {:?}, tangential {:?} N, spin {:.9e} rad/s, discrete hits {}, particles {:?}",
                normal.map(|n| 1000. * n),
                traction.map(|v| 1000. * v),
                g.state().angular_velocity[2],
                g.material().articulated_impact_count(),
                g.material()
                    .particles()
                    .iter()
                    .map(|p| (p.position[1], p.position[2]))
                    .collect::<Vec<_>>()
            );
        }
    }
    Ok(())
}
