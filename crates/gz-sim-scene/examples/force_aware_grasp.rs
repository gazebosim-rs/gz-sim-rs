// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/force_aware.rs"]
mod force;
#[allow(dead_code)]
#[path = "support/capsule_grasp.rs"]
mod grasp;
use force::*;
use std::error::Error;
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let value = |key: &str| args.iter().find_map(|arg| arg.strip_prefix(key));
    let requested_ticks = value("--ticks=").map(str::parse::<usize>).transpose()?;
    let height = value("--height=").unwrap_or("0.0625").parse::<f64>()?;
    let ns: Vec<u32> = if args.iter().any(|arg| arg == "--sweep") {
        vec![1, 2, 4, 8, 16]
    } else {
        vec![value("--substeps=").unwrap_or("4").parse()?]
    };
    if let Some(choice) = args.iter().find(|arg| !arg.starts_with("--"))
        && !["constant", "tape", "balanced", "feedback"].contains(&choice.as_str())
    {
        return Err(format!("unknown fixture {choice}").into());
    }
    for n in ns {
        for case in [
            CarryInput::Balanced,
            CarryInput::Constant,
            CarryInput::IdenticalTape,
            CarryInput::Feedback,
        ] {
            let which = match case {
                CarryInput::Balanced => "balanced",
                CarryInput::Constant => "constant",
                CarryInput::IdenticalTape => "tape",
                CarryInput::Feedback => "feedback",
            };
            let chosen = args.iter().find(|arg| !arg.starts_with("--"));
            if chosen.is_some_and(|a| a != which)
                || (chosen.is_none()
                    && matches!(case, CarryInput::Constant | CarryInput::IdenticalTape))
            {
                continue;
            }
            let ticks = requested_ticks.unwrap_or(match case {
                CarryInput::Constant => 500,
                CarryInput::IdenticalTape => 250,
                _ => 1050,
            });
            if ticks == 0 || !height.is_finite() {
                return Err("ticks must be positive and jaw height finite".into());
            }
            let mut chord = pair_at(false, n, 0.001, case, height)?;
            let mut aware = pair_at(true, n, 0.001, case, height)?;
            let mut window = [Window::default(), Window::default()];
            let mut center = [glam::DVec3::ZERO; 2];
            let mut count = 0;
            let (mut valid, mut fallback, mut difference) = (0, 0, 0f64);
            for tick in 1..=ticks {
                let a = command(case, &chord, &chord);
                let b = command(case, &chord, &aware);
                for (l, r) in a.into_iter().zip(b) {
                    difference = difference.max(l.0.distance(r.0)).max(l.1.distance(r.1));
                }
                chord
                    .step_wrenches(a)
                    .map_err(|e| format!("{case:?} chord n={n}: {e}"))?;
                aware
                    .step_wrenches(b)
                    .map_err(|e| format!("{case:?} force-aware n={n}: {e}"))?;
                for sample in aware.cloth().capsule_forcing() {
                    if sample.used {
                        valid += 1
                    } else {
                        fallback += 1;
                    }
                }
                if (if ticks >= 690 {
                    570..=690
                } else {
                    ticks.saturating_sub(120).max(1)..=ticks
                })
                .contains(&tick)
                {
                    for (i, r) in [&chord, &aware].into_iter().enumerate() {
                        center[i] += r.center();
                        window[i].add(r.cloth(), 0.001, 2, grasp::SQUEEZE);
                    }
                    count += 1;
                }
            }
            for (i, w) in window.iter().enumerate() {
                println!(
                    "{case:?} n={n} aware={} accepted/fallback {valid}/{fallback}, max actual command difference {difference:e}, hold {} substep contacts: duty {:.5}%, mean normal {:.8} N, rms {:.8} N, peak {:.8} N; weighted slip {:e} m/s, max loaded slip {:e} m/s, support {:.8} N, mean COM {:?}",
                    i == 1,
                    w.count,
                    100. * w.duty / w.duration,
                    w.normal / w.duration,
                    w.rms_variation(),
                    w.max_force,
                    w.weighted_slip(),
                    w.max_slip,
                    2. * w.received / w.duration,
                    center[i] / count as f64
                );
            }
        }
    }
    Ok(())
}
