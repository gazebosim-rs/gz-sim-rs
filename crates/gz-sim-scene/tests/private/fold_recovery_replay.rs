// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Recompute signed margins using the archived node spheres and the two oriented jaw boxes.
fn native_clearances(root: &Path, start: u64, dt: u64, margins: &[[f64; 2]]) -> Result<usize> {
    let mut verified = 0;
    let mut last = 0;
    let frames = fs::read_to_string(root.join("replay/frames.jsonl"))?;
    for line in frames.lines() {
        let frame: Value = serde_json::from_str(line)?;
        if frame["kind"] != "frame" {
            continue;
        }
        let scene: Value = serde_json::from_str(frame["scene"].as_str().unwrap())?;
        let time = uint(&scene["seconds"]) * 1_000_000_000 + uint(&scene["nanoseconds"]);
        if time <= start {
            continue;
        }
        let visual = scene["visuals"].as_array().unwrap();
        let mut nodes = Vec::new();
        for v in visual {
            if v["name"]
                .as_str()
                .unwrap()
                .starts_with("default::source::body::sheet_")
            {
                nodes.push((
                    DVec3::from_array(serde_json::from_value(v["translation"].clone())?),
                    scalar(&v["shape"]["radius"]),
                ));
            }
        }
        assert!(!nodes.is_empty());
        let tick = usize::try_from((time - start) / dt)?;
        assert_eq!((time - start) % dt, 0);
        for (side, label) in ["left", "right"].into_iter().enumerate() {
            let name = format!("default::arm::{label}::{label}");
            let jaw = visual.iter().find(|v| v["name"] == name).unwrap();
            let half = scalar(&jaw["shape"]["size"][1]) / 2.;
            let center = DVec3::from_array(serde_json::from_value(jaw["translation"].clone())?);
            let rotation = DQuat::from_array(serde_json::from_value(jaw["rotation"].clone())?);
            let mut forward = f64::INFINITY;
            let mut backward = f64::INFINITY;
            for &(p, radius) in &nodes {
                let y = (rotation.conjugate() * (p - center)).y;
                forward = forward.min(y - radius - half);
                backward = backward.min(-y - radius - half);
            }
            near(margins[tick - 1][side], forward.max(backward));
            verified += 1;
        }
        last = tick;
    }
    assert_eq!(last, margins.len());
    Ok(verified)
}

/// Audit the guard and control against their native jaw geometry and the same force input.
#[test]
#[ignore = "requires saved fold-recovery replay archives"]
fn guarded_and_unguarded_native_traces_match_the_measured_timeline() -> Result<()> {
    let input = env::var_os("GZ_FOLD_RECOVERY").ok_or("set GZ_FOLD_RECOVERY")?;
    let root = Path::new(&input);
    let spec = read(&root.join("comparison.json"))?;
    let dt = uint(&spec["dt_ns"]);
    let horizon = uint(&spec["horizon_ticks"]) as usize;
    let mut results = vec![];
    for case in spec["cases"].as_array().ok_or("missing guard cases")? {
        let cols = uint(&case["columns"]);
        let load = uint(&case["peak_mn"]);
        let start = uint(&case["baseline_ns"]);
        let mut branches = vec![];
        for role in ["guard", "control"] {
            let name = format!("mesh-{cols}-{load}-{role}");
            let run = inspect(root, &name, cols, start)?;
            let post = &run.trace[run.baseline + 1..];
            let observed = post.len();
            let escaped = case[role]["escaped_after_tick"].as_u64();
            assert_eq!(case[role]["observed_postgrip_ticks"], observed);
            assert_eq!(observed as u64, escaped.unwrap_or(horizon as u64));
            assert!((20..=horizon).contains(&observed));
            let margins: Vec<[f64; 2]> =
                serde_json::from_slice(&fs::read(root.join(&name).join("escape-clearance.json"))?)?;
            assert_eq!(margins.len(), observed);
            let native_margin_checks = native_clearances(&root.join(&name), start, dt, &margins)?;
            let mut escape_count = 0;
            let mut first_escape = None;
            let baseline = &run.trace[run.baseline];
            let mut peak = [0_f64; 2];
            let mut mean400 = [0.; 2];
            let mut mean_final = [0.; 2];
            let mut first_slip = None;
            let mut bilateral = 0;
            let mut bench = 0;
            let mut lost = None;
            let mut bench_hold = 0;
            let mut support_at_open = None;
            for (i, t) in post.iter().enumerate() {
                let tick = i + 1;
                assert_eq!(uint(&t["time_ns"]), start + tick as u64 * dt);
                let force = scalar(&t["bench_up_n"]);
                let weight = scalar(&t["mass_kg"]) * 9.81;
                let supported = force >= 0.8 * weight;
                bench_hold = if supported { bench_hold + 1 } else { 0 };
                if case[role]["release_after_tick"].as_u64() == Some(tick as u64) {
                    support_at_open = Some(bench_hold);
                }
                if scalar(&t["min_bench_clearance_m"]) > 0.01
                    && t["jaw_squeeze_n"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .all(|v| scalar(v) < 0.05)
                {
                    lost.get_or_insert(tick);
                }
                let clear = margins[i].iter().all(|v| *v > 0.)
                    && scalar(&t["min_bench_clearance_m"]) > 0.01
                    && t["jaw_squeeze_n"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .all(|v| scalar(v) < 1e-9);
                escape_count = if clear { escape_count + 1 } else { 0 };
                if escape_count == 4 {
                    first_escape.get_or_insert(tick as u64);
                }
                for j in 0..2 {
                    let travel =
                        scalar(&t["pad_frame_y_m"][j]) - scalar(&baseline["pad_frame_y_m"][j]);
                    if travel.abs() > 0.00025 {
                        first_slip.get_or_insert(tick);
                    }
                    peak[j] = peak[j].max(travel.abs());
                    if (381..=400).contains(&tick) {
                        mean400[j] += travel / 20.;
                    }
                    if tick > observed - 20 {
                        mean_final[j] += travel / 20.;
                    }
                }
                if tick > observed - 20 {
                    let jaws = [scalar(&t["jaw_up_n"][0]), scalar(&t["jaw_up_n"][1])];
                    bilateral += usize::from(
                        force.abs() < 1e-6
                            && jaws.iter().all(|n| *n > 0.)
                            && ((jaws[0] + jaws[1]) / weight - 1.).abs() < 0.025
                            && scalar(&t["min_bench_clearance_m"]) > 0.01,
                    );
                    bench += usize::from(supported);
                }
            }
            if let Some(ticks) = support_at_open {
                assert!(ticks as u64 * dt >= 20_000_000);
            }
            for (key, expected) in [
                ("peak_patch_motion_m", peak),
                ("final_patch_motion_m", mean_final),
            ] {
                for (j, v) in expected.iter().enumerate() {
                    near(*v, scalar(&case[role][key][j]));
                }
            }
            if observed < 400 {
                assert!(case[role]["patch_motion_at_400ms_m"].is_null());
            } else {
                for (j, v) in mean400.into_iter().enumerate() {
                    near(v, scalar(&case[role]["patch_motion_at_400ms_m"][j]));
                }
            }
            assert_eq!(first_escape, escaped);
            if escaped.is_some() {
                assert_eq!(case[role]["outcome"], "lost_aloft");
            }
            assert_eq!(case[role]["lost_aloft_tick"], json!(lost));
            assert_eq!(case[role]["first_250um_tick"], json!(first_slip));
            assert_eq!(case[role]["final_bilateral_ticks"], bilateral);
            assert_eq!(case[role]["final_bench_supported_ticks"], bench);
            results.push(json!({
                "archive": name,
                "observations": run.summary,
                "observed_postgrip_ticks": observed,
                "first_complete_escape_tick": first_escape,
                "independent_jaw_margin_checks": native_margin_checks,
                "terminal_jaw_margins_m": margins.last(),
                "first_250um_tick": first_slip,
                "first_loss_in_air_tick": lost,
                "verified_bench_support_ms_at_open": support_at_open.map(|t| t as f64 * dt as f64 * 1e-6),
                "final_bilateral_ticks": bilateral,
                "final_bench_ticks": bench,
            }));
            branches.push(run);
        }
        let (g, c) = (&branches[0], &branches[1]);
        assert_eq!(g.baseline, c.baseline);
        assert_eq!(g.prefix, c.prefix);
        assert_eq!(g.trace[..=g.baseline], c.trace[..=c.baseline]);
        let matched = g
            .trace
            .iter()
            .zip(&c.trace)
            .take_while(|(a, b)| a == b)
            .count()
            - g.baseline
            - 1;
        assert_eq!(case["matching_postgrip_ticks"], matched);
        assert!(
            g.trace
                .iter()
                .zip(&c.trace)
                .all(|(a, b)| a["force_y_n"] == b["force_y_n"])
        );
    }
    if let Some(path) = env::var_os("GZ_FOLD_RECOVERY_AUDIT") {
        fs::write(path, serde_json::to_vec_pretty(&results)?)?;
    }
    Ok(())
}
