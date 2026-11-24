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
            let following = spec["follow_dropped"] == true;
            let first = escaped.and_then(|e| {
                let from = e as usize;
                post[from..]
                    .iter()
                    .position(|t| scalar(&t["bench_up_n"]) > 1e-9)
                    .map(|i| from + i + 1)
            });
            let expected = if spec["stop_at_first_landing"] == true {
                assert!(following);
                first.unwrap_or(horizon)
            } else if spec["stop_at_rest"] == true {
                let evidence = read(&root.join(&name).join("rest.json"))?;
                let rows: Vec<(usize, f64, f64)> =
                    serde_json::from_value(evidence["observations"].clone())?;
                let samples: Vec<Vec<[f64; 3]>> =
                    serde_json::from_value(evidence["quiet_particle_velocities_m_s"].clone())?;
                let mut quiet = 0;
                if let Some(first) = first {
                    assert_eq!(rows.first().unwrap().0, first);
                }
                for (tick, max_speed, weight_ratio) in &rows {
                    let t = &post[*tick - 1];
                    near(
                        *weight_ratio,
                        scalar(&t["bench_up_n"]) / (9.81 * scalar(&t["mass_kg"])),
                    );
                    let squeeze: [f64; 2] = serde_json::from_value(t["jaw_squeeze_n"].clone())?;
                    if *max_speed <= 0.001
                        && (*weight_ratio - 1.).abs() <= 0.025
                        && squeeze.iter().all(|n| n.abs() < 1e-9)
                    {
                        quiet += 1;
                    } else {
                        quiet = 0;
                    }
                    assert!(quiet < 50 || *tick == observed);
                }
                assert_eq!(quiet, samples.len());
                assert_eq!(evidence["consecutive_ticks"], quiet);
                for (v, row) in samples.iter().zip(&rows[rows.len() - quiet..]) {
                    assert_eq!(v.len(), (4 * cols) as usize);
                    let maximum = v
                        .iter()
                        .map(|v| DVec3::from(*v).length())
                        .fold(0., f64::max);
                    near(maximum, row.1);
                }
                if quiet == 50 {
                    rows.last().unwrap().0
                } else {
                    horizon
                }
            } else if following {
                horizon
            } else {
                escaped.map_or(horizon, |e| e as usize)
            };
            assert_eq!(observed, expected);
            let landing = if let Some(tick) = first.filter(|_| following) {
                Some(check_first_landing(
                    root,
                    &name,
                    &post[..tick],
                    escaped.unwrap() as usize,
                    dt,
                )?)
            } else {
                None
            };
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
            let slide = if following && first.is_some() && observed == horizon && observed >= 100 {
                let tail = &run.trace[run.trace.len() - 101..];
                let seconds = 100. * dt as f64 * 1e-9;
                let mass = scalar(&tail[0]["mass_kg"]);
                let normal = tail[1..]
                    .iter()
                    .map(|v| scalar(&v["bench_up_n"]))
                    .sum::<f64>()
                    / 100.;
                let final_y = scalar(&tail[100]["momentum_y_kg_m_s"]) / mass;
                let acceleration =
                    (final_y - scalar(&tail[0]["momentum_y_kg_m_s"]) / mass) / seconds;
                let supported = tail[1..]
                    .iter()
                    .filter(|v| scalar(&v["bench_up_n"]) > 1e-9)
                    .count();
                let residual = tail
                    .windows(2)
                    .map(|ab| {
                        let v = &ab[1];
                        let jaws: [f64; 2] =
                            serde_json::from_value(v["jaw_side_reaction_n"].clone()).unwrap();
                        (scalar(&v["momentum_y_kg_m_s"])
                            - scalar(&ab[0]["momentum_y_kg_m_s"])
                            - (scalar(&v["force_y_n"])
                                - scalar(&v["bench_side_reaction_n"])
                                - jaws.iter().sum::<f64>())
                                * dt as f64
                                * 1e-9)
                            .abs()
                    })
                    .fold(0., f64::max);
                assert!(residual < 1e-12);
                if (cols, load, horizon) == (5, 1250, 1200) {
                    assert_eq!(supported, 100);
                    assert!((normal / (9.81 * mass) - 1.).abs() < 0.005);
                    assert!((acceleration + 0.7 * 9.81).abs() < 0.03);
                }
                Some(
                    json!({"terminal_y_speed_m_s":final_y,"loaded_ticks_of_final_100":supported,
                    "mean_normal_n":normal,"lateral_deceleration_m_s2":-acceleration,
                    "largest_lateral_balance_error_ns":residual}),
                )
            } else {
                None
            };
            results.push(json!({
                "archive": name,
                "observations": run.summary,
                "observed_postgrip_ticks": observed,
                "first_complete_escape_tick": first_escape,
                "independent_jaw_margin_checks": native_margin_checks,
                "terminal_jaw_margins_m": margins.last(),
                "first_landing": landing,
                "last_100ms_slide": slide,
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

/// Physics observations at every public tick and an independent particle mass/velocity census.
fn check_first_landing(
    root: &Path,
    name: &str,
    post: &[Value],
    escaped: usize,
    dt_ns: u64,
) -> Result<Value> {
    let last = post.len() - 1;
    let h = dt_ns as f64 * 1e-9;
    let (a, b) = (&post[last - 1], &post[last]);
    assert!(
        post[escaped - 1..last]
            .iter()
            .all(|t| scalar(&t["bench_up_n"]).abs() < 1e-9)
    );
    assert!(scalar(&b["bench_up_n"]) > 1e-9);
    let hit = read(&root.join(name).join("landing.json"))?;
    assert_eq!(hit["tick"], post.len());
    near(
        scalar(&a["min_bench_clearance_m"]),
        scalar(&hit["incoming_clearance_m"]),
    );
    near(
        scalar(&b["min_bench_clearance_m"]),
        scalar(&hit["outgoing_clearance_m"]),
    );
    let material = Material::read(&fs::read_to_string(root.join(name).join("world.sdf"))?)?;
    let momentum = |key: &str| -> Result<f64> {
        let rows: Vec<[f64; 2]> = serde_json::from_value(hit[key].clone())?;
        assert_eq!(rows.len(), material.mass.len());
        Ok(rows
            .iter()
            .zip(&material.mass)
            .map(|(p, &m)| {
                near(p[0], m);
                m * p[1]
            })
            .sum())
    };
    let before = momentum("incoming_mass_kg_vz_m_s")?;
    let after = momentum("outgoing_mass_kg_vz_m_s")?;
    near(before, scalar(&hit["incoming_vertical_momentum_kg_m_s"]));
    near(after, scalar(&hit["outgoing_vertical_momentum_kg_m_s"]));
    let weight = 9.81 * material.mass.iter().sum::<f64>();
    let jaws = |key: &str| b[key].as_array().unwrap().iter().map(scalar).sum::<f64>();
    let normal = h * scalar(&b["bench_up_n"]);
    let tangent = -h * scalar(&b["bench_side_reaction_n"]);
    let dz = after - before - h * (jaws("jaw_up_n") - weight) - normal;
    let dy = scalar(&b["momentum_y_kg_m_s"])
        - scalar(&a["momentum_y_kg_m_s"])
        - h * (scalar(&b["force_y_n"]) - jaws("jaw_side_reaction_n"))
        - tangent;
    near(normal, scalar(&hit["bench_upward_impulse_ns"]));
    near(tangent, scalar(&hit["bench_lateral_impulse_ns"]));
    near(dz, scalar(&hit["vertical_balance_error_ns"]));
    near(dy, scalar(&hit["lateral_balance_error_ns"]));
    assert!(dz.abs() < 1e-12 && dy.abs() < 1e-12);
    Ok(
        json!({"time_ns":b["time_ns"],"postgrip_tick":post.len(),"normal_ns":normal,"tangent_ns":tangent,"normal_force_n":normal/h,"z_residual_ns":dz,"y_residual_ns":dy,"particles_per_state":material.mass.len()}),
    )
}
