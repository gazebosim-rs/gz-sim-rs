// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::EdgeStickMode;
const MIXED: &str = include_str!("../../../../examples/multifidelity_mixed_edge.sdf");
fn flight(ns: f64, mode: EdgeStickMode, g: f64) -> String {
    let (x, y) = match mode {
        EdgeStickMode::BothStick => (0.05, 0.4),
        EdgeStickMode::FirstSticks => (0.02, 0.08),
        EdgeStickMode::SecondSticks => (-0.02, 0.08),
    };
    let (s, c) = 0.67f64.sin_cos();
    let v = format!(
        "{:.17} {:.17} -1.05",
        0.17 + c * x - s * y,
        -0.09 + s * x + c * y
    );
    source_at(ns * 1e-9, g, 0.2)
        .replace("project_sliding_edge", "project_mixed_edge")
        .replace("3.6 -1.9 -1.05", &v)
}
#[test]
fn both_sticking_endpoints_report_identifiable_normal_and_total_with_shear_self_stress() {
    let mut s = try_load(MIXED).unwrap();
    s.step_exact(1).unwrap();
    let r = last_refinement(&s).unwrap().clone();
    let [h] = r.interval.mixed_edge_impacts.as_slice() else {
        panic!("missing cold edge impact")
    };
    assert_eq!(h.mode, EdgeStickMode::BothStick);
    assert_eq!(h.elapsed, Duration::ZERO);
    assert_eq!(r.interval.native_steps, 8);
    assert!(r.interval.sliding_edge_impacts.is_empty());
    let [lo, hi] = h.impulses.first_longitudinal_range;
    assert!((hi - 0.16128).abs() < 1e-12 && (lo + 0.16128).abs() < 1e-12);
    assert!((h.impulses.longitudinal_total - 0.01176).abs() < 1e-12);
    let total = DVec3::from(h.total_impulse_on_foot());
    let mut moments = vec![];
    for split in [lo, hi] {
        let j = h.split_world(split).unwrap().map(DVec3::from);
        assert!((j[0].z - 0.48).abs() < 1e-12 && (j[1].z - 0.72).abs() < 1e-12);
        assert!((j[0] + j[1]).distance(total) < 1e-12);
        moments.push(DVec3::from(h.points[0]).cross(j[0]) + DVec3::from(h.points[1]).cross(j[1]));
    }
    assert!(moments[0].distance(moments[1]) < 1e-12);
    assert!(h.split_world(hi + 1e-5).is_none());
    let pair = &r.requested_pairs[0];
    assert_eq!(pair.first_positive_end, Some(Duration::ZERO));
    assert!(pair.normal_impulse >= 1.2 - 1e-12 && pair.last_native_normal_impulse < 0.01);
    let foot = handles(&s, "default::rocker::foot").1.unwrap();
    let upper = handles(&s, "default::rocker::upper").1.unwrap();
    let initial = try_load(MIXED).unwrap();
    let mass_velocity = |sim: &Simulation| {
        DVec3::from(state(sim, foot).velocity) + DVec3::from(state(sim, upper).velocity)
    };
    let contact = DVec3::from(pair.normal_impulse_on_foot)
        + DVec3::from(pair.tangent_impulse_on_foot.unwrap());
    assert!(
        contact.distance(mass_velocity(&s) - mass_velocity(&initial) + DVec3::Z * 0.00981) < 6e-5
    );
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_refinement(&s).is_none());
    s.step_exact(1).unwrap();
    assert_eq!(last_refinement(&s).unwrap().interval, r.interval);
}
#[test]
fn mixed_modes_survive_fractional_free_flight_and_preserve_entire_public_clock() {
    for (ns, g) in [
        (62500.37, 0.),
        (499999.34, -9.81),
        (638197.61, 0.),
        (2138197.72, -9.81),
    ] {
        for mode in [
            EdgeStickMode::BothStick,
            EdgeStickMode::FirstSticks,
            EdgeStickMode::SecondSticks,
        ] {
            let mut s = try_load(&flight(ns, mode, g)).unwrap();
            let ticks = (ns / 500000.).ceil() as u64;
            for k in 1..ticks {
                s.step_exact(1).unwrap();
                let n = s
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                assert!(
                    n.rigid_contact_observations()
                        .iter()
                        .all(|p| p.normal_impulse == 0.)
                );
                assert!(
                    last_refinement(&s).is_none_or(
                        |r| r.iteration != k || r.interval.mixed_edge_impacts.is_empty()
                    )
                );
            }
            s.step_exact(1).unwrap();
            let rec = last_refinement(&s).unwrap();
            assert_eq!(rec.iteration, ticks);
            let [h] = rec.interval.mixed_edge_impacts.as_slice() else {
                panic!("missed {mode:?} at {ns} ns")
            };
            assert_eq!(h.mode, mode);
            assert_eq!((rec.start_time + h.elapsed).as_nanos(), ns.floor() as u128);
            assert_eq!(rec.requested_pairs[0].first_positive_end, Some(h.elapsed));
            assert_eq!(s.elapsed(), Duration::from_micros(500 * ticks));
            for i in 0..2 {
                let post = DVec3::from(h.after_relative_velocity[i]);
                assert!(post.z.abs() < 1e-12);
                let sticks = mode == EdgeStickMode::BothStick
                    || (mode == EdgeStickMode::FirstSticks) == (i == 0);
                if sticks {
                    assert!(post.length() < 1e-12);
                } else {
                    assert!(post.length() > 1e-5);
                }
            }
        }
    }
}
#[test]
fn disabled_uncertified_or_interrupted_mixed_contacts_keep_ordinary_ownership() {
    for other in [
        "project_mixed_edge",
        "project_corner",
        "project_planar",
        "project_sliding_edge",
        "follow_planar_edge",
    ] {
        assert!(
            try_load(&MIXED.replace("</pair>", &format!("<{other}>true</{other}></pair>")))
                .is_err()
        );
    }
    let mut unknown = try_load(&MIXED.replace(">0.35<", ">1.0<")).unwrap();
    let foot = handles(&unknown, "default::rocker::foot").1.unwrap();
    let pad = handles(&unknown, "default::lift::pad").1.unwrap();
    let mut owner = unknown.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    let before = n.state(foot);
    assert!(n.project_mixed_edge_impact(foot, pad).unwrap().is_none());
    let pp = [(foot, pad)];
    let a = BoxContactActions {
        sliding_edge_impacts: &pp,
        mixed_edge_impacts: &pp,
        ..Default::default()
    };
    assert!(
        n.advance_box_contacts(Duration::from_micros(500), 8, 4.into(), a, None)
            .is_err()
    );
    assert_eq!(before, n.state(foot));
    let mut paused = try_load(&flight(2138197.72, EdgeStickMode::BothStick, -9.81)).unwrap();
    paused.step_exact(2).unwrap();
    let uid = handles(&paused, "default::lift").0;
    paused.set_paused(true);
    assert!(paused.tick().unwrap().dt.is_zero());
    remove_model(&mut paused, uid).unwrap();
    paused.step_exact(10).unwrap();
    assert!(last_refinement(&paused).is_none());
}

#[test]
fn one_registered_edge_selects_each_certified_mode_and_preserves_uncertain_shear() {
    use gz_physics_multifidelity::EdgeImpactReceipt;
    for (src, expected) in [(MIXED, "mixed"), (SDF, "sliding")] {
        let tag = if expected == "mixed" {
            "project_mixed_edge"
        } else {
            "project_sliding_edge"
        };
        let mut plain = try_load(src).unwrap();
        let mut unified = try_load(&src.replace(tag, "project_edge")).unwrap();
        plain.step_exact(1).unwrap();
        unified.step_exact(1).unwrap();
        let a = last_refinement(&plain).unwrap();
        let b = last_refinement(&unified).unwrap();
        assert_eq!(b.interval.edge_impacts.len(), 1);
        assert!(
            b.interval.mixed_edge_impacts.is_empty() && b.interval.sliding_edge_impacts.is_empty()
        );
        match b.interval.edge_impacts[0] {
            EdgeImpactReceipt::Sticking(h) => {
                assert_eq!(expected, "mixed");
                assert_eq!(h, a.interval.mixed_edge_impacts[0]);
            }
            EdgeImpactReceipt::BothSlide(h) => {
                assert_eq!(expected, "sliding");
                for (p, q) in h.sites.iter().zip(a.interval.sliding_edge_impacts[0].sites) {
                    assert!(
                        DVec3::from(p.impulse_on_foot).distance(DVec3::from(q.impulse_on_foot))
                            < 1e-12
                    );
                }
            }
            _ => panic!("unexpected singleton on co-bearing edge"),
        }
        assert!(
            (a.requested_pairs[0].normal_impulse - b.requested_pairs[0].normal_impulse).abs()
                < 1e-12
        );
        assert!(
            try_load(&src.replace("</pair>", "<project_edge>true</project_edge></pair>")).is_err()
        );
    }
    let src = include_str!("../../../../examples/multifidelity_active_edge.sdf");
    let mut s = try_load(src).unwrap();
    for _ in 0..4 {
        s.step_exact(1).unwrap();
        let n = s
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        assert!(
            n.rigid_contact_observations()
                .iter()
                .all(|p| p.normal_impulse == 0.)
        );
    }
    s.step_exact(1).unwrap();
    let event = last_refinement(&s).unwrap();
    let [EdgeImpactReceipt::OneLoaded(h)] = event.interval.edge_impacts.as_slice() else {
        panic!("missing one-site impact from offset child");
    };
    assert_eq!(
        event.start_time + h.loaded.elapsed,
        Duration::from_nanos(2_138_197)
    );
    assert!((h.loaded.impulse_on_foot[2] - 1.2124970736247331).abs() < 1e-12);
    assert!(
        h.before_unloaded_relative_velocity[2] < -1.2
            && h.after_unloaded_relative_velocity[2] > 0.188
    );
    assert!(h.separating_margin.iter().all(|m| *m > 0.));
    let pair = &event.requested_pairs[0];
    assert_eq!(pair.first_positive_end, Some(h.loaded.elapsed));
    assert!((pair.normal_impulse - 1.216630983041424).abs() < 1e-10);
}

#[test]
fn stationary_bearing_and_arriving_vertex_receive_distinct_frictionless_impulses() {
    use gz_physics_multifidelity::EdgeImpactReceipt;
    let source = include_str!("../../../../examples/multifidelity_normal_recontact.sdf");
    let mut sim = try_load(source).unwrap();
    sim.step_exact(1).unwrap();
    let event = last_refinement(&sim).unwrap();
    let [EdgeImpactReceipt::Recontact(h)] = event.interval.edge_impacts.as_slice() else {
        panic!("missing measured two-normal load");
    };
    assert_eq!(
        event.requested_pairs[0].first_positive_end,
        Some(std::time::Duration::ZERO)
    );
    let order = [h.stationary_site, 1 - h.stationary_site];
    for (i, expected) in order
        .into_iter()
        .zip([0.0327300707844205, 1.1774415182476048])
    {
        let p = h.sites[i];
        assert_eq!(p.elapsed, std::time::Duration::ZERO);
        assert!((p.impulse_on_foot[2] - expected).abs() < 1e-12);
        assert!(p.after_relative_velocity[2].abs() < 1e-12);
    }
    assert!(event.requested_pairs[0].normal_impulse > 1.21017);
    sim.step_exact(1).unwrap();
    assert!(
        !last_refinement(&sim)
            .is_some_and(|e| e.iteration == 2 && !e.interval.edge_impacts.is_empty())
    );
    let other = source.replace("<mu>0</mu><mu2>0</mu2>", "<mu>0.3</mu><mu2>0.3</mu2>");
    let mut sim = try_load(&other).unwrap();
    sim.step_exact(1).unwrap();
    assert!(!last_refinement(&sim).is_some_and(|e| {
        e.interval
            .edge_impacts
            .iter()
            .any(|h| matches!(h, EdgeImpactReceipt::Recontact(_)))
    }));
}

#[test]
fn frictionless_two_bearing_keeps_per_site_integrals_until_the_next_geometric_change() {
    let base = include_str!("../../../../examples/multifidelity_normal_recontact.sdf");
    let source = base.replace(
        "</pair>",
        "<follow_frictionless_edge>true</follow_frictionless_edge></pair>",
    );
    let mut sim = try_load(&source).unwrap();
    sim.step_exact(1).unwrap();
    let result = last_refinement(&sim).unwrap();
    let interval = &result.interval;
    assert_eq!(interval.two_bearing_edges.len(), 8);
    let smooth = interval
        .two_bearing_edges
        .iter()
        .map(|s| {
            assert!(s.normal_force_bounds.iter().all(|i| i[0] > 0.));
            assert!(s.normal_impulses.iter().all(|v| *v > 0.));
            s.normal_impulses.iter().sum::<f64>()
        })
        .sum::<f64>();
    assert!(
        (result.requested_pairs[0].normal_impulse
            - (0.0327300707844205 + 1.1774415182476048 + smooth))
            .abs()
            < 1e-12,
        "public pair {:?}, actual smooth {smooth:e}, projection {:?}",
        result.requested_pairs[0],
        interval.edge_impacts
    );
    let mut last = 1;
    for t in 2..=112 {
        sim.step_exact(1).unwrap();
        if let Some(r) = last_refinement(&sim)
            .filter(|r| r.iteration == t && !r.interval.two_bearing_edges.is_empty())
        {
            last = t;
            assert!(r.interval.edge_impacts.is_empty());
        }
    }
    assert!((97..=100).contains(&last), "last smooth tick {last}");
    let invalid = source.replace("project_edge", "project_mixed_edge");
    assert!(try_load(&invalid).is_err());
}

#[test]
fn co_bearing_falls_back_for_queued_loads_and_forgets_a_removed_pad() {
    let source = include_str!("../../../../examples/multifidelity_two_bearing.sdf");
    let witness = r#"<model name="witness"><pose>3 0 2 0 0 0</pose><link name="b"><inertial><mass>2</mass></inertial><collision name="ball"><geometry><sphere><radius>0.08</radius></sphere></geometry></collision></link></model>"#;
    let mixed = source.replace("</world>", &format!("{witness}</world>"));
    let mut a = try_load(&mixed).unwrap();
    let mut b = try_load(&mixed.replace(
        "<follow_frictionless_edge>true</follow_frictionless_edge>",
        "",
    ))
    .unwrap();
    for sim in [&mut a, &mut b] {
        let (id, _) = handles(sim, "default::witness::b");
        queue_link_world_wrench(sim, id, [12., -6., 0.], [0.; 3]).unwrap();
        sim.step_exact(1).unwrap();
    }
    assert!(
        last_refinement(&a)
            .unwrap()
            .interval
            .two_bearing_edges
            .is_empty()
    );
    for name in [
        "default::rocker::foot",
        "default::rocker::upper",
        "default::witness::b",
    ] {
        assert_eq!(
            state(&a, handles(&a, name).1.unwrap()),
            state(&b, handles(&b, name).1.unwrap())
        );
    }
    let mut sim = try_load(source).unwrap();
    sim.step_exact(4).unwrap();
    assert!(
        !last_refinement(&sim)
            .unwrap()
            .interval
            .two_bearing_edges
            .is_empty()
    );
    let uid = handles(&sim, "default::rocker::foot").0;
    queue_link_world_wrench(&mut sim, uid, [10., 0., 3.], [0.01, 0.02, 0.]).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        last_refinement(&sim)
            .is_none_or(|r| r.iteration != 5 || r.interval.two_bearing_edges.is_empty())
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(2).unwrap();
    assert!(
        !last_refinement(&sim)
            .unwrap()
            .interval
            .two_bearing_edges
            .is_empty()
    );
    sim.set_paused(true);
    assert!(sim.tick().unwrap().dt.is_zero());
    let pad = handles(&sim, "default::lift").0;
    let previous = last_refinement(&sim).unwrap().iteration;
    let foot = handles(&sim, "default::rocker::foot").1.unwrap();
    let upper = handles(&sim, "default::rocker::upper").1.unwrap();
    let momentum = |s: &Simulation| {
        DVec3::from(state(s, foot).velocity) + DVec3::from(state(s, upper).velocity)
    };
    let before = momentum(&sim);
    // Rebuild the same unforced locked assembly from the measured rigid state
    // with the independent ordinary solver and its pad physically distant.
    let measured = state(&sim, foot);
    let pose = glam::DQuat::from_array(measured.orientation);
    let (yaw, pitch, roll) = pose.to_euler(glam::EulerRot::ZYX);
    let vector = |x: [f64; 3]| format!("{:.17} {:.17} {:.17}", x[0], x[1], x[2]);
    let mut control = source.to_string();
    let replace = |src: &mut String, model: &str, tag: &str, value: String| {
        let model = src.find(&format!("<model name=\"{model}\">")).unwrap();
        let open = format!("<{tag}>");
        let a = model + src[model..].find(&open).unwrap() + open.len();
        let b = a + src[a..].find(&format!("</{tag}>")).unwrap();
        src.replace_range(a..b, &value);
    };
    replace(
        &mut control,
        "rocker",
        "pose",
        format!(
            "{} {roll:.17} {pitch:.17} {yaw:.17}",
            vector(measured.position)
        ),
    );
    replace(
        &mut control,
        "rocker",
        "linear_velocity",
        vector((0.5 * before).to_array()),
    );
    replace(
        &mut control,
        "rocker",
        "angular_velocity",
        vector(measured.angular_velocity),
    );
    replace(&mut control, "lift", "pose", "0 0 -100 0 0 0".into());
    let mut baseline = try_load(&control).unwrap();
    let cf = handles(&baseline, "default::rocker::foot").1.unwrap();
    let cu = handles(&baseline, "default::rocker::upper").1.unwrap();
    baseline.step_exact(4).unwrap();
    remove_model(&mut sim, pad).unwrap();
    sim.step_exact(4).unwrap();
    // The latest historical record is retained within an epoch. Removing
    // the support must stop new records and return the assembly to flight.
    assert!(last_refinement(&sim).is_none_or(|r| r.iteration == previous));
    let after_baseline =
        DVec3::from(state(&baseline, cf).velocity) + DVec3::from(state(&baseline, cu).velocity);
    assert!(
        momentum(&sim).distance(after_baseline) < 2e-12,
        "delta {:?}, matched flight {:?}",
        momentum(&sim) - before,
        after_baseline - before
    );
}

#[test]
fn projected_frictionless_sole_reports_the_pressure_center_and_every_equivalent_diagonal_split() {
    let source = include_str!("../../../../examples/multifidelity_frictionless_sole.sdf");
    let mut sim = try_load(source).unwrap();
    sim.step_exact(1).unwrap();
    let r = last_refinement(&sim).unwrap();
    let [hit] = r.interval.face_pressure_impacts.as_slice() else {
        panic!("missing off-center face impulse");
    };
    assert!(hit.elapsed.is_zero() && (hit.normal_impulse - 0.87).abs() < 1e-12);
    assert!(
        (hit.normalized_center[0] - 0.28).abs() < 1e-12
            && (hit.normalized_center[1] - 0.17).abs() < 1e-12
    );
    let [lo, hi] = hit.equivalent_corner_loads.shift_range;
    assert!((hi - lo - 0.3132).abs() < 1e-12);
    for delta in [lo, 0.5 * (lo + hi), hi] {
        let j = hit.equivalent_corner_loads.at(delta).unwrap();
        let point = hit
            .points
            .into_iter()
            .zip(j)
            .map(|(p, j)| j * DVec3::from(p))
            .sum::<DVec3>()
            / hit.normal_impulse;
        assert!(point.distance(hit.center_of_pressure.into()) < 1e-14);
    }
    assert!(r.interval.planar_impacts.is_empty() && r.interval.edge_impacts.is_empty());
    assert!(r.requested_pairs[0].normal_impulse > hit.normal_impulse);
    let previous = hit.normal_impulse;
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        last_refinement(&sim)
            .unwrap()
            .interval
            .face_pressure_impacts[0]
            .normal_impulse,
        previous
    );
    let mut dry =
        try_load(&source.replace("<mu>0</mu><mu2>0</mu2>", "<mu>0.4</mu><mu2>0.4</mu2>")).unwrap();
    dry.step_exact(1).unwrap();
    assert!(last_refinement(&dry).is_none_or(
        |r| r.interval.face_pressure_impacts.is_empty() && r.interval.edge_impacts.is_empty()
    ));
    assert!(
        try_load(&source.replace("</pair>", "<project_edge>true</project_edge></pair>")).is_err()
    );
}

#[test]
fn loaded_edge_reenrolls_after_reset_and_transfers_inside_the_public_step() {
    const TRANSFER: &str = include_str!("../../../../examples/multifidelity_sole_transfer.sdf");
    let mut sim = try_load(TRANSFER).unwrap();
    let (mut history, mut foot_states) = (vec![], vec![]);
    for _ in 0..2 {
        let (_, f) = handles(&sim, "default::rocker::foot");
        sim.step_exact(1).unwrap();
        let report = last_refinement(&sim).unwrap();
        let [hit] = report.interval.sole_transfers.as_slice() else {
            panic!("missing internal change of bearing")
        };
        assert_eq!(
            report.interval.face_pressure_impacts.as_slice(),
            [hit.pressure]
        );
        assert!(
            hit.event_bracket_seconds[0] <= hit.pressure.physical_elapsed_seconds
                && hit.event_bracket_seconds[1] >= hit.pressure.physical_elapsed_seconds
        );
        assert!((hit.pressure.physical_elapsed_seconds - 0.000213819760).abs() < 1e-13);
        assert_eq!(hit.pressure.elapsed, Duration::from_nanos(213820));
        assert_eq!(
            report.requested_pairs[0].first_positive_end,
            Some(Duration::ZERO)
        );
        assert!((report.requested_pairs[0].normal_impulse - 1.194116668590228).abs() < 1e-11);
        assert!((hit.pressure.normal_impulse - 1.187962065180675).abs() < 1e-11);
        history.push(*hit);
        foot_states.push(state(&sim, f.unwrap()));
        sim.step_exact(1).unwrap();
        let follow = last_refinement(&sim).unwrap();
        assert!(follow.interval.sole_transfers.is_empty());
        assert!(!follow.interval.two_bearing_edges.is_empty());
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(last_refinement(&sim).is_none());
    }
    assert_eq!(
        history[0].pressure.physical_elapsed_seconds,
        history[1].pressure.physical_elapsed_seconds
    );
    assert_eq!(foot_states[0], foot_states[1]);
}
#[test]
fn enrolled_sole_hands_back_external_interruption_and_declines_positive_friction() {
    const TRANSFER: &str = include_str!("../../../../examples/multifidelity_sole_transfer.sdf");
    let witness = r#"<model name="intruder"><static>true</static><pose>0 0 0.36 0 0 0</pose><link name="link"><collision name="cap"><geometry><box><size>0.35 0.35 0.26</size></box></geometry></collision></link></model>"#;
    let cases = [
        TRANSFER.replace("<mu>0</mu><mu2>0</mu2>", "<mu>0.4</mu><mu2>0.4</mu2>"),
        TRANSFER.replace("</world>", &format!("{witness}</world>")),
        TRANSFER.into(),
    ];
    for (k, source) in cases.iter().enumerate() {
        let ordinary = source.replace(
            "<follow_frictionless_edge>true</follow_frictionless_edge>",
            "",
        );
        let mut states = vec![];
        for src in [source, &ordinary] {
            let mut s = try_load(src).unwrap();
            if k == 2 {
                let (uid, _) = handles(&s, "default::rocker::upper");
                queue_link_world_wrench(&mut s, uid, [0., 1., 0.], [0., 0.4, 0.]).unwrap();
            }
            s.step_exact(1).unwrap();
            assert!(
                last_refinement(&s).is_none_or(|r| r.interval.sole_transfers.is_empty()
                    && r.interval.two_bearing_edges.is_empty())
            );
            states.push(state(&s, handles(&s, "default::rocker::foot").1.unwrap()));
        }
        assert_eq!(states[0], states[1]);
    }
}

#[test]
fn repeated_rocking_arrests_inside_the_certified_interval_and_carries_full_static_weight() {
    const TRAIN: &str = include_str!("../../../../examples/multifidelity_rocking_limit.sdf");
    let mut s = try_load(TRAIN).unwrap();
    let mut jumps = 0.;
    let mut arrest = None;
    let mut first = None;
    for tick in 1..=104 {
        s.step_exact(1).unwrap();
        let r = last_refinement(&s).filter(|r| r.iteration == tick).unwrap();
        assert!(!r.interval.rocking_limits.is_empty());
        for h in &r.interval.rocking_limits {
            let x = h.response;
            jumps += x.integrated_impact_impulse;
            if let Some(hit) = x.first_impact {
                first.get_or_insert((r.start_time + h.start).as_secs_f64() + hit.seconds);
            }
            if let Some(window) = x.settle_time_bounds {
                assert!(x.includes_infinite_tail);
                assert!(
                    arrest
                        .replace(window.map(|t| (r.start_time + h.start).as_secs_f64() + t))
                        .is_none()
                );
            }
            if tick > 98 {
                assert!(x.settled);
                assert_eq!(x.angular_rate, 0.);
                assert_eq!(x.integrated_impact_impulse, 0.);
                assert!(
                    (x.integrated_normal_impulse - 19.62 * h.duration.as_secs_f64()).abs() < 1e-13
                );
            }
        }
        let p = r.requested_pairs[0];
        let total = r
            .interval
            .rocking_limits
            .iter()
            .map(|h| h.response.integrated_normal_impulse)
            .sum::<f64>();
        assert!((p.normal_impulse - total).abs() < 1e-12);
    }
    assert!((first.unwrap() - 0.0298769340967383).abs() < 1e-10);
    let end = arrest.unwrap();
    assert!(end[0] > 0.0485158 && end[1] < 0.0485160);
    let r = 5. / 13.;
    assert!((jumps - 0.2 * 0.45 * (1. + r) / (1. - r)).abs() < 1e-10);
}
#[test]
fn nonplanar_spin_and_external_forces_decline_only_the_rocking_limit() {
    const TRANSFER: &str = include_str!("../../../../examples/multifidelity_sole_transfer.sdf");
    const TRAIN: &str = include_str!("../../../../examples/multifidelity_rocking_limit.sdf");
    const FLAG: &str = "<resolve_rocking_limit>true</resolve_rocking_limit>";
    let tagged = TRANSFER.replace(
        "</follow_frictionless_edge>",
        &format!("</follow_frictionless_edge>{FLAG}"),
    );
    let mut a = try_load(&tagged).unwrap();
    let mut b = try_load(TRANSFER).unwrap();
    a.step_exact(1).unwrap();
    b.step_exact(1).unwrap();
    let r = last_refinement(&a).unwrap();
    assert!(r.interval.rocking_limits.is_empty());
    assert_eq!(r.interval.sole_transfers.len(), 1);
    assert_eq!(
        r.interval.sole_transfers,
        last_refinement(&b).unwrap().interval.sole_transfers
    );
    let mut states = vec![];
    for src in [TRAIN.to_string(), TRAIN.replace(FLAG, "")] {
        let mut sim = try_load(&src).unwrap();
        let (uid, foot) = handles(&sim, "default::rocker::foot");
        queue_link_world_wrench(&mut sim, uid, [1., -2., 0.1], [0.3, 0.2, 0.]).unwrap();
        sim.step_exact(1).unwrap();
        assert!(last_refinement(&sim).is_none_or(|r| r.interval.rocking_limits.is_empty()));
        states.push(state(&sim, foot.unwrap()));
    }
    assert_eq!(states[0], states[1]);
    let invalid = TRAIN.replace(
        "<follow_frictionless_edge>true</follow_frictionless_edge>",
        "",
    );
    assert!(try_load(&invalid).is_err());
}
#[test]
fn finite_face_pressure_follows_yaw_through_overhang_then_releases_on_the_first_native_boundary() {
    const SAMPLE: &str = include_str!("../../../../examples/multifidelity_face_bearing.sdf");
    let mut sim = try_load(SAMPLE).unwrap();
    let mut history = vec![];
    for _ in 0..2 {
        sim.step_exact(182).unwrap();
        let r = last_refinement(&sim).unwrap();
        let end = r.interval.face_bearings.last().unwrap();
        assert_eq!(r.iteration, 182);
        assert!((end.normal_force - 19.62).abs() < 1e-11 && end.end_overlap.area < 0.042);
        assert!((r.requested_pairs[0].normal_impulse - 0.00981).abs() < 1e-12);
        history.push(*end);
        sim.step_exact(1).unwrap();
        assert!(
            last_refinement(&sim)
                .is_none_or(|r| r.iteration < 183 || r.interval.face_bearings.is_empty())
        );
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
    assert_eq!(history[0].end_pressure_world, history[1].end_pressure_world);
}
#[test]
fn finite_face_enrollment_preserves_positive_friction_and_upper_body_commands() {
    const SAMPLE: &str = include_str!("../../../../examples/multifidelity_face_bearing.sdf");
    for friction in [false, true] {
        let src = if friction {
            SAMPLE.replace("<mu>0</mu><mu2>0</mu2>", "<mu>0.25</mu><mu2>0.25</mu2>")
        } else {
            SAMPLE.to_string()
        };
        let mut outputs = vec![];
        for text in [
            src.clone(),
            src.replace(
                "<follow_frictionless_face>true</follow_frictionless_face>",
                "",
            ),
        ] {
            let mut sim = try_load(&text).unwrap();
            if !friction {
                let (uid, _) = handles(&sim, "default::rocker::upper");
                queue_link_world_wrench(&mut sim, uid, [2., 0., 0.], [0.1, 0., 0.]).unwrap();
            }
            sim.step_exact(1).unwrap();
            assert!(last_refinement(&sim).is_none_or(|r| r.interval.face_bearings.is_empty()));
            outputs.push(state(
                &sim,
                handles(&sim, "default::rocker::foot").1.unwrap(),
            ));
        }
        assert_eq!(outputs[0], outputs[1]);
    }
}

#[test]
fn registered_deck_edge_splits_the_public_tick_and_accumulates_moving_normal_vectors() {
    const SOURCE: &str = include_str!("../../../../examples/multifidelity_deck_edge_bearing.sdf");
    let mut sim = try_load(SOURCE).unwrap();
    sim.step_exact(1).unwrap();
    let first = last_refinement(&sim).unwrap();
    let split = first.interval.deck_edge_bearings.first().unwrap();
    assert!(split.face_exit_bracket_seconds.is_some());
    assert!((split.face_seconds - 0.00021381976).abs() < 1e-12);
    assert!((split.face_seconds + split.edge_seconds - split.duration.as_secs_f64()).abs() < 1e-15);
    sim.step_exact(180).unwrap();
    let r = last_refinement(&sim).unwrap();
    let row = r.interval.deck_edge_bearings.last().unwrap();
    assert!(row.face_exit_bracket_seconds.is_none() && row.end_normal[0] > 1e-3);
    assert!(row.impulse_on_foot[0] > 1e-5 && row.end_equivalent_force.iter().all(|x| *x > 0.));
    assert!((row.normal_impulse() - r.requested_pairs[0].normal_impulse).abs() < 1e-12);
    let p = r
        .interval
        .pairs
        .iter()
        .find(|p| [p.a, p.b].contains(&row.foot) && [p.a, p.b].contains(&row.support))
        .unwrap();
    let sign = if p.a == row.foot { 1. } else { -1. };
    assert!(
        DVec3::from(p.normal_impulse_on_a).distance(sign * DVec3::from(row.impulse_on_foot))
            < 1e-14
    );
    let (uid, _) = handles(&sim, "default::rocker::upper");
    queue_link_world_wrench(&mut sim, uid, [0., 0., 0.], [0.2, 0., 0.]).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        last_refinement(&sim)
            .is_none_or(|r| r.iteration < 182 || r.interval.deck_edge_bearings.is_empty())
    );
}
#[test]
fn shared_pressure_preserves_the_surviving_support_after_paused_model_removal() {
    const SOURCE: &str = include_str!("../../../../examples/multifidelity_shared_supports.sdf");
    let mut sim = try_load(SOURCE).unwrap();
    sim.step_exact(444).unwrap();
    let pair = last_refinement(&sim).unwrap();
    assert_eq!(pair.interval.shared_faces.len(), 1);
    let load = pair.interval.shared_faces[0];
    assert!((load.total_normal_impulse - 0.00981).abs() < 1e-12);
    for site in load.loads {
        assert!(
            site.selected_normal_impulse >= site.normal_impulse_bounds[0] - 1e-13
                && site.selected_normal_impulse <= site.normal_impulse_bounds[1] + 1e-13
        );
    }
    let toe = handles(&sim, "default::toe::pad").1.unwrap();
    let heel = handles(&sim, "default::lift").0;
    sim.set_paused(true);
    assert!(sim.tick().unwrap().dt.is_zero());
    remove_model(&mut sim, heel).unwrap();
    sim.step_exact(1).unwrap();
    let now = last_refinement(&sim).unwrap();
    assert!(now.interval.shared_faces.is_empty());
    assert_eq!(now.interval.face_bearings[0].support, toe);
    assert!((now.interval.face_bearings[0].normal_impulse - 0.00981).abs() < 1e-12);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        last_refinement(&sim).unwrap().interval.shared_faces.len(),
        1
    );
}
#[test]
fn shared_pair_registration_rejects_incomplete_binding_and_checks_the_full_neighborhood() {
    const SOURCE: &str = include_str!("../../../../examples/multifidelity_shared_supports.sdf");
    let one = SOURCE.replacen("<share_coplanar_face>true</share_coplanar_face>", "", 1);
    assert!(try_load(&one).is_err());
    let intruder = r#"<model name="obstacle"><static>true</static><pose>0 0 0.27 0 0 0</pose><link name="link"><collision name="c"><geometry><box><size>0.13 0.10 0.12</size></box></geometry></collision></link></model>"#;
    let mut sim = try_load(&SOURCE.replace("</world>", &format!("{intruder}</world>"))).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        last_refinement(&sim).is_none_or(
            |r| r.interval.shared_faces.is_empty() && r.interval.face_bearings.is_empty()
        )
    );
}

#[test]
fn shared_kinetic_supports_report_actual_tangents_and_delegate_the_first_stopping_belt() {
    const SRC: &str = include_str!("../../../../examples/multifidelity_uniform_shared_slide.sdf");
    let mut sim = try_load(SRC).unwrap();
    sim.step_exact(165).unwrap();
    let r = last_refinement(&sim).unwrap();
    let phase = r.interval.shared_faces.last().unwrap();
    assert!(
        phase.dissipated_energy > 0. && phase.slip_start_end.unwrap()[1].iter().all(|s| *s > 0.)
    );
    for site in phase.loads {
        let pair = r
            .interval
            .pairs
            .iter()
            .find(|p| [p.a, p.b].contains(&site.support) && [p.a, p.b].contains(&phase.foot))
            .unwrap();
        let sign = if pair.a == phase.foot { 1. } else { -1. };
        let tangent = DVec3::from(site.tangent_impulse_on_foot);
        assert!((tangent.length() - 0.46 * site.selected_normal_impulse).abs() < 1e-14);
        assert!(tangent.distance(sign * DVec3::from(pair.tangent_impulse_on_a.unwrap())) < 1e-14);
    }
    sim.step_exact(1).unwrap();
    assert!(
        last_refinement(&sim)
            .is_none_or(|r| r.iteration != 166 || r.interval.shared_faces.is_empty())
    );
    let tag = "<share_uniform_sliding>true</share_uniform_sliding>";
    let mut disabled = try_load(&SRC.replace(tag, "")).unwrap();
    disabled.step_exact(1).unwrap();
    assert!(last_refinement(&disabled).is_none_or(|r| r.interval.shared_faces.is_empty()));
    assert!(try_load(&SRC.replacen(tag, "", 1)).is_err());
    let mut hit = try_load(SRC).unwrap();
    let upper = handles(&hit, "default::rocker::upper").0;
    queue_link_world_wrench(&mut hit, upper, [0.35, -0.74, 0.18], [0., 0.05, 0.]).unwrap();
    hit.step_exact(1).unwrap();
    assert!(last_refinement(&hit).is_none_or(|r| r.interval.shared_faces.is_empty()));
}

#[test]
fn shared_comoving_stop_preserves_both_subtick_phases_and_releases_after_one_belt_changes() {
    const SOURCE: &str =
        include_str!("../../../../examples/multifidelity_comoving_support_arrest.sdf");
    let mut sim = try_load(SOURCE).unwrap();
    sim.step_exact(75).unwrap();
    let r = last_refinement(&sim).unwrap();
    assert_eq!(r.iteration, 75);
    let [slide, rest]: &[_; 2] = r.interval.shared_faces.as_slice().try_into().unwrap();
    assert!(!slide.comoving_rest && rest.comoving_rest);
    assert!((slide.physical_duration_seconds - 0.00021381976).abs() < 2e-14);
    assert_eq!(rest.physical_start_seconds, slide.physical_duration_seconds);
    assert_eq!(rest.duration + slide.duration, Duration::from_micros(500));
    assert_eq!(rest.slip_start_end, Some([[0.; 2]; 2]));
    assert!(slide.dissipated_energy > 0. && rest.dissipated_energy == 0.);
    for s in rest.loads {
        assert_eq!(s.tangent_impulse_on_foot, [0.; 3]);
    }
    sim.step_exact(1).unwrap();
    assert!(
        last_refinement(&sim)
            .unwrap()
            .interval
            .shared_faces
            .iter()
            .all(|p| p.comoving_rest)
    );
    let toe = handles(&sim, "default::toe::pad").0;
    sim.set_paused(true);
    assert!(sim.tick().unwrap().dt.is_zero());
    gz_sim_scene::queue_link_world_velocity(&mut sim, toe, [-0.7, 0.3, 0.2], [0.; 3]).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        last_refinement(&sim)
            .is_none_or(|r| r.iteration != 77 || r.interval.shared_faces.is_empty())
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(75).unwrap();
    assert!(
        last_refinement(&sim)
            .unwrap()
            .interval
            .shared_faces
            .last()
            .unwrap()
            .comoving_rest
    );
    let tag = "<settle_comoving_supports>true</settle_comoving_supports>";
    assert!(try_load(&SOURCE.replacen(tag, "", 1)).is_err());
    let mut regular = try_load(&SOURCE.replace(tag, "")).unwrap();
    regular.step_exact(75).unwrap();
    assert!(
        last_refinement(&regular)
            .is_none_or(|r| r.iteration != 75 || r.interval.shared_faces.is_empty())
    );
}
