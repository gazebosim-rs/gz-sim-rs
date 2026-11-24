// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn reject_unresolvable_clocks_or_unreachable_offsets_before_capture() {
    let d = tempfile::tempdir().unwrap();
    for (shift, rate) in [(80, 0), (80, 3), (80, 1000), (80, 1001), (201, 16)] {
        assert!(run(d.path(), shift, rate, true, false, None).is_err());
        assert!(d.path().read_dir().unwrap().next().is_none());
    }
    assert!(compare(d.path(), d.path()).is_err());
}

#[test]
fn an_overhang_releases_onto_the_table_and_an_unbalanced_one_falls() -> Result<()> {
    for (shift, outcome, nodes) in [
        (80, Outcome::Resting, 4),
        (95, Outcome::Fell, 8),
        (165, Outcome::Held, 0),
    ] {
        let xml = world(16, true, false)?;
        let mut rec = Recording::new(&xml, "edge-test.sdf", "default")?;
        let (rows, mut samples) = measure(&mut rec, shift, &mut None, None)?;
        let result = summarize(&rows, &samples, shift, 16, 0.06, MS / 16, false)?;
        assert_eq!(result.outcome, outcome);
        assert!(result.max_linear_balance_residual_ns < 1e-13);
        if outcome == Outcome::Held {
            assert!(result.pad_free_tick.is_none() && result.last_table_contact_tick.is_none());
            let acceleration_load =
                (samples[END - 1].momentum_ns[2] - samples[END - 101].momentum_ns[2]) / 0.1;
            assert!((result.mean_final_jaws_n - WEIGHT - acceleration_load).abs() < 1e-12);
            assert!(acceleration_load.abs() < 0.01 * WEIGHT);
        } else {
            assert_eq!(result.overhanging_nodes_at_release, Some(nodes));
            assert!(result.peak_simultaneous_support_n > 0.35 * WEIGHT);
            let [lo, hi] = result.released_line_of_action_inside_m.unwrap();
            if outcome == Outcome::Resting {
                assert!((lo..hi).contains(&0.01));
                assert!(lo > 0.009 && hi < 0.011);
                assert!((result.mean_final_table_n - WEIGHT).abs() < 1e-10);
            } else {
                assert!(lo.abs().max(hi.abs()) < 1e-6);
                assert!(
                    result.final_lowest_skin_m < -1.
                        && result.free_gravity_residual_m_s2.unwrap() < 1e-8
                );
            }
        }
        assert!(summarize(&rows[..END - 1], &samples, shift, 16, 0.06, MS / 16, false).is_err());
        assert!(summarize(&rows, &samples, shift + 1, 16, 0.06, MS / 16, false).is_err());
        if outcome == Outcome::Resting {
            samples[END - 1].peak_speed_m_s = 0.5;
            assert_eq!(
                summarize(&rows, &samples, shift, 16, 0.06, MS / 16, false)?.outcome,
                Outcome::Unsettled
            );
        }
        samples[2000].deck_impulse_ns[0] += 0.001;
        assert!(summarize(&rows, &samples, shift, 16, 0.06, MS / 16, false).is_err());
    }
    Ok(())
}

#[test]
fn resettable_native_recordings_compare_only_at_compatible_public_ticks() {
    let tmp = tempfile::tempdir().unwrap();
    let coarse = tmp.path().join("coarse");
    let fine = tmp.path().join("fine");
    fs::create_dir(&coarse).unwrap();
    fs::create_dir(&fine).unwrap();
    run(&coarse, 80, 16, true, true, None).expect("repeatable public recording");
    run(&fine, 80, 32, false, false, None).expect("independent finer native recording");
    let pair = compare(&coarse, &fine).expect("two compatible completed trajectories");
    assert_eq!(pair["outcomes"], json!(["Resting", "Resting"]));
    assert!(pair["max_release_aligned_com_error_m"].as_f64().unwrap() < 0.001);
    assert!(compare(&fine, &coarse).is_err());
    let path = fine.join("comparison.json");
    let summary: Value = serde_json::from_slice(&fs::read(&path).expect("fine receipt"))
        .expect("complete measured summary");
    for (key, value) in [
        ("actuator_ns", json!(500_000)),
        ("deck_shift_mm", json!(95)),
        ("pad_free_tick", json!(0)),
        ("pad_free_tick", json!(END + 1)),
    ] {
        let mut invalid = summary.clone();
        invalid[key] = value;
        write_json(&path, &invalid).unwrap();
        assert!(compare(&coarse, &fine).is_err());
    }
    write_json(&path, &summary).unwrap();
    write_json(&fine.join("edge.json"), &json!([])).unwrap();
    assert!(compare(&coarse, &fine).is_err());
    assert!(compare(&fine, &coarse).is_err());
    fs::remove_file(fine.join("edge.json")).unwrap();
    assert!(compare(&coarse, &fine).is_err());
    fs::write(fine.join("edge.json"), b"{").unwrap();
    assert!(compare(&coarse, &fine).is_err());
    fs::write(fine.join("comparison.json"), b"{").unwrap();
    assert!(compare(&coarse, &fine).is_err());
}

#[test]
fn sliding_loaded_receiver_preserves_resultant_about_the_public_start_origin() -> Result<()> {
    let xml = world(4, true, true)?;
    let mut outer = Recording::new(&xml, "public.sdf", "default")?;
    let mut native = Recording::new(&world(4, true, false)?, "fine.sdf", "default")?;
    let deck = entity_id(&outer.sim, "default::source::body")?;
    assert!(
        gz_sim_scene::queue_link_world_velocity(&mut outer.sim, deck, [0.; 3], [0., 0., 1.])
            .is_err()
    );
    let turntable = xml.replace(
        "coupling=\"one_way\"/>",
        "coupling=\"one_way\" motion=\"turntable\"/>",
    );
    assert!(Recording::new(&turntable, "rotating.sdf", "default").is_err());

    let mut normal = 0.;
    let mut shear = 0.;
    let mut shift_moment = 0.;
    for tick in 0..60 {
        if tick == 10 {
            for r in [&mut native, &mut outer] {
                gz_sim_scene::queue_link_world_velocity(&mut r.sim, deck, [0., 0.02, 0.], [0.; 3])?;
            }
        }
        let origin = DVec3::from(state(&outer.sim, deck)?.position);
        let mut impulse = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        for _ in 0..4 {
            let before = DVec3::from(state(&native.sim, deck)?.position);
            native.advance()?;
            let j = arm_sheet::cloth(&native.sim)?.last_boundary_impulses()[2];
            let shift = (before - origin).cross(j.linear.into());
            moment += DVec3::from(j.angular) + shift;
            impulse += DVec3::from(j.linear);
            shift_moment += shift.length();
        }
        outer.advance()?;
        let measured = arm_sheet::cloth(&outer.sim)?;
        assert_eq!(measured.rigid_material_exchanges(), 4);
        let j = measured.last_boundary_impulses()[2];
        assert!(DVec3::from(j.linear).distance(impulse) < 1e-12);
        assert!(DVec3::from(j.angular).distance(moment) < 1e-12);
        assert_eq!(
            measured.particles(),
            arm_sheet::cloth(&native.sim)?.particles()
        );
        if tick >= 10 {
            normal -= j.linear[2];
            shear += j.linear[1].abs();
        }
    }
    assert!(normal > 0.002 && shear > 1e-6 && shift_moment > 1e-9);
    Ok(())
}

#[test]
fn a_coarse_contact_fault_or_failed_archive_cannot_publish_an_edge_trial() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let path = temp.path().join("unresolved");
    let error = write_new(&path, |out| run(out, 95, 1, true, false, None)).unwrap_err();
    assert!(error.to_string().contains("swept surface self-contact"));
    assert!(!path.exists());
    let error = write_new(&path, |out| {
        fs::create_dir(out.join("world.sdf"))?;
        run(out, 80, 16, true, false, None)
    })
    .unwrap_err();
    assert!(error.downcast_ref::<std::io::Error>().is_some());
    assert!(temp.path().read_dir()?.next().is_none());
    Ok(())
}

#[test]
fn reject_a_missing_receiver_and_a_round_receiver_before_recording_an_edge() -> Result<()> {
    let xml = sheet_jaws::setdown_world(16, false, true)?;
    let mut missing = Recording::new(&xml, "missing-link.sdf", "default")?;
    let id = entity_id(&missing.sim, "default::source::body")?;
    let ent = entity_by_id(missing.sim.world(), id).unwrap();
    missing
        .sim
        .world_mut()
        .entity_mut(ent)
        .remove::<BodyHandle>();
    assert!(
        measure(&mut missing, 80, &mut None, None)
            .unwrap_err()
            .to_string()
            .contains("missing body")
    );
    missing.sim.world_mut().entity_mut(ent).remove::<Name>();
    assert!(
        measure(&mut missing, 80, &mut None, None)
            .unwrap_err()
            .to_string()
            .contains("missing default::source::body")
    );

    let mut round = xml;
    let model = round.find(r#"<model name="source">"#).unwrap();
    let start = model + round[model..].find("<box>").unwrap();
    let end = start + round[start..].find("</box>").unwrap() + 6;
    round.replace_range(start..end, "<sphere><radius>0.06</radius></sphere>");
    let mut r = Recording::new(&round, "round.sdf", "default")?;
    assert!(
        measure(&mut r, 80, &mut None, None)
            .unwrap_err()
            .to_string()
            .contains("missing box geometry")
    );
    Ok(())
}
