// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DMat3, DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step,
    primary_contact::last_opposing_corner, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;
const SDF: &str = include_str!("../../../examples/multifidelity_local_opposing_normals.sdf");
const FLAG: &str = "<project_opposing_normals>true</project_opposing_normals>";
fn load(s: &str) -> Simulation {
    load_world_with_sdf_step(
        s,
        "opposing.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn foot(s: &Simulation) -> BodyState {
    #[allow(deprecated)]
    let id = s
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == "default::rocker::foot")
                .then(|| e.get::<BodyHandle>().unwrap().id())
        })
        .unwrap();
    s.world().resource::<PhysicsBackend>().state(id).unwrap()
}
#[test]
fn one_native_call_reports_both_ordered_normal_sums_and_commits_their_spatial_impulses() {
    let mut s = load(SDF);
    let initial = foot(&s);
    s.step_exact(1).unwrap();
    let record = last_opposing_corner(&s).unwrap().clone();
    assert_eq!(record.interval.native_steps, 1);
    assert_eq!(record.sim_time, Duration::from_nanos(44));
    let [r] = record.interval.opposing_normal_series.as_slice() else {
        panic!("expected finite opposing series");
    };
    assert_eq!(r.end, foot(&s));
    assert_eq!(r.corners, [3, 0]);
    let [h] = r.finite.as_slice() else {
        panic!("expected one finite roof strike");
    };
    assert_eq!(h.site, 1);
    let expected = [4.038286397053222, 0.3236683156507053];
    let independent_limit = 1.1053364612258339e-8;
    assert!(r.terminal_time_s[0] <= independent_limit && independent_limit <= r.terminal_time_s[1]);
    assert!(
        r.min_face_margin_m > 0.
            && r.min_other_gap_m > 0.
            && r.separation_acceleration.into_iter().all(|v| v > 0.)
    );
    let measured = r.first_lower.point.impact;
    let nq = r.normals.map(DVec3::from);
    let mut net = DVec3::ZERO;
    for k in 0..2 {
        let delta = (r.normal_impulse[k] - expected[k]).abs();
        assert!(delta < r.impulse_radius[k] + 2e-11 && r.impulse_radius[k] <= 1e-4);
        let foot = measured.foot;
        let support = r.supports[k];
        let p = record
            .interval
            .pairs
            .iter()
            .find(|p| (p.a, p.b) == (foot.min(support), foot.max(support)))
            .unwrap();
        assert!(!p.active_at_end);
        assert_eq!(
            p.first_positive_end,
            Some(if k == 0 {
                measured.elapsed
            } else {
                Duration::from_secs_f64(h.time_s)
            })
        );
        let j = nq[k] * r.normal_impulse[k];
        let sign = if foot == p.a { 1. } else { -1. };
        assert!(j.distance(sign * DVec3::from(p.normal_impulse_on_a)) < 1e-12);
        assert!(DVec3::from(p.tangent_impulse_on_a.unwrap()).length() < 1e-12);
        net += j;
    }
    let g = DVec3::new(3.891319042256626, -3.533032893078129, -8.28320075138826);
    let applied = 2. * (DVec3::from(r.end.velocity) - DVec3::from(initial.velocity) - g * 44e-9);
    assert!(applied.distance(net) < 1e-12);
    let tensor = DMat3::from_cols_array(&TENSOR);
    let momentum = |b: BodyState| {
        let q = DQuat::from_array(b.orientation);
        q * (tensor * (q.conjugate() * DVec3::from(b.angular_velocity)))
    };
    let pre = r.first_lower.incoming;
    let com =
        DVec3::from(pre.position) + DQuat::from_array(pre.orientation) * DVec3::new(0., 0., 0.07);
    let mut dl = (DVec3::from(measured.point) - com).cross(DVec3::from(measured.impulse_on_foot));
    dl += DVec3::from(h.lever_from_com).cross(nq[1] * h.impulse);
    for (k, j) in r.terminal_normal.iter().enumerate() {
        dl += DVec3::from(r.terminal_levers[k]).cross(nq[k] * *j);
    }
    assert!(dl.distance(momentum(r.end) - momentum(initial)) < 1e-11);
    s.set_paused(true);
    assert!(s.tick().unwrap().dt.is_zero());
    assert_eq!(Some(&record), last_opposing_corner(&s));
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_opposing_corner(&s).is_none());
    s.step_exact(1).unwrap();
    let again = last_opposing_corner(&s).unwrap();
    assert_eq!(again.epoch, record.epoch + 1);
    assert_eq!(again.interval, record.interval);
}
#[test]
fn unsupported_worlds_keep_the_same_ordinary_step_and_receipt() {
    for fixture in [
        SDF.replace("<mu>0</mu><mu2>0</mu2>","<mu>0.3</mu><mu2>0.3</mu2>"),
        SDF.replace("</world>",r#"<model name="other"><pose>0 0 0.7 0 0 0</pose><link name="extra"><collision name="ball"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model></world>"#),
        SDF.replace("0.000000044000","0.000000003000")
    ]{
        let mut original=load(&fixture.replace(FLAG,""));
        let mut attempted=load(&fixture);
        original.step_exact(1).unwrap();attempted.step_exact(1).unwrap();
        assert_eq!(foot(&original),foot(&attempted));assert_eq!(original.elapsed(),attempted.elapsed());
        let a=last_opposing_corner(&original).map(|s|&s.interval);
        let b=last_opposing_corner(&attempted).map(|s|&s.interval);
        let mut comparison=b.cloned();
        if let Some(r)=&mut comparison{r.opposing_normal_declines.clear();}
        assert_eq!(a,comparison.as_ref());assert!(b.is_none_or(|s|s.opposing_normal_series.is_empty()));
    }
}
#[test]
fn enrollment_requires_a_unique_enabled_opposing_pair() {
    for invalid in [
        SDF.replace(FLAG, &format!("{FLAG}{FLAG}")),
        SDF.replace(
            FLAG,
            "<project_opposing_normals>yes</project_opposing_normals>",
        ),
        SDF.replace(
            "<project_opponent_corner>true</project_opponent_corner>",
            "",
        ),
    ] {
        assert!(
            load_world_with_sdf_step(
                &invalid,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

const TENSOR: [f64; 9] = [
    0.041674869460494465,
    -0.013871218300212176,
    0.006130513913955311,
    -0.013871218300212176,
    0.05787429028537743,
    0.011986955040662856,
    0.006130513913955315,
    0.011986955040662852,
    0.05405084025412814,
];

#[test]
fn independently_moving_supports_accumulate_inside_one_nanosecond_after_a_cold_approach() {
    let xml = include_str!("../../../examples/multifidelity_local_fractional_opposing_normals.sdf");
    let mut sim = load(xml);
    sim.step_exact(1).unwrap();
    let obs = last_opposing_corner(&sim).unwrap();
    let [hit] = obs.interval.opposing_normal_series.as_slice() else {
        panic!("the isolated sub-clock return should be enclosed");
    };
    assert_eq!(obs.sim_time, Duration::from_nanos(97));
    assert_eq!(obs.interval.native_steps, 1);
    let floor = hit.first_lower;
    assert!((floor.point.after_cut_s - 64e-9).abs() <= floor.timing_error_s);
    assert_eq!(floor.point.impact.elapsed, Duration::from_nanos(64));
    let at = 6.443_437_032_284_175e-8;
    assert!(
        hit.terminal_time_s[0] <= at
            && at <= hit.terminal_time_s[1]
            && hit.terminal_time_s[1] < 65e-9
    );
    assert!(hit.finite.iter().all(|e| e.time_s > floor.point.after_cut_s
        && e.time_s < 65e-9
        && e.dissipated_energy_j >= 0.));
    let expected = [2.8359393880918966, 0.2361003010859354];
    for (k, expected) in expected.into_iter().enumerate() {
        assert!((hit.normal_impulse[k] - expected).abs() <= hit.impulse_radius[k]);
    }
    let external = hit.support_work_j.iter().sum::<f64>() + hit.gravity_work_j;
    assert!((external - hit.kinetic_energy_change_j - hit.dissipated_energy_j).abs() < 1e-11);
    assert!(hit.dissipated_energy_j > hit.energy_error_j && hit.energy_error_j > 0.);
    let integrals = obs.interval.pairs.clone();
    assert!(integrals.iter().all(|p| !p.active_at_end));
    assert_eq!(
        integrals
            .iter()
            .filter(|p| p.first_positive_end == Some(Duration::from_nanos(64)))
            .count(),
        2
    );
    sim.step_exact(1).unwrap();
    if let Some(next) = last_opposing_corner(&sim).filter(|r| r.sim_time == sim.elapsed()) {
        assert!(next.interval.opposing_normal_series.is_empty());
        assert!(next.interval.pairs.iter().all(|p| p.normal_impulse < 1e-11));
    }
}
