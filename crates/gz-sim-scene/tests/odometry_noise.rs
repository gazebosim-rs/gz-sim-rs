// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, StepError, StepFault};
use gz_sim_scene::{
    GroundTruthOdometryDimensions as Dimensions, GroundTruthOdometryError,
    GroundTruthOdometryNoise as Noise, GroundTruthOdometryPublication as Publication,
    GroundTruthOdometrySample as Sample, GroundTruthOdometryScheduleFault, Name, PhysicsEngine,
    SceneResetError, Velocity, WorldPose, load_world_seeded, queue_link_world_wrench, remove_model,
    reset_loaded_world,
};
use std::time::Duration;
const TWO: &str = include_str!("../../../examples/m34_odometry_publisher.sdf");
const THREE: &str = include_str!("../../../examples/m38_odometry_3d.sdf");
const TRANSLATED: &str = include_str!("../../../examples/m39_odometry_translation_offsets.sdf");
const ROTATED: &str = include_str!("../../../examples/m46_odometry_rotation_offsets.sdf");
fn with_wrench(source: &str) -> String {
    if source.contains("gz-sim-apply-link-wrench-system") {
        source.to_owned()
    } else {
        source.replace("<model name=\"rig\">", "<plugin name=\"gz::sim::systems::ApplyLinkWrench\" filename=\"gz-sim-apply-link-wrench-system\"/><model name=\"rig\">")
    }
}
fn with_noise(source: &str, sigma: &str) -> String {
    source.replace(
        "</plugin>",
        &format!("<gaussian_noise>{sigma}</gaussian_noise></plugin>"),
    )
}
fn load(source: &str, seed: [u8; 32]) -> Simulation {
    load_world_seeded(
        source,
        "m47.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        seed,
    )
    .unwrap()
}
fn entity(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn model(sim: &Simulation) -> bevy_ecs::entity::Entity {
    entity(sim, "default::rig")
}
fn body(sim: &Simulation) -> bevy_ecs::entity::Entity {
    entity(sim, "default::rig::body")
}
fn sample(sim: &Simulation) -> Sample {
    *sim.world().get::<Sample>(model(sim)).unwrap()
}
fn twist(s: Sample) -> ([u64; 3], [u64; 3]) {
    (s.linear.map(f64::to_bits), s.angular.map(f64::to_bits))
}
fn pose(s: Sample) -> ([u64; 3], [u64; 4]) {
    (
        s.pose.translation.map(f64::to_bits),
        s.pose.rotation.map(f64::to_bits),
    )
}
fn physical(sim: &Simulation) -> ([u64; 3], [u64; 4], [u64; 3], [u64; 3]) {
    let pose = sim.world().get::<WorldPose>(model(sim)).unwrap();
    let velocity = sim.world().get::<Velocity>(body(sim)).unwrap();
    (
        pose.translation.map(f64::to_bits),
        pose.rotation.map(f64::to_bits),
        velocity.linear.map(f64::to_bits),
        velocity.angular.map(f64::to_bits),
    )
}
fn drive(sim: &mut Simulation) {
    let id = sim.world().get::<EntityId>(body(sim)).unwrap().get();
    queue_link_world_wrench(sim, id, [4.0, -2.0, 6.0], [2.0, 4.0, 6.0]).unwrap();
    sim.step_exact(20).unwrap();
    let velocity = sim.world().get::<Velocity>(body(sim)).unwrap();
    assert!(velocity.linear.into_iter().any(|v| v.abs() > 0.0001));
    assert!(velocity.angular.into_iter().any(|v| v.abs() > 0.0001));
}
#[test]
fn all_five_exact_prior_noise_refusals_load_move_and_replay_healthy_reset() {
    // The private configured() case retains the exact original M34 model and
    // raw plugin contents (sigma1, default Two, no other plugin fields).
    let start = TWO.find("      <plugin").unwrap();
    let end = TWO[start..].find("</plugin>").unwrap() + start + "</plugin>".len();
    let private = format!(
        "{}<plugin name='gz::sim::systems::OdometryPublisher' filename='gz-sim-odometry-publisher-system'><gaussian_noise>1</gaussian_noise></plugin>{}",
        &TWO[..start],
        &TWO[end..]
    );
    let cases = [
        (private, 1.0, Dimensions::Two),
        (
            TWO.replace(
                "<dimensions>2</dimensions>",
                "<dimensions>2</dimensions><gaussian_noise>0.01</gaussian_noise>",
            ),
            0.01,
            Dimensions::Two,
        ),
        (
            THREE.replace(
                "<dimensions>3</dimensions>",
                "<dimensions>3</dimensions><gaussian_noise>0.01</gaussian_noise>",
            ),
            0.01,
            Dimensions::Three,
        ),
        (
            TRANSLATED.replace(
                "<dimensions>3</dimensions>",
                "<dimensions>3</dimensions><gaussian_noise>0.01</gaussian_noise>",
            ),
            0.01,
            Dimensions::Three,
        ),
        (
            ROTATED.replace(
                "<dimensions>3</dimensions>",
                "<dimensions>3</dimensions><gaussian_noise>0.01</gaussian_noise>",
            ),
            0.01,
            Dimensions::Three,
        ),
    ];
    for (source, sigma, dimensions) in cases {
        let source = with_wrench(&source);
        let quiet_source = source.replace(&format!("<gaussian_noise>{sigma}</gaussian_noise>"), "");
        let mut noisy = load(&source, [47; 32]);
        let mut quiet = load(&quiet_source, [47; 32]);
        let owner = model(&noisy);
        let id = noisy.world().get::<EntityId>(owner).unwrap().get();
        let metadata = noisy.world().get::<Publication>(owner).unwrap().clone();
        assert_eq!(metadata.dimensions, dimensions);
        assert_eq!(metadata.gaussian_noise, Noise::from_stddev(sigma).unwrap());
        assert_eq!(
            metadata.gaussian_noise.variance().to_bits(),
            ((sigma * sigma) as f32).to_bits()
        );
        noisy.step_exact(20).unwrap();
        quiet.step_exact(20).unwrap();
        let initial = sample(&noisy);
        assert_eq!(pose(initial), pose(sample(&quiet)));
        assert_ne!(twist(initial), twist(sample(&quiet)));
        assert!(
            initial
                .linear
                .into_iter()
                .chain(initial.angular)
                .all(f64::is_finite)
        );
        drive(&mut noisy);
        drive(&mut quiet);
        assert_eq!(physical(&noisy), physical(&quiet));
        assert_eq!(pose(sample(&noisy)), pose(sample(&quiet)));
        assert_ne!(twist(sample(&noisy)), twist(sample(&quiet)));
        reset_loaded_world(&mut noisy, |_| ()).unwrap();
        let owner = model(&noisy);
        assert_eq!(noisy.world().get::<EntityId>(owner).unwrap().get(), id);
        assert_eq!(noisy.world().get::<Publication>(owner).unwrap(), &metadata);
        assert!(noisy.world().get::<Sample>(owner).is_none());
        noisy.step_exact(20).unwrap();
        assert_eq!(pose(sample(&noisy)), pose(initial));
        assert_eq!(twist(sample(&noisy)), twist(initial));
        assert_eq!(
            (sample(&noisy).epoch, sample(&noisy).revision),
            (1, initial.revision)
        );
    }
}
#[test]
fn same_seed_replays_and_other_seed_diverges_without_perturbing_body_or_pose() {
    for source in [with_wrench(TWO), with_wrench(ROTATED)] {
        let source = with_noise(&source, "0.1");
        let mut a = load(&source, [47; 32]);
        let mut b = load(&source, [47; 32]);
        let mut c = load(&source, [48; 32]);
        a.step_exact(20).unwrap();
        b.step_exact(20).unwrap();
        c.step_exact(20).unwrap();
        assert_eq!(twist(sample(&a)), twist(sample(&b)));
        assert_ne!(twist(sample(&a)), twist(sample(&c)));
        drive(&mut a);
        drive(&mut b);
        drive(&mut c);
        assert_eq!(physical(&a), physical(&b));
        assert_eq!(physical(&a), physical(&c));
        assert_eq!(pose(sample(&a)), pose(sample(&c)));
        assert_eq!(twist(sample(&a)), twist(sample(&b)));
        assert_ne!(twist(sample(&a)), twist(sample(&c)));
    }
}
#[test]
fn zero_tiny_noise_and_omission_keep_exact_pose_twist_and_no_draw_domain() {
    for source in [with_wrench(TWO), with_wrench(ROTATED)] {
        for sigma in ["0", "-0", "0.0000000000000002220446049250313"] {
            let mut quiet = load(&source, [47; 32]);
            let mut tiny = load(&with_noise(&source, sigma), [48; 32]);
            quiet.step_exact(20).unwrap();
            tiny.step_exact(20).unwrap();
            assert_eq!(pose(sample(&quiet)), pose(sample(&tiny)));
            assert_eq!(twist(sample(&quiet)), twist(sample(&tiny)));
            drive(&mut quiet);
            drive(&mut tiny);
            assert_eq!(physical(&quiet), physical(&tiny));
            assert_eq!(pose(sample(&quiet)), pose(sample(&tiny)));
            assert_eq!(twist(sample(&quiet)), twist(sample(&tiny)));
            let expected = Noise::from_stddev(sigma.parse().unwrap()).unwrap();
            assert_eq!(
                tiny.world()
                    .get::<Publication>(model(&tiny))
                    .unwrap()
                    .gaussian_noise,
                expected
            );
        }
    }
}
#[test]
fn pause_and_publication_throttle_preserve_seeded_history_before_delivery() {
    let source = with_noise(&with_wrench(ROTATED), "0.1");
    let mut limited = load(&source, [47; 32]);
    let mut every = load(
        &source.replace(
            "<odom_publish_frequency>50</odom_publish_frequency>",
            "<odom_publish_frequency>0</odom_publish_frequency>",
        ),
        [47; 32],
    );
    limited.step_exact(20).unwrap();
    every.step_exact(20).unwrap();
    assert_eq!(twist(sample(&limited)), twist(sample(&every)));
    limited.set_paused(true);
    every.set_paused(true);
    let held = sample(&limited);
    for _ in 0..3 {
        limited.tick().unwrap();
        every.tick().unwrap();
    }
    assert_eq!(twist(sample(&limited)), twist(held));
    assert_eq!(
        (sample(&limited).time, sample(&limited).revision),
        (held.time, held.revision)
    );
    limited.set_paused(false);
    every.set_paused(false);
    drive(&mut limited);
    drive(&mut every);
    assert_eq!(twist(sample(&limited)), twist(sample(&every)));
    assert_eq!(pose(sample(&limited)), pose(sample(&every)));
    assert_ne!(sample(&limited).revision, sample(&every).revision);
}
#[test]
fn sigma_bit_identity_faults_before_stale_sample_and_terminal_reset_refuses() {
    let mut sim = load(&with_noise(&with_wrench(ROTATED), "0.1"), [47; 32]);
    sim.step_exact(20).unwrap();
    let owner = model(&sim);
    let id = sim.world().get::<EntityId>(owner).unwrap().get();
    sim.world_mut()
        .get_mut::<Publication>(owner)
        .unwrap()
        .gaussian_noise = Noise::from_stddev(f64::from_bits(0.1_f64.to_bits() + 1)).unwrap();
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    assert!(sim.world().get::<Sample>(owner).is_none());
    let fault = sim
        .world()
        .resource::<StepFault>()
        .error()
        .unwrap()
        .downcast_ref::<GroundTruthOdometryScheduleFault>()
        .unwrap();
    assert_eq!(
        (fault.entity, fault.source),
        (id, GroundTruthOdometryError::Identity)
    );
    let metadata = sim.world().get::<Publication>(owner).unwrap().clone();
    let clock = (sim.epoch(), sim.elapsed());
    let mut called = false;
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| called = true),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert!(!called);
    assert_eq!((sim.epoch(), sim.elapsed()), clock);
    assert_eq!(sim.world().get::<EntityId>(owner).unwrap().get(), id);
    assert_eq!(sim.world().get::<Publication>(owner).unwrap(), &metadata);
    assert!(sim.world().get::<Sample>(owner).is_none());
}
#[test]
fn removal_and_loaded_reconstruction_restore_initial_seed_and_public_ids() {
    let mut sim = load(&with_noise(&with_wrench(ROTATED), "0.1"), [47; 32]);
    sim.step_exact(20).unwrap();
    let owner = model(&sim);
    let id = sim.world().get::<EntityId>(owner).unwrap().get();
    let link = sim.world().get::<EntityId>(body(&sim)).unwrap().get();
    let metadata = sim.world().get::<Publication>(owner).unwrap().clone();
    let initial = sample(&sim);
    queue_link_world_wrench(&mut sim, link, [1.0; 3], [2.0; 3]).unwrap();
    remove_model(&mut sim, id).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<Publication>() || e.contains::<Sample>())
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let owner = model(&sim);
    assert_eq!(sim.world().get::<EntityId>(owner).unwrap().get(), id);
    assert_eq!(sim.world().get::<EntityId>(body(&sim)).unwrap().get(), link);
    assert_eq!(sim.world().get::<Publication>(owner).unwrap(), &metadata);
    assert!(sim.world().get::<Sample>(owner).is_none());
    sim.step_exact(20).unwrap();
    assert_eq!(pose(sample(&sim)), pose(initial));
    assert_eq!(twist(sample(&sim)), twist(initial));
}
#[test]
fn invalid_noise_is_refused_without_expanding_other_domains() {
    for bad in [
        "-0.01",
        "NaN",
        "inf",
        "-inf",
        "1e200",
        "1e30",
        "bad",
        "",
        "0 1",
        "<nested/>",
        "<![CDATA[0.1]]>",
    ] {
        let source = with_noise(ROTATED, bad);
        assert!(
            load_world_seeded(
                &source,
                "bad.sdf",
                "default",
                Some(Duration::from_millis(1)),
                &sdformat::NoResources,
                PhysicsEngine::Rapier,
                [47; 32]
            )
            .is_err(),
            "{bad}"
        );
    }
    for extra in [
        "<gaussian_noise>0.1</gaussian_noise><gaussian_noise>0.1</gaussian_noise>",
        "<gaussian_noise value='1'>0.1</gaussian_noise>",
    ] {
        let source = ROTATED.replace("</plugin>", &format!("{extra}</plugin>"));
        assert!(
            load_world_seeded(
                &source,
                "bad.sdf",
                "default",
                Some(Duration::from_millis(1)),
                &sdformat::NoResources,
                PhysicsEngine::Rapier,
                [47; 32]
            )
            .is_err()
        );
    }
    assert!(
        load_world_seeded(
            &with_noise(ROTATED, "0.1"),
            "bad.sdf",
            "default",
            Some(Duration::from_millis(1)),
            &sdformat::NoResources,
            PhysicsEngine::Tpe,
            [47; 32]
        )
        .is_err()
    );
}
