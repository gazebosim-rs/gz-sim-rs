// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]
use bevy_ecs::prelude::Entity;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, InputError, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    receiving_support::{ReceivingSupport, SupportCriteria, SupportReading},
    reset_loaded_world,
};
use std::time::Duration;
fn load(moving: bool, reversed: bool) -> Simulation {
    load_on(moving, reversed, PhysicsEngine::MultiFidelity)
}
fn load_on(moving: bool, reversed: bool, engine: PhysicsEngine) -> Simulation {
    let pad = r#"<link name="body"><pose>0 0 -0.02 0 0 0</pose><collision name="shape"><geometry><box><size>0.4 0.4 0.04</size></box></geometry></collision></link>"#;
    let motion = r#"<plugin filename="gz-sim-prescribed-motion-system" name="gz::sim::systems::PrescribedMotion"><trajectory end="hold"><waypoint time_ns="0" displacement="0 0 0" velocity="0 0 0"/><waypoint time_ns="300000000" displacement="0 0 0" velocity="0 0 0"/><waypoint time_ns="400000000" displacement="0.02 0 0" velocity="0 0 0"/></trajectory></plugin>"#;
    let shelf = if moving {
        format!(r#"<model name="shelf">{pad}{motion}</model>"#)
    } else {
        format!(r#"<model name="shelf"><static>true</static>{pad}</model>"#)
    };
    let load = r#"<model name="load"><link name="body"><pose>0 0 0.051 0 0 0</pose><inertial><mass>0.2</mass><inertia><ixx>0.00034</ixx><iyy>0.00034</iyy><izz>0.00034</izz></inertia></inertial><collision name="shape"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model>"#;
    let pair = if reversed {
        format!("{load}{shelf}")
    } else {
        format!("{shelf}{load}")
    };
    let xml = format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity><physics name="p" type="ignored"><max_step_size>0.001</max_step_size></physics>{pair}<model name="other"><static>true</static><pose>1 0 0 0 0 0</pose>{pad}</model></world></sdf>"#
    );
    load_world_with_sdf_step(
        &xml,
        "receiving_support.sdf",
        "default",
        &sdformat::NoResources,
        engine,
    )
    .unwrap()
}
fn link(sim: &Simulation, name: &str) -> (u64, Entity) {
    let name = format!("default::{name}::body");
    let row = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (row.get::<EntityId>().unwrap().get(), row.id())
}
fn criteria() -> SupportCriteria {
    SupportCriteria::for_weight(1.962, Duration::from_micros(30_500))
}
fn bind(sim: &Simulation, rcv: &str, criteria: SupportCriteria) -> ReceivingSupport {
    ReceivingSupport::new(sim.world(), link(sim, "load").0, link(sim, rcv).0, criteria).unwrap()
}
fn sample(sim: &mut Simulation, sensor: &mut ReceivingSupport) -> SupportReading {
    sim.step_exact(1).unwrap();
    sensor.sample(sim.world()).unwrap()
}

#[test]
fn static_and_substepped_receivers_deliver_signed_weight_and_require_an_uninterrupted_dwell() {
    for (moving, reversed) in [(false, false), (false, true), (true, false), (true, true)] {
        let mut sim = load(moving, reversed);
        let mut sensor = bind(&sim, "shelf", criteria());
        let mut other = bind(&sim, "other", criteria());
        assert_eq!(
            sensor.sample(sim.world()).unwrap(),
            SupportReading::default()
        );
        sim.step_exact(150).unwrap();
        for i in 1..=31 {
            let r = sample(&mut sim, &mut sensor);
            assert_eq!(r.ready, i == 31, "{moving} {i} {r:?}");
            assert_eq!(r.sustained, Duration::from_millis(i).min(criteria().dwell));
            assert!(
                r.face_loaded && !r.receiver_moving && (r.normal_force_n - 1.962).abs() < 0.001
            );
            assert_eq!(sensor.sample(sim.world()).unwrap(), r);
            assert_eq!(other.sample(sim.world()).unwrap().normal_force_n, 0.);
        }
        let body = |name| {
            sim.world()
                .get::<BodyHandle>(link(&sim, name).1)
                .unwrap()
                .id()
        };
        let (a, b) = (body("load"), body("shelf"));
        let solver = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let x = solver.normal_reaction_on(a, b).unwrap().unwrap();
        let y = solver.normal_reaction_on(b, a).unwrap().unwrap();
        assert_eq!(x.magnitude, y.magnitude);
        assert_eq!(x.impulse, y.impulse.map(|x| -x));
        assert!((x.impulse[2] - 0.001962).abs() < 1e-6 && x.loaded_at_end);
        assert!(solver.normal_reaction_on(a, a).is_err());
        assert!(
            solver
                .normal_reaction_on(BodyId::from_raw(u64::MAX), b)
                .is_err()
        );
        sim.step_exact(2).unwrap();
        assert_eq!(
            sensor.sample(sim.world()).unwrap().sustained,
            Duration::from_millis(1)
        );
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(
            sensor.sample(sim.world()).unwrap(),
            SupportReading::default()
        );
    }
}

#[test]
fn a_translating_receiver_must_park_and_qualify_again() {
    let mut sim = load(true, false);
    let mut sensor = bind(&sim, "shelf", criteria());
    sim.step_exact(250).unwrap();
    for _ in 0..45 {
        sample(&mut sim, &mut sensor);
    }
    assert!(sensor.sample(sim.world()).unwrap().ready);
    let mut loaded_while_moving = 0;
    for _ in 0..105 {
        let r = sample(&mut sim, &mut sensor);
        if r.receiver_moving && r.face_loaded {
            assert!(!r.ready && r.sustained.is_zero());
            loaded_while_moving += 1;
        }
    }
    assert!(loaded_while_moving > 90);
    for _ in 0..35 {
        sample(&mut sim, &mut sensor);
    }
    assert!(sensor.sample(sim.world()).unwrap().ready);
}

#[test]
fn a_wrong_face_speed_pose_jump_or_broken_binding_cannot_authorize_release() {
    let mut sim = load(false, true);
    let foreign = load(false, true);
    let mut sensor = bind(&sim, "shelf", criteria());
    let mut sideways = bind(
        &sim,
        "shelf",
        SupportCriteria {
            up: [2., 0., 0.],
            ..criteria()
        },
    );
    sim.step_exact(130).unwrap();
    for _ in 0..35 {
        sample(&mut sim, &mut sensor);
    }
    assert!(sensor.sample(sim.world()).unwrap().ready);
    assert!(!sideways.sample(sim.world()).unwrap().face_loaded);
    let (_, rcv) = link(&sim, "shelf");
    let (_, load) = link(&sim, "load");
    let initial = *sim.world().get::<WorldPose>(rcv).unwrap();
    sim.world_mut().get_mut::<WorldPose>(rcv).unwrap().rotation = initial.rotation.map(|x| -x);
    assert!(sensor.sample(sim.world()).unwrap().ready);
    *sim.world_mut().get_mut::<WorldPose>(rcv).unwrap() = initial;
    sim.world_mut()
        .get_mut::<WorldPose>(rcv)
        .unwrap()
        .translation[0] += 0.001;
    let moved = sensor.sample(sim.world()).unwrap();
    assert!(moved.receiver_moving && !moved.ready);
    *sim.world_mut().get_mut::<WorldPose>(rcv).unwrap() = initial;
    let moved = sensor.sample(sim.world()).unwrap();
    assert!(moved.receiver_moving);
    sim.world_mut().get_mut::<Velocity>(load).unwrap().linear[2] = 0.2;
    let moving_load = sensor.sample(sim.world()).unwrap();
    assert!(moving_load.sustained.is_zero());
    sim.world_mut().get_mut::<Velocity>(rcv).unwrap().angular[0] = 1.;
    assert!(sensor.sample(sim.world()).unwrap().receiver_moving);
    sim.world_mut().get_mut::<Velocity>(rcv).unwrap().angular[0] = f64::NAN;
    assert!(sensor.sample(sim.world()).is_err());
    assert!(sensor.sample(foreign.world()).is_err());
    sim.world_mut().entity_mut(rcv).remove::<BodyHandle>();
    assert!(sensor.sample(sim.world()).is_err());
    assert!(
        ReceivingSupport::new(
            sim.world(),
            link(&sim, "load").0,
            link(&sim, "shelf").0,
            criteria()
        )
        .is_err()
    );
}

#[test]
fn criteria_and_pairs_are_validated_before_sampling() {
    let sim = load(false, false);
    let (a, b) = (link(&sim, "load").0, link(&sim, "shelf").0);
    assert!(matches!(
        ReceivingSupport::new(sim.world(), a, a, criteria()),
        Err(InputError::InvalidValue)
    ));
    assert!(ReceivingSupport::new(sim.world(), a, u64::MAX, criteria()).is_err());
    let other = load_on(false, false, PhysicsEngine::Rapier);
    assert!(ReceivingSupport::new(other.world(), a, b, criteria()).is_err());
    for x in [
        SupportCriteria {
            up: [0.; 3],
            ..criteria()
        },
        SupportCriteria {
            dwell: Duration::ZERO,
            ..criteria()
        },
        SupportCriteria {
            weight_n: f64::INFINITY,
            ..criteria()
        },
        SupportCriteria {
            weight_n: -1.,
            ..criteria()
        },
        SupportCriteria {
            minimum_alignment: 1.001,
            ..criteria()
        },
        SupportCriteria {
            minimum_sample_fraction: f64::NAN,
            ..criteria()
        },
        SupportCriteria {
            mean_weight_fraction: [0., 1.],
            ..criteria()
        },
        SupportCriteria {
            mean_weight_fraction: [2., 1.],
            ..criteria()
        },
        SupportCriteria {
            minimum_sample_fraction: 2.,
            ..criteria()
        },
        SupportCriteria {
            weight_n: f64::MAX,
            ..criteria()
        },
    ] {
        assert!(matches!(
            ReceivingSupport::new(sim.world(), a, b, x),
            Err(InputError::InvalidValue)
        ));
    }
}

#[test]
fn a_finished_oblique_impact_reports_its_full_vector_without_admitting_resting_support() {
    let ball = |radius: f64| {
        format!(
            "<collision name='ball'><geometry><sphere><radius>{radius}</radius></sphere></geometry><surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>"
        )
    };
    let xml = format!(
        r#"<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ignored'><max_step_size>0.12</max_step_size></physics>
        <model name='load'><link name='body'><pose>0.18 0.045 0 0 0 0</pose><inertial><mass>0.2</mass><inertia><ixx>0.00015</ixx><iyy>0.00015</iyy><izz>0.00015</izz></inertia></inertial>{}</link></model>
        <model name='shelf'><link name='body'>{}</link><plugin filename='gz-sim-prescribed-motion-system' name='gz::sim::systems::PrescribedMotion'><trajectory end='hold'>
        <waypoint time_ns='0' displacement='0 0 0' velocity='0 0 0'/><waypoint time_ns='100000000' displacement='0.20 0 0' velocity='0 0 0'/></trajectory></plugin></model></world></sdf>"#,
        ball(0.043),
        ball(0.075)
    );
    let mut sim = load_world_with_sdf_step(
        &xml,
        "oblique.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let mut detector = bind(
        &sim,
        "shelf",
        SupportCriteria {
            up: [1., 0., 0.],
            ..criteria()
        },
    );
    sim.step_exact(1).unwrap();
    let ids =
        ["load", "shelf"].map(|n| sim.world().get::<BodyHandle>(link(&sim, n).1).unwrap().id());
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    let j = native.normal_reaction_on(ids[0], ids[1]).unwrap().unwrap();
    let v = sim.world().get::<Velocity>(link(&sim, "load").1).unwrap();
    let residual = (glam::DVec3::from(j.impulse) - 0.2 * glam::DVec3::from(v.linear)).length();
    eprintln!(
        "oblique contact: slices={}, impulse={j:?}, momentum residual={residual:e}",
        native.translation_interval().0
    );
    assert_eq!(native.translation_interval().0, 240);
    assert!(j.impulse[0] > 0.1 && j.impulse[1] > 0.05 && residual < 1e-9);
    assert!(j.magnitude >= glam::DVec3::from(j.impulse).length() - 1e-12 && !j.loaded_at_end);
    let r = detector.sample(sim.world()).unwrap();
    assert!(r.normal_force_n > 0. && !r.face_loaded && !r.ready);
}

#[test]
fn a_runtime_spawn_has_its_own_support_binding_and_retirement_invalidates_it() {
    use gz_sim_scene::{prepare_model_template, remove_model, spawn_model_template};
    let xml = r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity><physics name="p" type="ignored"><max_step_size>0.001</max_step_size></physics>
      <model name="source"><pose>1 0 0.1 0 0 0</pose><link name="body"><inertial><mass>0.2</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <collision name="c"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></visual></link></model>
      <model name="shelf"><static>true</static><link name="body"><pose>0 0 -0.02 0 0 0</pose><collision name="deck"><geometry><box><size>0.5 0.5 0.04</size></box></geometry></collision></link></model></world></sdf>"#;
    let parsed = sdformat::load_str(xml, "spawn.sdf", &sdformat::NoResources).unwrap();
    let mut sim = load_world_with_sdf_step(
        xml,
        "spawn.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let model = spawn_model_template(
        &mut sim,
        &template,
        WorldPose {
            translation: [0., 0., 0.051],
            rotation: [0., 0., 0., 1.],
        },
    )
    .unwrap();
    let body = sim
        .world()
        .get::<bevy_ecs::prelude::Children>(gz_sim_core::entity_by_id(sim.world(), model).unwrap())
        .unwrap()
        .iter()
        .find(|e| sim.world().get::<BodyHandle>(**e).is_some())
        .unwrap();
    let payload = sim.world().get::<EntityId>(*body).unwrap().get();
    let mut detector =
        ReceivingSupport::new(sim.world(), payload, link(&sim, "shelf").0, criteria()).unwrap();
    sim.step_exact(150).unwrap();
    for _ in 0..31 {
        sample(&mut sim, &mut detector);
    }
    assert!(detector.sample(sim.world()).unwrap().ready);
    remove_model(&mut sim, model).unwrap();
    assert!(
        matches!(detector.sample(sim.world()),Err(InputError::MissingEntity(id)) if id==payload)
    );
}

#[test]
fn incomplete_observations_forget_the_previous_decision() {
    let mut sim = load(false, false);
    let mut detector = bind(&sim, "shelf", criteria());
    let s = link(&sim, "shelf").1;
    let p = link(&sim, "load").1;
    let pose = sim.world_mut().entity_mut(p).take::<WorldPose>().unwrap();
    assert!(detector.sample(sim.world()).is_err());
    sim.world_mut().entity_mut(p).insert(pose);
    let v = sim.world_mut().entity_mut(s).take::<Velocity>().unwrap();
    assert!(detector.sample(sim.world()).is_err());
    sim.world_mut().entity_mut(s).insert(v);
    sim.world_mut().remove_resource::<gz_sim_core::UpdateInfo>();
    assert!(detector.sample(sim.world()).is_err());
}

#[test]
fn an_adaptive_sorter_reports_the_whole_bounce_without_starting_a_support_dwell() {
    let xml = include_str!("../../../examples/worlds/local_sorter_crossing.sdf")
        .replace("<link name=\"deck\">", "<link name=\"body\">")
        .replace("::deck</support>", "::body</support>")
        .replace(r#"<model name="parcel">"#, r#"<model name="parcel"><plugin name="gz::sim::systems::ContactMaterial" filename="gz-sim-contact-material-system"><restitution link="body">0.8</restitution></plugin>"#);
    let mut sim = load_world_with_sdf_step(
        &xml,
        "sorter.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let red = link(&sim, "parcel").0;
    let pad = link(&sim, "pad").0;
    let bypass = link(&sim, "bypass").0;
    let mut receiving = ReceivingSupport::new(sim.world(), red, pad, criteria()).unwrap();
    let mut opposite = ReceivingSupport::new(sim.world(), pad, red, criteria()).unwrap();
    let mut free = ReceivingSupport::new(sim.world(), bypass, pad, criteria()).unwrap();
    for _ in 0..2 {
        sim.step_exact(1).unwrap();
        let velocity = sim
            .world()
            .get::<Velocity>(link(&sim, "parcel").1)
            .unwrap()
            .linear[2];
        let average = 0.01 * (velocity + 4.) / 0.02;
        let r = receiving.sample(sim.world()).unwrap();
        assert!(velocity > 2. && average > 3.);
        assert!(
            (r.normal_force_n - average).abs() < 1e-9,
            "measured {r:?}, momentum predicts {average}"
        );
        assert!(!r.face_loaded && !r.ready && r.sustained.is_zero());
        assert_eq!(receiving.sample(sim.world()).unwrap(), r);
        assert!((opposite.sample(sim.world()).unwrap().normal_force_n + average).abs() < 1e-9);
        assert_eq!(free.sample(sim.world()).unwrap().normal_force_n, 0.);
        sim.step_exact(1).unwrap();
        assert_eq!(receiving.sample(sim.world()).unwrap().normal_force_n, 0.);
        sim.reset().unwrap();
        assert!(!receiving.sample(sim.world()).unwrap().measurement_available);
    }
}
