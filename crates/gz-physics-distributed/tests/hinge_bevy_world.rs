// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise the public Bevy/SDF surface against an ordinary local solver.
use gz_physics::PhysicsBackend;
use gz_physics_distributed::articulation::{ArticulationObservation, ArticulationSpec};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    JointSample, LocalPose, PhysicsEngine, Velocity, WorldPose,
    distributed_hinge::{
        DistributedHingeBackend, distributed_hinge_spec, load_distributed_hinge_world,
    },
    load_world_with_sdf_step, queue_joint_effort, reset_loaded_world,
};
use std::{error::Error, fs, path::Path};

const SDF: &str = include_str!("../../../examples/multifidelity_articulation.sdf");
const SOURCE: &str = "multifidelity_articulation.sdf";
const WORLD: &str = "default";

fn hosted(dir: &Path) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_distributed_hinge_world(
        SDF,
        SOURCE,
        WORLD,
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        dir,
        None,
    )
}

fn observed(sim: &Simulation) -> ArticulationObservation {
    *sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedHingeBackend>()
        .unwrap()
        .pool()
        .observation()
}

fn compare(host: &Simulation, native: &Simulation, spec: &ArticulationSpec) {
    compare_public(host, native, spec, &observed(host))
}

fn compare_public(
    host: &Simulation,
    native: &Simulation,
    spec: &ArticulationSpec,
    cut: &ArticulationObservation,
) {
    let time = native.committed_update_info().unwrap();
    assert_eq!(host.epoch(), time.epoch);
    assert_eq!(host.elapsed(), time.sim_time);
    assert_eq!(cut.tick, time.iteration);
    let sample_at = |sim: &Simulation| {
        *sim.world()
            .get::<JointSample>(entity_by_id(sim.world(), spec.joint_id).unwrap())
            .unwrap()
    };
    let sample = sample_at(host);
    let expected = sample_at(native);
    assert_eq!(
        (sample.state, sample.time, sample.epoch),
        (expected.state, expected.time, expected.epoch)
    );
    assert_eq!(cut.q.to_bits(), expected.state.position.to_bits());
    assert_eq!(cut.qdot.to_bits(), expected.state.velocity.to_bits());
    for entity in native.world().iter_entities() {
        let Some(id) = entity.get::<EntityId>() else {
            continue;
        };
        let other = host
            .world()
            .entity(entity_by_id(host.world(), id.get()).unwrap());
        if let Some(pose) = entity.get::<WorldPose>() {
            let got = other.get::<WorldPose>().unwrap();
            assert_eq!(
                (got.translation, got.rotation),
                (pose.translation, pose.rotation),
                "world pose for {}",
                id.get()
            );
        }
        if let Some(local) = entity.get::<LocalPose>() {
            let got = other.get::<LocalPose>().unwrap().0;
            assert_eq!(
                (got.translation, got.rotation),
                (local.0.translation, local.0.rotation),
                "local pose for {}",
                id.get()
            );
        }
        if let Some(vel) = entity.get::<Velocity>() {
            let got = other.get::<Velocity>().unwrap();
            assert_eq!((got.linear, got.angular), (vel.linear, vel.angular));
        }
    }
}

#[test]
fn ordinary_ecs_queue_step_transfer_restart_reset_and_reopen() -> Result<(), Box<dyn Error>> {
    let directory = tempfile::tempdir()?;
    let path = directory.path();
    let spec = distributed_hinge_spec(SDF, SOURCE, WORLD)?;
    let mut native = load_world_with_sdf_step(
        SDF,
        SOURCE,
        WORLD,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut host = hosted(path)?;
    assert_eq!(
        gz_sim_scene::remove_model(&mut host, spec.model_id),
        Err(gz_sim_scene::RemoveError::ImmutableModel)
    );
    assert!(gz_sim_scene::set_joint_force(&mut host, spec.joint_id, 3.).is_err());
    compare(&host, &native, &spec);
    for tick in 1..=36 {
        let effort = if tick < 20 { 2.625 } else { -1.875 };
        queue_joint_effort(&mut host, spec.joint_id, effort)?;
        queue_joint_effort(&mut native, spec.joint_id, effort)?;
        match tick {
            1 | 18 => {
                drop(host);
                host = hosted(path)?;
                assert!(host.is_paused());
                compare(&host, &native, &spec);
            }
            7 | 23 => {
                let mut physics = host.world_mut().resource_mut::<PhysicsBackend>();
                let backend = physics.downcast_mut::<DistributedHingeBackend>().unwrap();
                backend.transfer(1 - backend.pool().owner())?;
            }
            14 => {
                let mut physics = host.world_mut().resource_mut::<PhysicsBackend>();
                let backend = physics.downcast_mut::<DistributedHingeBackend>().unwrap();
                backend.replace_worker(backend.pool().owner())?;
            }
            _ => {}
        }
        if tick == 9 {
            let before = observed(&host);
            assert!(host.tick()?.dt.is_zero());
            assert_eq!(observed(&host), before);
        }
        host.step_exact(1)?;
        native.step_exact(1)?;
        compare(&host, &native, &spec);
    }
    assert!(observed(&host).q.abs() > 0.0001);
    queue_joint_effort(&mut host, spec.joint_id, 14.)?;
    reset_loaded_world(&mut host, |_| ())?;
    reset_loaded_world(&mut native, |_| ())?;
    compare(&host, &native, &spec);
    assert_eq!(host.epoch(), 1);
    drop(host);
    host = hosted(path)?;
    compare(&host, &native, &spec);
    for _ in 0..3 {
        host.step_exact(1)?;
        native.step_exact(1)?;
        compare(&host, &native, &spec);
    }
    host.reset()?;
    native.reset()?;
    assert_eq!(host.epoch(), 2);
    compare(&host, &native, &spec);
    Ok(())
}

#[test]
fn failed_reset_publication_preserves_ecs_and_prior_session() -> Result<(), Box<dyn Error>> {
    let directory = tempfile::tempdir()?;
    let path = directory.path();
    let mut sim = hosted(path)?;
    let joint_id = observed(&sim).joint_id;
    queue_joint_effort(&mut sim, joint_id, 4.)?;
    sim.step_exact(2)?;
    let cut = observed(&sim);
    let old_clock = sim.committed_update_info()?;
    let old_entity = entity_by_id(sim.world(), joint_id);
    // Reserve the ledger staging name to force publication failure without
    // disturbing the committed file. The session has to be reopened.
    let stage = path.join("barrier.pending.gzhinge");
    fs::create_dir(&stage)?;
    assert!(reset_loaded_world(&mut sim, |_| panic!("failed reset was published")).is_err());
    assert_eq!(sim.committed_update_info()?, old_clock);
    assert_eq!(old_entity, entity_by_id(sim.world(), joint_id));
    assert_eq!(observed(&sim), cut);
    drop(sim);
    fs::remove_dir(&stage)?;
    let mut sim = hosted(path)?;
    assert_eq!(sim.committed_update_info()?, old_clock);
    assert_eq!(observed(&sim), cut);
    sim.step_exact(1)?;
    Ok(())
}

// On Unix the test can unlink a private name for an executing binary.
#[cfg(unix)]
#[test]
fn worker_restart_failure_preserves_scene_and_durable_cut() -> Result<(), Box<dyn Error>> {
    let stock = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let executables = tempfile::tempdir_in(stock.parent().unwrap())?;
    let worker = executables.path().join("worker");
    fs::hard_link(stock, &worker)?;
    let directory = tempfile::tempdir()?;
    let path = directory.path();
    let mut host = load_distributed_hinge_world(SDF, SOURCE, WORLD, &worker, path, None)?;
    let joint = observed(&host).joint_id;
    queue_joint_effort(&mut host, joint, 1.25)?;
    host.step_exact(3)?;
    queue_joint_effort(&mut host, joint, -0.875)?;
    let prior = observed(&host);
    let entity = entity_by_id(host.world(), joint);
    let clock = host.committed_update_info()?;
    fs::remove_file(&worker)?;
    assert!(reset_loaded_world(&mut host, |_| panic!("worker startup failed")).is_err());
    assert_eq!(host.committed_update_info()?, clock);
    assert_eq!(observed(&host), prior);
    assert_eq!(entity_by_id(host.world(), joint), entity);
    drop(host);
    let mut resumed = hosted(path)?;
    assert_eq!(observed(&resumed), prior);
    let mut native = load_world_with_sdf_step(
        SDF,
        SOURCE,
        WORLD,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    for (effort, steps) in [(1.25, 3), (-0.875, 1)] {
        queue_joint_effort(&mut native, joint, effort)?;
        native.step_exact(steps)?;
    }
    resumed.step_exact(1)?;
    compare(
        &resumed,
        &native,
        &distributed_hinge_spec(SDF, SOURCE, WORLD)?,
    );
    Ok(())
}

const GROUND_SDF: &str = include_str!("../../../examples/distributed_grounded_hinge.sdf");

#[test]
fn grounded_scene_keeps_public_floor_and_loaded_hinge_through_restart_and_reset()
-> Result<(), Box<dyn Error>> {
    use gz_physics_multifidelity::MultiFidelityBackend;
    use gz_sim_scene::BodyHandle;
    let directory = tempfile::tempdir()?;
    let source = "grounded.sdf";
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_hinge_spec(GROUND_SDF, source, WORLD)?;
    let load = |xml: &str| {
        load_distributed_hinge_world(xml, source, WORLD, exe, directory.path(), Some(3))
    };
    let mut native = load_world_with_sdf_step(
        GROUND_SDF,
        source,
        WORLD,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut host = load(GROUND_SDF)?;
    let ground_id = spec.ground.unwrap().link_id;
    let handle = |id| {
        native
            .world()
            .get::<BodyHandle>(entity_by_id(native.world(), id).unwrap())
            .unwrap()
            .id()
    };
    let floor = handle(ground_id);
    let links = spec.links.map(|l| handle(l.id));
    let ground_model = host
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<gz_sim_scene::Name>()?.0 == "default::ground")
                .then(|| e.get::<EntityId>().unwrap().get())
        })
        .unwrap();
    assert_eq!(
        gz_sim_scene::remove_model(&mut host, ground_model),
        Err(gz_sim_scene::RemoveError::ImmutableModel)
    );
    let mut loaded_steps = 0;
    for _ in 0..470 {
        host.step_exact(1)?;
        native.step_exact(1)?;
        compare(&host, &native, &spec);
        let state = observed(&host);
        let solver = native
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let actual = state.ground_reactions.unwrap();
        for (link, measured) in links.into_iter().zip(actual) {
            assert_eq!(solver.ground_reaction(link, floor)?, measured);
        }
        if actual.iter().any(|sample| sample.normal_impulse > 0.) {
            loaded_steps += 1;
            match loaded_steps {
                1 => {
                    queue_joint_effort(&mut host, spec.joint_id, 1.25)?;
                    queue_joint_effort(&mut native, spec.joint_id, 1.25)?;
                    host.world_mut()
                        .resource_mut::<PhysicsBackend>()
                        .downcast_mut::<DistributedHingeBackend>()
                        .unwrap()
                        .transfer(2)?;
                }
                10 => {
                    drop(host);
                    let before = fs::read(directory.path().join("barrier.gzhinge"))?;
                    assert!(load(&GROUND_SDF.replace("-9.81", "-7.25")).is_err());
                    assert_eq!(before, fs::read(directory.path().join("barrier.gzhinge"))?);
                    host = load(GROUND_SDF)?;
                    compare(&host, &native, &spec);
                    assert_eq!(observed(&host), state);
                }
                18 => {
                    host.world_mut()
                        .resource_mut::<PhysicsBackend>()
                        .downcast_mut::<DistributedHingeBackend>()
                        .unwrap()
                        .replace_worker(2)?;
                }
                25 => break,
                _ => {}
            }
        }
    }
    assert!(loaded_steps >= 25, "expected sustained contact");
    reset_loaded_world(&mut host, |_| ())?;
    reset_loaded_world(&mut native, |_| ())?;
    compare(&host, &native, &spec);
    assert!(
        observed(&host)
            .ground_reactions
            .unwrap()
            .into_iter()
            .all(|r| !r.active && r.normal_impulse == 0.)
    );
    drop(host);
    let reopened = load(GROUND_SDF)?;
    compare(&reopened, &native, &spec);
    Ok(())
}

#[test]
fn grounded_sdf_admission_rejects_unrepresented_environments_before_starting_workers() {
    for xml in [
        GROUND_SDF.replace("<static>true", "<static>false"),
        GROUND_SDF.replace("0 0 -9.81", "0.1 0 -9.81"),
        GROUND_SDF.replace("0 0 -9.81", "0 0 0"),
        GROUND_SDF.replace("<normal>0 0 1", "<normal>0 1 0"),
        GROUND_SDF.replace(
            "<link name=\"floor\">",
            "<link name=\"floor\"><pose>0 0 0 0.5 0 0</pose>",
        ),
        GROUND_SDF.replace(
            "</collision>",
            "<surface><friction><ode><mu>0</mu></ode></friction></surface></collision>",
        ),
    ] {
        assert!(distributed_hinge_spec(&xml, "inadmissible.sdf", WORLD).is_err());
    }
}

#[test]
fn moving_hinge_bevy_scene_matches_through_impact_release_and_both_reset_paths()
-> Result<(), Box<dyn Error>> {
    use gz_sim_scene::distributed_hinge::{
        DistributedMovingHingeBackend, distributed_moving_hinge_spec,
    };
    let xml = include_str!("../../../examples/distributed_hinge_impact.sdf");
    let source = "distributed_hinge_impact.sdf";
    let spec = distributed_moving_hinge_spec(xml, source, WORLD)?;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let directory = tempfile::tempdir()?;
    let load = |xml: &str| {
        load_distributed_hinge_world(xml, source, WORLD, exe, directory.path(), Some(3))
    };
    let mut host = load(xml)?;
    let mut native = load_world_with_sdf_step(
        xml,
        source,
        WORLD,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let sample = |sim: &Simulation| {
        let p = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedMovingHingeBackend>()
            .unwrap()
            .pool();
        (p.owners(), *p.observation())
    };
    let ball_model = host
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<gz_sim_scene::Name>()?.0 == "default::ball")
                .then(|| e.get::<EntityId>().unwrap().get())
        })
        .unwrap();
    for model in [spec.hinge.model_id, ball_model] {
        assert_eq!(
            gz_sim_scene::remove_model(&mut host, model),
            Err(gz_sim_scene::RemoveError::ImmutableModel)
        );
    }
    assert!(gz_sim_scene::set_joint_force(&mut host, spec.hinge.joint_id, 1.).is_err());
    let mut impact = false;
    let mut released = false;
    for _ in 0..1600 {
        host.step_exact(1)?;
        native.step_exact(1)?;
        let (owners, state) = sample(&host);
        compare_public(&host, &native, &spec.hinge, &state.hinge);
        if !impact
            && state
                .reactions
                .iter()
                .any(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
        {
            impact = true;
            assert_eq!(owners[0], owners[1]);
            queue_joint_effort(&mut host, spec.hinge.joint_id, 2.25)?;
            queue_joint_effort(&mut native, spec.hinge.joint_id, 2.25)?;
            host.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<DistributedMovingHingeBackend>()
                .unwrap()
                .transfer((owners[0] + 1) % 3)?;
            drop(host);
            assert!(load(&xml.replace("-1.8 0 0", "-1.7 0 0")).is_err());
            host = load(xml)?;
            assert!(host.is_paused());
            assert_eq!(sample(&host).1, state);
            compare_public(&host, &native, &spec.hinge, &state.hinge);
        } else if impact && owners[0] != owners[1] {
            released = true;
            drop(host);
            host = load(xml)?;
            assert_eq!(sample(&host), (owners, state));
            break;
        }
    }
    assert!(impact && released);
    for reconstruct in [true, false] {
        queue_joint_effort(&mut host, spec.hinge.joint_id, -4.)?;
        if reconstruct {
            reset_loaded_world(&mut host, |_| ())?;
            reset_loaded_world(&mut native, |_| ())?;
        } else {
            host.reset()?;
            native.reset()?;
        }
        assert_eq!(sample(&host).0, [0, 1]);
        compare_public(&host, &native, &spec.hinge, &sample(&host).1.hinge);
        host.step_exact(1)?;
        native.step_exact(1)?;
        compare_public(&host, &native, &spec.hinge, &sample(&host).1.hinge);
    }
    Ok(())
}

#[test]
fn moving_hinge_scene_admission_rejects_unsupported_construction() {
    use gz_sim_scene::distributed_hinge::distributed_moving_hinge_spec;
    let xml = include_str!("../../../examples/distributed_hinge_impact.sdf");
    for invalid in [
        xml.replacen("allow_auto_disable>false", "allow_auto_disable>true", 1),
        xml.replace("<iyy>0.009", "<iyy>0.01"),
        xml.replace("<gravity>0 0 0", "<gravity>0 0 -9.8"),
        xml.replace("<linear_velocity>-1.8 0 0", "<linear_velocity>nan 0 0"),
        xml.replace("<max_step_size>0.001", "<max_step_size>0.01"),
        xml.replace(
            "<link name=\"body\">",
            "<link name=\"body\"><kinematic>true</kinematic>",
        ),
    ] {
        assert_ne!(invalid, xml);
        assert!(distributed_moving_hinge_spec(&invalid, "case.sdf", WORLD).is_err());
    }
}

#[test]
fn two_sphere_host_preserves_both_ecs_links_on_reopen_and_reset() -> Result<(), Box<dyn Error>> {
    use gz_sim_scene::distributed_hinge::{
        DistributedMultiSphereBackend, distributed_multi_sphere_spec,
    };
    let xml = include_str!("../../../examples/distributed_hinge_two_spheres.sdf");
    let src = "distributed_hinge_two_spheres.sdf";
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir()?;
    let load =
        |xml: &str, width| load_distributed_hinge_world(xml, src, WORLD, worker, dir.path(), width);
    let spec = distributed_multi_sphere_spec(xml, src, WORLD)?;
    assert_ne!(spec.spheres[0].id, spec.spheres[1].id);
    assert!(distributed_hinge_spec(xml, src, WORLD).is_err());
    assert!(
        gz_sim_scene::distributed_hinge::distributed_moving_hinge_spec(xml, src, WORLD).is_err()
    );
    let mut host = load(xml, None)?;
    let mut native = load_world_with_sdf_step(
        xml,
        src,
        WORLD,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let inspect = |sim: &Simulation| {
        let pool = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedMultiSphereBackend>()
            .unwrap()
            .pool();
        (
            pool.owners(),
            pool.observation().hinge,
            pool.pending_joint_effort(),
        )
    };
    assert_eq!(inspect(&host).0, [0, 1, 2]);
    for name in ["default::robot", "default::ball", "default::ball2"] {
        let id = host
            .world()
            .iter_entities()
            .find_map(|e| {
                (e.get::<gz_sim_scene::Name>()?.0 == name).then_some(e.get::<EntityId>()?.get())
            })
            .unwrap();
        assert_eq!(
            gz_sim_scene::remove_model(&mut host, id),
            Err(gz_sim_scene::RemoveError::ImmutableModel)
        );
    }
    host.step_exact(3)?;
    native.step_exact(3)?;
    let check = |host: &Simulation, native: &Simulation| {
        compare_public(host, native, &spec.hinge, &inspect(host).1)
    };
    check(&host, &native);
    for sim in [&mut host, &mut native] {
        queue_joint_effort(sim, spec.hinge.joint_id, 0.01)?;
    }
    drop(host);
    let archive = dir.path().join("barrier.gzmultisphere");
    let before = fs::read(&archive)?;
    assert!(load(xml, Some(4)).is_err());
    assert!(load(&xml.replacen("<mass>2</mass>", "<mass>2.4</mass>", 1), None).is_err());
    assert!(
        load(
            include_str!("../../../examples/distributed_hinge_impact.sdf"),
            None
        )
        .is_err()
    );
    host = load(xml, None)?;
    assert_eq!(fs::read(&archive)?, before);
    assert_eq!(inspect(&host).2, Some(0.01));
    host.step_exact(2)?;
    native.step_exact(2)?;
    check(&host, &native);
    reset_loaded_world(&mut host, |_| ())?;
    reset_loaded_world(&mut native, |_| ())?;
    check(&host, &native);
    assert_eq!(inspect(&host).0, [0, 1, 2]);
    assert_eq!(inspect(&host).2, None);
    Ok(())
}

#[test]
fn ordinary_two_sphere_world_accepts_construction_outside_worker_profile()
-> Result<(), Box<dyn Error>> {
    use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
    let xml = include_str!("../../../examples/distributed_hinge_two_spheres.sdf");
    for scene in [
        xml.replacen("<ixx>0.009</ixx>", "<ixx>0.010</ixx>", 1),
        xml.replace(
            "</axis>",
            "<limit><lower>-1</lower><upper>1</upper></limit></axis>",
        ),
    ] {
        assert_ne!(scene, xml);
        let mut ordinary = load_world_with_sdf_step(
            &scene,
            "ordinary.sdf",
            WORLD,
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        ordinary.step_exact(4)?;
        assert_eq!(ordinary.committed_update_info()?.iteration, 4);
        assert!(distributed_multi_sphere_spec(&scene, "ordinary.sdf", WORLD).is_err());
    }
    Ok(())
}

#[test]
fn independent_sphere_island_migrates_through_the_bevy_backend() -> Result<(), Box<dyn Error>> {
    use gz_sim_scene::distributed_hinge::{
        DistributedMultiSphereBackend, distributed_multi_sphere_spec,
    };
    let xml = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
    let source = "distributed_sphere_pair_island.sdf";
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let spec = distributed_multi_sphere_spec(xml, source, WORLD)?;
    let open = || load_distributed_hinge_world(xml, source, WORLD, worker, dir.path(), Some(4));
    let mut host = open()?;
    let mut ordinary = load_world_with_sdf_step(
        xml,
        source,
        WORLD,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut transferred = false;
    let mut merged = false;
    for _ in 0..320 {
        for sim in [&mut host, &mut ordinary] {
            queue_joint_effort(sim, spec.hinge.joint_id, 0.01)?;
            sim.step_exact(1)?;
        }
        let pool = host
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedMultiSphereBackend>()
            .unwrap()
            .pool();
        compare_public(&host, &ordinary, &spec.hinge, &pool.observation().hinge);
        let [hinge, a, b] = pool.owners();
        if !transferred
            && a == b
            && a != hinge
            && pool.observation().sphere_reaction.normal_impulse > 0.
        {
            let image = pool.pair_image().unwrap().to_vec();
            let tick = host.elapsed();
            let free = (0..4).find(|slot| !pool.owners().contains(slot)).unwrap();
            host.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<DistributedMultiSphereBackend>()
                .unwrap()
                .transfer_pair(free)?;
            drop(host);
            host = open()?;
            let pool = host
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<DistributedMultiSphereBackend>()
                .unwrap()
                .pool();
            assert_eq!(pool.owners(), [hinge, free, free]);
            assert_eq!(pool.pair_image(), Some(image.as_slice()));
            assert_eq!(host.elapsed(), tick);
            compare_public(&host, &ordinary, &spec.hinge, &pool.observation().hinge);
            transferred = true;
        } else if transferred
            && hinge == a
            && a == b
            && pool.observation().sphere_reaction.normal_impulse > 0.
            && pool
                .observation()
                .reactions
                .iter()
                .flatten()
                .any(|r| r.normal_impulse > 0.)
        {
            merged = true;
        }
    }
    assert!(transferred && merged);
    Ok(())
}
