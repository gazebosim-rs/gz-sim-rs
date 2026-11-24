// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
type Visuals = BTreeMap<String, ([u64; 3], [u64; 4])>;
type Run = (Outcome, Vec<(u128, Visuals)>);

fn measured(sim: &mut Simulation, sdf: &sdformat::Sdf, trial: Trial) -> Result<Run> {
    let route = Passage::bind(sim, sdf)?;
    let mut frames = Vec::new();
    let result = drive(sim, trial, Some(route), WEIGHT, false, |sim| {
        let actual = sim
            .world()
            .iter_entities()
            .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Visual))
            .map(|e| {
                let name = e.get::<Name>().unwrap().0.clone();
                let pose = e.get::<WorldPose>().unwrap();
                (
                    name,
                    (
                        pose.translation.map(f64::to_bits),
                        pose.rotation.map(f64::to_bits),
                    ),
                )
            })
            .collect::<Visuals>();
        let (scene, _) = gz_sim_cli::primitive_frame(sim, sdf, "default", frames.len() as u64 + 1)?;
        let scene: serde_json::Value = serde_json::from_str(&scene)?;
        let poses = scene["visuals"].as_array().unwrap();
        assert_eq!(poses.len(), actual.len());
        for visual in poses {
            let (position, rotation) = &actual[visual["name"].as_str().unwrap()];
            for (observed, expected) in visual["translation"]
                .as_array()
                .unwrap()
                .iter()
                .chain(visual["rotation"].as_array().unwrap())
                .zip(position.iter().chain(rotation))
            {
                assert!((observed.as_f64().unwrap() - f64::from_bits(*expected)).abs() < 1e-12);
            }
        }
        frames.push((sim.elapsed().as_millis(), actual));
        Ok(())
    })?;
    Ok((result, frames))
}

#[test]
fn lifted_and_blocked_routes_repeat_decisions_loads_and_all_visual_frames_after_reset() -> Result<()>
{
    for trial in [
        Trial::ClearObstacle,
        Trial::LowClearance,
        Trial::ContactStop,
    ] {
        let xml = trial.xml();
        let sdf = sdformat::load_str(xml, "route.sdf", &sdformat::NoResources)?;
        let mut sim = load_world_with_sdf_step(
            xml,
            "route.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let first = measured(&mut sim, &sdf, trial)?;
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
        assert_eq!(measured(&mut sim, &sdf, trial)?, first);
        let (out, frames) = first;
        let passage = out.obstacle.as_ref().unwrap();
        assert_eq!(passage.parts.len(), 6);
        let parcel = &passage.parts["default::parcel::box"];
        if trial == Trial::ClearObstacle {
            assert!(passage.first_sweep_hit.is_none() && passage.abort_tick.is_none());
            assert!((0.018..0.020).contains(&parcel.minimum_gap_m));
            assert!(
                passage
                    .parts
                    .values()
                    .all(|r| r.minimum_gap_m >= parcel.minimum_gap_m && r.normal_impulse_ns == 0.)
            );
            assert_eq!(out.parcel_carried_on_both_pads_ticks, out.traverse_ticks);
            assert_eq!(out.dwell_ms_at_release, Some(30));
            assert_eq!(out.shelf_unloaded_ticks_after_release, 0);
            assert!((out.final_shelf_load_n - WEIGHT).abs() < 0.001);
            assert_eq!(out.final_pad_load_n, 0.);
        } else {
            let rejected = passage.first_sweep_hit.as_ref().unwrap();
            assert_eq!(rejected.tick, MOVE_START);
            assert_eq!(rejected.part, "default::gripper::right");
            assert!((0.16..0.18).contains(&rejected.travel_m));
            assert!(out.release_tick.is_none() && out.shelf_mean_at_release_n.is_none());
            assert_eq!(out.final_shelf_load_n, 0.);
            assert_eq!(
                passage.post_abort_ticks,
                STEPS + 1 - passage.abort_tick.unwrap()
            );
            assert_eq!(
                passage.both_pads_supporting_after_abort_ticks,
                passage.post_abort_ticks
            );
            assert!(out.final_parcel_position_m[1].abs() < 0.001);
            if trial == Trial::LowClearance {
                assert_eq!(passage.abort_tick, Some(MOVE_START));
                assert!(
                    !passage.abort_on_measured_contact
                        && passage.first_obstacle_reaction_tick.is_none()
                );
                assert!(passage.peak_carriage_travel_m < 0.001);
                assert!(
                    passage
                        .parts
                        .values()
                        .all(|r| r.minimum_gap_m > 0. && r.normal_impulse_ns == 0.)
                );
            } else {
                assert!(passage.abort_on_measured_contact);
                assert_eq!(passage.abort_tick, passage.first_obstacle_reaction_tick);
                assert!(passage.abort_tick.unwrap() > MOVE_START);
                assert!(passage.peak_carriage_travel_m > 0.16);
                let jaw = &passage.parts["default::gripper::right"];
                assert!(jaw.minimum_gap_m < 0. && jaw.normal_impulse_ns > 0.);
                assert!(jaw.peak_force_vector_n[1] < -60.);
                assert_eq!(parcel.normal_impulse_ns, 0.);
            }
        }
        let event = passage.abort_tick.or(out.release_tick).unwrap() as u128;
        assert!(frames.iter().any(|(ms, _)| *ms == event));
    }
    Ok(())
}

#[test]
fn rejected_route_geometry_and_missing_observations_fail_before_commanding_travel() -> Result<()> {
    let xml = Trial::ClearObstacle.xml();
    let sdf = sdformat::load_str(xml, "route.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        xml,
        "route.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let reject = |bad: &str| {
        let parsed = sdformat::load_str(bad, "route.sdf", &sdformat::NoResources).unwrap();
        Passage::bind(&sim, &parsed).err().unwrap().to_string()
    };
    assert!(reject(&xml.replace("name=\"default\"", "name=\"other\"")).contains("route world"));
    assert!(reject(&xml.replace("name=\"obstacle\"", "name=\"other\"")).contains("missing fixed"));
    assert!(
        reject(&xml.replace(
            "<model name=\"obstacle\"><static>true",
            "<model name=\"obstacle\"><static>false"
        ))
        .contains("fixed obstacle")
    );
    assert!(
        reject(&xml.replace(
            "<box><size>0.26 0.024 0.107</size></box>",
            "<sphere><radius>0.01</radius></sphere>"
        ))
        .contains("cuboid")
    );
    assert!(
        reject(&xml.replace("<collision name=\"bar\">", "<collision name=\"missing\">"))
            .contains("collision")
    );
    assert!(
        reject(
            &xml.replace("<model name=\"parcel\">", "<model name=\"object\">")
                .replace("<model name=\"gripper\">", "<model name=\"tool\">")
        )
        .contains("no moving")
    );
    let wall = Passage::bind(&sim, &sdf)?.wall.collision;
    sim.world_mut().entity_mut(wall).remove::<WorldPose>();
    assert_eq!(
        Passage::bind(&sim, &sdf).err().unwrap().to_string(),
        "missing obstacle-route collision pose"
    );
    Ok(())
}

#[test]
fn incompletely_observed_obstacle_routes_never_issue_the_next_rail_command() -> Result<()> {
    let trial = Trial::ClearObstacle;
    let xml = trial.xml();
    let sdf = sdformat::load_str(xml, "route.sdf", &sdformat::NoResources)?;
    let load = |s: &str| {
        load_world_with_sdf_step(
            s,
            "route.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
    };
    let dest = tempfile::tempdir()?;
    let output = dest.path().join("incomplete");
    assert!(
        record(
            Args {
                trial,
                output: output.clone(),
                world: None,
                travel_ms: 4000,
                pull: None,
            },
            XML
        )
        .is_err()
    );
    assert!(!output.exists());

    let same_surface =
        "<surface><friction><ode><mu>0.85</mu><mu2>0.85</mu2></ode></friction></surface>";
    let extra = format!(
        "<collision name=\"extra\"><pose>0 0 0.01 0 0 0</pose><geometry><box><size>0.01 0.01 0.01</size></box></geometry>{same_surface}</collision><collision name=\"left\">"
    );
    let duplicate = xml.replace("<collision name=\"left\">", &extra);
    let duplicated = load(&duplicate)?;
    let parsed = sdformat::load_str(&duplicate, "route.sdf", &sdformat::NoResources)?;
    assert!(
        Passage::bind(&duplicated, &parsed)
            .err()
            .unwrap()
            .to_string()
            .contains("one contact box")
    );

    let mut sim = load(xml)?;
    let mut route = Passage::bind(&sim, &sdf)?;
    let collision = route.moving[0].collision;
    sim.world_mut().entity_mut(collision).remove::<WorldPose>();
    assert!(Passage::bind(&sim, &sdf).is_err());
    let solver = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    assert!(
        route
            .observe(&sim, solver, 1, Some(0.46), false, 0.001)
            .is_err()
    );

    let mut sim = load(xml)?;
    let route = Passage::bind(&sim, &sdf)?;
    let wall = route.wall.collision;
    sim.add_systems(
        gz_sim_core::Phase::Finalize,
        move |world: &mut bevy_ecs::world::World| {
            world.entity_mut(wall).remove::<WorldPose>();
        },
    );
    assert_eq!(
        drive(&mut sim, trial, Some(route), WEIGHT, false, |_| Ok(()))
            .unwrap_err()
            .to_string(),
        "missing obstacle-route collision pose"
    );
    assert_eq!(sim.elapsed(), Duration::from_millis(1));
    Ok(())
}

#[test]
fn stale_obstacle_body_binding_prevents_advancing_the_controller() -> Result<()> {
    let trial = Trial::ClearObstacle;
    let xml = trial.xml();
    let sdf = sdformat::load_str(xml, "route.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        xml,
        "route.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut route = Passage::bind(&sim, &sdf)?;
    route.wall.body = BodyId::from_raw(u64::MAX);
    assert!(drive(&mut sim, trial, Some(route), WEIGHT, false, |_| Ok(())).is_err());
    assert_eq!(sim.elapsed(), Duration::ZERO);
    Ok(())
}

#[test]
fn return_projection_tests_only_the_remaining_signed_travel() -> Result<()> {
    let xml = Trial::LowClearance
        .xml()
        .replace("<pose>0 0.23 0.5535", "<pose>0 -0.23 0.5535");
    let sdf = sdformat::load_str(&xml, "return.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "return.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    sim.step_exact(1)?;
    let solver = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    let cast = |delta| -> Result<_> {
        let mut route = Passage::bind(&sim, &sdf)?;
        route.observe(&sim, solver, MOVE_START, Some(delta), false, 0.001)?;
        Ok(route.outcome.first_sweep_hit)
    };
    assert!(cast(0.46)?.is_none());
    assert!(cast(-0.05)?.is_none());
    let hit = cast(-0.46)?.expect("obstacle on return to the pickup");
    assert!((0.14..0.23).contains(&hit.travel_m), "{hit:?}");
    Ok(())
}

#[test]
fn missing_return_geometry_refuses_an_unobserved_route() -> Result<()> {
    let xml = Trial::GuardedCarry.xml();
    let sdf = sdformat::load_str(xml, "path.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        xml,
        "path.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let route = Passage::bind(&sim, &sdf)?;
    let face = route.moving[0].collision;
    sim.world_mut().entity_mut(face).remove::<WorldPose>();
    assert!(route.path_clear(&sim, 0.1).is_err());
    sim.world_mut()
        .entity_mut(route.wall.collision)
        .remove::<WorldPose>();
    assert!(route.path_clear(&sim, 0.1).is_err());
    Ok(())
}
