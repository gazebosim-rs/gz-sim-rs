// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
const BASE: &str = include_str!("../../../../../examples/worlds/local_serial_cable.sdf");
const PLUGIN: &str = "filename=\"gz-sim-deformables-system\"";
fn clock(n: &str) -> String {
    BASE.replace(PLUGIN, &format!("{PLUGIN} rigid_exchanges=\"{n}\""))
}
fn load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "exchange-test.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn body(sim: &Simulation, id: u64) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
        .id()
}
fn read(sim: &Simulation, id: u64) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(body(sim, id))
        .unwrap()
}

#[test]
fn reject_invalid_exchange_clocks_owners_and_material_resolution() {
    for n in ["0", "257", "-1", "sixteen"] {
        assert!(load(&clock(n)).is_err());
    }
    let no_pads = clock("4")
        .lines()
        .filter(|l| !l.contains("coupling=\"articulated_wrench\""))
        .collect::<Vec<_>>()
        .join("\n");
    assert!(load(&no_pads).is_err());
    assert!(load(&clock("4").replace(PLUGIN, &format!("{PLUGIN} worker_threads=\"1\""))).is_err());
    assert!(
        load(&clock("4").replace("0.001</max_step_size>", "0.000000004</max_step_size>")).is_err()
    );
    for (n, want) in [("1", false), ("3", true), ("256", true)] {
        let mut sim = load(&clock(n)).unwrap();
        assert_eq!(enabled(sim.world()), want);
        if want {
            assert_eq!(
                set_material_resolution(&mut sim, "cable", 1_000_000, 2),
                Err(MaterialResolutionError::InvalidResolution)
            );
        }
    }
}

const PROBE: &str = r#"<model name="probe"><pose>0.20 0 0.56 0 0 0</pose><link name="box">
<inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
<collision name="box"><geometry><box><size>0.04 0.04 0.12</size></box></geometry></collision></link></model>"#;
fn add_probe(mut xml: String) -> String {
    // Both solvers receive the same prescribed public effort. Independently
    // running a 4 kHz screw/wrist feedback controller would change the experiment.
    let mut feedback = roxmltree::Document::parse(&xml)
        .unwrap()
        .descendants()
        .filter(|n| {
            n.has_tag_name("plugin")
                && n.attribute("filename") == Some("gz-sim-joint-position-controller-system")
        })
        .map(|n| n.range())
        .collect::<Vec<_>>();
    for r in feedback.drain(..).rev() {
        xml.replace_range(r, "");
    }
    xml.replace("</world>", &format!("{PROBE}</world>"))
}
#[test]
fn held_forces_initial_impulses_and_real_rigid_reactions_agree_at_every_barrier() {
    let mut outer = load(&add_probe(clock("4"))).unwrap();
    let mut fine = load(&add_probe(
        BASE.replace("0.001</max_step_size>", "0.00025</max_step_size>"),
    ))
    .unwrap();
    let probe = id(&outer, "default::probe::box");
    let bench = id(&outer, "default::stand::post");
    let jaw = id(&outer, "default::gripper::close_left");
    for s in [&mut outer, &mut fine] {
        // A direct native impulse and a queued persistent effort reach different native channels.
        let b = body(s, probe);
        s.world_mut()
            .resource_mut::<PhysicsBackend>()
            .apply_impulse(b, [0.01, 0., 0.], [0., 0., 0.00001])
            .unwrap();
        JointForceCommand::set_batch(s, [(jaw, -2.)]).unwrap();
    }
    let rigid = |s: &Simulation, a, b| {
        s.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .normal_reaction_on(body(s, a), body(s, b))
            .unwrap()
            .unwrap()
    };
    let mut contact = 0.;
    for tick in 0..80 {
        let force = [0.5, -0.2, -1.];
        let torque = [0.00035, 0., 0.0002];
        let sheet = [0., 0., 0.008];
        if tick == 0 {
            queue_link_world_wrench(&mut outer, probe, force, torque).unwrap();
            set_material_force(&mut outer, "cable", sheet).unwrap();
            outer.set_paused(true);
            assert!(outer.tick().unwrap().dt.is_zero());
            assert_eq!(
                bodies(&outer).next().unwrap().1.rigid_material_exchanges(),
                0
            );
        }
        let mut j = DVec3::ZERO;
        let mut normal = 0.;
        let mut end = false;
        for _ in 0..4 {
            if tick == 0 {
                queue_link_world_wrench(&mut fine, probe, force, torque).unwrap();
                set_material_force(&mut fine, "cable", sheet).unwrap();
            }
            fine.step_exact(1).unwrap();
            let r = rigid(&fine, probe, bench);
            j += DVec3::from(r.impulse);
            normal += r.magnitude;
            end = r.loaded_at_end;
        }
        outer.step_exact(1).unwrap();
        assert_eq!(read(&outer, probe), read(&fine, probe));
        assert_eq!(
            bodies(&outer).next().unwrap().1.particles(),
            bodies(&fine).next().unwrap().1.particles()
        );
        let r = rigid(&outer, probe, bench);
        let reverse = rigid(&outer, bench, probe);
        assert!((DVec3::from(r.impulse) - j).length() < 1e-13);
        assert!((DVec3::from(reverse.impulse) + j).length() < 1e-13);
        assert!((r.magnitude - normal).abs() < 1e-13);
        assert_eq!(r.loaded_at_end, end);
        let native = outer
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let (a, b) = (body(&outer, probe), body(&outer, bench));
        let scalar = native
            .rigid_contact_observations()
            .into_iter()
            .find(|p| p.a == a.min(b) && p.b == a.max(b))
            .map_or(0., |p| p.normal_impulse);
        assert!((scalar - normal).abs() < 1e-13);
        contact += normal;
    }
    assert!(contact > 0.5);
    set_material_force(&mut outer, "cable", [2., 0., 0.]).unwrap();
    queue_link_world_wrench(&mut outer, probe, [0., 0., 2.], [0.; 3]).unwrap();
    reset_loaded_world(&mut outer, |_| ()).unwrap();
    let mut fresh = load(&add_probe(clock("4"))).unwrap();
    assert_eq!(
        bodies(&outer).next().unwrap().1.rigid_material_exchanges(),
        0
    );
    for s in [&mut fresh, &mut outer] {
        s.step_exact(3).unwrap();
    }
    assert_eq!(read(&outer, probe), read(&fresh, probe));
    assert_eq!(
        bodies(&outer).next().unwrap().1.particles(),
        bodies(&fresh).next().unwrap().1.particles()
    );
    let retired = body(&outer, probe);
    let mut backend = outer.world_mut().resource_mut::<PhysicsBackend>();
    assert!(backend.remove(retired));
    let rows = backend
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .rigid_contact_observations();
    assert!(rows.iter().all(|r| r.a != retired && r.b != retired));
}

#[test]
fn damaged_public_binding_faults_before_any_new_exchange_sample() {
    let mut sim = load(&clock("3")).unwrap();
    sim.step_exact(1).unwrap();
    let prior = bodies(&sim).next().unwrap().1.particles().to_vec();
    let pad = id(&sim, "default::gripper::left");
    let e = entity_by_id(sim.world(), pad).unwrap();
    sim.world_mut().entity_mut(e).remove::<BodyHandle>();
    assert!(sim.step_exact(1).is_err());
    assert!(sim.is_faulted());
    let sample = bodies(&sim).next().unwrap().1;
    assert_eq!(sample.sim_time(), Duration::from_millis(1));
    assert_eq!(sample.particles(), prior);
    assert!(articulated_pad_wrench(&sim, pad).is_none());
}

#[test]
fn native_exchange_receipts_belong_to_a_single_owner_and_complete_interval() {
    let mut one = load(&clock("3")).unwrap();
    let mut two = load(&clock("3")).unwrap();
    let mut pa = one.world_mut().resource_mut::<PhysicsBackend>();
    let a = pa.downcast_mut::<MultiFidelityBackend>().unwrap();
    let mut pb = two.world_mut().resource_mut::<PhysicsBackend>();
    let b = pb.downcast_mut::<MultiFidelityBackend>().unwrap();
    let h = Duration::from_micros(20);
    assert!(a.exchange_inputs(Duration::ZERO, 2).is_err());
    assert!(a.exchange_inputs(h, 0).is_err());
    let mut first = a.exchange_inputs(h * 2, 2).unwrap();
    assert!(a.repeat_exchange_inputs(&first).is_err());
    assert!(b.observe_exchange(&mut first).is_err());
    assert!(b.finish_exchanges(first).is_err());
    let mut second = a.exchange_inputs(h * 2, 2).unwrap();
    a.step(h).unwrap();
    a.observe_exchange(&mut second).unwrap();
    assert!(a.finish_exchanges(second).is_err());
    let mut last = a.exchange_inputs(h, 1).unwrap();
    a.step(h).unwrap();
    a.observe_exchange(&mut last).unwrap();
    assert!(a.observe_exchange(&mut last).is_err());
    assert!(b.repeat_exchange_inputs(&last).is_err());
    a.finish_exchanges(last).unwrap();
    let mut stale = a.exchange_inputs(h * 2, 2).unwrap();
    a.step(h).unwrap();
    a.observe_exchange(&mut stale).unwrap();
    a.reset().unwrap();
    assert!(a.repeat_exchange_inputs(&stale).is_err());
    assert!(a.observe_exchange(&mut stale).is_err());
}

#[test]
fn resetting_during_a_loaded_grasp_drops_deferred_reactions_and_repeats_pickup() {
    let xml = clock("3");
    let mut sim = load(&xml).unwrap();
    let close = |sim: &mut Simulation| {
        let left = id(sim, "default::gripper::close_left");
        let right = id(sim, "default::gripper::close_right");
        JointForceCommand::set_batch(sim, [(left, -4.2), (right, -4.2)]).unwrap();
        for tick in 1..=700 {
            sim.step_exact(1).unwrap();
            let soft = sim.world().resource::<Deformables>();
            if soft
                .pending_pad_impulses
                .values()
                .filter(|(j, _)| j.length() > 1e-5)
                .count()
                == 2
            {
                return (
                    tick,
                    soft.pending_pad_impulses.clone(),
                    soft.pad_wrenches.clone(),
                    bodies(sim).next().unwrap().1.particles().to_vec(),
                );
            }
        }
        panic!("both pads must carry a real load");
    };
    let loaded = close(&mut sim);
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(
        sim.world().resource::<Deformables>().pending_pad_impulses,
        loaded.1
    );
    let at = sim.elapsed();
    set_material_force(&mut sim, "cable", [0., 5., 2.]).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let soft = sim.world().resource::<Deformables>();
    assert!(
        soft.pending_pad_impulses.is_empty()
            && soft.pad_wrenches.is_empty()
            && soft.pending_accelerations.is_empty()
    );
    let again = close(&mut sim);
    assert_eq!(loaded, again);
    assert_eq!(sim.elapsed(), at);
}

#[test]
fn odd_nanosecond_grid_delivers_the_complete_force_once() {
    let xml = add_probe(clock("3")).replace("0.20 0 0.56", "5 0 2");
    let mut sim = load(&xml).unwrap();
    let probe = id(&sim, "default::probe::box");
    queue_link_world_wrench(&mut sim, probe, [2., 0., 0.], [0.; 3]).unwrap();
    sim.step_exact(1).unwrap();
    let ifirst = read(&sim, probe);
    assert!((ifirst.velocity[0] - 0.002).abs() < 1e-15);
    sim.step_exact(1).unwrap();
    assert_eq!(read(&sim, probe).velocity[0], ifirst.velocity[0]);
    assert_eq!(bodies(&sim).next().unwrap().1.rigid_material_exchanges(), 3);
}

#[test]
fn remote_owner_and_free_two_way_spheres_cannot_share_the_material_grid() {
    let xml = clock("4");
    let remote = load_world_configured(
        &xml,
        "grasp.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            noise_seed: [0; 32],
            step: None,
            distributed_worker: Some(std::path::Path::new("unused-worker")),
        },
    );
    let remote_error = remote
        .err()
        .expect("conflicting solver ownership")
        .to_string();
    let expected = if cfg!(feature = "distributed-physics") {
        "local material ownership"
    } else {
        "explicit physics worker requires"
    };
    assert!(remote_error.contains(expected), "{remote_error}");
    let sphere = PROBE.replace(
        "<box><size>0.04 0.04 0.12</size></box>",
        "<sphere><radius>0.025</radius></sphere>",
    );
    let moving = xml
        .replace("</world>", &format!("{sphere}</world>"))
        .replace(
            "<cable name=\"cable\"",
            "<sphere link=\"default::probe::box\" coupling=\"two_way\"/><cable name=\"cable\"",
        );
    let error = load(&moving).err().unwrap();
    assert!(
        error
            .to_string()
            .contains("articulated pads and optional fixed or linearly translating supports"),
        "{error}"
    );
}

#[test]
fn unresolved_material_step_keeps_the_last_committed_public_measurements() {
    let mut sim = load(&clock("4")).unwrap();
    sim.step_exact(1).unwrap();
    let before = bodies(&sim).next().unwrap().1.particles().to_vec();
    set_material_force(&mut sim, "cable", [0., 0., 1e175]).unwrap();
    assert!(sim.step_exact(1).is_err());
    assert_eq!(bodies(&sim).next().unwrap().1.particles(), before);
    assert!(sim.is_faulted());
}

#[test]
fn absent_or_faulted_material_has_no_integrated_constraint_sample() {
    let empty = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(shell_constraint_impulses(&empty, "sheet").is_none());
    for xml in [BASE.to_owned(), clock("3")] {
        let sim = load(&xml).unwrap();
        assert!(shell_constraint_impulses(&sim, "missing").is_none());
        sim.world()
            .resource::<StepFault>()
            .report(CouplingError::Binding);
        assert!(shell_constraint_impulses(&sim, "cable").is_none());
    }
}
