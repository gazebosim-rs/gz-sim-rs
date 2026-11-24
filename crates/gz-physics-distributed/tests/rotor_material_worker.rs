// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/rotating_pad.rs"]
mod fixture;
use fixture::{CABLE, Rotor};
use gz_physics_distributed::floating_tree::{AutomaticRotorMaterialSpec, RotorMaterialPool};
use gz_sim_scene::deformable::articulated_impact::distributed_rotor_material_contact_spec;
use std::{error::Error, path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[test]
fn a_world_supported_rotor_transfers_the_actual_angular_impulse_over_recovery() -> Result<()> {
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    for public in ["0.4", "0.027", "0.007"] {
        let xml = CABLE.replace("0.4</max_step_size>", &format!("{public}</max_step_size>"));
        let r = distributed_rotor_material_contact_spec(&xml, "rotor.sdf", "default")?;
        let mut ordinary = Rotor::new(&xml)?;
        let s = AutomaticRotorMaterialSpec {
            pad: r.pad,
            material_name: r.material_name,
            step_ns: r.step.as_nanos() as u64,
            restitution: r.restitution,
        };
        let d = tempfile::tempdir()?;
        let mut p = RotorMaterialPool::create(exe, d.path(), 3, s, r.material.clone())?;
        let mut prior = None;
        let mut hit = false;
        let mut recovered = false;
        for t in 0..(500_000_000u64 / p.spec().step_ns + 4) {
            if t == 1 || t == 17 || (hit && !recovered) {
                p.transfer((p.owner() + 1) % p.width())?;
                let pid = p.worker_pid(p.owner()).unwrap();
                assert!(
                    Command::new("kill")
                        .args(["-KILL", &pid.to_string()])
                        .status()?
                        .success()
                );
                if hit {
                    recovered = true;
                }
            }
            let save = std::env::var_os("GZ_ROTOR_WITNESS")
                .map(std::path::PathBuf::from)
                .filter(|_| public == "0.4" && t == 0);
            if let Some(dir) = &save {
                std::fs::create_dir_all(dir)?;
                std::fs::copy(
                    d.path().join("barrier.gzrotmaterial"),
                    dir.join("prior.gzrotmaterial"),
                )?;
            }
            ordinary.advance(1)?;
            p.advance_at(0, t)?;
            if let Some(dir) = &save {
                std::fs::copy(
                    d.path().join("barrier.gzrotmaterial"),
                    dir.join("accepted.gzrotmaterial"),
                )?;
            }
            let current = ordinary.state();
            let b = &p.observation().links[0];
            assert_eq!(
                b.position.map(f64::to_bits),
                current.position.map(f64::to_bits),
                "ordinary position at {public}:{t}"
            );
            assert_eq!(
                b.orientation.map(f64::to_bits),
                current.orientation.map(f64::to_bits)
            );
            assert_eq!(
                b.velocity.map(f64::to_bits),
                current.velocity.map(f64::to_bits)
            );
            assert_eq!(
                b.angular_velocity.map(f64::to_bits),
                current.angular_velocity.map(f64::to_bits)
            );
            for (a, b) in ordinary.material().particles().iter().zip(p.particles()) {
                assert_eq!(a.position.map(f64::to_bits), b.position.map(f64::to_bits));
                assert_eq!(a.velocity.map(f64::to_bits), b.velocity.map(f64::to_bits));
            }
            if !hit && let Some(event) = p.last_impact() {
                let measured = event.input_tick * p.spec().step_ns + event.head_ns;
                assert!((measured as i64 - 177770455).abs() <= 2);
                let z = event.contact.loads[0].on_link;
                println!(
                    "dt={public} absolute_impact_ns={measured} native_spin={} before_impulse={:?} signed_native_Ns={z:?}",
                    b.angular_velocity[2], event.contact.basis.velocities
                );
                let reference = ordinary.material().latest_articulated_impact().unwrap();
                assert_eq!(
                    event.contact.loads[0].on_link.map(f64::to_bits),
                    reference.material.loads[0].on_rigid.map(f64::to_bits)
                );
                let image = p.native_image().unwrap().to_vec();
                let soft = p.material_image().to_vec();
                drop(p);
                p = RotorMaterialPool::resume(exe, d.path(), r.material.clone())?;
                assert_eq!(p.native_image(), Some(&image[..]));
                assert_eq!(p.material_image(), soft);
                prior = Some(measured);
                hit = true;
            }
        }
        assert!(hit && recovered, "actual transfer and restore after impact");
        p.reset()?;
        assert_eq!((p.epoch(), p.tick()), (1, 0));
        assert!(p.last_impact().is_none());
        for _ in 0..(prior.unwrap() / p.spec().step_ns + 1) {
            p.advance()?;
        }
        let q = p.last_impact().unwrap();
        assert_eq!(q.head_ns + q.input_tick * p.spec().step_ns, prior.unwrap());
    }
    Ok(())
}

#[test]
fn native_rotor_images_bind_duration_construction_and_independent_body_motion() -> Result<()> {
    use gz_physics::PhysicsBackend;
    use gz_physics_multifidelity::MultiFidelityBackend;
    use sha2::{Digest, Sha256};
    use std::time::Duration;
    let mut sim = Rotor::new(CABLE)?;
    sim.advance(1)?;
    let event = sim.material().latest_articulated_impact().unwrap();
    let last = Duration::from_millis(400) - event.time;
    let image = sim
        .sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .checkpoint_timed_world_revolute(last)?;
    assert_eq!(&image[..8], b"GZRTOI01");
    let mut destination = Rotor::new(CABLE)?;
    let mut b = destination.sim.world_mut().resource_mut::<PhysicsBackend>();
    let be = b.downcast_mut::<MultiFidelityBackend>().unwrap();
    assert!(be.restore_timed_revolute_tree(&image).is_err());
    assert_eq!(be.restore_timed_world_revolute(&image)?, last);
    for offset in [40, 72, 80, 88, 96, image.len() - 8] {
        let mut bad = image.clone();
        bad[offset + 5] ^= 0x04;
        let digest = Sha256::digest(&bad[40..]);
        bad[8..40].copy_from_slice(&digest);
        assert!(
            be.restore_timed_world_revolute(&bad).is_err(),
            "accepted corrupt coordinate or solver witness at {offset}"
        );
        assert_eq!(be.checkpoint_timed_world_revolute(last)?, image);
    }
    for xml in [
        CABLE.replace("<mass>1.2", "<mass>1.3"),
        CABLE.replace("<radius>0.035", "<radius>0.036"),
        CABLE.replace("<upper>100", "<upper>90"),
    ] {
        let mut world = Rotor::new(&xml)?;
        let mut backend = world.sim.world_mut().resource_mut::<PhysicsBackend>();
        assert!(
            backend
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .restore_timed_world_revolute(&image)
                .is_err()
        );
    }
    Ok(())
}

#[test]
fn opposite_signed_rotors_hold_the_unwrapped_coordinate_and_countertorque_across_a_full_turn()
-> Result<()> {
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let mut impulses = vec![];
    for sign in [1., -1.] {
        let mut xml = CABLE.replace("0.4</max_step_size>", "0.013</max_step_size>");
        if sign < 0. {
            xml = xml
                .replace("0 1.8 0</linear_velocity>", "0 -1.8 0</linear_velocity>")
                .replace("0 0 4</angular_velocity>", "0 0 -4</angular_velocity>")
                .replace("0.32281024090478527 ", "-0.32281024090478527 ");
        }
        let r = distributed_rotor_material_contact_spec(&xml, "rotor.sdf", "default")?;
        let mut ordinary = Rotor::new(&xml)?;
        let s = AutomaticRotorMaterialSpec {
            pad: r.pad,
            material_name: r.material_name,
            step_ns: r.step.as_nanos() as u64,
            restitution: r.restitution,
        };
        let d = tempfile::tempdir()?;
        let mut p = RotorMaterialPool::create(exe, d.path(), 3, s, r.material.clone())?;
        let mut wound = false;
        for t in 0..250 {
            ordinary.advance(1)?;
            p.advance_at(0, t)?;
            let b = &p.observation().links[0];
            let o = ordinary.state();
            assert_eq!(b.position.map(f64::to_bits), o.position.map(f64::to_bits));
            assert_eq!(b.velocity.map(f64::to_bits), o.velocity.map(f64::to_bits));
            assert_eq!(
                b.orientation.map(f64::to_bits),
                o.orientation.map(f64::to_bits)
            );
            assert_eq!(
                b.angular_velocity.map(f64::to_bits),
                o.angular_velocity.map(f64::to_bits)
            );
            if t == 13 {
                let event = p.last_impact().unwrap();
                assert_eq!(event.input_tick * 13_000_000 + event.head_ns, 177_770_455);
                let load = &event.contact.loads[0];
                let lever = glam::DVec3::from(load.point) - glam::DVec3::from(p.spec().pad.anchor);
                let axial = lever.cross(load.on_link.into()).z;
                assert!((axial + sign * 0.387343187933).abs() < 1e-9);
                impulses.push(axial);
            }
            if t == 11
                || t == 20
                || (!wound && p.observation().joints[0].q.abs() > std::f64::consts::TAU)
            {
                wound |= p.observation().joints[0].q.abs() > std::f64::consts::TAU;
                p.transfer((p.owner() + 1) % 3)?;
                drop(p);
                p = RotorMaterialPool::resume(exe, d.path(), r.material.clone())?;
            }
        }
        assert!(wound);
        eprintln!(
            "initial_spin={} final_unwrapped_rad={:.9} physical_intervals=250 moment_Nms={:.12}",
            sign * 4.,
            p.observation().joints[0].q,
            impulses.last().unwrap()
        );
    }
    assert!((impulses[0] + impulses[1]).abs() < 1e-10);
    Ok(())
}

#[test]
fn stock_bevy_sdf_selects_the_managed_rotor_and_republishes_the_ordinary_event() -> Result<()> {
    use gz_physics::PhysicsBackend;
    use gz_sim_scene::{
        PhysicsEngine, WorldLoadOptions,
        deformable::{
            bodies,
            managed_impact::{ManagedAutomaticMaterial, transfer},
        },
        distributed_hinge::load_distributed_articulation_world_with_step,
        load_world_configured,
    };
    use std::{fs, time::Duration};
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let xml = CABLE.replace("0.4</max_step_size>", "0.007</max_step_size>");
    let dir = tempfile::tempdir()?;
    let mut ordinary = Rotor::new(&xml)?;
    let mut transient = load_world_configured(
        &xml,
        "source.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(exe),
        },
    )?;
    let mut durable = load_distributed_articulation_world_with_step(
        &xml,
        "source.sdf",
        "default",
        exe,
        dir.path(),
        Some(3),
        Some(Duration::from_millis(7)),
    )?;
    let barrier = dir.path().join("barrier.gzrotmaterial");
    assert!(barrier.exists());
    for t in 0..70u64 {
        if [24, 25, 26, 47].contains(&t) {
            for sim in [&mut transient, &mut durable] {
                let owner = sim.world().resource::<ManagedAutomaticMaterial>();
                let slot = owner.owner();
                let pid = owner.worker_pid(slot).unwrap();
                assert!(
                    Command::new("kill")
                        .args(["-KILL", &pid.to_string()])
                        .status()?
                        .success()
                );
                transfer(sim, 0, t, if slot == 0 { 1 } else { 0 })?;
            }
            let prior = fs::read(&barrier)?;
            drop(durable);
            durable = load_distributed_articulation_world_with_step(
                &xml,
                "source.sdf",
                "default",
                exe,
                dir.path(),
                None,
                None,
            )?;
            assert_eq!(prior, fs::read(&barrier)?);
        }
        ordinary.advance(1)?;
        transient.step_exact(1)?;
        durable.step_exact(1)?;
        for sim in [&transient, &durable] {
            let b = sim
                .world()
                .resource::<PhysicsBackend>()
                .state(ordinary.body)
                .unwrap();
            let q = ordinary.state();
            assert_eq!(
                b.position.map(f64::to_bits),
                q.position.map(f64::to_bits),
                "stock Cartesian t={t}"
            );
            assert_eq!(
                b.angular_velocity.map(f64::to_bits),
                q.angular_velocity.map(f64::to_bits)
            );
            let soft = bodies(sim).next().unwrap().1;
            for (n, o) in soft.particles().iter().zip(ordinary.material().particles()) {
                assert_eq!(n.position.map(f64::to_bits), o.position.map(f64::to_bits));
                assert_eq!(n.velocity.map(f64::to_bits), o.velocity.map(f64::to_bits));
            }
            assert_eq!(
                soft.articulated_impact_count(),
                ordinary.material().articulated_impact_count()
            );
            assert_eq!(
                soft.impacts_this_step().len(),
                ordinary.material().impacts_this_step().len()
            );
            if let Some(h) = soft.latest_articulated_impact() {
                let r = ordinary.material().latest_articulated_impact().unwrap();
                assert_eq!(h.time, r.time);
                assert_eq!(
                    h.material.loads[0].on_rigid.map(f64::to_bits),
                    r.material.loads[0].on_rigid.map(f64::to_bits)
                );
                assert_eq!(
                    h.native.kinetic_energy_change.to_bits(),
                    r.native.kinetic_energy_change.to_bits()
                );
            }
        }
    }
    transient.reset()?;
    durable.reset()?;
    for sim in [&transient, &durable] {
        assert_eq!(
            sim.world()
                .resource::<ManagedAutomaticMaterial>()
                .interval(),
            (1, 0)
        );
    }
    Ok(())
}

#[test]
fn stock_visuals_follow_the_rotor_and_both_material_nodes_after_reopen() -> Result<()> {
    use gz_sim_core::EntityId;
    use gz_sim_scene::{
        Name, VisualGeometry, WorldPose,
        distributed_hinge::load_distributed_articulation_world_with_step as open,
    };
    use std::io::Write;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let xml = include_str!("../../../examples/distributed_rotating_pad_cable.sdf");
    let dir = tempfile::tempdir()?;
    let mut managed = open(xml, "rotor.sdf", "default", exe, dir.path(), Some(3), None)?;
    let mut control = Rotor::new(xml)?;
    let mut output = std::env::var_os("GZ_ROTOR_LIVE_REFERENCE")
        .map(std::fs::File::create)
        .transpose()?
        .map(std::io::BufWriter::new);
    let mut max_geometry_residual = 0.0_f64;
    for tick in 0..=180 {
        if [25, 26, 63].contains(&tick) {
            drop(managed);
            managed = open(xml, "rotor.sdf", "default", exe, dir.path(), None, None)?;
        }
        let mut visuals = vec![];
        let mut nodes = 0;
        for e in control
            .sim
            .world()
            .iter_entities()
            .filter(|e| e.contains::<VisualGeometry>())
        {
            let id = e.get::<EntityId>().unwrap();
            let name = &e.get::<Name>().unwrap().0;
            let pose = e.get::<WorldPose>().unwrap();
            let other = managed
                .world()
                .iter_entities()
                .find(|entity| {
                    entity
                        .get::<EntityId>()
                        .is_some_and(|candidate| candidate.get() == id.get())
                })
                .and_then(|entity| entity.get::<WorldPose>())
                .unwrap();
            assert_eq!(
                (pose.translation, pose.rotation),
                (other.translation, other.rotation),
                "visual {name}, tick {tick}"
            );
            if let Some((_, part)) = name.rsplit_once("cable_node_") {
                nodes += 1;
                let target = control.material().particles()[part.parse::<usize>()?].position;
                for (a, b) in pose.translation.into_iter().zip(target) {
                    max_geometry_residual = max_geometry_residual.max((a - b).abs());
                }
            }
            visuals.push(serde_json::json!({"id":id.get(),"name":name,"position":pose.translation,"orientation":pose.rotation}));
        }
        assert_eq!((nodes, visuals.len()), (2, 4));
        if let Some(w) = &mut output {
            writeln!(
                w,
                "{}",
                serde_json::json!({"tick":tick*7,"visuals":visuals})
            )?;
        }
        if tick < 180 {
            control.advance(1)?;
            managed.step_exact(1)?;
        }
    }
    assert!(max_geometry_residual < 3e-14, "{max_geometry_residual}");
    eprintln!(
        "visual_frames=181 independently_checked_transforms=724 actual_material_nodes=362 max_geometry_error_m={max_geometry_residual} disk_reopens=3"
    );
    Ok(())
}

/// A rigid change of world frame leaves the contact time and axial impulse invariant.
#[test]
fn tilted_translated_world_axes_preserve_the_signed_impact_and_recovered_orbit() -> Result<()> {
    use glam::{DQuat, DVec3};
    fn vector(v: DVec3) -> String {
        format!("{} {} {}", v.x, v.y, v.z)
    }
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    for (angle, shift) in [
        (0.31, DVec3::new(1., -0.22, 0.53)),
        (-0.71, DVec3::new(-0.39, 0.04, 0.8)),
        (1.23, DVec3::new(0.125, 0.375, -0.25)),
    ] {
        let q = DQuat::from_rotation_x(angle);
        let point = |p| vector(q * p + shift);
        let mut xml = CABLE
            .replace("0.4</max_step_size>", "0.007</max_step_size>")
            .replace(
                "0.45 0 0.62 0 0 0",
                &format!("{} {angle} 0 0", point(DVec3::new(0.45, 0., 0.62))),
            )
            .replace(
                ">0 1.8 0</linear_velocity>",
                &format!(">{}</linear_velocity>", vector(q * DVec3::Y * 1.8)),
            )
            .replace(
                ">0 0 4</angular_velocity>",
                &format!(">{}</angular_velocity>", vector(q * DVec3::Z * 4.)),
            )
            .replace(">0 0 1</xyz>", &format!(">{}</xyz>", vector(q * DVec3::Z)));
        for (src, z) in [("0.27", 0.27), ("0.97", 0.97)] {
            xml = xml.replace(
                &format!("0.31351801920622441 0.32281024090478527 {src}"),
                &point(DVec3::new(0.3135180192062244, 0.322_810_240_904_785_3, z)),
            );
        }
        let r = distributed_rotor_material_contact_spec(&xml, "tilted.sdf", "default")?;
        let spec = AutomaticRotorMaterialSpec {
            pad: r.pad,
            material_name: r.material_name,
            step_ns: r.step.as_nanos() as u64,
            restitution: r.restitution,
        };
        let dir = tempfile::tempdir()?;
        let mut p = RotorMaterialPool::create(exe, dir.path(), 3, spec, r.material.clone())?;
        let mut ordinary = Rotor::new(&xml)?;
        let mut checked = false;
        for t in 0..70 {
            if [23, 26, 43].contains(&t) {
                p.transfer((p.owner() + 1) % p.width())?;
                drop(p);
                p = RotorMaterialPool::resume(exe, dir.path(), r.material.clone())?;
            }
            p.advance_at(0, t)?;
            ordinary.advance(1)?;
            let s = ordinary.state();
            let a = &p.observation().links[0];
            assert_eq!(
                (a.position, a.orientation, a.velocity, a.angular_velocity),
                (s.position, s.orientation, s.velocity, s.angular_velocity),
                "ordinary/worker tilt {angle}, tick {t}"
            );
            for (n, o) in p.particles().iter().zip(ordinary.material().particles()) {
                assert_eq!((n.position, n.velocity), (o.position, o.velocity));
            }
            if !checked && let Some(event) = p.last_impact() {
                let when = event.input_tick * p.spec().step_ns + event.head_ns;
                let load = &event.contact.loads[0];
                let moment = (DVec3::from(load.point) - DVec3::from(p.spec().pad.anchor))
                    .cross(DVec3::from(load.on_link))
                    .dot(q * DVec3::Z);
                let spin = DVec3::from(a.angular_velocity).dot(q * DVec3::Z);
                assert!((when as i64 - 177770455).abs() <= 2);
                assert!(
                    (moment + 0.387343187933).abs() < 1e-10 && (spin - 2.591479316607).abs() < 1e-9
                );
                println!(
                    "tilt_rad={angle} measured_ns={when} signed_axial_impulse_Nms={moment:.12} post_spin_rad_s={spin:.12}"
                );
                checked = true;
            }
        }
        assert!(checked, "missing transformed collision");
    }
    Ok(())
}
