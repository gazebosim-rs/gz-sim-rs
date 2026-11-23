// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DMat3, DQuat, DVec3};
use gz_physics::PhysicsBackend;
use gz_physics_distributed::floating_tree::{
    AutomaticMaterialPool, AutomaticMaterialSpec, ContactTreeSpec,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::entity_by_id;
use gz_sim_scene::{
    BodyHandle, PhysicsEngine,
    deformable::{self, articulated_impact::distributed_automatic_material_contact_tree_spec},
    load_world_with_sdf_step,
};
use std::{error::Error, fs, path::Path, process::Command, time::Duration};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const FAST: &str = include_str!("../../../examples/articulated_transient_cable.sdf");
const CLOTH: &str = include_str!("../../../examples/articulated_first_impact.sdf");
fn recipe(xml: &str) -> Result<(AutomaticMaterialSpec, gz_physics_xpbd::SoftBody)> {
    let s = distributed_automatic_material_contact_tree_spec(xml, "automatic.sdf", "default")?;
    Ok((
        AutomaticMaterialSpec {
            tree: ContactTreeSpec {
                tree: s.tree,
                limits: s.limits,
            },
            material_name: s.material_name,
            step_ns: s.step.as_nanos() as u64,
            links: s.links,
            restitution: s.restitution,
        },
        s.material,
    ))
}
fn momentum(p: &AutomaticMaterialPool) -> (DVec3, DVec3, f64) {
    let (mut total, mut angular, mut energy) = (DVec3::ZERO, DVec3::ZERO, 0.);
    for (l, b) in p.spec().tree.tree.links.iter().zip(&p.observation().links) {
        let q = DQuat::from_array(b.orientation);
        let center = DVec3::from(b.position) + q * DVec3::from(l.inertia_center);
        let r = DMat3::from_quat(q * DQuat::from_array(l.inertia_orientation));
        let [xx, yy, zz, xy, xz, yz] = l.inertia_tensor;
        let inertia =
            r * DMat3::from_cols(
                DVec3::new(xx, xy, xz),
                DVec3::new(xy, yy, yz),
                DVec3::new(xz, yz, zz),
            ) * r.transpose();
        let v = DVec3::from(b.velocity);
        let w = DVec3::from(b.angular_velocity);
        let m = l.mass * v;
        total += m;
        angular += center.cross(m) + inertia * w;
        energy += 0.5 * (m.dot(v) + w.dot(inertia * w));
    }
    for n in p.particles() {
        let m = DVec3::from(n.velocity) / n.inverse_mass;
        total += m;
        angular += DVec3::from(n.position).cross(m);
        energy += 0.5 * m.dot(DVec3::from(n.velocity));
    }
    (total, angular, energy)
}
/// Independently reconstruct spatial and kinetic impulse changes from the published
/// event-head geometry, dense native mobility and material masses.
fn impulse_audit(p: &AutomaticMaterialPool) -> [f64; 3] {
    let hit = p.last_impact().unwrap();
    let r = &hit.contact;
    let mut material = vec![DVec3::ZERO; hit.head_particles.len()];
    let (mut linear, mut angular, mut energy) = (DVec3::ZERO, DVec3::ZERO, 0.);
    for (i, row) in r.loads.iter().enumerate() {
        let impulse = DVec3::from(row.on_link);
        linear += impulse;
        angular += DVec3::from(row.point).cross(impulse);
        let dv = r
            .loads
            .iter()
            .enumerate()
            .fold(DVec3::ZERO, |acc, (j, other)| {
                let block = r.basis.mobility(i, j).unwrap();
                let inc =
                    std::array::from_fn(|k| DVec3::from(block[k]).dot(DVec3::from(other.on_link)));
                acc + DVec3::from(inc)
            });
        energy += impulse.dot(DVec3::from(r.basis.velocities[i]) + 0.5 * dv);
        for (id, j) in &row.on_particles {
            material[*id] += DVec3::from(*j);
        }
    }
    for (id, impulse) in material.into_iter().enumerate() {
        let (x, v) = hit.head_particles[id];
        linear += impulse;
        angular += DVec3::from(x).cross(impulse);
        energy += impulse.dot(DVec3::from(v))
            + 0.5 * impulse.length_squared() * p.particles()[id].inverse_mass;
    }
    [
        linear.length(),
        angular.length(),
        (energy + r.dissipated_energy - r.prescribed_support_work).abs(),
    ]
}
#[test]
fn entering_and_leaving_within_ten_ms_preserves_true_impact_time_across_recovery() -> Result<()> {
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let cloth = CLOTH
        .replace("0.001</max_step_size>", "0.01</max_step_size>")
        .replace("friction=\"0.9\"", "friction=\"0\"");
    for (label, xml, expected_ns) in [
        ("cable", FAST, 2_371_250u64),
        ("cloth", cloth.as_str(), 100_375_123),
    ] {
        let (spec, cold) = recipe(xml)?;
        assert_eq!(spec.step_ns, 10_000_000);
        let mut sim = load_world_with_sdf_step(
            xml,
            "automatic.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let ids = spec
            .tree
            .tree
            .links
            .iter()
            .map(|l| {
                sim.world()
                    .get::<BodyHandle>(entity_by_id(sim.world(), l.id).unwrap())
                    .unwrap()
                    .id()
            })
            .collect::<Vec<_>>();
        let d = tempfile::tempdir()?;
        let mut p = AutomaticMaterialPool::create(exe, d.path(), 3, spec.clone(), cold.clone())?;
        let file = d.path().join("barrier.gzautmaterial");
        let input = expected_ns / spec.step_ns;
        let initial = momentum(&p);
        let mut preserved = None;
        let mut kills = 0;
        let mut replacements = 0;
        for t in 0..40 {
            if [0, input, input + 1, 24, 33].contains(&t) {
                let original = p.worker_pid(p.owner()).unwrap();
                assert!(
                    Command::new("kill")
                        .args(["-KILL", &original.to_string()])
                        .status()?
                        .success()
                );
                kills += 1;
            }
            if [input + 1, input + 2, 27].contains(&t) {
                let old = p.native_image().map(<[u8]>::to_owned);
                p.transfer((p.owner() + 1) % p.width())?;
                assert_eq!(p.native_image(), old.as_deref());
                drop(p);
                p = AutomaticMaterialPool::resume(exe, d.path(), cold.clone())?;
                replacements += 1;
            }
            if t == input {
                p.drop_next_acknowledgement(p.owner())?;
            }
            let prior = (t == input).then(|| fs::read(&file)).transpose()?;
            sim.step_exact(1)?;
            p.advance_at(0, t)?;
            let be = sim.world().resource::<PhysicsBackend>();
            let n = be.downcast_ref::<MultiFidelityBackend>().unwrap();
            let expected_dt = if t == input {
                Duration::from_nanos((input + 1) * spec.step_ns - expected_ns)
            } else {
                Duration::from_nanos(spec.step_ns)
            };
            assert_eq!(p.last_native_duration(), expected_dt);
            let expected = n.checkpoint_timed_revolute_tree(expected_dt)?;
            assert!(
                p.native_image() == Some(&expected),
                "native full timed graph differs at {label} {}",
                t + 1
            );
            assert!(
                n.checkpoint_floating_revolute_tree().is_err(),
                "fixed 1ms semantics"
            );
            for (id, row) in ids.iter().zip(&p.observation().links) {
                let b = be.state(*id).unwrap();
                assert!(
                    row.position
                        .iter()
                        .chain(&row.orientation)
                        .chain(&row.velocity)
                        .chain(&row.angular_velocity)
                        .zip(
                            b.position
                                .iter()
                                .chain(&b.orientation)
                                .chain(&b.velocity)
                                .chain(&b.angular_velocity)
                        )
                        .all(|(a, b)| a.to_bits() == b.to_bits())
                );
            }
            let soft = deformable::bodies(&sim).next().unwrap().1;
            assert_eq!(p.particles(), soft.particles());
            assert_eq!(p.last_impact().is_some(), t >= input);
            if t == input {
                let r = p.last_impact().unwrap();
                let o = soft.latest_articulated_impact().unwrap();
                let det = o.detection.as_ref().unwrap();
                assert_eq!(input * spec.step_ns + r.head_ns, expected_ns);
                assert_eq!(o.time, Duration::from_nanos(expected_ns));
                assert_eq!(
                    r.continuous_seconds.to_bits(),
                    det.seconds_after_start.to_bits()
                );
                assert_eq!(
                    r.rounding_distance.to_bits(),
                    det.rounding_distance.to_bits()
                );
                assert_eq!(r.contact.loads.len(), 2);
                let mut closure = 0.0f64;
                for (a, b) in r.contact.loads.iter().zip(&o.material.loads) {
                    assert_eq!(a.point, b.site.point);
                    assert_eq!(a.on_link, b.on_rigid);
                    assert_eq!(a.on_particles, b.on_particles);
                    assert_eq!(
                        a.initial_normal_velocity.to_bits(),
                        b.initial_normal_velocity.to_bits()
                    );
                    assert_eq!(
                        a.final_normal_velocity.to_bits(),
                        b.final_normal_velocity.to_bits()
                    );
                    let sum = a
                        .on_particles
                        .iter()
                        .fold(DVec3::from(a.on_link), |x, (_, v)| x + DVec3::from(*v));
                    closure = closure.max(sum.length());
                }
                assert!(closure < 1e-13);
                let immediate = impulse_audit(&p);
                assert!(
                    immediate.iter().all(|v| *v < 2e-12),
                    "event-head spatial/energy audit {immediate:?}"
                );
                let m = momentum(&p);
                let residual = [
                    (m.0 - initial.0).length(),
                    (m.1 - initial.1).length(),
                    (m.2 - initial.2 + r.contact.dissipated_energy).abs(),
                ];
                println!(
                    "{label} public_input_tick={input} interval_ns={} head_ns={} tail_ns={} continuous_s={} rounding_m={} active_sites={} immediate_equal_opposite_Ns={closure} event_head_residual_p_L_E={immediate:?} physical_endpoint_residual_p_L_E={residual:?}",
                    spec.step_ns,
                    r.head_ns,
                    r.tail_ns,
                    r.continuous_seconds,
                    r.rounding_distance,
                    r.contact.loads.len()
                );
                if let Some(base) = std::env::var_os("GZ_AUTOMATIC_WITNESS") {
                    let root = Path::new(&base).join(label);
                    fs::create_dir_all(&root)?;
                    fs::write(root.join("prior.gzautmaterial"), prior.unwrap())?;
                    fs::copy(&file, root.join("accepted.gzautmaterial"))?;
                    fs::write(root.join("receipt.json"), serde_json::to_vec_pretty(r)?)?;
                }
                preserved = Some(serde_json::to_vec(r)?);
            }
            if t > input {
                assert_eq!(
                    serde_json::to_vec(p.last_impact().unwrap())?,
                    *preserved.as_ref().unwrap()
                );
            }
        }
        let before = fs::read(&file)?;
        assert!(p.advance_at(1, 40).is_err());
        assert_eq!(fs::read(&file)?, before);
        p.reset()?;
        assert_eq!((p.tick(), p.epoch()), (0, 1));
        assert!(p.last_impact().is_none());
        for t in 0..=input {
            p.advance_at(1, t)?;
        }
        assert_eq!(p.last_impact().unwrap().contact.epoch, 1);
        println!(
            "{label}: exact_ordinary_public_cuts=40 real_worker_kills={kills} transfer_and_resume={replacements} repeated_epoch_impact=true"
        );
    }
    Ok(())
}

#[test]
fn stock_bevy_worker_selection_and_persistent_restart_publish_the_actual_off_grid_receipt()
-> Result<()> {
    use gz_sim_scene::{
        WorldLoadOptions,
        deformable::managed_impact::{self, ManagedAutomaticMaterial},
        load_world_configured,
    };
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let cloth = CLOTH
        .replace("0.001</max_step_size>", "0.01</max_step_size>")
        .replace("friction=\"0.9\"", "friction=\"0\"");
    for xml in [FAST, cloth.as_str()] {
        let options = WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(exe),
        };
        let mut hosted =
            load_world_configured(xml, "host.sdf", "default", &sdformat::NoResources, options)?;
        assert!(
            hosted
                .world()
                .contains_resource::<ManagedAutomaticMaterial>()
        );
        let dir = tempfile::tempdir()?;
        let mut persistent =
            managed_impact::load_persistent(xml, "host.sdf", "default", exe, dir.path(), 3)?;
        let mut direct = load_world_with_sdf_step(
            xml,
            "host.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let mut t = 0;
        for k in 0..60 {
            if [0, 9, 10, 11, 23, 40, 41, 59].contains(&k) {
                let epoch = persistent.epoch();
                let p = persistent.world().resource::<ManagedAutomaticMaterial>();
                let pid = p.worker_pid(p.owner()).unwrap();
                assert!(
                    Command::new("kill")
                        .args(["-KILL", &pid.to_string()])
                        .status()?
                        .success()
                );
                let destination = (persistent
                    .world()
                    .resource::<ManagedAutomaticMaterial>()
                    .owner()
                    + 1)
                    % 3;
                managed_impact::transfer(&mut persistent, epoch, t, destination)?;
                drop(persistent);
                persistent = managed_impact::load_persistent(
                    xml,
                    "host.sdf",
                    "default",
                    exe,
                    dir.path(),
                    3,
                )?;
                assert_eq!(persistent.committed_update_info()?.iteration, t);
                let a = deformable::bodies(&persistent).next().unwrap().1;
                let b = deformable::bodies(&direct).next().unwrap().1;
                assert_eq!(a.particles(), b.particles());
                assert_eq!(a.latest_articulated_impact(), b.latest_articulated_impact());
                assert_eq!(a.impacts_this_step(), b.impacts_this_step());
                assert_eq!(a.articulated_impact_count(), b.articulated_impact_count());
            }
            if k == 31 {
                for sim in [&mut persistent, &mut direct, &mut hosted] {
                    sim.reset()
                        .map_err(|e| format!("reset at loop {k}, {e}, {:?}", sim.fault()))?;
                }
                t = 0;
            }
            for (lane, s) in [
                ("direct", &mut direct),
                ("hosted", &mut hosted),
                ("persistent", &mut persistent),
            ] {
                s.step_exact(1).map_err(|e| {
                    format!(
                        "{lane} at epoch={}, tick={t}, loop={k}: {e}, {:?}",
                        s.epoch(),
                        s.fault()
                    )
                })?;
            }
            t += 1;
            for s in [&hosted, &persistent] {
                assert_eq!(
                    s.world().resource::<ManagedAutomaticMaterial>().interval(),
                    (direct.epoch(), t)
                );
                let a = deformable::bodies(s).next().unwrap().1;
                let b = deformable::bodies(&direct).next().unwrap().1;
                assert_eq!(a.particles(), b.particles());
                assert_eq!(a.latest_articulated_impact(), b.latest_articulated_impact());
                assert_eq!(a.impacts_this_step(), b.impacts_this_step());
                assert_eq!(a.articulated_impact_count(), b.articulated_impact_count());
                let expected = direct.world().resource::<PhysicsBackend>();
                let accepted = s.world().resource::<PhysicsBackend>();
                for e in direct.world().iter_entities() {
                    if let Some(h) = e.get::<BodyHandle>() {
                        let original = expected.state(h.id()).unwrap();
                        let actual = accepted.state(h.id()).unwrap();
                        assert_eq!(
                            (
                                original.position,
                                original.orientation,
                                original.velocity,
                                original.angular_velocity
                            ),
                            (
                                actual.position,
                                actual.orientation,
                                actual.velocity,
                                actual.angular_velocity
                            )
                        );
                    }
                }
            }
        }
        println!(
            "configured_stock_worker_public_steps=60 persistent_public_steps=60 real_pid_kills=8 durable_reopens=8 exact_body_particle_and_typed_receipts=true"
        );
    }
    let xml = CLOTH;
    let opt = WorldLoadOptions {
        engine: PhysicsEngine::MultiFidelity,
        step: None,
        noise_seed: [0; 32],
        distributed_worker: Some(exe),
    };
    assert!(
        load_world_configured(
            xml,
            "frictional.sdf",
            "default",
            &sdformat::NoResources,
            opt
        )
        .is_err()
    );
    Ok(())
}

#[test]
fn shrinking_public_time_quantifies_post_impact_error_and_preserves_physical_time() -> Result<()> {
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let cloth = CLOTH.replace("friction=\"0.9\"", "friction=\"0\"");
    for (label, source, time, first) in [
        ("cable", FAST, "0.01", 2_371_250u64),
        ("cloth", cloth.as_str(), "0.001", 100_375_123),
    ] {
        for (ns, step) in [
            (10_000_000, "0.01"),
            (5_000_000, "0.005"),
            (2_000_000, "0.002"),
            (1_000_000, "0.001"),
            (500_000, "0.0005"),
        ] {
            let xml = source.replace(
                &format!("{time}</max_step_size>"),
                &format!("{step}</max_step_size>"),
            );
            let (spec, cold) = recipe(&xml)?;
            assert_eq!(spec.step_ns, ns);
            let dir = tempfile::tempdir()?;
            let mut p = AutomaticMaterialPool::create(exe, dir.path(), 2, spec, cold)?;
            let original = momentum(&p);
            let compare = if label == "cable" {
                10_000_000
            } else {
                110_000_000
            };
            for t in 0..compare / ns {
                p.advance_at(0, t)?;
            }
            let hit = p.last_impact().unwrap();
            let at = hit.input_tick * ns + hit.head_ns;
            assert_eq!(at, first);
            let immediate = impulse_audit(&p);
            assert!(immediate.iter().all(|v| *v < 2e-12));
            let close = momentum(&p);
            println!(
                "convergence material={label} public_dt_ns={ns} impact_ns={at} comparison_ns={compare} linear_Ns={} angular_Nms={} kinetic_only_plus_contact_dissipation_J={} immediate_p_L_E={immediate:?}",
                (close.0 - original.0).length(),
                (close.1 - original.1).length(),
                (close.2 - original.2 + hit.contact.dissipated_energy).abs()
            );
        }
    }
    Ok(())
}

#[test]
fn quantum_adjacent_impacts_allow_a_zero_length_private_head_or_tail() -> Result<()> {
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    for target_ns in [0.2f64, 9_999_999.8] {
        let shift = (target_ns - 2_371_250.) * 1e-9;
        let x = 0.32259511 + 0.504 * shift;
        let z = 0.58889752 + 1.728 * shift;
        let xml = FAST
            .replace("0.32259511", &format!("{x:.17}"))
            .replace("0.58889752", &format!("{z:.17}"));
        let (spec, cold) = recipe(&xml)?;
        let dir = tempfile::tempdir()?;
        let mut p = AutomaticMaterialPool::create(exe, dir.path(), 2, spec, cold.clone())?;
        let mut sim = load_world_with_sdf_step(
            &xml,
            "quantum.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        p.advance()?;
        sim.step_exact(1)?;
        let observed = p.last_impact().ok_or("missing subnanosecond edge event")?;
        assert_eq!(observed.head_ns, target_ns.round() as u64);
        assert!(observed.head_ns == 0 || observed.tail_ns == 0);
        assert_eq!(p.last_native_duration(), Duration::from_millis(10));
        assert!(impulse_audit(&p).iter().all(|e| *e < 2e-12));
        p.transfer(1)?;
        drop(p);
        let mut p = AutomaticMaterialPool::resume(exe, dir.path(), cold)?;
        for _ in 0..24 {
            p.advance()?;
            sim.step_exact(1)?;
            let native = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            assert_eq!(
                p.native_image().unwrap(),
                native.checkpoint_timed_revolute_tree(Duration::from_millis(10))?
            );
            assert_eq!(
                p.particles(),
                deformable::bodies(&sim).next().unwrap().1.particles()
            );
        }
        eprintln!(
            "continuous_target_ns={target_ns} rounded_head_ns={} zero_subinterval=true exact_post_transfer_steps=24",
            p.last_impact().unwrap().head_ns
        );
    }
    Ok(())
}

#[test]
fn stock_articulation_session_hosts_and_recovers_named_deformable_visuals() -> Result<()> {
    use gz_sim_core::EntityId;
    use gz_sim_scene::{
        Name, VisualGeometry, WorldPose,
        distributed_hinge::load_distributed_articulation_world_with_step as open,
    };
    use std::io::Write;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let xml = include_str!("../../../examples/distributed_first_impact_cloth.sdf");
    let dir = tempfile::tempdir()?;
    assert!(
        open(
            xml,
            "cloth.sdf",
            "default",
            exe,
            dir.path(),
            Some(3),
            Some(Duration::from_millis(1))
        )
        .is_err()
    );
    assert!(!dir.path().join("barrier.gzautmaterial").exists());
    let mut managed = open(
        xml,
        "cloth.sdf",
        "default",
        exe,
        dir.path(),
        Some(3),
        Some(Duration::from_millis(10)),
    )?;
    let mut reference = load_world_with_sdf_step(
        xml,
        "cloth.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut out = std::env::var_os("GZ_AUTOMATIC_LIVE_REFERENCE")
        .map(fs::File::create)
        .transpose()?
        .map(std::io::BufWriter::new);
    for tick in 0..=180 {
        if [11, 41].contains(&tick) {
            drop(managed);
            managed = open(xml, "cloth.sdf", "default", exe, dir.path(), None, None)?;
        }
        let nodes = deformable::bodies(&reference).next().unwrap().1.particles();
        let mut visuals = vec![];
        let mut particles = 0;
        for e in reference
            .world()
            .iter_entities()
            .filter(|e| e.contains::<VisualGeometry>())
        {
            let id = e.get::<EntityId>().unwrap().get();
            let name = &e.get::<Name>().unwrap().0;
            let pose = e.get::<WorldPose>().unwrap();
            let native = managed
                .world()
                .get::<WorldPose>(entity_by_id(managed.world(), id).unwrap())
                .unwrap();
            assert_eq!(
                (native.translation, native.rotation),
                (pose.translation, pose.rotation),
                "visual {name} at tick {tick}"
            );
            if let Some(i) = name.split("cloth_corner_").nth(1) {
                assert_eq!(pose.translation, nodes[i.parse::<usize>()?].position);
                particles += 1;
            }
            visuals.push(serde_json::json!({"id":id,"name":name,"position":pose.translation,"orientation":pose.rotation}));
        }
        assert_eq!(particles, 4);
        if let Some(w) = &mut out {
            writeln!(
                w,
                "{}",
                serde_json::json!({"tick":tick*10,"visuals":visuals})
            )?;
        }
        if tick < 180 {
            managed.step_exact(1)?;
            reference.step_exact(1)?;
        }
    }
    eprintln!("ordinary_and_stock_visual_frames=181 four_true_material_corners=724 live_reopens=2");
    Ok(())
}
