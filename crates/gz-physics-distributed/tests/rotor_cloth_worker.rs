// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/rotating_pad.rs"]
mod fixture;
use fixture::Rotor;
use glam::DVec3;
use gz_physics_distributed::floating_tree::{AutomaticRotorMaterialSpec, RotorMaterialPool};
use gz_sim_scene::deformable::articulated_impact::distributed_rotor_material_contact_spec;
use std::{error::Error, path::Path, process::Command};
const SDF: &str = include_str!("../../../examples/distributed_rotating_pad_cloth.sdf");
const MASSES: [f64; 4] = [0.17, 0.09, 0.26, 0.14];
const I: f64 = 0.275;
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn first_contact() -> f64 {
    // Lower face y = 0.32 - 0.12t. The pad enters it when y_center + R + skin reaches that plane.
    let (mut lo, mut hi) = (0., 0.30);
    for _ in 0..64 {
        let t = (lo + hi) * 0.5_f64;
        if 0.45 * (4. * t).sin() + 0.04 < 0.32 - 0.12 * t {
            lo = t;
        } else {
            hi = t;
        }
    }
    0.5 * (lo + hi)
}

#[test]
fn interior_rotor_face_couples_three_unequal_node_masses_and_preserves_the_deformed_sheet()
-> Result<()> {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let t_hit = first_contact();
    let edges = [(0, 1), (1, 3), (2, 3), (0, 2), (0, 3), (1, 2)];
    for period in ["0.011", "0.031", "0.20"] {
        let xml = SDF.replace(
            "0.011</max_step_size>",
            &format!("{period}</max_step_size>"),
        );
        let r = distributed_rotor_material_contact_spec(&xml, "face.sdf", "default")?;
        let mut control = Rotor::new(&xml)?;
        let spec = AutomaticRotorMaterialSpec {
            pad: r.pad,
            material_name: r.material_name,
            step_ns: r.step.as_nanos() as u64,
            restitution: r.restitution,
        };
        let rest: Vec<_> = edges
            .iter()
            .map(|&(a, b)| {
                DVec3::from(r.material.particles()[a].position)
                    .distance(r.material.particles()[b].position.into())
            })
            .collect();
        let d = tempfile::tempdir()?;
        let mut p = RotorMaterialPool::create(worker, d.path(), 3, spec, r.material.clone())?;
        let mut event_checked = false;
        let mut reacquired = false;
        let mut stretch = 0.0_f64;
        let mut warm = 0;
        for t in 0..(450_000_000 / p.spec().step_ns + 3) {
            if t == 1 || t == 9 || (event_checked && !reacquired) {
                let next = (p.owner() + 1) % p.width();
                p.transfer(next)?;
                let pid = p.worker_pid(next).ok_or("lost worker")?;
                assert!(
                    Command::new("kill")
                        .args(["-KILL", &pid.to_string()])
                        .status()?
                        .success()
                );
                let stored = p.native_image().map(ToOwned::to_owned);
                let before = p.material_image().to_owned();
                drop(p);
                p = RotorMaterialPool::resume(worker, d.path(), r.material.clone())?;
                assert_eq!(p.native_image(), stored.as_deref());
                assert_eq!(p.material_image(), before);
                if event_checked {
                    reacquired = true;
                }
            }
            let witness = std::env::var_os("GZ_ROTOR_CLOTH_WITNESS")
                .map(std::path::PathBuf::from)
                .filter(|_| period == "0.20" && t == 0);
            if let Some(dir) = &witness {
                std::fs::create_dir_all(dir)?;
                std::fs::copy(
                    d.path().join("barrier.gzrotmaterial"),
                    dir.join("prior.gzrotmaterial"),
                )?;
            }
            p.advance_at(0, t)?;
            if let Some(dir) = &witness {
                std::fs::copy(
                    d.path().join("barrier.gzrotmaterial"),
                    dir.join("accepted.gzrotmaterial"),
                )?;
            }
            control.advance(1)?;
            let state = control.state();
            let remote = &p.observation().links[0];
            assert_eq!(
                (
                    remote.position,
                    remote.orientation,
                    remote.velocity,
                    remote.angular_velocity
                ),
                (
                    state.position,
                    state.orientation,
                    state.velocity,
                    state.angular_velocity
                )
            );
            for (a, b) in p.particles().iter().zip(control.material().particles()) {
                assert_eq!((a.position, a.velocity), (b.position, b.velocity));
            }
            if let Some(hit) = p.last_impact() {
                warm += 1;
                for (j, &(a, b)) in edges.iter().enumerate() {
                    let l = DVec3::from(p.particles()[a].position)
                        .distance(p.particles()[b].position.into());
                    stretch = stretch.max((l / rest[j] - 1.).abs());
                }
                if event_checked {
                    continue;
                }
                let abs = (hit.input_tick * p.spec().step_ns + hit.head_ns) as f64 * 1e-9;
                assert!((abs - t_hit).abs() <= 1.01e-9, "{period} {abs}, {t_hit}");
                assert_eq!(hit.contact.loads.len(), 1);
                let row = &hit.contact.loads[0];
                assert_eq!(row.feature.len(), 3);
                assert!(
                    row.weights.iter().all(|&x| x > 0.05 && x < 0.9),
                    "{:?}",
                    row.weights
                );
                let pad = DVec3::from(hit.head_links[0].position);
                let site = DVec3::from(row.point);
                assert!((pad.x - 0.45 * (4. * abs).cos()).abs() < 1e-12);
                let cloud = &hit.head_particles;
                let face: DVec3 = row
                    .feature
                    .iter()
                    .zip(row.weights)
                    .map(|(&i, w)| DVec3::from(cloud[i].0) * w)
                    .sum();
                assert!((face.y - site.y - 0.005).abs() < 3e-9);
                let node_clearance = cloud
                    .iter()
                    .map(|n| DVec3::from(n.0).distance(pad) - 0.04)
                    .fold(f64::INFINITY, f64::min);
                assert!(node_clearance > 0.05);
                let denom = site.x * site.x / I
                    + row
                        .feature
                        .iter()
                        .zip(row.weights)
                        .map(|(&i, w)| w * w / MASSES[i])
                        .sum::<f64>();
                let expected = (1. + p.spec().restitution) * (4. * site.x + 0.12) / denom;
                let on_link = DVec3::from(row.on_link);
                assert!(
                    (on_link + DVec3::Y * expected).length() < 3e-8,
                    "{on_link:?}, {expected}"
                );
                let rotor_dl = I * (remote.angular_velocity[2] - 4.);
                let nodes_dl = row
                    .on_particles
                    .iter()
                    .map(|(i, j)| DVec3::from(cloud[*i].0).cross((*j).into()).z)
                    .sum::<f64>();
                assert!((rotor_dl + nodes_dl).abs() < 1e-9);
                for ((i, j), w) in row.on_particles.iter().zip(row.weights) {
                    assert!(
                        (DVec3::from(*j) - DVec3::Y * w * expected).length() < 3e-8,
                        "{i}"
                    );
                }
                let ordinary = control.material().latest_articulated_impact().unwrap();
                assert_eq!(
                    ordinary.material.loads[0].on_rigid.map(f64::to_bits),
                    row.on_link.map(f64::to_bits)
                );
                println!(
                    "public_dt_s={period} contact_s={abs:.9} analytic_s={t_hit:.12} face={:?} weights={:?} nearest_node_clearance_m={node_clearance:.9} signed_impulse_y_Ns={:.12} analytic_J_Ns={expected:.12} axial_residual_Nms={:.3e} dissipated_J={:.9}",
                    row.feature,
                    row.weights,
                    on_link.y,
                    (nodes_dl + rotor_dl).abs(),
                    hit.contact.dissipated_energy
                );
                event_checked = true;
            }
        }
        assert!(
            event_checked && reacquired && warm > 1 && stretch > 1e-4,
            "checked={event_checked}, warm={warm}, maximum strain={stretch:e}"
        );
        println!(
            "public_dt_s={period} exact_warm_intervals={warm} maximum_measured_edge_or_diagonal_strain={stretch:.8}"
        );
    }
    Ok(())
}
