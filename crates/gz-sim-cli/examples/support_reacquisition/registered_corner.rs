// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise the registered corner solver with both physical bodies present.
#[path = "registered_arrivals.rs"]
pub(crate) mod arrivals;
use super::*;
use gz_physics_multifidelity::BoxContactActions;
use gz_sim_scene::{load_world_with_sdf_step, primary_contact};
const SDF: &str = include_str!("../../../../examples/multifidelity_registered_corner.sdf");
fn rotate_body(b: Body, q: DQuat, shift: DVec3) -> Body {
    Body {
        position: (shift + q * DVec3::from(b.position)).to_array(),
        orientation: (q * DQuat::from_array(b.orientation)).to_array(),
        velocity: (q * DVec3::from(b.velocity)).to_array(),
        angular_velocity: (q * DVec3::from(b.angular_velocity)).to_array(),
    }
}
fn physical(c: Approach, q: DQuat, pad_first: bool) -> Result<[f64; 4]> {
    let mut n = MultiFidelityBackend::new([0.; 3])?;
    let shift = DVec3::new(0.71, -1.13, 0.29);
    let frame = c.frame;
    let states = c.states().map(|s| rotate_body(s, q, shift));
    let f = Setup {
        centers: frame.centers.map(|p| shift + q * p),
        anchor: shift + q * frame.anchor,
        axis: q * frame.axis,
        q: frame.q.map(|a| q * a),
        ..frame
    };
    let make_pad = |n: &mut MultiFidelityBackend| -> Result<BodyId> {
        let support = rotate_body(
            Body {
                position: [0., 0., -PAD_HALF[2]],
                orientation: DQuat::IDENTITY.to_array(),
                velocity: DECK.to_array(),
                angular_velocity: [0.; 3],
            },
            q,
            shift,
        );
        Ok(n.spawn_material_link(
            cube(PAD_HALF, support, 1.),
            orientation(support),
            None,
            MaterialLinkMotion::Prescribed,
            IsotropicFriction::new(c.mu)?,
        )?)
    };
    let pad = if pad_first {
        Some(make_pad(&mut n)?)
    } else {
        None
    };
    let ([a, b], _) = add_tree(&mut n, f, states, false, Some(IsotropicFriction::new(1.5)?))?;
    let pad = if let Some(id) = pad {
        id
    } else {
        make_pad(&mut n)?
    };
    let law = c.target()?;
    let predicted = DVec3::from(law.impulse);
    let delta = frame.response(predicted, false);
    let h = n
        .project_corner_impact(a, pad)?
        .ok_or("certified corner unexpectedly refused")?;
    let measured = DVec3::from(h.impulse_on_foot);
    let jerr = measured.distance(q * predicted);
    let mut twist = 0.0_f64;
    for (i, id) in [a, b].into_iter().enumerate() {
        let v = n.state(id).unwrap();
        twist = twist
            .max(DVec3::from(v.velocity).distance(DVec3::from(states[i].velocity) + q * delta.v[i]))
            .max(
                DVec3::from(v.angular_velocity)
                    .distance(DVec3::from(states[i].angular_velocity) + q * delta.w[i]),
            );
    }
    let energy = (h.kinetic_energy_change - (DECK.dot(predicted) - law.dissipated_energy)).abs();
    let velocity =
        DVec3::from(h.after_relative_velocity).distance(q * DVec3::from(law.relative_velocity));
    if [jerr, twist, energy, velocity]
        .into_iter()
        .any(|v| v > 1e-9)
        || h.sticking != law.sticking
        || n.project_corner_impact(a, pad)?.is_some()
    {
        return Err(
            "registered projection disagrees with independent operator or delivered twice".into(),
        );
    }
    Ok([jerr, twist, energy, velocity])
}
fn scene(out: &Path) -> Result<()> {
    let f = Setup::new(1., DVec3::new(0.2, 1., 0.25), 0.31, 1.4);
    // The SDF starts both free links at rest and translates the support up and sideways.
    let exact = Approach {
        frame: f,
        slip: DVec3::new(-1.1, 0.7, 0.),
        spin: DVec3::ZERO,
        hinge: 0.,
        mu: 0.6,
    }
    .target()?;
    let mut rows = vec![];
    let mut lanes = vec![];
    for (label, name) in [
        ("ordinary", "SDF, native contact"),
        ("equal", "Eight native calls from same SDF"),
        ("project", "SDF, isolated corner projection"),
    ] {
        let mut sim = load_world_with_sdf_step(
            SDF,
            "corner.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        if label == "ordinary" {
            primary_contact::set_enabled(&mut sim, false)?;
        }
        #[allow(deprecated)]
        let id = |name: &str| {
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
                .get::<BodyHandle>()
                .unwrap()
                .id()
        };
        let [foot, upper, pad] = [
            "default::rocker::foot",
            "default::rocker::upper",
            "default::lift::pad",
        ]
        .map(id);
        let mut frames = vec![];
        let mut first = None;
        let mut sum = 0.;
        for tick in 0..=32 {
            let mut normal = 0.;
            let mut calls = 0;
            if tick > 0 {
                if label == "equal" && tick == 1 {
                    let mut res = sim.world_mut().resource_mut::<PhysicsBackend>();
                    let n = res.downcast_mut::<MultiFidelityBackend>().unwrap();
                    let r = n.advance_box_contacts(
                        DT,
                        8,
                        4.into(),
                        BoxContactActions::default(),
                        None,
                    )?;
                    normal = r.interval.pairs.iter().map(|p| p.normal_impulse).sum();
                    calls = 8;
                } else if label == "equal" {
                    let mut res = sim.world_mut().resource_mut::<PhysicsBackend>();
                    let n = res.downcast_mut::<MultiFidelityBackend>().unwrap();
                    n.step(DT)?;
                    normal = n
                        .rigid_contact_observations()
                        .iter()
                        .map(|p| p.normal_impulse)
                        .sum();
                    calls = 1;
                } else {
                    sim.step_exact(1)?;
                    if let Some(r) =
                        primary_contact::last_refinement(&sim).filter(|r| r.iteration == tick)
                    {
                        calls = r.interval.native_steps;
                        normal = r.interval.pairs.iter().map(|p| p.normal_impulse).sum();
                        if tick == 1 {
                            let h = r
                                .interval
                                .corner_impacts
                                .first()
                                .ok_or("authored projection missing")?;
                            let err = DVec3::from(h.impulse_on_foot).distance(exact.impulse.into());
                            if err > 1e-10 {
                                return Err(format!("authored impulse and independent analytic response differ {err}").into());
                            }
                        }
                    } else {
                        calls = 1;
                        normal = sim
                            .world()
                            .resource::<PhysicsBackend>()
                            .downcast_ref::<MultiFidelityBackend>()
                            .unwrap()
                            .rigid_contact_observations()
                            .iter()
                            .map(|p| p.normal_impulse)
                            .sum();
                    }
                }
            }
            let res = sim.world().resource::<PhysicsBackend>();
            let [a, b, d] = [foot, upper, pad].map(|i| Body::from(res.state(i).unwrap()));
            if tick == 1 {
                let momentum = DVec3::from(a.velocity) + DVec3::from(b.velocity);
                first = Some(
                    json!({"world_reaction_from_momentum_ns":momentum.to_array(),"error_vs_instantaneous_ns":momentum.distance(exact.impulse.into()),"normal_ns":normal}),
                );
            }
            sum += normal;
            frames.push(json!({"tick":tick,"time_us":tick*500,"foot":a,"upper":b,"pad":d,"native_steps":calls,"solver_iterations":4,"normal_impulse_ns":normal}));
        }
        rows.push(json!({"lane":label,"first_public":first,"cumulative_scalar_normal_ns":sum}));
        write_json(
            &out.join("traces").join(format!("{label}.json")),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":UPPER_HALF,"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"name":name,"label":label,"file":format!("traces/{label}.json")}));
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Registered frictional corner on a free ankle","pair_totals_only":true,"lanes":lanes,"instantaneous_target_world_ns":exact.impulse,"paths":rows}),
    )
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut max = [0.0_f64; 4];
    let mut count = 0;
    let mut sticking = 0;
    for (index, q) in [
        DQuat::IDENTITY,
        DQuat::from_rotation_x(0.67),
        DQuat::from_euler(glam::EulerRot::ZYX, -1.4, 0.72, -0.55),
        DQuat::from_rotation_y(2.81),
    ]
    .into_iter()
    .enumerate()
    {
        for pad_first in [false, true] {
            for mass in [0.4, 1., 2.5] {
                for tilt in [-0.25, 0.31] {
                    for dir in 0..13 {
                        for mu in [0., 0.2, 0.6, 1.] {
                            let angle = (dir as f64 + 0.13) * std::f64::consts::TAU / 13.;
                            let f = Setup::new(
                                mass,
                                if dir % 2 == 0 {
                                    DVec3::X
                                } else {
                                    DVec3::new(0.2, 1., 0.25)
                                },
                                tilt,
                                if dir % 3 == 0 { 0.65 } else { 1.4 },
                            );
                            let c = Approach {
                                frame: f,
                                slip: DVec3::new(angle.cos(), angle.sin(), 0.)
                                    * if (dir + index) % 2 == 0 { 0.22 } else { 1.8 },
                                spin: DVec3::new(0.31, 0.8, -0.77),
                                hinge: -0.64,
                                mu,
                            };
                            let errs = physical(c, q, pad_first)?;
                            for (j, e) in errs.into_iter().enumerate() {
                                max[j] = max[j].max(e);
                            }
                            count += 1;
                            sticking += usize::from(c.target()?.sticking);
                        }
                    }
                }
            }
        }
    }
    scene(out)?;
    write_json(
        &out.join("physical.json"),
        &json!({"qualified":count,"sticking":sticking,"sliding":count-sticking,"max_impulse_error_ns":max[0],"max_twist_error":max[1],"max_work_error_j":max[2],"max_relative_speed_error_m_s":max[3]}),
    )
}
