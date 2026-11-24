// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reload exact continuous second-impact donors through the ordinary Bevy scene.
use super::super::super::{
    authored::xml,
    transaction::{Frame, inertial},
};
use super::*;
use gz_physics::{NormalSiteState, TwoNormalContact};
use gz_physics_multifidelity::{EdgeImpactReceipt, FrictionlessRecontactReceipt};

pub(super) fn frame(rotated: bool) -> Frame {
    Frame {
        q: if rotated {
            DQuat::from_rotation_x(0.23) * DQuat::from_rotation_y(-0.18)
        } else {
            DQuat::IDENTITY
        },
        shift: if rotated {
            DVec3::new(1.4, -1.9, 0.57)
        } else {
            DVec3::ZERO
        },
        spin: 0.,
    }
}
pub(super) fn donor(mut input: Input, s: State) -> Input {
    input.seed.state = PoseState {
        q: s.motion.q,
        c: s.c,
        v: s.v,
        w: s.motion.w,
    };
    input
}
fn force_check(path: Arc, j: Jump) -> Result<Value> {
    let follow = Arc {
        start: j.after,
        sites: j.new_sites,
        ..path
    };
    let expected = follow.load(j.after.motion)?;
    let Sites::Two(keys) = j.new_sites else {
        return Ok(json!({"active":[j.new],"following_n":[expected[0]]}));
    };
    let shape = path.shape;
    let q = j.after.motion.q;
    let w = j.after.motion.w;
    let arms = keys.map(|k| q * shape.vertices[k]);
    let inertia = DMat3::from_quat(q) * shape.inertia * DMat3::from_quat(q).transpose();
    let gyro = inertia.inverse() * (-w.cross(inertia * w));
    let torque = arms.map(|r| r.cross(DVec3::Z));
    let w = TwoNormalContact {
        inverse_mass: torque
            .map(|r| torque.map(|a| 1. / shape.mass + r.dot(inertia.inverse() * a))),
        free_normal: arms.map(|r| shape.gravity.z + (gyro.cross(r) + w.cross(w.cross(r))).z),
    }
    .solve()?;
    let err = (0..2)
        .map(|k| (w.load[k] - expected[k]).abs())
        .fold(0., f64::max);
    if w.state != [NormalSiteState::Loaded; 2] || err > 1e-10 {
        return Err(
            "following normal forces disagree with constrained Newton–Euler reference".into(),
        );
    }
    Ok(json!({"active":keys,"following_n":w.load,"independent_n":expected,"max_error_n":err}))
}
fn details(
    hit: FrictionlessRecontactReceipt,
    f: Frame,
    s: State,
    j: Jump,
    path: Arc,
) -> Result<Value> {
    let ids = [j.old, j.new];
    let order = [hit.stationary_site, 1 - hit.stationary_site];
    let mut max_p = 0f64;
    let mut max_u = 0f64;
    let mut loads = [0.; 2];
    for (k, (id, idx)) in ids.into_iter().zip(order).enumerate() {
        let site = hit.sites[idx];
        let (p, u) = path.shape.point(id, s);
        let (_, v) = path.shape.point(id, j.after);
        max_p = max_p.max(DVec3::from(site.point).distance(f.shift + f.q * p));
        max_u = max_u
            .max(DVec3::from(site.before_relative_velocity).distance(f.q * u))
            .max(DVec3::from(site.after_relative_velocity).distance(f.q * v));
        let impulse = DVec3::from(site.impulse_on_foot);
        loads[k] = impulse.dot(f.q * DVec3::Z);
        if impulse.distance(f.q * (j.normal[k] * DVec3::Z)) > 1e-10
            || !site.elapsed.is_zero()
            || site.friction != 0.
            || (hit.state[idx] == NormalSiteState::Loaded) != (j.normal[k] > 0.)
        {
            return Err("native per-point loading differs from independent return".into());
        }
    }
    if max_p.max(max_u).max((hit.dissipated_energy - j.loss).abs()) > 1e-10 {
        return Err(format!("return geometry {max_p:e}, twist {max_u:e}").into());
    }
    Ok(
        json!({"normal_old_new_ns":loads,"reference_ns":j.normal,"point_error_m":max_p,"speed_error_m_s":max_u,"loss_error_j":(hit.dissipated_energy-j.loss).abs(),"old_before_m_s":path.shape.point(j.old,s).1.z,"new_before_m_s":path.shape.point(j.new,s).1.z,"old_after_m_s":path.shape.point(j.old,j.after).1.z,"new_after_m_s":path.shape.point(j.new,j.after).1.z}),
    )
}
fn exact(i: Input, path: Arc, e: Arrival, j: Jump, f: Frame) -> Result<Value> {
    let src = xml(i, f, false, f.q * path.shape.gravity, "edge");
    let mut sim = load(&src, true)?;
    let ids = ids(&sim)?;
    let mut owner = sim.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    let before = departure::ingest(n, &ids);
    let Some(EdgeImpactReceipt::Recontact(hit)) = n.project_edge_impact(ids.foot, ids.pad)? else {
        return Err(format!(
            "no native frictionless return {}→{} at {}",
            j.old, j.new, e.time
        )
        .into());
    };
    let after = departure::ingest(n, &ids);
    if n.project_edge_impact(ids.foot, ids.pad)?.is_some() || after != departure::ingest(n, &ids) {
        return Err("stationary return incorrectly committed twice".into());
    }
    let (p0, h0, k0) = inertial(before);
    let (p1, h1, k1) = inertial(after);
    let p = (p1 - p0).distance(f.q * (j.normal.iter().sum::<f64>() * DVec3::Z));
    let h = (h1 - h0).distance(
        [j.old, j.new]
            .into_iter()
            .zip(j.normal)
            .map(|(id, v)| f.q * ((e.state.motion.q * path.shape.vertices[id]).cross(v * DVec3::Z)))
            .sum(),
    );
    let k = (k1 - k0 - hit.kinetic_energy_change).abs();
    if p.max(h).max(k) > 1e-10 {
        return Err("native two-site return momentum or work mismatch".into());
    }
    let mut authored = load(&src, true)?;
    authored.step_exact(1)?;
    let r = last_refinement(&authored).ok_or("missing authored recontact interval")?;
    if r.interval.edge_impacts.as_slice() != [EdgeImpactReceipt::Recontact(hit)]
        || r.requested_pairs[0].first_positive_end != Some(Duration::ZERO)
    {
        return Err("Bevy and direct recontact receipts differ".into());
    }
    let mut data = details(hit, f, e.state, j, path)?;
    data["linear_error_ns"] = p.into();
    data["angular_error_nms"] = h.into();
    data["work_error_j"] = k.into();
    data["pair_public_ns"] = r.requested_pairs[0].normal_impulse.into();
    Ok(data)
}
pub(in super::super::super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    let mut demo = None;
    for active in [0, 1] {
        for pitch in [-0.18, 0.18] {
            for opening in [0.09, 0.3, 0.5] {
                for pitch_rate in [-3., -0.6, 0.6, 3.] {
                    for yaw in [0., 0.67] {
                        let (i, path, _) = fixture(Case {
                            active,
                            pitch,
                            yaw,
                            opening,
                            pitch_rate,
                        });
                        if solve(i.edge).is_err() {
                            continue;
                        }
                        let e = path.next(0.0000625, 0.4)?;
                        let End::Contact(new) = e.end else {
                            return Err("verified rocking donor lost second event".into());
                        };
                        let j = path.shape.recollide(e.state, path.sites.first(), new)?;
                        let input = donor(i, e.state);
                        let mut observations = vec![];
                        for rotated in [false, true] {
                            observations.push(exact(input, path, e, j, frame(rotated))?);
                        }
                        let force = force_check(path, j)?;
                        if demo.is_none() && matches!(j.new_sites, Sites::Two(_)) && yaw > 0. {
                            demo = Some((input, path, e, j));
                        }
                        rows.push(json!({"initial_site":active,"pitch":pitch,"opening":opening,"pitch_rate":pitch_rate,"yaw":yaw,"time_s":e.time,"old":j.old,"new":new,"co_bearing":matches!(j.new_sites,Sites::Two(_)),"force":force,"native":observations}));
                    }
                }
            }
        }
    }
    fs::write(out.join("returns.json"), serde_json::to_vec(&rows)?)?;
    let (i, path, e, j) = demo.ok_or("missing co-bearing replay")?;
    use super::super::super::edge_view::{self, Replay};
    edge_view::record(
        out,
        Replay {
            registered_name: "Qualified edge active set",
            refinement_name: "Predictive refinement",
            title: "A returning sole vertex joins the existing bearing",
            capture_name: "second_impact",
            impact_tick:1,
            capture_tick: 1,
            last_tick: 26,
            sdf: "return.sdf",
        },
        |label| {
            xml(i,frame(false),false,path.shape.gravity,label)
              .replace("A translating deck meets two vertices of a tilted sliding sole at the initial boundary.","A frictionless moving support bears one vertex of a rocking foot. A second vertex is at touchdown in this measured return state.")
        },
        |h, corners| {
            let EdgeImpactReceipt::Recontact(r) = h else {
                return Err("expected stationary-bearing return".into());
            };
            for (k, id) in [(r.stationary_site, j.old), (1 - r.stationary_site, j.new)] {
                let site = r.sites[k];
                corners[id] += DVec3::from(site.impulse_on_foot).dot(DVec3::from(site.normal));
            }
            Ok((
                r.sites[0].elapsed,
                json!({"original_rocking_time_s":e.time,"old_quadrant":j.old,"new_quadrant":j.new,"native_jump_old_new_ns":[r.sites[r.stationary_site].impulse_on_foot[2],r.sites[1-r.stationary_site].impulse_on_foot[2]],"independent_jump_old_new_ns":j.normal}),
            ))
        },
    )
}
