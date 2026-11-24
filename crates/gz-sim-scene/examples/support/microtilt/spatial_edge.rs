// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Coupled reactions at two simultaneous contacts on a tilted sole.
use super::*;
use glam::DMat3;
#[path = "spatial_edge/authored.rs"]
pub(in super::super) mod authored;
#[path = "spatial_edge/opening.rs"]
pub(in super::super) mod opening;
#[path = "spatial_edge/oracle.rs"]
mod law;
#[path = "spatial_edge/mixed.rs"]
pub(in super::super) mod mixed;
#[path = "spatial_edge/native.rs"]
pub(in super::super) mod transaction;
#[path="spatial_edge/edge_view.rs"]
mod edge_view;
use law::{Edge, Impact};

#[derive(Clone, Copy)]
struct Input {
    seed: Seed,
    edge: Edge,
    indices: [usize; 2],
    desired: [DVec3; 2],
    after_v: DVec3,
    after_w: DVec3,
}
fn make(mu: f64, yaw: f64, roll: f64, load: [f64; 2], reversed: bool) -> Input {
    let qz = DQuat::from_rotation_z(yaw);
    let q = qz * DQuat::from_rotation_x(roll);
    let y = usize::from(roll < 0.);
    let indices = [y, y + 2];
    let arm = indices.map(|k| q * local_arms()[k]);
    let sign = if reversed { -1. } else { 1. };
    let spin = qz * DVec3::new(0.4 * sign, 0., 3.5 * sign);
    let slide = qz * DVec3::new(3. * sign, -1.7, 0.);
    let velocity = DVec3::new(slide.x, slide.y, -spin.cross(arm[0]).z);
    let desired = std::array::from_fn(|i| {
        let vt = (velocity + spin.cross(arm[i])).truncate();
        (-mu * load[i] * vt.normalize()).extend(load[i])
    });
    let mut edge = Edge {
        mass: 2.,
        inertia: DMat3::from_diagonal(DVec3::splat(0.05)),
        arm,
        velocity,
        spin,
        friction: [mu; 2],
    };
    edge.velocity -= (desired[0] + desired[1]) * 0.5;
    edge.spin -= (arm[0].cross(desired[0]) + arm[1].cross(desired[1])) / 0.05;
    let pad = BodyState {
        position: [0., 0., -0.06],
        velocity: [0.17, -0.09, 0.22],
        orientation: [0., 0., 0., 1.],
        angular_velocity: [0.; 3],
    };
    let seed = Seed {
        pad,
        state: PoseState {
            q,
            c: DVec3::Z * (-arm[0].z),
            v: edge.velocity,
            w: edge.spin,
        },
    };
    Input {
        seed,
        edge,
        indices,
        desired,
        after_v: velocity,
        after_w: spin,
    }
}
fn scene(seed: Seed, mu: f64, rigid: bool, policy: &str) -> String {
    seed.scene(mu, rigid, policy)
        .replace("0.66 0.36 0.12", "0.82 0.82 0.12")
}
fn points(patches: &[RigidContactPatch], ids: &Reading, q: DQuat) -> Result<[DVec3; 4]> {
    let mut j = [DVec3::ZERO; 4];
    for patch in patches {
        let sign = if patch.a == ids.foot && patch.b == ids.pad {
            1.
        } else if patch.b == ids.foot && patch.a == ids.pad {
            -1.
        } else {
            continue;
        };
        for p in &patch.points {
            let r = q.conjugate() * DVec3::from(if sign > 0. { p.lever_a } else { p.lever_b });
            let k = usize::from(r.x > 0.) * 2 + usize::from(r.y > 0.);
            let t = DVec3::from(
                p.tangent_impulse_on_a
                    .ok_or("edge point friction observations unavailable")?,
            );
            j[k] += sign * (DVec3::from(patch.normal_on_a) * p.normal_impulse + t);
        }
    }
    Ok(j)
}
struct NativePoints {
    impulses: [DVec3; 2],
    end: PoseState,
    start_error: f64,
    angular_balance: f64,
}
fn native_points(input: Input, rigid: bool, passes: usize) -> Result<NativePoints> {
    let mut sim = load(
        &scene(input.seed, input.edge.friction[0], rigid, "ordinary"),
        true,
    )?;
    let ids = ids(&sim)?;
    let mut p = sim.world_mut().resource_mut::<PhysicsBackend>();
    let n = p.downcast_mut::<MultiFidelityBackend>().unwrap();
    let initial = Seed::read(departure::ingest(n, &ids)).state;
    let delta = near(initial, input.seed.state);
    if delta > 1e-12 {
        return Err(format!("edge incoming states differ {delta:e}").into());
    }
    let mut observed = None;
    n.observe_contact_interval(
        Duration::from_nanos(1),
        1,
        ContactSolverIterations {
            temporal: 4,
            pgs: Some(passes),
            relaxation: Some(passes),
        },
        |s| observed = Some(points(&s.contact_patches(), &ids, initial.q)),
    )?;
    let all = observed.ok_or("missing edge solver point data")??;
    let j = input.indices.map(|k| all[k]);
    if (0..4).any(|k| !input.indices.contains(&k) && all[k].length() > 1e-12) {
        return Err("non-touching edge vertex received a native reaction".into());
    }
    let committed = Seed::read(departure::ingest(n, &ids)).state;
    let angular = (committed.w - initial.w) * 0.05;
    let balance = angular.distance(input.edge.arm[0].cross(j[0]) + input.edge.arm[1].cross(j[1]));
    Ok(NativePoints {
        impulses: j,
        end: committed,
        start_error: delta,
        angular_balance: balance,
    })
}
fn native(input: Input, ideal: &Impact, rigid: bool, passes: usize) -> Result<Value> {
    let NativePoints {
        impulses: j,
        end: committed,
        start_error: delta,
        angular_balance: balance,
    } = native_points(input, rigid, passes)?;
    let per = std::array::from_fn::<_, 2, _>(|i| j[i].distance(ideal.impulse[i]));
    let point_v = input.edge.arm.map(|r| committed.v + committed.w.cross(r));
    Ok(
        json!({"single_rigid":rigid,"passes":passes,"start_error":delta,"impulses_ns":j.map(|j|j.to_array()),"point_velocities_m_s":point_v.map(|u|u.to_array()),
        "site_errors_ns":per,"total_error_ns":(j[0]+j[1]).distance(ideal.impulse[0]+ideal.impulse[1]),"spin_error_rad_s":committed.w.distance(ideal.spin),"angular_balance_nms":balance,"state":committed.wire()}),
    )
}
fn sequential(mut e: Edge, reverse: bool) -> Result<Value> {
    let mut j = [DVec3::ZERO; 2];
    for i in if reverse { [1, 0] } else { [0, 1] } {
        if e.point(i).z >= 0. {
            continue;
        }
        let m = DMat3::from_cols_array_2d(&DVec3::AXES.map(|v| e.mobility(i, i, v).to_array()));
        let one = gz_physics::PointImpact {
            inverse_mass: m.transpose().to_cols_array_2d(),
            relative_velocity: e.point(i).to_array(),
            friction: e.friction[i],
        }
        .solve()?;
        j[i] = one.impulse.into();
        e.velocity += j[i] / e.mass;
        e.spin += e.inertia.inverse() * e.arm[i].cross(j[i]);
    }
    Ok(
        json!({"reverse":reverse,"impulses_ns":j.map(|a|a.to_array()),"point_velocities_m_s":[e.point(0).to_array(),e.point(1).to_array()]}),
    )
}
fn record(out: &Path) -> Result<()> {
    let i = make(0.35, 0.67, 0.18, [0.4, 0.8], false);
    let ideal = i.edge.solve().map_err(|e| format!("{e:?}"))?;
    let mut all = vec![];
    for label in ["ordinary", "pgs32", "coupled"] {
        let xml = scene(i.seed, 0.35, false, "ordinary");
        let mut s = load(&xml, true)?;
        let mut r = ids(&s)?;
        if label == "pgs32" {
            s.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .set_contact_pgs_passes(32)?;
        }
        let mut rows = vec![];
        let mut corners = [0.; 4];
        let mut native = 0;
        for tick in 0..=40 {
            if tick > 0 {
                if label == "coupled" && tick == 1 {
                    let after = Seed {
                        state: PoseState {
                            v: ideal.velocity,
                            w: ideal.spin,
                            ..i.seed.state
                        },
                        ..i.seed
                    };
                    s = load(&scene(after, 0.35, false, "ordinary"), true)?;
                    r = ids(&s)?;
                    if near(
                        Seed::read(departure::ingest(engine(&s), &r)).state,
                        after.state,
                    ) > 1e-12
                    {
                        return Err("coupled jump native endpoint mismatch".into());
                    }
                }
                s.step_exact(1)?;
                native = 1;
                let b = engine(&s);
                let q = DQuat::from_array(b.state(r.foot).unwrap().orientation);
                corners = points(&b.rigid_contact_patches(), &r, q)?.map(|v| v.z);
                if tick == 1 && label == "coupled" {
                    for k in 0..2 {
                        corners[i.indices[k]] += ideal.normal[k];
                    }
                }
            }
            let (f, u, p) = departure::ingest(engine(&s), &r);
            let loaded = corners.iter().sum::<f64>();
            rows.push(json!({"tick":tick,"time_us":tick*500,"native_steps":native,"solver_iterations":4,"selected":native>1,"weak_start":false,"secondary_start":false,"normal_impulse_ns":loaded,"corner_ns":corners,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"gap_m":f.position[2]-p.position[2]-0.06-q_up(f).abs().dot(DVec3::new(0.18,0.1,0.06))}));
        }
        fs::write(
            out.join(format!("traces/{label}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
            )?,
        )?;
        all.push(json!({"label":label,"first":rows[1],"last":rows.last()}));
        if label == "ordinary" {
            fs::write(out.join("edge.sdf"), &xml)?;
        }
    }
    let lanes = [
        ("ordinary", "Ordinary locked edge"),
        ("pgs32", "32 fixed-time PGS passes"),
        ("coupled", "Coupled physical jump, then native"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Two simultaneous sliding edge contacts","capture_ticks":{"first_interval":1},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&all)?)?;
    Ok(())
}
pub(in super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    let mut declines = vec![];
    for (family, load, reverse, roll) in [
        ("outer-heavy", [0.4, 0.8], false, 0.18),
        ("inner-heavy", [0.75, 0.45], true, 0.18),
        ("other-edge", [0.65, 0.55], false, -0.18),
        ("reverse-slide", [0.5, 0.7], true, -0.18),
    ] {
        for mu in [0.08, 0.2, 0.35, 0.5] {
            for yaw_deg in [0., 27., 68., 112.] {
                let input = make(mu, f64::to_radians(yaw_deg), roll, load, reverse);
                let e = input.edge;
                let ideal = match e.solve() {
                    Ok(r) => r,
                    Err(why) => {
                        declines.push(json!({"case":family,"mu":mu,"yaw_deg":yaw_deg,"reason":format!("{why:?}")}));
                        continue;
                    }
                };
                let mut controls = vec![];
                for single in [false, true] {
                    for passes in [1, 8, 32] {
                        controls.push(native(input, &ideal, single, passes)?);
                    }
                }
                let exact = ideal
                    .impulse
                    .iter()
                    .zip(input.desired)
                    .map(|(a, b)| a.distance(b))
                    .fold(0., f64::max);
                if exact > 1e-12
                    || ideal.velocity.distance(input.after_v) > 1e-12
                    || ideal.spin.distance(input.after_w) > 1e-12
                {
                    return Err("manufactured simultaneous edge solution mismatch".into());
                }
                rows.push(json!({"case":family,"mu":mu,"yaw_deg":yaw_deg,"indices":input.indices,"arms_m":e.arm.map(|r|r.to_array()),"before":input.seed.state.wire(),"target_loads_ns":load,"desired_ns":input.desired.map(|v|v.to_array()),
                "impulses_ns":ideal.impulse.map(|v|v.to_array()),"point_velocities_m_s":ideal.point_velocity.map(|v|v.to_array()),"normal_ns":ideal.normal,"velocity_m_s":ideal.velocity.to_array(),"spin_rad_s":ideal.spin.to_array(),"dissipation_j":ideal.dissipated,
                "contraction_bound":ideal.contraction,"normal_speed_upper_if_other_alone":ideal.singleton_opening_upper,"slip_speed_floor":ideal.slip_floor,"fixed_point_iterations":ideal.iterations,"iterate_error_bound_ns":ideal.impulse_error_bound,
                "native":controls,"sequential":[sequential(e,false)?,sequential(e,true)?]}));
            }
        }
    }
    fs::write(out.join("edge.json"), serde_json::to_vec(&rows)?)?;
    fs::write(
        out.join("declines.json"),
        serde_json::to_vec_pretty(&declines)?,
    )?;
    record(out)
}
