// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{grasp::*, soft_replay::Frame};
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
const SCALE: f64 = 8.;
const GOLD: [f64; 4] = [1., 0.72, 0.20, 1.];
const BLUE: [f64; 4] = [0.17, 0.37, 0.66, 1.];
type Command = [(DVec3, DVec3); 2];
pub(super) fn sample(run: &Grasp, name: &str, dy: f64, f: &mut Frame, cmd: Command) -> Value {
    let color = if matches!(name, "loaded" | "force_aware") {
        [0.12, 0.83, 0.78, 1.]
    } else {
        [0.94, 0.43, 0.16, 1.]
    };
    let pos = |p: DVec3| SCALE * p + DVec3::Y * dy;
    let mut jaws = vec![];
    for (i, s) in run.jaws().into_iter().enumerate() {
        let c = pos(s.position.into());
        let q = DQuat::from_array(s.orientation);
        jaws.push(json!({"center":s.position,"orientation":s.orientation,"velocity":s.velocity,"spin":s.angular_velocity,"force":cmd[i].0.to_array(),"torque":cmd[i].1.to_array()}));
        for (j, sign) in [("base", -1.), ("tip", 1.)] {
            f.add(
                &format!("{name}_jaw_{i}_{j}"),
                c + q * (DVec3::Z * (sign * SCALE * HALF_LENGTH)),
                q,
                json!({"kind":"sphere","radius":SCALE*RADIUS}),
                BLUE,
            );
        }
        for j in 0..8 {
            let o = q * DQuat::from_rotation_z(j as f64 * std::f64::consts::TAU / 8.);
            f.add(&format!("{name}_jaw_{i}_barrel_{j}"),c+o*(DVec3::X*SCALE*0.96*RADIUS),o,json!({"kind":"box","size":[SCALE*0.0007,2.*SCALE*RADIUS*(std::f64::consts::PI/8.).tan(),2.*SCALE*HALF_LENGTH]}),BLUE);
        }
    }
    let cloth = run.cloth();
    let nodes = cloth
        .particles()
        .iter()
        .map(|p| pos(p.position.into()))
        .collect::<Vec<_>>();
    for (i, &p) in nodes.iter().enumerate() {
        f.add(
            &format!("{name}_vertex_{i}"),
            p,
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":SCALE*SKIN}),
            color,
        );
    }
    let edge = |f: &mut Frame, name: &str, p: DVec3, q: DVec3, length: f64, w: f64| {
        f.edge(name, p, q, length, w, color);
    };
    let rest = |a: usize, b: usize| {
        let xy = |i: usize| DVec3::new((i % 2) as f64, 0., (i / 2) as f64);
        0.12 * SCALE * xy(a).distance(xy(b))
    };
    for (i, (a, b)) in [(0, 1), (1, 3), (3, 2), (2, 0), (0, 3)]
        .into_iter()
        .enumerate()
    {
        edge(
            f,
            &format!("{name}_edge_{i}"),
            nodes[a],
            nodes[b],
            rest(a, b),
            0.025,
        );
    }
    // Each line is an affine material coordinate in a real triangle.
    for (k, [a, b, c]) in [[0, 1, 3], [0, 3, 2]].into_iter().enumerate() {
        for i in 1..6 {
            let s = i as f64 / 6.;
            edge(
                f,
                &format!("{name}_weft_{k}_{i}"),
                nodes[a].lerp(nodes[b], s),
                nodes[c].lerp(nodes[b], s),
                (1. - s) * rest(a, c),
                0.013,
            );
            edge(
                f,
                &format!("{name}_warp_{k}_{i}"),
                nodes[b].lerp(nodes[a], s),
                nodes[c].lerp(nodes[a], s),
                (1. - s) * rest(b, c),
                0.013,
            );
        }
    }
    let dt = run.sim.step_duration().as_secs_f64();
    let support = -cloth
        .last_capsule_friction_loads()
        .iter()
        .map(|l| l.linear[2])
        .sum::<f64>()
        / dt;
    let origin = DVec3::new(-0.50, dy - 0.24, 0.055);
    f.add(
        &format!("{name}_weight_scale"),
        origin + DVec3::X * 0.4,
        DQuat::IDENTITY,
        json!({"kind":"box","size":[0.8,0.030,0.025]}),
        [0.25, 0.31, 0.36, 1.],
    );
    f.add(
        &format!("{name}_one_weight"),
        origin + DVec3::X * 0.52,
        DQuat::IDENTITY,
        json!({"kind":"box","size":[0.016,0.075,0.050]}),
        [0.93, 0.93, 0.89, 1.],
    );
    f.add(
        &format!("{name}_traction"),
        origin
            + DVec3::X * (0.52 * (support / (CLOTH_MASS * GRAVITY)).clamp(0., 1.5))
            + DVec3::Z * 0.028,
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":0.032}),
        GOLD,
    );
    f.add(
        &format!("{name}_table"),
        DVec3::new(0.17, dy, -0.025),
        DQuat::IDENTITY,
        json!({"kind":"box","size":[1.75,0.90,0.05]}),
        [0.20, 0.24, 0.27, 1.],
    );
    let particles = cloth
        .particles()
        .iter()
        .map(|p| json!({"position":p.position,"velocity":p.velocity,"mass":1./p.inverse_mass}))
        .collect::<Vec<_>>();
    let reactions = cloth
        .last_boundary_impulses()
        .iter()
        .map(|r| json!({"boundary":r.boundary,"linear":r.linear,"angular":r.angular}))
        .collect::<Vec<_>>();
    let normal=cloth.last_capsule_material_loads().iter().map(|l|json!({"boundary":l.boundary,"vertices":l.feature.vertices(),"region":format!("{:?}",l.region),"two_way":l.two_way,"linear":l.linear,"angular":l.angular,"nodal":l.nodal,"first_time":l.first_time})).collect::<Vec<_>>();
    let friction=cloth.last_capsule_friction_loads().iter().map(|l|json!({"boundary":l.boundary,"vertices":l.feature.vertices(),"region":format!("{:?}",l.region),"two_way":l.two_way,"linear":l.linear,"angular":l.angular,"nodal":l.nodal,"normal_sum":l.normal_impulse_sum,"tangent_sum":l.tangent_impulse_sum,"last_time":l.last_time,"weights":l.last_weights,"point":l.last_point,"normal":l.last_normal,"slip":l.last_slip_velocity,"last_jn":l.last_normal_impulse,"last_jt":l.last_tangent_impulse,"normal_weighted_slip":l.normal_weighted_slip,"slip_distance":l.slip_distance})).collect::<Vec<_>>();
    json!({"jaws":jaws,"particles":particles,"reactions":reactions,"normal":normal,"friction":friction,"support_n":support,"center":run.center().to_array(),"squeeze_n":run.force,"refinements":cloth.last_sweep_refinements()})
}
