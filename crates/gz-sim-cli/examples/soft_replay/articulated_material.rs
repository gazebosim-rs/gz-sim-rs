// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{fixture::*, soft_replay::Frame};
use glam::{DQuat, DVec3};
use gz_sim_scene::deformable;
use serde_json::{Value, json};
const STEEL: [f64; 4] = [0.40, 0.46, 0.54, 1.];
const BLUE: [f64; 4] = [0.13, 0.69, 0.90, 1.];
const GREEN: [f64; 4] = [0.12, 0.71, 0.52, 1.];
const GOLD: [f64; 4] = [0.99, 0.72, 0.15, 1.];
const CORAL: [f64; 4] = [0.99, 0.32, 0.19, 1.];
#[path = "articulated_event.rs"]
mod impact;
pub use impact::event;
pub fn record(g: &Gripper, name: &str, shift: f64, ground: bool, f: &mut Frame) -> Value {
    record_impulses(g, name, shift, ground, f, false)
}
pub fn record_impulses(
    g: &Gripper,
    name: &str,
    shift: f64,
    ground: bool,
    f: &mut Frame,
    net: bool,
) -> Value {
    record_frame(g, name, shift, ground, f, Some(net))
}
pub(super) fn record_frame(
    g: &Gripper,
    name: &str,
    shift: f64,
    ground: bool,
    f: &mut Frame,
    markers: Option<bool>,
) -> Value {
    let sample = deformable::bodies(&g.sim).next().unwrap().1;
    let nodes = sample.particles();
    let pos: Vec<_> = nodes.iter().map(|p| DVec3::from(p.position)).collect();
    let states = g.states();
    let native = g
        .sim
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
        .expect("native articulated material scene");
    let radii = g.ids.map(|(_, id)| {
        native
            .articulated_sphere_geometry(id)
            .expect("spherical gripper")
            .1
    });
    let cloth = pos.len() == 4;
    let turn = DQuat::from_rotation_z(-0.72);
    let display = |p: DVec3| turn * p + shift * DVec3::Y;
    let add_ball = |f: &mut Frame, key: String, p: DVec3, r: f64, c| {
        f.add(
            &format!("{name}_{key}"),
            display(p),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":r}),
            c,
        )
    };
    for (i, s) in states.into_iter().enumerate() {
        let center = s.position.into();
        add_ball(
            f,
            format!("body_{i}"),
            center,
            radii[i],
            if i == 0 { STEEL } else { BLUE },
        );
        if i > 0 {
            let a = pose(states[0], [0.04, if i == 1 { 0.29 } else { -0.29 }, 0.]);
            f.edge(
                &format!("{name}_shank_{i}"),
                display(a),
                display(center),
                0.60,
                0.052,
                STEEL,
            );
            add_ball(f, format!("hinge_{i}"), a, 0.046, STEEL);
        }
    }
    f.edge(
        &format!("{name}_palm"),
        display(pose(states[0], [0.04, 0.29, 0.])),
        display(pose(states[0], [0.04, -0.29, 0.])),
        0.58,
        0.073,
        STEEL,
    );
    if ground {
        f.add(
            &format!("{name}_mount"),
            display(DVec3::new(-0.32, 0., 0.25)),
            turn,
            json!({"kind":"box","size":[0.075,0.075,0.5]}),
            [0.70, 0.73, 0.76, 1.],
        );
    }
    f.add(
        &format!("{name}_base"),
        display(DVec3::new(0., 0., -0.02)),
        turn,
        json!({"kind":"box","size":[1.6,1.19,0.04]}),
        [0.22, 0.25, 0.28, 1.],
    );
    for (i, &p) in pos.iter().enumerate() {
        add_ball(f, format!("node_{i}"), p, 0.022, GREEN);
    }
    let edge = |f: &mut Frame, key: String, a: DVec3, b: DVec3, rest_length: f64, width: f64| {
        f.edge(
            &format!("{name}_{key}"),
            display(a),
            display(b),
            if markers.is_none() {
                a.distance(b)
            } else {
                rest_length
            },
            width,
            GREEN,
        )
    };
    if cloth {
        for (face, ids) in [[0, 1, 3], [0, 3, 2]].into_iter().enumerate() {
            let [a, b, c] = ids.map(|i| pos[i]);
            let span: f64 = 0.52;
            let width: f64 = 0.98;
            let rest = if face == 0 {
                [width, span.hypot(width), span]
            } else {
                [span, width, span.hypot(width)]
            };
            for (dir, (o, p, q)) in [(a, b, c), (b, a, c), (c, a, b)].into_iter().enumerate() {
                for k in 1..=4 {
                    let w = k as f64 / 4.;
                    edge(
                        f,
                        format!("weave_{face}_{dir}_{k}"),
                        o.lerp(p, w),
                        o.lerp(q, w),
                        rest[dir] * w,
                        if k == 4 { 0.011 } else { 0.005 },
                    );
                }
            }
        }
    } else {
        edge(f, "strand".into(), pos[0], pos[1], 1., 0.024);
    }
    let latest = sample.latest_articulated_impact();
    if let Some(net) = markers {
        for tip in 0..2 {
            let rigid = states[tip + 1];
            for side in ["link", "soft"] {
                let (at, j) = latest.map_or((DVec3::from(rigid.position), DVec3::ZERO), |r| {
                    let Some((i, l)) = r
                        .material
                        .loads
                        .iter()
                        .enumerate()
                        .find(|(_, l)| l.site.sphere == tip)
                    else {
                        return (DVec3::from(rigid.position), DVec3::ZERO);
                    };
                    let impulse = l
                        .friction
                        .filter(|_| !net)
                        .map_or(DVec3::from(l.on_rigid), |p| -DVec3::from(p.on_material));
                    if side == "link" {
                        (pose(rigid, r.local_points[i]), impulse)
                    } else {
                        let p = l
                            .site
                            .feature
                            .vertices()
                            .iter()
                            .zip(l.site.weights)
                            .map(|(&i, w)| w * pos[i])
                            .sum::<DVec3>();
                        (p, -impulse)
                    }
                });
                for step in 1..=3 {
                    add_ball(
                        f,
                        format!("{side}_{tip}_{step}"),
                        at + (step as f64 / 3.) * j * 0.85,
                        if step == 3 { 0.023 } else { 0.013 },
                        if side == "link" { CORAL } else { GOLD },
                    );
                }
            }
        }
    }
    let bodies=states.map(|s|json!({"position":s.position,"orientation":s.orientation,"v":s.velocity,"w":s.angular_velocity}));
    let particles = nodes
        .iter()
        .map(|p| json!({"position":p.position,"v":p.velocity,"inverse_mass":p.inverse_mass}))
        .collect::<Vec<_>>();
    json!({"radii":radii,"bodies":bodies,"particles":particles,"event":latest.filter(|r|r.time==g.sim.elapsed()).map(event)})
}
