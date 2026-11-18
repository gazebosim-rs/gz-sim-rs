// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Source controls for resolved link inertia; qualification waits for the parent.
use gz_physics::{CollisionSpec, Shape};
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;

fn quaternion([roll, pitch, yaw]: [f64; 3]) -> [f64; 4] {
    (glam::DQuat::from_rotation_z(yaw)
        * glam::DQuat::from_rotation_y(pitch)
        * glam::DQuat::from_rotation_x(roll))
    .to_array()
}
fn center(count: usize) -> [f64; 3] {
    match count {
        1 => [0.03, -0.02, 0.05],
        2 => [0.1, -0.05, 0.2],
        3 => [0., 0., 0.5],
        _ => unreachable!(),
    }
}
fn fixtures(count: usize) -> (String, Vec<CollisionSpec>) {
    let collision = |index: usize, pose: &str, geometry: &str| {
        format!(
            "<collision name='c{index}'><pose>{pose}</pose><geometry>{geometry}</geometry></collision>"
        )
    };
    let identity = [0., 0., 0., 1.];
    if count == 1 {
        return (
            collision(0, "0 0 0 0 0 0", "<sphere><radius>0.2</radius></sphere>"),
            vec![CollisionSpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.; 3],
                orientation: identity,
            }],
        );
    }
    let cylinder_x = if count == 2 { 0.4 } else { 0. };
    let mut xml = collision(0, "-0.4 0 0 0 0 0", "<box><size>0.6 0.4 0.8</size></box>")
        + &collision(
            1,
            &format!("{cylinder_x} 0 0 0 1.5708 0.3"),
            "<cylinder><radius>0.2</radius><length>1</length></cylinder>",
        );
    let mut specs = vec![
        CollisionSpec {
            shape: Shape::Box {
                half_extents: [0.3, 0.2, 0.4],
            },
            position: [-0.4, 0., 0.],
            orientation: identity,
        },
        CollisionSpec {
            shape: Shape::Cylinder {
                radius: 0.2,
                half_height: 0.5,
            },
            position: [cylinder_x, 0., 0.],
            orientation: quaternion([0., "1.5708".parse().unwrap(), 0.3]),
        },
    ];
    if count == 3 {
        xml += &collision(
            2,
            "0.4 0 0 0.2 0.4 -0.1",
            "<capsule><radius>0.2</radius><length>0.8</length></capsule>",
        );
        specs.push(CollisionSpec {
            shape: Shape::Capsule {
                radius: 0.2,
                half_length: 0.4,
            },
            position: [0.4, 0., 0.],
            orientation: quaternion([0.2, 0.4, -0.1]),
        });
    }
    (xml, specs)
}
fn xml(count: usize) -> String {
    let [x, y, z] = center(count);
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='compound'><link name='body'><pose>0 0 2 0.2 0.1 -0.3</pose><inertial><pose>{x} {y} {z} 0.1 0.2 0.3</pose><mass>2</mass><inertia><ixx>1</ixx><iyy>1.5</iyy><izz>2</izz></inertia></inertial>{}</link></model></world></sdf>",
        fixtures(count).0
    )
}

#[test]
fn offset_com_stays_outside_other_engines_and_planes() {
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        assert!(matches!(
            load_world_with_sdf_step(&xml(1), "offset-com.sdf", "default", &NoResources, engine),
            Err(gz_sim_scene::LoadError::Unsupported {
                feature: "offset center of mass",
                ..
            })
        ));
    }
}
