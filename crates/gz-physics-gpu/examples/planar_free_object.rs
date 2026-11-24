// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a free sphere struck by a joint-driven GPU tool with independent native Rapier.
#[allow(dead_code)]
#[path = "support/planar_arm/native.rs"]
mod native;
#[allow(dead_code)]
#[path = "support/planar_arm/reference.rs"]
mod reference;
use gz_physics_gpu::{
    FreeSphere, PlanarArmLane, PlanarObjectBatch, PlanarObjectLane, PlanarToolLane,
};
#[path = "support/rolling_object.rs"]
mod rolling;
use native::Native;
use rapier3d_f64::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path};

fn fixture(mass: f64, x: f64) -> PlanarObjectLane {
    let arm = PlanarArmLane {
        angle: [0.48, -0.62].map(reference::turn),
        speed: [0.; 2],
        length: [0.30, 0.21],
        center: [0.15, 0.12],
        mass: [0.8, 0.4],
        inertia: [0.0062, 0.0034],
        tip_mass: 0.,
        gravity: 9.81,
        target: [-0.36, -0.50].map(reference::turn),
        stiffness: [16., 5.],
        damping: [2.3, 0.41],
        effort_limit: [8., 2.8],
        effort: [0.; 2],
    };
    PlanarObjectLane {
        tool: PlanarToolLane {
            arm,
            ground_z: 0.05,
            radius: 0.018,
            friction: 0.,
            normal_impulse: 0.,
            tangent_impulse: 0.,
            first_contact_offset: -1.,
            first_normal_impulse: 0.,
            first_tangent_impulse: 0.,
            turns: [0.; 2],
            velocity_mode: [0.; 2],
            velocity_target: [0.; 2],
            stops: Default::default(),
            stop_impulse: [0.; 2],
        },
        object: FreeSphere {
            position: [x, 0.072],
            velocity: [0.; 2],
            radius: 0.022,
            mass,
            normal_impulse: 0.,
            tool_impulse: [0.; 2],
            floor_impulse: 0.,
            surface: Default::default(),
        },
    }
}

struct Oracle {
    arm: Native,
    ball: RigidBodyHandle,
    ball_shape: ColliderHandle,
    tool: ColliderHandle,
}
impl Oracle {
    fn new(l: &PlanarObjectLane) -> Self {
        let mut arm = Native::new(&l.tool.arm);
        let world = &mut arm.world;
        world.integration_parameters.dt = 0.001 / 32.;
        world.integration_parameters.normalized_prediction_distance = 0.;
        // Recompute the round contact normals at every independent native solve.
        world
            .integration_parameters
            .normalized_contact_recycle_distance = 0.;
        world.integration_parameters.normalized_allowed_linear_error = 0.;
        let tool = world.insert_collider(
            ColliderBuilder::ball(l.tool.radius)
                .translation(Vector::X * l.tool.arm.length[1])
                .density(0.)
                .friction(l.tool.friction.max(l.object.surface.tool_mu))
                .friction_combine_rule(CoefficientCombineRule::Min)
                .restitution(0.),
            Some(arm.links[1]),
        );
        world.insert_collider(
            ColliderBuilder::cuboid(10., 2., 0.03)
                .translation(Vector::Z * (l.tool.ground_z - 0.03))
                .friction(l.tool.friction.max(l.object.surface.floor_mu))
                .friction_combine_rule(CoefficientCombineRule::Min)
                .restitution(0.),
            None,
        );
        let [x, z] = l.object.position;
        let [u, v] = l.object.velocity;
        let ball = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, 0., z))
                .linvel(Vector::new(u, 0., v))
                .angvel(-Vector::Y * l.object.surface.spin)
                .can_sleep(false),
        );
        let ball_shape = world.insert_collider(
            ColliderBuilder::ball(l.object.radius)
                .mass(l.object.mass)
                .friction(l.object.surface.tool_mu.max(l.object.surface.floor_mu))
                .friction_combine_rule(CoefficientCombineRule::Min)
                .restitution(0.),
            Some(ball),
        );
        Self {
            arm,
            ball,
            ball_shape,
            tool,
        }
    }
    fn step(&mut self, l: &PlanarObjectLane) -> f64 {
        let mut sum = 0.;
        for _ in 0..32 {
            self.arm
                .step_with_motor(&l.tool.arm, l.tool.velocity_mode, l.tool.velocity_target);
            if let Some(p) = self
                .arm
                .world
                .narrow_phase
                .contact_pair(self.ball_shape, self.tool)
            {
                for m in p.solver_manifolds() {
                    sum += m.points.iter().map(|p| p.data.impulse).sum::<f64>();
                }
            }
        }
        sum
    }
    fn position(&self) -> [f64; 2] {
        let p = self.arm.world.bodies[self.ball].translation();
        [p.x, p.z]
    }
}
#[derive(Default)]
struct Stats {
    first: [Option<usize>; 2],
    total: [f64; 2],
    max_position: f64,
    max_tip: f64,
    max_balance: f64,
    last_gpu: [f64; 2],
    last_native: [f64; 2],
}
fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
/// Independent forward kinematics of the fixture's six authored visuals, including their offsets.
fn visual_sample(tick: usize, lane: &PlanarObjectLane) -> Value {
    let [q1, q2] = lane.tool.joint_angles();
    let [elbow, tip] = lane.tool.arm.points();
    let [x, z] = lane.object.position;
    let pose = |x: f64, z: f64, angle: f64| {
        let (sy, cy) = (-angle * 0.5).sin_cos();
        json!({"translation":[x,0.,z],"rotation":[0.,sy,0.,cy]})
    };
    json!({"time_ns":tick*1_000_000,"poses":{
        "surface":pose(0.51,0.637,0.),
        "hub":pose(0.,0.6,q1),
        "upper":pose(0.5*elbow[0],0.6+0.5*elbow[1],q1),
        "forearm":pose(0.5*(elbow[0]+tip[0]),0.6+0.5*(elbow[1]+tip[1]),q1+q2),
        "tool":pose(tip[0],0.6+tip[1],q1+q2),
        "ball":pose(x,0.6+z,0.)
    }})
}
fn run(adapter: &str, dir: &Path) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(dir)?;
    let inputs = [
        fixture(0.08, 0.503),
        fixture(0.25, 0.503),
        fixture(0.08, 0.90),
    ];
    let mut batch = PlanarObjectBatch::new(adapter, inputs.to_vec())?;
    let mut native = inputs.each_ref().map(Oracle::new);
    let mut stats = inputs.map(|_| Stats::default());
    let mut previous = inputs.map(|l| l.object.velocity);
    let mut frames = Vec::new();
    let mut history = Vec::new();
    let mut visuals = vec![visual_sample(0, &inputs[0])];
    for tick in 1..=400 {
        if tick == 301 {
            for i in 0..inputs.len() {
                batch.set_target_angles(i, [0.82, -0.15])?;
            }
        }
        batch.step()?;
        history.push(batch.lanes().to_vec());
        visuals.push(visual_sample(tick, &batch.lanes()[0]));
        for (i, l) in batch.lanes().iter().enumerate() {
            let s = &mut stats[i];
            let bn = &mut native[i];
            let n = bn.step(l);
            for (j, p) in [l.object.normal_impulse, n].into_iter().enumerate() {
                if p > 0. {
                    s.first[j].get_or_insert(tick);
                }
                s.total[j] += p;
            }
            let tip = l.tool.arm.points()[1];
            let pos = l.object.position;
            s.max_position = s.max_position.max(distance(pos, bn.position()));
            s.max_tip = s.max_tip.max(distance(tip, bn.arm.tip(&l.tool.arm)));
            let dv = [
                l.object.velocity[0] - previous[i][0],
                l.object.velocity[1] - previous[i][1] + 0.001 * l.tool.arm.gravity,
            ];
            let momentum = dv.map(|v| v * l.object.mass);
            let contact = [
                l.object.tool_impulse[0],
                l.object.tool_impulse[1] + l.object.floor_impulse,
            ];
            s.max_balance = s.max_balance.max(distance(momentum, contact));
            previous[i] = l.object.velocity;
            s.last_gpu = pos;
            s.last_native = bn.position();
            if i == 0 {
                frames.push(json!({"tick":tick,"tool":tip,"object":pos,"normal_ns":l.object.normal_impulse,"object_floor_ns":l.object.floor_impulse,"native_object":s.last_native,"native_tool":bn.arm.tip(&l.tool.arm),"native_normal_ns":n}));
            }
        }
    }
    batch.reset();
    for (idx, expected) in history.iter().enumerate() {
        if idx + 1 == 301 {
            for i in 0..inputs.len() {
                batch.set_target_angles(i, [0.82, -0.15])?;
            }
        }
        batch.step()?;
        assert_eq!(
            bytemuck::cast_slice::<_, u8>(batch.lanes()),
            bytemuck::cast_slice::<_, u8>(expected)
        );
    }
    let summary:Vec<Value>=stats.iter().enumerate().map(|(i,s)|json!({"mass_kg":inputs[i].object.mass,"first_gpu_native_ms":s.first,"total_gpu_native_ns":s.total,"max_object_error_m":s.max_position,"max_tool_error_m":s.max_tip,"max_object_momentum_residual_ns":s.max_balance,"final_gpu_xz":s.last_gpu,"final_native_xz":s.last_native})).collect();
    fs::write(dir.join("source-poses.json"), serde_json::to_vec(&visuals)?)?;
    fs::write(dir.join("frames.json"), serde_json::to_vec(&frames)?)?;
    let device = batch.device_info();
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(
            &json!({"adapter":device.name,"backend":device.backend,"driver":device.driver,"driver_info":device.driver_info,"shader_precision":"f64","gpu_dt_us":250,"rapier_dt_us":31.25,"ticks":400,"rows":summary}),
        )?,
    )?;
    println!("{}", serde_json::to_string_pretty(&summary)?);
    for s in &stats[..2] {
        assert_eq!(s.first, [Some(21); 2]);
        assert!(s.total[0] > 0.1 && (s.total[0] / s.total[1] - 1.).abs() < 0.005);
        assert!(s.max_position < 0.00075 && s.max_tip < 0.00025);
    }
    assert!(stats[0].last_gpu[0] > stats[1].last_gpu[0] + 0.05);
    assert_eq!(stats[2].first, [None; 2]);
    assert_eq!(stats[2].total, [0.; 2]);
    assert_eq!(stats[2].last_gpu[0], inputs[2].object.position[0]);
    assert!(stats.iter().all(|s| s.max_balance < 1e-10));
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if let [adapter, dir, mode] = args.as_slice()
        && mode == "--friction"
    {
        return rolling::run(adapter, Path::new(dir));
    }
    if args.len() != 2 {
        return Err("expected adapter name and output directory".into());
    }
    run(&args[0], Path::new(&args[1]))
}
