// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Drive the ordinary SDF GPU arm over Gazebo transport and compare every
//! published joint state with the same commands applied at local ECS boundaries.
use gz_sim_proto::{
    msgs::{Boolean, WorldControl},
    prost::Message,
};
use gz_sim_scene::{
    PhysicsEngine, WorldLoadOptions, gpu_scene::planar_tool::observe,
    initial_joint_controller_configurations, load_world_on_adapter, set_joint_controller_command,
};
use serde_json::json;
use std::{
    error::Error,
    fs::File,
    net::TcpListener,
    path::PathBuf,
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};
use zenoh::Wait;
#[path = "support/gpu_joint_wire.rs"]
mod wire;

type Result<T> = std::result::Result<T, Box<dyn Error + Send + Sync>>;
const WAIT: Duration = Duration::from_secs(10);
const STOPPED_WORLD: &str = include_str!("../../../examples/gpu_stopped_tool_world.sdf");
const WORLD: &str = include_str!("../../../examples/gpu_commanded_tool_world.sdf");
struct Server(Child);
impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
fn advance(session: &zenoh::Session, key: &str, count: u32) -> Result<()> {
    control(
        session,
        key,
        WorldControl {
            pause: true,
            multi_step: count,
            ..Default::default()
        },
    )
}
fn control(session: &zenoh::Session, key: &str, control: WorldControl) -> Result<()> {
    let replies = session
        .get(key)
        .payload(control.encode_to_vec())
        .timeout(WAIT)
        .wait()?;
    let response = replies
        .recv_timeout(WAIT)?
        .ok_or("missing world-control response")?
        .into_result()
        .map_err(|_| "world-control service error")?;
    if !Boolean::decode(response.payload().to_bytes().as_ref())?.data {
        return Err("world control refused".into());
    }
    Ok(())
}
fn main() -> Result<()> {
    let mut args: Vec<_> = std::env::args().skip(1).collect();
    let after_reset = args.last().is_some_and(|a| a == "--after-reset");
    if after_reset {
        args.pop();
    }
    let stopped = args.last().is_some_and(|a| a == "--stops");
    if stopped {
        args.pop();
    }
    let scene = if stopped { STOPPED_WORLD } else { WORLD };
    let [exe, adapter, path]: [String; 3] = args.try_into().map_err(
        |_| "usage: gpu_joint_probe GZ_SIM_RS ADAPTER NEW_OUTPUT_DIRECTORY [--stops] [--after-reset]",
    )?;
    let out = PathBuf::from(path);
    std::fs::create_dir(&out)?;
    let server_log = File::create_new(out.join("server.log"))?;
    let mut sim = load_world_on_adapter(
        scene,
        "gpu_commanded_tool_world.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(&adapter),
    )?;
    let ids: Vec<_> = initial_joint_controller_configurations(&sim)?
        .map(|(id, c)| (id, c.topic.clone()))
        .collect();
    let network = zenoh::Config::from_json5(
        r#"{mode:"peer",listen:{endpoints:["tcp/127.0.0.1:0"]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#,
    )?;
    let session = zenoh::open(network).wait()?;
    let endpoints: Vec<_> = session
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|l| l.to_string())
        .collect();
    let scratch = tempfile::tempdir()?;
    let config = scratch.path().join("zenoh.json");
    wire::save(
        &config,
        &json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":endpoints,"timeout_ms":2000},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"transport":{"shared_memory":{"enabled":false}}}),
    )?;
    let world = scratch.path().join("world.sdf");
    std::fs::write(&world, scene)?;
    let partition = format!("arm-command-{}", std::process::id());
    let prefix = format!("@/{partition}@");
    let state = session
        .declare_subscriber(format!("{prefix}/world/default/model/arm/joint_state"))
        .wait()?;
    let publishers = ids
        .iter()
        .map(|(_, topic)| session.declare_publisher(format!("{prefix}{topic}")).wait())
        .collect::<std::result::Result<Vec<_>, _>>()?;
    let port = TcpListener::bind(("127.0.0.1", 0))?.local_addr()?.port();
    let mut command = Command::new(exe);
    command.arg(&world).args([
        "--server",
        "--physics-engine",
        "gpu-batch",
        "--gpu-adapter",
        &adapter,
        "--iterations",
        if after_reset { "701" } else { "601" },
    ]);
    if !after_reset {
        command
            .args([
                "--web-viewer",
                &port.to_string(),
                "--web-lifecycle",
                "--record-scene",
            ])
            .arg(out.join("replay"));
    }
    let mut server = Server(
        command
            .env("ZENOH_CONFIG", config)
            .env("GZ_PARTITION", &partition)
            .stdout(server_log.try_clone()?)
            .stderr(server_log)
            .stdin(Stdio::null())
            .spawn()?,
    );
    let deadline = Instant::now() + WAIT;
    for publisher in &publishers {
        while !publisher.matching_status().wait()?.matching() {
            if server.0.try_wait()?.is_some() || Instant::now() >= deadline {
                return Err("joint-command subscriber unavailable; see server.log".into());
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }
    let control_key = format!("{prefix}/world/default/control");
    if after_reset {
        wire::reset_loaded(&session, &prefix, &state, &publishers[0])?;
    }
    let mut trace = Vec::new();
    let mut tick = 0_u32;
    let mut first = None;
    let mut last = None;
    let mut impulse = 0.;
    let mut peak_effort = [0_f64; 2];
    let mut max_speed = [0_f64; 2];
    let mut max_target = [0_f64; 2];
    let mut max_residual = 0_f64;
    let mut target = [0.; 2];
    loop {
        if tick == 1 || tick == 300 {
            target = if tick == 1 { [-8., 0.12] } else { [8., 0.47] };
            for (i, publisher) in publishers.iter().enumerate() {
                wire::command(publisher, target[i])?;
                set_joint_controller_command(&mut sim, ids[i].0, target[i])?;
            }
            // Round-trip through the paused server before the next stepped interval.
            advance(&session, &control_key, 0)?;
        }
        let next = match tick {
            0 => 1,
            1..=299 => (tick + 10).min(300),
            300..=599 => (tick + 10).min(600),
            _ => break,
        };
        advance(&session, &control_key, next - tick)?;
        while tick < next {
            let sample = wire::Feedback::at(&state, tick + 1)?;
            tick = sample.tick;
            sim.step_exact(1)?;
            let lane = observe(&sim, 7)?;
            let position = sample.position;
            let velocity = sample.velocity;
            for i in 0..2 {
                max_residual = max_residual
                    .max((position[i] - (lane.joint_angles()[i] - [0.48, -0.62][i])).abs())
                    .max((velocity[i] - lane.arm.speed[i]).abs());
                peak_effort[i] = peak_effort[i].max(lane.arm.effort[i].abs());
                max_speed[i] = max_speed[i].max(velocity[i].abs());
                max_target[i] = max_target[i].max(lane.velocity_target[i].abs());
            }
            if lane.normal_impulse > 0. {
                first.get_or_insert(tick);
                last = Some(tick);
                impulse += lane.normal_impulse;
            }
            trace.push(json!({"tick_ms":tick,"command_rad":target,"position_rad":position,"velocity_rad_s":velocity,"motor_demand_rad_s":lane.velocity_target,"motor_torque_nm":lane.arm.effort,"stop_impulse_nms":lane.stop_impulse,"normal_impulse_ns":lane.normal_impulse,"gap_m":lane.gap()}));
        }
    }
    advance(&session, &control_key, 1)?;
    sim.step_exact(1)?;
    let terminal = observe(&sim, 7)?;
    let position: [f64; 2] =
        std::array::from_fn(|axis| terminal.joint_angles()[axis] - [0.48, -0.62][axis]);
    let final_state = json!({"tick_ms":601,"source":"matched local hardware","position_rad":position,"velocity_rad_s":terminal.arm.speed,"normal_impulse_ns":terminal.normal_impulse,"gap_m":terminal.gap()});
    wire::save(&out.join("terminal.json"), &final_state)?;
    let deadline = Instant::now() + WAIT;
    loop {
        if let Some(status) = server.0.try_wait()? {
            assert!(status.success());
            break;
        }
        if Instant::now() >= deadline {
            return Err("server did not finish its last iteration".into());
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(max_residual < 1e-12 && first.is_some() && last.unwrap() < 600);
    let summary = json!({"adapter":adapter,"authored_stops":stopped,"after_transport_reset":after_reset,"frames":trace.len(),"first_loaded_tick":first,"last_loaded_tick":last,"normal_impulse_ns":impulse,"max_torque_nm":peak_effort,"max_measured_speed_rad_s":max_speed,"max_motor_demand_rad_s":max_target,"max_joint_transport_residual_si":max_residual});
    wire::save(&out.join("trace.json"), &serde_json::Value::Array(trace))?;
    wire::save(&out.join("comparison.json"), &summary)?;
    println!("{summary}");
    Ok(())
}
