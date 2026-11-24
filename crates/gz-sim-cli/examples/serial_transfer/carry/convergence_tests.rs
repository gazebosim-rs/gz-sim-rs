// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::prelude::{Res, ResMut};
use gz_physics::BodyState;
use gz_physics_multifidelity::{ContactSegment, RigidContactPatch};
use gz_sim_core::{Phase, StepFault, UpdateInfo};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};

fn fixture(us: u64) -> Result<String> {
    Ok(diagonal_fixture(0.4, 0.28, 10., 8.)?.replace(
        "<max_step_size>0.001<",
        &format!("<max_step_size>{}<", us as f64 / 1e6),
    ))
}

#[derive(Clone, Copy)]
struct Parts {
    parcel: BodyId,
    shelf: BodyId,
    tray: BodyId,
    jaws: [BodyId; 2],
}
impl Parts {
    fn bind(sim: &Simulation) -> Result<Self> {
        let id = |name| part(sim, name)?.body(sim);
        Ok(Self {
            parcel: id("parcel::box")?,
            shelf: id("receiver::shelf")?,
            tray: id("stand::post")?,
            jaws: [id("gripper::left")?, id("gripper::right")?],
        })
    }
    // Upward shelf and tray impulses on the parcel, total shelf normal,
    // and the two unsigned jaw normal impulses from the actual native points.
    fn impacts(&self, patches: &[RigidContactPatch]) -> [f64; 5] {
        let mut values = [0.; 5];
        for p in patches {
            let (other, sign) = if self.parcel == p.a {
                (p.b, 1.)
            } else if self.parcel == p.b {
                (p.a, -1.)
            } else {
                continue;
            };
            for pt in &p.points {
                let j = pt.normal_impulse;
                if self.shelf == other {
                    values[0] += j * sign * p.normal_on_a[2];
                    values[2] += j;
                }
                if self.tray == other {
                    values[1] += j * sign * p.normal_on_a[2];
                }
                if self.jaws[0] == other {
                    values[3] += j;
                }
                if self.jaws[1] == other {
                    values[4] += j;
                }
            }
        }
        values
    }
}

fn state(s: BodyState) -> Value {
    json!({"position_m":s.position,"velocity_m_s":s.velocity,"rotation":s.orientation,"rotation_rad_s":s.angular_velocity})
}
fn incoming(backend: &PhysicsBackend, handles: &[BodyId]) -> Value {
    let bodies = handles
        .iter()
        .map(|id| (id.raw(), state(backend.state(*id).unwrap())))
        .collect::<Vec<_>>();
    let native = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
    let contacts = native
        .rigid_contact_patches()
        .into_iter()
        .map(|p| {
            let points = p
                .points
                .into_iter()
                .map(|pt| {
                    (
                        pt.lever_a,
                        pt.lever_b,
                        pt.normal_impulse,
                        pt.tangent_impulse_on_a,
                    )
                })
                .collect::<Vec<_>>();
            (p.a.raw(), p.b.raw(), p.normal_on_a, points)
        })
        .collect::<Vec<_>>();
    let count = contacts.iter().map(|p| p.3.len()).sum::<usize>();
    let bytes = serde_json::to_vec(&(bodies, contacts)).unwrap();
    json!({"sha256":format!("{:x}",Sha256::digest(bytes)),"bodies":handles.len(),"warm_points":count})
}

#[derive(Debug)]
struct Prepared;
impl std::fmt::Display for Prepared {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("stopped immediately before the prepared native solve")
    }
}
impl std::error::Error for Prepared {}

fn prepared_interval(anchor_ms: u64, native_us: u64, load: u8) -> Result<Value> {
    let xml = fixture(500)?;
    let sdf = sdformat::load_str(&xml, "matched.sdf", &sdformat::NoResources)?;
    let mut sim = world(&xml)?;
    configure_clock(&mut sim, Trial::GuardedCarry, 4000, 2)?;
    format!("6201,64,{load}")
        .parse::<pull::Pull>()?
        .install(&mut sim, Trial::GuardedCarry)?;
    let parts = Parts::bind(&sim)?;
    let mut handles = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<BodyHandle>().map(BodyHandle::id))
        .collect::<Vec<_>>();
    handles.sort_unstable();
    // This disposable ordinary world runs the standard step throughout the prefix.
    // Stop between the actual ordered preparation of inputs and the next solve.
    sim.initialize_schedules()?;
    assert_eq!(
        sim.remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?,
        1
    );
    sim.add_systems(
        Phase::PhysicsStep,
        move |clock: Res<UpdateInfo>, mut engine: ResMut<PhysicsBackend>, fault: Res<StepFault>| {
            if clock.start_time == Duration::from_millis(anchor_ms) {
                fault.report(Prepared);
            } else if let Err(error) = engine.step(clock.dt) {
                fault.report(error);
            }
        },
    );
    let route = Passage::bind(&sim, &sdf)?;
    let stopped = drive(
        &mut sim,
        Trial::GuardedCarry,
        Some(route),
        payload::weight(&sdf)?,
        true,
        |_| Ok(()),
    );
    assert!(stopped.is_err() && sim.fault().is_some_and(|e| e.is::<Prepared>()));
    let efforts = sim
        .world()
        .iter_entities()
        .filter_map(|e| {
            Some((
                e.get::<EntityId>()?.get(),
                e.get::<JointForceCommand>()?.torque(),
            ))
        })
        .collect::<std::collections::BTreeMap<_, _>>();
    let clock = sim.world().resource::<UpdateInfo>();
    assert_eq!(clock.sim_time, Duration::from_millis(anchor_ms));
    // Take the actual initialized engine with warm joints, constraints, motors,
    // and just-queued commands. No ECS clock or foreign physics owner is edited.
    let mut backend = sim.world_mut().remove_resource::<PhysicsBackend>().unwrap();
    let initial = incoming(&backend, &handles);
    let initial_parcel = state(backend.state(parts.parcel).unwrap());
    let engine = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
    let segment = ContactSegment {
        duration: Duration::from_millis(2),
        native_steps: (2000 / native_us) as u32,
        iterations: engine.ordinary_contact_iterations(),
    };
    let mut samples = Vec::new();
    let mut impulses = [0.; 5];
    engine.observe_contact_schedule(segment.duration, &[segment], &[], |end| {
        let impulse = parts.impacts(&end.contact_patches());
        for (total, j) in impulses.iter_mut().zip(impulse) {
            *total += j;
        }
        samples.push(
            json!({"end_us":end.elapsed.as_micros(),"impulse_ns":impulse,
            "parcel":state(end.state(parts.parcel).unwrap())}),
        );
    })?;
    assert_eq!(samples.len(), segment.native_steps as usize);
    Ok(
        json!({"anchor_ms":anchor_ms,"native_us":native_us,"force_n":load,"prepared":initial,
        "input_efforts":efforts,"incoming_parcel":initial_parcel,"samples":samples,"impulse_ns":impulses}),
    )
}

#[test]
#[ignore = "matched ordinary 500 µs prefixes followed by isolated 2 ms native forks"]
fn serial_convergence_matched() -> Result<()> {
    let mut report = Vec::new();
    for (load, anchors) in [
        (8, &[6200, 6308, 6710, 8746, 9128, 9400][..]),
        (12, &[6200, 6262][..]),
    ] {
        for &anchor in anchors {
            let mut lanes = Vec::new();
            for us in [250, 500, 1000, 2000] {
                lanes.push(prepared_interval(anchor, us, load)?);
            }
            for lane in &lanes[1..] {
                assert_eq!(lane["prepared"], lanes[0]["prepared"]);
                assert_eq!(lane["incoming_parcel"], lanes[0]["incoming_parcel"]);
                assert_eq!(lane["input_efforts"], lanes[0]["input_efforts"]);
            }
            report.extend(lanes);
        }
    }
    if let Some(path) = std::env::var_os("GZ_SERIAL_MATCHED_OUTPUT") {
        std::fs::write(path, serde_json::to_vec_pretty(&report)?)?;
    }
    Ok(())
}

#[derive(Default, Resource)]
struct Trace {
    current_impulses: [f64; 5],
    rows: Vec<Value>,
    native: Vec<[f64; 5]>,
}
fn trace_route(us: u64, n: u8) -> Result<(Outcome, Trace)> {
    let xml = fixture(us)?;
    let sdf = sdformat::load_str(&xml, "trace.sdf", &sdformat::NoResources)?;
    let mut sim = world(&xml)?;
    configure_clock(&mut sim, Trial::GuardedCarry, 4000, 2)?;
    format!("6201,64,{n}")
        .parse::<pull::Pull>()?
        .install(&mut sim, Trial::GuardedCarry)?;
    let parts = Parts::bind(&sim)?;
    sim.world_mut().init_resource::<Trace>();
    sim.add_systems(
        Phase::Finalize,
        move |clock: Res<UpdateInfo>, backend: Res<PhysicsBackend>, mut trace: ResMut<Trace>| {
            let engine = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
            let impulse = parts.impacts(&engine.rigid_contact_patches());
            trace.native.push(impulse);
            for (sum, j) in trace.current_impulses.iter_mut().zip(impulse) {
                *sum += j;
            }
            let now = clock.sim_time.as_micros();
            if now.is_multiple_of(2000) {
                let total = std::mem::take(&mut trace.current_impulses);
                let parcel = backend.state(parts.parcel).unwrap();
                trace
                    .rows
                    .push(json!({"ms":now/1000,"impulse_ns":total,"parcel":state(parcel)}));
            }
        },
    );
    let passage = Passage::bind(&sim, &sdf)?;
    let result = drive(
        &mut sim,
        Trial::GuardedCarry,
        Some(passage),
        payload::weight(&sdf)?,
        true,
        |_| Ok(()),
    )?;
    let trace = sim.world_mut().remove_resource::<Trace>().unwrap();
    Ok((result, trace))
}
#[test]
#[ignore = "sixteen full fixed-clock physical set-downs and independent native force measurements"]
fn serial_convergence_traces() -> Result<()> {
    let dir = std::env::var_os("GZ_SERIAL_CONVERGENCE_TRACES").map(PathBuf::from);
    if let Some(d) = &dir {
        std::fs::create_dir_all(d)?;
    }
    for us in [250, 500, 1000, 2000] {
        for n in [0, 6, 8, 12] {
            let (out, trace) = trace_route(us, n)?;
            assert_eq!(trace.rows.len(), 9000);
            assert_eq!(trace.native.len(), 18_000_000 / us as usize);
            let c = out.carry.as_ref().unwrap();
            let dt = us as f64 / 1e6;
            let average = |data: &[[f64; 5]], i| {
                data.iter().map(|row| row[i]).sum::<f64>() / (dt * data.len() as f64)
            };
            let tail = &trace.native[trace.native.len() - 500_000 / us as usize..];
            assert!((average(tail, 0) - out.final_shelf_load_n).abs() < 1e-9);
            assert!((average(tail, 1) - out.final_tray_load_n).abs() < 1e-9);
            assert!((average(tail, 3) + average(tail, 4) - out.final_pad_load_n).abs() < 1e-9);
            let opening = c.supported_open_ms.or(out.release_tick);
            let window=opening.map(|ms| {
                let end=ms*1000/us as usize;
                let data=&trace.native[end-30_000/us as usize..end];
                let mean=average(data,0);
                assert!((0.94*3.924..=1.25*3.924).contains(&mean));
                assert!(data.iter().all(|r|r[2]>0. && r[0]>=0.999*r[2] && r[0]/dt>=0.2*3.924));
                assert!((mean-c.load_at_open_n.or(out.shelf_mean_at_release_n).unwrap()).abs()<1e-9);
                json!({"ms":ms,"mean_up_n":mean,"min_up_n":data.iter().map(|r|r[0]/dt).fold(f64::INFINITY,f64::min),
                    "max_up_n":data.iter().map(|r|r[0]/dt).fold(0.,f64::max),"solves":data.len()})
            });
            assert_eq!(opening.is_some(), n != 12);
            assert_eq!(c.first_alarm_ms.is_some(), n > 0);
            if n != 12 {
                assert!((average(tail, 0) - 3.924).abs() < 0.001 && out.final_pad_load_n == 0.);
            }
            if let Some(d) = &dir {
                let file = d.join(format!("p{us}-n{n}.json"));
                std::fs::write(
                    file,
                    serde_json::to_vec(
                        &json!({"outcome":out,"open_window":window,"trace":trace.rows}),
                    )?,
                )?;
            }
        }
    }
    Ok(())
}

#[test]
fn response_period_validates_the_batch_and_repeats_the_native_motor_after_world_reset() -> Result<()>
{
    use gz_sim_scene::{reset_loaded_world, set_joint_position_response_period};
    let xml = fixture(250)?;
    let mut sim = world(&xml)?;
    let spindle = part(&sim, "gripper::spindle")?;
    let wrist = part(&sim, "gripper::wrist")?;
    let ids = [spindle.id, wrist.id];
    let run = |sim: &mut Simulation| -> Result<Vec<[u64; 2]>> {
        let spindle = part(sim, "gripper::spindle")?;
        set_joint_controller_command(sim, spindle.id, 0.00012)?;
        let mut samples = Vec::new();
        for _ in 0..24 {
            sim.step_exact(1)?;
            let q = sim
                .world()
                .get::<JointSample>(spindle.entity)
                .unwrap()
                .state;
            samples.push([q.position.to_bits(), q.velocity.to_bits()]);
        }
        Ok(samples)
    };
    let horizon = Some(Duration::from_millis(2));
    assert!(set_joint_position_response_period(&mut sim, &ids, Some(Duration::ZERO)).is_err());
    assert!(
        set_joint_position_response_period(&mut sim, &[spindle.id, u64::MAX], horizon).is_err()
    );
    let default = run(&mut sim)?;
    sim.reset()?;
    set_joint_position_response_period(&mut sim, &ids, horizon)?;
    let fixed = run(&mut sim)?;
    assert_ne!(default, fixed);
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(run(&mut sim)?, fixed);
    sim.reset()?;
    set_joint_position_response_period(&mut sim, &ids, None)?;
    assert_eq!(run(&mut sim)?, default);
    let wide = xml
        .replace("<velocity>0.8</velocity>", "<velocity>1e295</velocity>")
        .replace("<cmd_max>0.65</cmd_max>", "<cmd_max>1e295</cmd_max>");
    let mut fast = world(&wide)?;
    let fast_id = part(&fast, "gripper::spindle")?.id;
    assert!(
        set_joint_position_response_period(&mut fast, &[fast_id], Some(Duration::MAX)).is_err()
    );
    let mut reduced = load_world_with_sdf_step(
        include_str!("../../../../../examples/m58_multiple_joint_controllers.sdf"),
        "other.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )?;
    let other = reduced
        .world()
        .iter_entities()
        .find(|e| e.contains::<gz_sim_scene::JointControllerConfiguration>())
        .and_then(|e| e.get::<EntityId>())
        .unwrap()
        .get();
    assert!(set_joint_position_response_period(&mut reduced, &[other], horizon).is_err());
    Ok(())
}

#[test]
fn a_finer_heldout_route_lands_after_slip_and_rejects_overload_on_reset() -> Result<()> {
    let xml = fixture(250)?;
    for n in [8, 12] {
        let pulse = format!("6201,64,{n}").parse()?;
        let out = repeated_clock(&xml, Trial::GuardedCarry, 4000, 2, Some(pulse))?;
        let c = out.carry.unwrap();
        assert_eq!(c.supported_open_ms.is_some(), n == 8);
        assert!(c.first_alarm_ms.is_some() && c.repeated_alarm_ms.is_some());
        if n == 8 {
            assert_eq!(c.sustained_at_open_ms, Some(30));
            assert!((out.final_shelf_load_n - 3.924).abs() < 0.001 && out.final_pad_load_n == 0.);
        }
    }
    Ok(())
}

#[test]
#[ignore = "interleaved fixed-affinity costs for the four fixed public physics clocks"]
fn serial_convergence_cost() -> Result<()> {
    let mut blocks = Vec::new();
    let clocks = [250, 500, 1000, 2000];
    for rotation in 0..clocks.len() {
        for index in 0..clocks.len() {
            let us = clocks[(rotation + index) % clocks.len()];
            let mut result = cost_block(&fixture(us)?, Trial::GuardedCarry, 2, 6201, 64, 8)?;
            result["physics_us"] = us.into();
            blocks.push(result);
        }
    }
    if let Some(p) = std::env::var_os("GZ_SERIAL_CONVERGENCE_COST") {
        std::fs::write(p, serde_json::to_vec_pretty(&blocks)?)?;
    }
    Ok(())
}

fn reset_clock(us: u64) -> Result<()> {
    let xml = fixture(us)?;
    let directory = std::env::var_os("GZ_SERIAL_CONVERGENCE_RESETS").map(PathBuf::from);
    if let Some(dir) = &directory {
        std::fs::create_dir_all(dir)?;
        std::fs::write(dir.join(format!("p{us}.sdf")), &xml)?;
    }
    for n in [0, 6, 8, 12] {
        let pull = format!("6201,64,{n}").parse()?;
        let out = repeated_clock(&xml, Trial::GuardedCarry, 4000, 2, Some(pull))?;
        if let Some(dir) = &directory {
            std::fs::write(
                dir.join(format!("p{us}-n{n}.json")),
                serde_json::to_vec_pretty(&out)?,
            )?;
        }
    }
    Ok(())
}
#[test]
#[ignore = "four full 250 µs routes with exact native pose and controller reset comparison"]
fn serial_convergence_reset_quarter() -> Result<()> {
    reset_clock(250)
}
#[test]
#[ignore = "four full 500 µs routes with exact native pose and controller reset comparison"]
fn serial_convergence_reset_half() -> Result<()> {
    reset_clock(500)
}
#[test]
#[ignore = "four full 1 ms routes with exact native pose and controller reset comparison"]
fn serial_convergence_reset_one() -> Result<()> {
    reset_clock(1000)
}
#[test]
#[ignore = "four full 2 ms routes with exact native pose and controller reset comparison"]
fn serial_convergence_reset_two() -> Result<()> {
    reset_clock(2000)
}

#[test]
fn a_monitored_carry_refuses_a_missing_spindle_or_wrist_before_enabling_its_clock() -> Result<()> {
    let xml = fixture(250)?;
    for joint in ["gripper::spindle", "gripper::wrist"] {
        let mut sim = world(&xml)?;
        let owner = part(&sim, joint)?.entity;
        sim.world_mut().entity_mut(owner).remove::<Name>();
        assert!(configure_clock(&mut sim, Trial::GuardedCarry, 4000, 2).is_err());
        assert!(!sim.world().contains_resource::<cycle::Cadence>());
    }
    let mut sim = world(&xml)?;
    let wrist = part(&sim, "gripper::wrist")?.entity;
    sim.world_mut()
        .entity_mut(wrist)
        .remove::<gz_sim_scene::JointControllerConfiguration>();
    assert!(configure_clock(&mut sim, Trial::GuardedCarry, 4000, 2).is_err());
    assert!(!sim.world().contains_resource::<cycle::Cadence>());
    Ok(())
}
