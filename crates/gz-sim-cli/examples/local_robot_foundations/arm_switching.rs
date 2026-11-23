// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DQuat;
use gz_physics::{BodyId, JointState};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{
    ArticulationTier as Tier, JointHandle, model_fidelity, set_joint_force, set_model_fidelity,
};

struct Observers {
    joint_ids: [u64; 4],
    springs: [(f64, f64); 4],
    links: Vec<(u64, f64, DVec3)>,
    contact_ids: [BodyId; 3],
    model: u64,
}
impl Observers {
    fn new(r: &Recording) -> Result<Self> {
        let id = |s: &str| entity_id(&r.sim, &format!("default::{s}"));
        let joints = [
            "arm::shoulder",
            "arm::lift",
            "arm::close_left",
            "arm::close_right",
        ];
        let mut joint_ids = [0; 4];
        let mut springs = [(0., 0.); 4];
        let robot = r.sdf.worlds[0]
            .models
            .iter()
            .find(|m| m.name == "arm")
            .ok_or("missing SDF arm")?;
        for ((out, spring), name) in joint_ids.iter_mut().zip(&mut springs).zip(joints) {
            *out = id(name)?;
            let joint = robot
                .joints
                .iter()
                .find(|j| name.strip_prefix("arm::") == Some(j.name.as_str()))
                .ok_or("missing SDF joint")?;
            let axis = joint.axis.as_ref().ok_or("missing actuator axis")?;
            *spring = (
                axis.dynamics.spring_stiffness,
                axis.dynamics.spring_reference,
            );
        }
        let mut links = Vec::new();
        for model in &r.sdf.worlds[0].models {
            if model.name != "arm" && model.name != "parcel" {
                continue;
            }
            for link in &model.links {
                let m = &link.inertial;
                let [x, y, z, xy, xz, yz] = m.inertia;
                if [xy, xz, yz] != [0.; 3]
                    || m.pose.transform.translation != [0.; 3]
                    || m.pose.transform.rotation != [0., 0., 0., 1.]
                {
                    return Err("fixture expects centered body-aligned inertia".into());
                }
                links.push((
                    id(&format!("{}::{}", model.name, link.name))?,
                    m.mass,
                    DVec3::new(x, y, z),
                ));
            }
        }
        let get = |s| -> Result<_> {
            let entity = entity_by_id(r.sim.world(), id(s)?).ok_or("missing link")?;
            Ok(r.sim
                .world()
                .get::<BodyHandle>(entity)
                .ok_or("missing handle")?
                .id())
        };
        Ok(Self {
            model: id("arm")?,
            joint_ids,
            springs,
            links,
            contact_ids: [get("arm::left")?, get("arm::right")?, get("parcel::box")?],
        })
    }
    fn sample(&self, s: &Simulation) -> Result<Snapshot> {
        let mut bodies = Vec::new();
        let (mut p, mut l, mut e) = (DVec3::ZERO, DVec3::ZERO, 0.);
        for &(id, m, i) in &self.links {
            let b = state(s, id)?;
            let q = DQuat::from_array(b.orientation);
            let (x, v, w) = (
                DVec3::from(b.position),
                DVec3::from(b.velocity),
                DVec3::from(b.angular_velocity),
            );
            let spin = q * (i * (q.inverse() * w));
            let mv = m * v;
            p += mv;
            l += x.cross(mv) + spin;
            e += m * 9.81 * x.z + 0.5 * (mv.dot(v) + spin.dot(w));
            bodies.push(b);
        }
        let mut joints = Vec::new();
        for id in self.joint_ids {
            let ent = entity_by_id(s.world(), id).ok_or("missing actuator")?;
            let joint = s.world().get::<JointHandle>(ent).ok_or("missing joint")?;
            joints.push(
                s.world()
                    .resource::<PhysicsBackend>()
                    .joint_state(joint.id())
                    .ok_or("missing native coordinate")?,
            );
        }
        for (joint, (stiffness, reference)) in joints.iter().zip(self.springs) {
            e += 0.5 * stiffness * (joint.position - reference).powi(2);
        }
        let backend = s
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("expected local physics")?;
        let [left, right, load] = self.contact_ids;
        let mut impulses = [0.; 2];
        for c in backend.rigid_contact_observations() {
            for (axis, jaw) in [left, right].into_iter().enumerate() {
                if [c.a, c.b] == [jaw, load] || [c.b, c.a] == [jaw, load] {
                    impulses[axis] += c.normal_impulse;
                }
            }
        }
        Ok(Snapshot {
            bodies,
            joints,
            p,
            l,
            e,
            impulses,
        })
    }
}
#[derive(Clone, Debug, PartialEq)]
struct Snapshot {
    bodies: Vec<BodyState>,
    joints: Vec<JointState>,
    p: DVec3,
    l: DVec3,
    e: f64,
    impulses: [f64; 2],
}
#[derive(Default)]
struct ErrorMax {
    position: f64,
    rotation: f64,
    velocity: f64,
    angular: f64,
    coordinate: f64,
    rate: f64,
    momentum: f64,
    moment: f64,
    energy: f64,
    impulse: f64,
}
impl ErrorMax {
    fn compare(&mut self, a: &Snapshot, b: &Snapshot) {
        for (s, t) in a.bodies.iter().zip(&b.bodies) {
            self.position = self
                .position
                .max(DVec3::from(s.position).distance(DVec3::from(t.position)));
            self.rotation = self.rotation.max(
                DQuat::from_array(s.orientation).angle_between(DQuat::from_array(t.orientation)),
            );
            self.velocity = self
                .velocity
                .max(DVec3::from(s.velocity).distance(DVec3::from(t.velocity)));
            self.angular = self
                .angular
                .max(DVec3::from(s.angular_velocity).distance(DVec3::from(t.angular_velocity)));
        }
        for (s, t) in a.joints.iter().zip(&b.joints) {
            self.coordinate = self.coordinate.max((s.position - t.position).abs());
            self.rate = self.rate.max((s.velocity - t.velocity).abs());
        }
        self.momentum = self.momentum.max(a.p.distance(b.p));
        self.moment = self.moment.max(a.l.distance(b.l));
        self.energy = self.energy.max((a.e - b.e).abs());
        for i in 0..2 {
            self.impulse = self.impulse.max((a.impulses[i] - b.impulses[i]).abs());
        }
    }
    fn json(&self) -> Value {
        json!({"position_m":self.position,"rotation_rad":self.rotation,"velocity_m_s":self.velocity,"angular_velocity_rad_s":self.angular,"joint_coordinate_m_or_rad":self.coordinate,"joint_rate_m_s_or_rad_s":self.rate,"linear_momentum_kg_m_s":self.momentum,"angular_momentum_kg_m2_s":self.moment,"mechanical_energy_j":self.energy,"jaw_normal_impulse_ns":self.impulse})
    }
}
const PRE: usize = 1000;
const END: usize = PRE + 6100;

fn compare_runs(
    reference: &mut Recording,
    switched: &mut Recording,
    record: bool,
) -> Result<(Value, Vec<Value>)> {
    let (a, b) = (Observers::new(reference)?, Observers::new(switched)?);
    if a.model != b.model
        || a.joint_ids != b.joint_ids
        || a.links != b.links
        || a.contact_ids != b.contact_ids
    {
        return Err("public identity mismatch".into());
    }
    let (ca, cb) = (
        arm::Controller::new(&reference.sim)?,
        arm::Controller::new(&switched.sim)?,
    );
    let mut errors = ErrorMax::default();
    let mut cuts = ErrorMax::default();
    let mut parks = [None; 2];
    let mut parked = [0; 2];
    let mut wakes = Vec::new();
    let mut refused = None;
    let mut held = None;
    let mut first = [[None; 2]; 2];
    let mut trace = vec![];
    let mut prior = Tier::Dynamic;
    if record {
        reference.capture()?;
        switched.capture()?;
    }
    for tick in 0..END {
        let t = (tick as f64 - PRE as f64) * 0.001;
        if tick >= PRE {
            ca.command(&mut reference.sim, t, false)?;
            cb.command(&mut switched.sim, t, false)?;
        }
        let index = if tick < PRE { 0 } else { 1 };
        let in_window = (650..PRE).contains(&tick) || (PRE + 4900..PRE + 5500).contains(&tick);
        if in_window && parks[index].is_none() {
            let h = b.sample(&switched.sim)?;
            match set_model_fidelity(&mut switched.sim, b.model, Tier::Prescribed) {
                Ok(()) => {
                    parks[index] = Some(tick);
                    cuts.compare(&h, &b.sample(&switched.sim)?);
                }
                Err(e) if tick + 1 == PRE || tick + 1 == PRE + 5500 => {
                    return Err(format!(
                        "never parked in window {index}: {e}; joints={:?}",
                        h.joints
                    )
                    .into());
                }
                Err(gz_sim_scene::FidelityError::Physics(
                    gz_physics::PhysicsError::TransitionDeferred { .. },
                )) => {}
                Err(e) => return Err(e.into()),
            }
        }
        if tick == PRE + 1700 {
            let before = b.sample(&switched.sim)?;
            if before.impulses.iter().any(|f| *f <= 0.) {
                return Err("expected a loaded two-sided grasp".into());
            }
            // Remove the command gate momentarily at the same simulation instant so
            // the physical refusal also has to protect the free parcel.
            let cmds = cb.command(&mut switched.sim, t, false)?;
            for id in b.joint_ids {
                set_joint_force(&mut switched.sim, id, 0.)?;
            }
            let err = set_model_fidelity(&mut switched.sim, b.model, Tier::Prescribed)
                .expect_err("loaded arm unexpectedly parked");
            for (id, effort) in b.joint_ids.into_iter().zip(cmds) {
                set_joint_force(&mut switched.sim, id, effort)?;
            }
            if !err.to_string().contains("external contact")
                || b.sample(&switched.sim)? != before
                || model_fidelity(&switched.sim, b.model)? != Tier::Dynamic
            {
                return Err("refusal mutated state".into());
            }
            held = Some([a.sample(&reference.sim)?.impulses, before.impulses]);
            refused = Some(err.to_string());
        }
        let tier = model_fidelity(&switched.sim, b.model)?;
        reference.advance()?;
        switched.advance()?;
        let after = model_fidelity(&switched.sim, b.model)?;
        if tier == Tier::Prescribed && after == Tier::Prescribed {
            parked[index] += 1;
        }
        if prior == Tier::Prescribed && after == Tier::Dynamic {
            wakes.push(tick + 1);
        }
        prior = after;
        let (x, y) = (a.sample(&reference.sim)?, b.sample(&switched.sim)?);
        errors.compare(&x, &y);
        for (run, obs) in [&x, &y].into_iter().enumerate() {
            for (entry, impulse) in first[run].iter_mut().zip(obs.impulses) {
                if impulse > 0. {
                    entry.get_or_insert((tick + 1, impulse));
                }
            }
        }
        if (tick + 1) % 50 == 0 {
            if record {
                reference.capture()?;
                switched.capture()?;
            }
            trace.push(json!({"tick":tick+1,"prescribed":after==Tier::Prescribed,"reference_jaw_ns":x.impulses,"switched_jaw_ns":y.impulses,"reference_parcel_m":x.bodies.last().unwrap().position,"switched_parcel_m":y.bodies.last().unwrap().position}));
        }
    }
    let final_pose = b.sample(&switched.sim)?.bodies.last().unwrap().position;
    let metrics = json!({"refused":refused,"held_reference_and_switched_jaw_ns":held,"park_tick":parks,"parked_steps":parked,"wake_tick":wakes,"first_jaw_impulse":first,"instant_switch_errors":cuts.json(),"full_trajectory_errors":errors.json(),"placed_m":final_pose});
    if parked[0] < 50
        || parked[1] < 50
        || wakes != [PRE + 1, PRE + 5501]
        || first.iter().flatten().any(Option::is_none)
        || DVec3::from(final_pose).distance(DVec3::new(0., 0.32, 0.17)) > 0.005
        || first[0]
            .iter()
            .zip(first[1])
            .any(|(a, b)| a.map(|v| v.0) != b.map(|v| v.0))
        || cuts.position > 1e-12
        || cuts.rotation > 1e-7
        || cuts.impulse > 1e-12
        || errors.position > 0.001
        || errors.rotation > 0.01
        || errors.velocity > 0.02
        || errors.angular > 0.1
        || errors.impulse > 0.001
        || errors.momentum > 0.01
        || errors.moment > 0.01
        || errors.energy > 0.002
    {
        return Err(format!("arm switching comparison: {metrics}").into());
    }
    Ok((metrics, trace))
}
pub(super) fn run(output: &Path) -> Result<()> {
    let new = || Recording::new(arm::SCENE, "local_manipulation_arm.sdf", "default");
    let (mut a, mut b) = (new()?, new()?);
    let (metrics, trace) = compare_runs(&mut a, &mut b, true)?;
    gz_sim_scene::reset_loaded_world(&mut a.sim, |_| ())?;
    gz_sim_scene::reset_loaded_world(&mut b.sim, |_| ())?;
    let repeated = compare_runs(&mut a, &mut b, false)?;
    if repeated != (metrics.clone(), trace.clone()) {
        return Err("loaded world reset changed switching results".into());
    }
    for r in [&mut a, &mut b] {
        r.costs_ns.truncate(END);
    }
    let mut timing = serde_json::Map::new();
    for (name, r) in [("reference", &a), ("switched", &b)] {
        let sample = |mut times: Vec<u64>| {
            times.sort_unstable();
            json!({"samples":times.len(),"median_ns":times[(times.len()-1)/2],"p95_ns":times[(95*times.len()).div_ceil(100)-1]})
        };
        let indices = metrics["park_tick"]
            .as_array()
            .ok_or("missing park times")?;
        let ranges = [
            (indices[0].as_u64().ok_or("initial park")? as usize, PRE),
            (
                indices[1].as_u64().ok_or("final park")? as usize,
                PRE + 5500,
            ),
        ];
        let parked = ranges
            .into_iter()
            .flat_map(|(lo, hi)| r.costs_ns[lo..hi].iter().copied())
            .collect();
        timing.insert(
            name.to_string(),
            json!({"full":sample(r.costs_ns.clone()),"parked_windows":sample(parked)}),
        );
    }
    write_json(&output.join("timing.json"), &Value::Object(timing))?;
    write_json(&output.join("comparison.json"), &metrics)?;
    write_json(&output.join("trajectory.json"), &json!(trace))?;
    a.save(&output.join("reference"))?;
    b.save(&output.join("switched"))?;
    let views = json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"captures":{"parked_before_approach":16,"grasp":34,"carry":56,"placed":90,"parked_after_release":124,"end":142}});
    for name in ["reference", "switched"] {
        write_json(&output.join(name).join("frames.json"), &views)?;
    }
    Ok(())
}
