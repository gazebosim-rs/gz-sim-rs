// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure a local, automatically promoted arm against an always-dynamic reference.
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, JointState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, JointSample, Name, PhysicsEngine, load_world_with_sdf_step,
    model_fidelity,
};
use serde::Serialize;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_free_payload_impact.sdf");
const NAMES: [&str; 3] = ["arm::base", "arm::tip", "payload::body"];
const STEPS: u64 = 600;
const DT: f64 = 0.001;

struct Lane {
    sim: Simulation,
    arm: u64,
    joint: u64,
    ids: [u64; 3],
    handles: [BodyId; 3],
}

impl Lane {
    fn load(xml: &str) -> Result<Self> {
        let sim = load_world_with_sdf_step(
            xml,
            "free_payload_impact.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let id = |name: &str| -> Result<u64> {
            let name = format!("default::{name}");
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .and_then(|e| e.get::<EntityId>())
                .map(|v| v.get())
                .ok_or_else(|| format!("missing {name}").into())
        };
        let ids = [id(NAMES[0])?, id(NAMES[1])?, id(NAMES[2])?];
        let handles = ids.map(|id| {
            sim.world()
                .get::<BodyHandle>(entity_by_id(sim.world(), id).unwrap())
                .unwrap()
                .id()
        });
        Ok(Self {
            arm: id("arm")?,
            joint: id("arm::elbow")?,
            sim,
            ids,
            handles,
        })
    }
    fn observe(&self) -> Result<Observation> {
        let backend = self.sim.world().resource::<PhysicsBackend>();
        let [base, tip, body] = self.handles;
        let bodies = [
            backend.state(base).ok_or("missing base state")?,
            backend.state(tip).ok_or("missing tip state")?,
            backend.state(body).ok_or("missing payload state")?,
        ];
        let contacts = backend
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local backend")?
            .rigid_contact_observations();
        let impulse = contacts
            .iter()
            .filter(|c| {
                (c.a == body && [base, tip].contains(&c.b))
                    || (c.b == body && [base, tip].contains(&c.a))
            })
            .map(|c| c.normal_impulse)
            .sum();
        let joint = self
            .sim
            .world()
            .get::<JointSample>(entity_by_id(self.sim.world(), self.joint).ok_or("missing elbow")?)
            .ok_or("missing elbow sample")?
            .state;
        Ok(Observation {
            bodies,
            impulse,
            joint,
            tier: model_fidelity(&self.sim, self.arm)?,
        })
    }
}

struct Observation {
    bodies: [BodyState; 3],
    joint: JointState,
    impulse: f64,
    tier: ArticulationTier,
}

fn pose_rows(obs: &Observation, ids: &[u64; 3]) -> Value {
    let rows: Vec<_> = obs
        .bodies
        .iter()
        .enumerate()
        .map(|(i, body)| {
            json!({
                "id": ids[i],
                "name": NAMES[i],
                "position": body.position,
                "orientation": body.orientation,
                "velocity": body.velocity,
                "angular_velocity": body.angular_velocity,
            })
        })
        .collect();
    Value::Array(rows)
}

#[derive(Clone, Copy)]
struct Mass {
    mass: f64,
    spherical_inertia: f64,
}

fn masses() -> Result<[Mass; 3]> {
    let doc = sdformat::load_str(SDF, "free_payload_impact.sdf", &sdformat::NoResources)?;
    let mut values = Vec::new();
    for name in NAMES {
        let (model, link) = name.split_once("::").unwrap();
        let inertia = &doc.worlds[0]
            .models
            .iter()
            .find(|m| m.name == model)
            .and_then(|m| m.links.iter().find(|l| l.name == link))
            .ok_or("missing authored inertia")?
            .inertial;
        let [ix, iy, iz, ixy, ixz, iyz] = inertia.inertia;
        if ix != iy
            || iy != iz
            || [ixy, ixz, iyz] != [0.; 3]
            || inertia.pose != sdformat::Pose::default()
        {
            return Err(
                "this fixture's independent momentum oracle requires centered spherical inertia"
                    .into(),
            );
        }
        values.push(Mass {
            mass: inertia.mass,
            spherical_inertia: ix,
        });
    }
    values
        .try_into()
        .map_err(|_| "expected three mass properties".into())
}

fn momentum(s: &Observation, props: &[Mass; 3]) -> (DVec3, DVec3) {
    let mut linear = DVec3::ZERO;
    let mut angular = DVec3::ZERO;
    for (b, m) in s.bodies.iter().zip(props) {
        let p = m.mass * DVec3::from(b.velocity);
        linear += p;
        angular += DVec3::from(b.position).cross(p)
            + m.spherical_inertia * DVec3::from(b.angular_velocity);
    }
    (linear, angular)
}
#[derive(Default, Serialize)]
struct Errors {
    position_m: f64,
    orientation_rad: f64,
    linear_velocity_m_s: f64,
    angular_velocity_rad_s: f64,
    hinge_angle_rad: f64,
    hinge_rate_rad_s: f64,
    normal_impulse_ns: f64,
}
impl Errors {
    fn compare(&mut self, a: &Observation, b: &Observation) {
        self.hinge_angle_rad = self
            .hinge_angle_rad
            .max((a.joint.position - b.joint.position).abs());
        self.hinge_rate_rad_s = self
            .hinge_rate_rad_s
            .max((a.joint.velocity - b.joint.velocity).abs());
        self.normal_impulse_ns = self.normal_impulse_ns.max((a.impulse - b.impulse).abs());
        for (a, b) in a.bodies.iter().zip(&b.bodies) {
            self.position_m = self
                .position_m
                .max(DVec3::from(a.position).distance(b.position.into()));
            self.linear_velocity_m_s = self
                .linear_velocity_m_s
                .max(DVec3::from(a.velocity).distance(b.velocity.into()));
            self.angular_velocity_rad_s = self
                .angular_velocity_rad_s
                .max(DVec3::from(a.angular_velocity).distance(b.angular_velocity.into()));
            let dq =
                DQuat::from_array(a.orientation).conjugate() * DQuat::from_array(b.orientation);
            self.orientation_rad = self
                .orientation_rad
                .max(2. * dq.xyz().length().atan2(dq.w.abs()));
        }
    }
}

/// Use a separate unguarded simulation at each resolution, with the same 600 ms horizon.
fn reference_drift(source: &str, refinement: u64, props: &[Mass; 3]) -> Result<(f64, f64)> {
    let xml = source.replace(
        "<max_step_size>0.001</max_step_size>",
        &format!("<max_step_size>{}</max_step_size>", DT / refinement as f64),
    );
    let mut lane = Lane::load(&xml)?;
    let initial = momentum(&lane.observe()?, props);
    let mut drift = (0f64, 0f64);
    for _ in 0..(STEPS * refinement) {
        lane.sim.step_exact(1)?;
        let (p, l) = momentum(&lane.observe()?, props);
        drift.0 = drift.0.max(initial.0.distance(p));
        drift.1 = drift.1.max(initial.1.distance(l));
    }
    Ok(drift)
}

fn measure() -> Result<(Value, Value)> {
    // Remove the world policy altogether so the control has no promotion schedule.
    let mut reference_xml = SDF.to_owned();
    let begin = reference_xml
        .find("<plugin name=\"gz::sim::systems::ContactPromotion\"")
        .ok_or("missing authored policy")?;
    let end = begin
        + reference_xml[begin..]
            .find("</plugin>")
            .ok_or("missing end of policy")?
        + "</plugin>".len();
    reference_xml.replace_range(begin..end, "");
    let miss_xml = SDF.replace("-0.86 0.71 0.5", "-0.86 1.1 0.5");
    let mut automatic = Lane::load(SDF)?;
    let mut reference = Lane::load(&reference_xml)?;
    let mut miss = Lane::load(&miss_xml)?;
    let props = masses()?;
    let initial = automatic.observe()?;
    let p0 = momentum(&initial, &props);
    assert_eq!(initial.tier, ArticulationTier::Prescribed);
    let mut samples = Vec::new();
    let (mut promotion, mut impact, mut ref_impact) = (None, None, None);
    let mut first_impulse = 0.;
    let mut peak_hinge_rate: f64 = 0.;
    let mut errors = Errors::default();
    let mut linear_drift: f64 = 0.;
    let mut angular_drift: f64 = 0.;
    let mut reference_drift_1ms = (0f64, 0f64);
    let mut momentum_difference = (0f64, 0f64);
    let mut post_impact = None;
    let mut last_contact = None;
    let mut total_impulse = 0.;
    let mut miss_clearance = f64::INFINITY;
    let mut peak_contact_impulse: f64 = 0.;
    for tick in 0..=STEPS {
        if tick > 0 {
            for lane in [&mut automatic, &mut reference, &mut miss] {
                lane.sim.step_exact(1)?;
            }
        }
        let a = automatic.observe()?;
        let b = reference.observe()?;
        let m = miss.observe()?;
        assert_eq!(
            m.tier,
            ArticulationTier::Prescribed,
            "near miss promoted at {tick}"
        );
        let payload = DVec3::from(m.bodies[2].position);
        for arm_link in &m.bodies[..2] {
            miss_clearance =
                miss_clearance.min(payload.distance(DVec3::from(arm_link.position)) - 0.14 - 0.1);
        }
        peak_contact_impulse = peak_contact_impulse.max(a.impulse);
        assert_eq!(m.impulse, 0., "near miss contacted at {tick}");
        assert_eq!(b.tier, ArticulationTier::Dynamic);
        if a.tier == ArticulationTier::Dynamic {
            promotion.get_or_insert(tick);
        }
        if b.impulse > 0. {
            ref_impact.get_or_insert(tick);
        }
        if a.impulse > 0. {
            if impact.is_none() {
                impact = Some(tick);
                first_impulse = a.impulse;
            }
            last_contact = Some(tick);
            total_impulse += a.impulse;
        } else if last_contact == tick.checked_sub(1) && impact.is_some() {
            post_impact = Some(json!({
                "tick": tick,
                "velocity_m_s": a.bodies[2].velocity,
                "hinge_angle_rad": a.joint.position,
                "hinge_rate_rad_s": a.joint.velocity,
            }));
        }
        let (p, l) = momentum(&a, &props);
        let (pr, lr) = momentum(&b, &props);
        reference_drift_1ms.0 = reference_drift_1ms.0.max(pr.distance(p0.0));
        reference_drift_1ms.1 = reference_drift_1ms.1.max(lr.distance(p0.1));
        momentum_difference.0 = momentum_difference.0.max(pr.distance(p));
        momentum_difference.1 = momentum_difference.1.max(lr.distance(l));
        linear_drift = linear_drift.max(p.distance(p0.0));
        angular_drift = angular_drift.max(l.distance(p0.1));
        peak_hinge_rate = peak_hinge_rate.max(a.joint.velocity.abs());
        errors.compare(&a, &b);
        samples.push(json!({
            "tick": tick,
            "tier": if a.tier == ArticulationTier::Dynamic { "dynamic" } else { "prescribed" },
            "automatic": pose_rows(&a, &automatic.ids),
            "reference": pose_rows(&b, &reference.ids),
            "near_miss": pose_rows(&m, &miss.ids),
            "hinge_angle_rad": a.joint.position,
            "hinge_rate_rad_s": a.joint.velocity,
            "reference_hinge_rate_rad_s": b.joint.velocity,
            "near_miss_hinge_rate_rad_s": m.joint.velocity,
            "automatic_impulse_ns": a.impulse,
            "reference_impulse_ns": b.impulse,
        }));
    }
    let promotion = promotion.ok_or("missing promotion")?;
    let impact = impact.ok_or("missing impact")?;
    let released = post_impact.ok_or("missing separation")?;
    let end = automatic.observe()?;
    let ref_end = reference.observe()?;
    let reference_drift_half = reference_drift(&reference_xml, 2, &props)?;
    let reference_drift_quarter = reference_drift(&reference_xml, 4, &props)?;
    let convergence = [
        (DT, reference_drift_1ms),
        (DT / 2., reference_drift_half),
        (DT / 4., reference_drift_quarter),
    ]
    .map(|(dt, (linear, angular))| {
        json!({
            "dt_s": dt,
            "max_linear_momentum_drift_kg_m_s": linear,
            "max_angular_momentum_drift_kg_m2_s": angular,
        })
    });
    let summary = json!({
        "dt_s": DT,
        "steps": STEPS,
        "first_promotion_tick": promotion,
        "first_contact_tick": impact,
        "reference_first_contact_tick": ref_impact,
        "last_contact_tick": last_contact,
        "first_native_normal_impulse_ns": first_impulse,
        "summed_native_normal_impulse_ns": total_impulse,
        "payload_initial_velocity_m_s": initial.bodies[2].velocity,
        "first_unloaded_after_last_contact": released,
        "peak_native_normal_impulse_ns": peak_contact_impulse,
        "final_payload_velocity_m_s": end.bodies[2].velocity,
        "reference_final_payload_velocity_m_s": ref_end.bodies[2].velocity,
        "reference_final_hinge_angle_rad": ref_end.joint.position,
        "reference_final_hinge_rate_rad_s": ref_end.joint.velocity,
        "final_hinge_angle_rad": end.joint.position,
        "final_hinge_rate_rad_s": end.joint.velocity,
        "peak_abs_hinge_rate_rad_s": peak_hinge_rate,
        "initial_linear_momentum_kg_m_s": p0.0.to_array(),
        "initial_angular_momentum_kg_m2_s": p0.1.to_array(),
        "max_linear_momentum_drift_kg_m_s": linear_drift,
        "max_angular_momentum_drift_kg_m2_s": angular_drift,
        "max_reference_errors": errors,
        "always_dynamic_momentum_convergence": convergence,
        "max_linear_momentum_difference_kg_m_s": momentum_difference.0,
        "max_angular_momentum_difference_kg_m2_s": momentum_difference.1,
        "near_miss": {
            "initial_payload_y_m": 1.1,
            "promotions": 0,
            "contact_impulse_ns": 0.0,
            "minimum_surface_clearance_m": miss_clearance,
        },
    });
    println!("{}", serde_json::to_string_pretty(&summary)?);
    assert!(miss_clearance > 0.2 && miss_clearance < 0.22);
    assert!(promotion < impact && ref_impact == Some(impact));
    assert!(first_impulse > 0. && peak_hinge_rate > 0.1 && end.joint.position.abs() > 0.01);
    assert!(
        end.bodies[2].velocity[0] < 0.,
        "payload should reverse its horizontal travel"
    );
    assert!(
        errors.position_m < 1e-8
            && errors.orientation_rad < 1e-8
            && errors.linear_velocity_m_s < 1e-8
            && errors.angular_velocity_rad_s < 1e-8
    );
    assert!(
        errors.hinge_angle_rad < 1e-8
            && errors.hinge_rate_rad_s < 1e-8
            && errors.normal_impulse_ns < 1e-8
    );
    // Momentum added by promotion and absolute drift of the underlying solver are
    // different errors. Check both, with tighter absolute limits at the finer step.
    assert!(momentum_difference.0 < 1e-10 && momentum_difference.1 < 1e-10);
    assert!(linear_drift < 0.001 * p0.0.length() && angular_drift < 0.001 * p0.1.length());
    assert!(reference_drift_quarter.0 < 1e-4 && reference_drift_quarter.1 < 1e-4);
    Ok((summary, json!({"dt_s":DT,"samples":samples})))
}

fn main() -> Result<()> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: free_payload_promotion OUTPUT_DIRECTORY")?,
    );
    let (summary, trajectory) = measure()?;
    fs::create_dir_all(&dir)?;
    fs::write(
        dir.join("trajectory.json"),
        serde_json::to_vec(&trajectory)?,
    )?;
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_free_payload_matches_the_always_dynamic_reference() -> Result<()> {
        measure()?;
        Ok(())
    }
}
