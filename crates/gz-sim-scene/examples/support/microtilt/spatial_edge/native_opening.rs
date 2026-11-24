// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native generalized opening and offset-payload free-flight qualification.
use super::super::{
    authored::xml,
    transaction::{Frame, inertial},
};
use super::*;
use gz_physics_multifidelity::{EdgeImpactReceipt, OpeningEdgeImpactReceipt};
fn checked(h: OpeningEdgeImpactReceipt, law: Solution, e: Edge, frame: Frame) -> Result<Value> {
    let i = law.site;
    let k = 1 - i;
    let n = frame.q * DVec3::Z;
    let j = DVec3::from(h.loaded.impulse_on_foot);
    let point = DVec3::from(h.loaded.point);
    let free = DVec3::from(h.unloaded_point);
    let endpoint = j.distance(frame.q * law.j);
    let after = DVec3::from(h.after_unloaded_relative_velocity);
    let speed = after
        .distance(frame.q * law.post[k])
        .max(DVec3::from(h.loaded.after_relative_velocity).distance(frame.q * law.post[i]));
    let before = DVec3::from(h.before_unloaded_relative_velocity);
    let pre = before
        .distance(frame.q * e.point(k))
        .max(DVec3::from(h.loaded.before_relative_velocity).distance(frame.q * e.point(i)));
    let geometry = (point - free).distance(frame.q * (e.arm[i] - e.arm[k]));
    let energy = (h.loaded.dissipated_energy - law.loss).abs();
    if h.loaded_site != i
        || h.loaded.sticking != law.sticking
        || after.dot(n) <= 0.
        || before.dot(n) >= 0.
        || [endpoint, speed, pre, geometry, energy]
            .iter()
            .any(|u| !u.is_finite() || *u > 1e-10)
    {
        return Err(format!("one-site physical error {endpoint:e}, {speed:e}, {pre:e}, geometry {geometry:e}, energy {energy:e}").into());
    }
    Ok(
        json!({"site":i,"sticking":law.sticking,"point_m":h.loaded.point,"unloaded_point_m":h.unloaded_point,"impulse_ns":h.loaded.impulse_on_foot,"oracle_ns":(frame.q*law.j).to_array(),"endpoint_error_ns":endpoint,"outgoing_velocity_error_m_s":speed,"incoming_velocity_error_m_s":pre,"geometric_residual_m":geometry,"loss_residual_j":energy,"opening_before_m_s":before.dot(n),"opening_after_m_s":after.dot(n),"loaded_after_m_s":h.loaded.after_relative_velocity,"normal":h.loaded.normal,"cone_margins_kg_inv":h.separating_margin}),
    )
}
fn exact(mut i: Input, active: usize, free: bool, frame: Frame) -> Result<Value> {
    if free {
        let axis = i.seed.state.q * DVec3::X;
        i.edge.inertia -= DMat3::from_cols(axis * axis.x, axis * axis.y, axis * axis.z) * 0.0201;
        i.edge.spin =
            i.after_w - i.edge.inertia.inverse() * i.edge.arm[active].cross(i.desired[active]);
        i.seed.state.w = i.edge.spin;
    }
    let oracle = solve(i.edge);
    let source = xml(i, frame, free, frame.q * DVec3::new(0., 0., -9.81), "edge");
    let mut direct = load(&source, true)?;
    let id = ids(&direct)?;
    let mut owner = direct.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    let before = departure::ingest(n, &id);
    let hit = n.project_edge_impact(id.foot, id.pad)?;
    let after = departure::ingest(n, &id);
    let mut sim = load(&source, true)?;
    sim.step_exact(1)?;
    let r = last_refinement(&sim);
    let via = r.and_then(|e| e.interval.edge_impacts.first());
    match (oracle,hit,via){
    (Err(note),None,None)=>{
      if before!=after{return Err("unresolved opening mutated state".into());}
      Ok(json!({"admitted":false,"reason":note.to_string()}))
    }
    (Ok(law),Some(EdgeImpactReceipt::OneLoaded(a)),Some(EdgeImpactReceipt::OneLoaded(b)))=>{
      if a!=*b||!a.loaded.elapsed.is_zero(){return Err("direct and authored edge transaction differ".into());}
      let mut details=checked(a,law,i.edge,frame)?;
      let (p0,h0,k0)=inertial(before);let(p1,h1,k1)=inertial(after);
      let j=DVec3::from(a.loaded.impulse_on_foot);let dp=(p1-p0).distance(j);let dh=(h1-h0).distance((frame.q*i.edge.arm[law.site]).cross(j));let dk=(k1-k0-a.loaded.kinetic_energy_change).abs();
      if dp.max(dh).max(dk)>1e-10||n.project_edge_impact(id.foot,id.pad)?.is_some()||after!=departure::ingest(n,&id){return Err("duplicate commit or native momentum/work mismatch".into());}
      let rec=r.unwrap();let pair=&rec.requested_pairs[0];
      if pair.first_positive_end!=Some(Duration::ZERO)||pair.normal_impulse+1e-12<law.j.z || rec.interval.native_steps!=8{return Err("public impact attribution failed".into());}
      details["admitted"]=true.into();details["momentum_residual_ns"]=dp.into();details["angular_residual_nms"]=dh.into();details["energy_residual_j"]=dk.into();details["pair_scalar_ns"]=pair.normal_impulse.into();Ok(details)
    }
    (why,a,b)=>Err(format!("native registration disagrees with circumscribed reference oracle={} direct={:?} authored={:?}",why.is_ok(),a,b).into())
 }
}
#[derive(Clone, Copy)]
struct Ballast {
    input: Input,
    e: Edge,
    offset: f64,
    gravity: DVec3,
    t: f64,
}
impl Ballast {
    fn new(yaw: f64, offset: f64, mu: f64, g: DVec3, t: f64) -> Self {
        let mut i = make_opening(Setup {
            active: 0,
            sticking: false,
            mu,
            yaw,
            pitch: 0.18,
            opening: 0.3,
        });
        i.edge.velocity = DQuat::from_rotation_z(yaw) * DVec3::new(0.44, -0.22, -1.27);
        i.edge.spin = DVec3::ZERO;
        i.seed.state.v = i.edge.velocity;
        i.seed.state.w = DVec3::ZERO;
        i.seed.state.c.z -= i.edge.velocity.z * t + 0.5 * g.z * t * t;
        let q = DMat3::from_quat(i.seed.state.q);
        let d = DVec3::new(0., offset / 2., 0.07);
        let intrinsic = DMat3::from_diagonal(DVec3::new(0.0402, 0.0402, 0.05));
        let orbital = 2.
            * (DMat3::IDENTITY * d.length_squared() - DMat3::from_cols(d * d.x, d * d.y, d * d.z));
        let e = Edge {
            inertia: q * (intrinsic + orbital) * q.transpose(),
            arm: i.edge.arm.map(|r| r - q * (0.5 * offset * DVec3::Y)),
            ..i.edge
        };
        Self {
            input: i,
            e,
            offset,
            gravity: g,
            t,
        }
    }
    fn xml(&self, frame: Frame, policy: &str) -> String {
        xml(self.input,frame,false,frame.q*self.gravity,policy)
    .replace("<pose>0 0 0.14 0 0 0</pose>",&format!("<pose>0 {:.17} 0.14 0 0 0</pose>",self.offset))
    .replace("A translating deck meets two vertices of a tilted sliding sole at the initial boundary.","An offset payload lands on two co-level vertices of a tilted sole. One vertex takes the impact and the other opens.")
    }
    fn arrival(self, frame: Frame) -> Result<Value> {
        let mut s = load(&self.xml(frame, "edge"), true)?;
        let steps = ((self.t / DT).ceil() as u64).max(1);
        let mut early = 0.;
        for tick in 1..=steps {
            s.step_exact(1)?;
            let rec = last_refinement(&s).filter(|r| r.iteration == tick);
            if let Some((r, h)) = rec.and_then(|r| r.interval.edge_impacts.first().map(|h| (r, h)))
            {
                let EdgeImpactReceipt::OneLoaded(h) = h else {
                    return Err("offset ballast had unexpected edge active set".into());
                };
                let at = r.start_time + h.loaded.elapsed;
                if early != 0. || tick != steps || (at.as_secs_f64() - self.t).abs() > 1.001e-9 {
                    return Err(format!(
                        "ballast event at {at:?} target {} earlier kick {early}",
                        self.t
                    )
                    .into());
                }
                let mut e = self.e;
                e.velocity += self.gravity * at.as_secs_f64();
                let law = solve(e)?;
                let mut data = checked(*h, law, e, frame)?;
                if r.requested_pairs[0].first_positive_end != Some(h.loaded.elapsed) {
                    return Err("pair clock differs from edge clock".into());
                }
                data["tick"] = tick.into();
                data["actual_ns"] = (at.as_nanos() as u64).into();
                data["known_ns"] = (self.t * 1e9).into();
                data["prior_reaction_ns"] = early.into();
                data["interval_ns"] = r.requested_pairs[0].normal_impulse.into();
                data["last_ns"] = r.requested_pairs[0].last_native_normal_impulse.into();
                return Ok(data);
            }
            early += rec.map_or_else(
                || {
                    engine(&s)
                        .rigid_contact_observations()
                        .iter()
                        .map(|p| p.normal_impulse)
                        .sum::<f64>()
                },
                |r| r.interval.pairs.iter().map(|p| p.normal_impulse).sum(),
            );
        }
        Err("missed offset ballast edge arrival".into())
    }
}
fn view(out: &Path, b: Ballast) -> Result<()> {
    use super::super::edge_view::{self, Replay};
    let frame = Frame {
        q: DQuat::IDENTITY,
        shift: DVec3::ZERO,
        spin: 0.,
    };
    edge_view::record(
        out,
        Replay {
            registered_name: "Qualified edge active set",
            refinement_name: "Eight native calls",
            title: "Offset payload lands with one bearing vertex",
            capture_name: "first_landing",
            impact_tick:5,
            capture_tick: 5,
            last_tick: 55,
            sdf: "edge.sdf",
        },
        |label| b.xml(frame, label),
        |h, sites| {
            let EdgeImpactReceipt::OneLoaded(h) = h else {
                return Err("unexpected public edge mode".into());
            };
            sites[b.input.indices[h.loaded_site]] +=
                DVec3::from(h.loaded.impulse_on_foot).dot(DVec3::from(h.loaded.normal));
            Ok((
                h.loaded.elapsed,
                json!({"loaded_quadrant":b.input.indices[h.loaded_site],"released_quadrant":b.input.indices[1-h.loaded_site],"j_ns":h.loaded.impulse_on_foot,"unloaded_after":h.after_unloaded_relative_velocity}),
            ))
        },
    )
}
pub(in super::super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let base = Frame {
        q: DQuat::IDENTITY,
        shift: DVec3::ZERO,
        spin: 0.,
    };
    let moved = Frame {
        q: DQuat::from_euler(glam::EulerRot::ZYX, -0.4, 0.34, -0.18),
        shift: DVec3::new(-1.1, 1.8, -0.2),
        spin: 0.,
    };
    let mut exacts = vec![];
    for mu in [0., 0.08, 0.2, 0.35, 0.55] {
        for pitch in [-0.18, 0.18] {
            for yaw in [0., 0.47, 1.19] {
                for active in 0..2 {
                    for sticking in [false, true] {
                        if sticking && mu == 0. {
                            continue;
                        }
                        for opening in [0.04, 0.13, 0.3, 0.5, 0.72] {
                            let i = make_opening(Setup {
                                active,
                                sticking,
                                mu,
                                yaw,
                                pitch,
                                opening,
                            });
                            for free in [false, true] {
                                for (label, frame) in [
                                    ("base", base),
                                    ("inclined spinning", Frame { spin: 1.3, ..moved }),
                                ] {
                                    let r=exact(i,active,free,frame).map_err(|e|format!("mu={mu} p={pitch} yaw={yaw} active={active} sticks={sticking} opens={opening} free={free} {label}: {e}"))?;
                                    exacts.push(json!({"mu":mu,"pitch":pitch,"active":active,"sticking":sticking,"opening_m_s":opening,"free":free,"yaw":yaw,"frame":label,"result":r}));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("exact.json"), serde_json::to_vec(&exacts)?)?;
    let mut flights = vec![];
    for ns in [
        0.25, 1.25, 62500.37, 249999.4, 499999.34, 500001.61, 638197.61, 2138197.72,
    ] {
        for g in [DVec3::ZERO, DVec3::new(0.35, -0.23, -9.81)] {
            for off in [-0.36, 0.36] {
                for mu in [0., 0.08, 0.2] {
                    for yaw in [0., 0.7] {
                        for (name, frame) in [("base", base), ("inclined", moved)] {
                            let b = Ballast::new(yaw, off, mu, g, ns * 1e-9);
                            let r=b.arrival(frame).map_err(|e|format!("ballast ns={ns} g={g:?} off={off} mu={mu} yaw={yaw} {name}: {e}"))?;
                            flights.push(json!({"offset_y_m":off,"yaw":yaw,"frame":name,"g":g.to_array(),"mu":mu,"result":r}));
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("flights.json"), serde_json::to_vec(&flights)?)?;
    view(
        out,
        Ballast::new(
            0.5,
            0.36,
            0.08,
            DVec3::new(0.35, -0.23, -9.81),
            0.00213819765,
        ),
    )
}
