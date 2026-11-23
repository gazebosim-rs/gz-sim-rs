// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Actual native force, spatial impulse, and matched-state response at the upper point.
use super::*;
#[path = "inertia_upper_projected.rs"]
pub(crate) mod projected;
use gz_physics_multifidelity::{ContactSegment, ContactSolverIterations};

pub(super) fn source(i: &Impact, g: &Gate, gate_mu: f64) -> String {
    text(i, g, true).replace(&g.xml, &material(g, gate_mu))
}
pub(super) fn at_foot(r: &Physical, x: Flight) -> BodyState {
    BodyState {
        position: (x.com - x.q * ARM).into(),
        velocity: x.v.into(),
        orientation: x.q.into(),
        angular_velocity: r.body.omega(x.q, x.l).into(),
    }
}
struct Cuts {
    sim: Simulation,
    ids: Bodies,
    gate: BodyId,
}
fn restart(
    i: &Impact,
    g: &Gate,
    raw: &str,
    at: f64,
    x: Flight,
    point: bool,
    remote: Option<usize>,
) -> crate::Result<Cuts> {
    let (mut sim, ids) = load(raw, point)?;
    let gate = id_named(&sim, "default::opponent::gate")?;
    let mut phy = sim.world_mut().resource_mut::<PhysicsBackend>();
    let e = phy.downcast_mut::<MultiFidelityBackend>().unwrap();
    let mut p = e.state(ids.pad).unwrap();
    p.position =
        (i.r.pad_v * at - i.r.n * (PAD_HALF[2] + if remote == Some(0) { 5. } else { 0. })).into();
    p.velocity = i.r.pad_v.into();
    e.set_body_state(ids.pad, p)?;
    let mut b = e.state(gate).unwrap();
    b.position = (g.center
        + g.velocity * at
        + if remote == Some(1) {
            i.r.n * 5.
        } else {
            DVec3::ZERO
        })
    .into();
    b.velocity = g.velocity.into();
    e.set_body_state(gate, b)?;
    e.set_body_state(ids.foot, at_foot(&i.r, x))?;
    Ok(Cuts { sim, ids, gate })
}

#[derive(Clone, Copy)]
pub(super) struct Wrench {
    normal: DVec3,
    pub(super) scalar: f64,
    tangent: Option<DVec3>,
    pub(super) corners: [f64; 8],
}
impl Default for Wrench {
    fn default() -> Self {
        Self {
            normal: DVec3::ZERO,
            scalar: 0.,
            tangent: Some(DVec3::ZERO),
            corners: [0.; 8],
        }
    }
}
impl Wrench {
    pub(super) fn add(&mut self, p: &RigidContactPatch, foot: BodyId, q: DQuat) {
        let a = p.a == foot;
        let sign = if a { 1. } else { -1. };
        for pt in &p.points {
            let z = pt.normal_impulse;
            if z == 0. {
                continue;
            }
            self.normal += sign * z * DVec3::from(p.normal_on_a);
            self.scalar += z;
            self.tangent = self
                .tangent
                .zip(pt.tangent_impulse_on_a)
                .map(|(j, add)| j + sign * DVec3::from(add));
            let local = q.conjugate() * DVec3::from(if a { pt.lever_a } else { pt.lever_b }) + ARM;
            let which = 2 * usize::from(local.x >= 0.)
                + usize::from(local.y >= 0.)
                + 4 * usize::from(local.z > 0.);
            self.corners[which] += z;
        }
    }
    fn data(self) -> Value {
        json!({"normal_vector":self.normal.to_array(),"normal_ns":self.scalar,"native_tangent":self.tangent.map(|j|j.to_array()),"corners":self.corners})
    }
}
#[derive(Clone, Copy)]
struct Profile {
    point: bool,
    pgs: usize,
    relax: usize,
    calls: u32,
    remote_other: bool,
}
const COLD: [Profile; 6] = [
    Profile {
        point: false,
        pgs: 1,
        relax: 16,
        calls: 1,
        remote_other: false,
    },
    Profile {
        point: false,
        pgs: 1,
        relax: 16,
        calls: 1,
        remote_other: true,
    },
    Profile {
        point: true,
        pgs: 1,
        relax: 16,
        calls: 1,
        remote_other: false,
    },
    Profile {
        point: true,
        pgs: 8,
        relax: 16,
        calls: 1,
        remote_other: false,
    },
    Profile {
        point: true,
        pgs: 8,
        relax: 16,
        calls: 1,
        remote_other: true,
    },
    Profile {
        point: true,
        pgs: 8,
        relax: 16,
        calls: 8,
        remote_other: false,
    },
];
fn added(
    e: &mut MultiFidelityBackend,
    ids: Bodies,
    gate: BodyId,
    h: u64,
    p: Profile,
) -> crate::Result<(Wrench, Wrench)> {
    let dur = Duration::from_nanos(h);
    let seg = ContactSegment {
        duration: dur,
        native_steps: p.calls,
        iterations: ContactSolverIterations {
            temporal: 4,
            pgs: Some(p.pgs),
            relaxation: Some(p.relax),
        },
    };
    let (mut bottom, mut upper) = (Wrench::default(), Wrench::default());
    let fresh = [(ids.foot, ids.pad), (ids.foot, gate)];
    let (r, _) = e.observe_contact_schedule(dur, &[seg], &fresh, |s| {
        let q = DQuat::from_array(s.state(ids.foot).unwrap().orientation);
        for patch in s.contact_patches() {
            if patch.a != ids.foot && patch.b != ids.foot {
                continue;
            }
            let other = if patch.a == ids.foot {
                patch.b
            } else {
                patch.a
            };
            if other == gate {
                upper.add(&patch, ids.foot, q);
            }
            if other == ids.pad {
                bottom.add(&patch, ids.foot, q);
            }
        }
    })?;
    for (id, normal) in [(ids.pad, bottom.scalar), (gate, upper.scalar)] {
        let sum: f64 = r
            .pairs
            .iter()
            .filter(|p| (p.a, p.b) == (id.min(ids.foot), id.max(ids.foot)))
            .map(|p| p.normal_impulse)
            .sum();
        assert!((sum - normal).abs() < 1e-10);
    }
    Ok((bottom, upper))
}
fn matched(
    i: &Impact,
    g: &Gate,
    raw: &str,
    hit: Crossing,
    h: u64,
    p: Profile,
) -> crate::Result<Value> {
    let mut c = restart(
        i,
        g,
        raw,
        hit.at,
        hit.before,
        p.point,
        p.remote_other.then_some(1 - hit.surface),
    )?;
    let mut phy = c.sim.world_mut().resource_mut::<PhysicsBackend>();
    let e = phy.downcast_mut::<MultiFidelityBackend>().unwrap();
    let start = e.state(c.ids.foot).unwrap();
    let (lower, upper) = added(e, c.ids, c.gate, h, p)?;
    let end = e.state(c.ids.foot).unwrap();
    let net = M
        * (DVec3::from(end.velocity)
            - DVec3::from(start.velocity)
            - i.r.gravity * (h as f64 * 1e-9));
    let (chosen, other) = if hit.surface == 1 {
        (upper, lower)
    } else {
        (lower, upper)
    };
    let explicit = chosen.tangent.map(|j| chosen.normal + j);
    let measured = explicit.or_else(|| (other.scalar == 0.).then_some(net));
    if let (Some(ja), Some(jb)) = (explicit, other.tangent.map(|j| other.normal + j)) {
        assert!((ja + jb).distance(net) < 1e-8);
    }
    let direction=measured.map(|j|{
        let local=hit.plane.frame.conjugate()*j;
        let after=hit.plane.frame.conjugate()*hit.law.before+hit.law.effective*local;
        let jt=local.truncate();let slip=after.truncate();
        json!({"unilateral_velocity_m_s":after.z,"post_slip_m_s":slip.length(),"cone_residual_ns":jt.length()-hit.mu*local.z,
            "opposition_cos":(jt.length()>1e-7 && slip.length()>1e-7).then(||-jt.normalize().dot(slip.normalize()))})
    });
    let startl = i.r.body.momentum(
        DQuat::from_array(start.orientation),
        start.angular_velocity.into(),
    );
    let endl = i.r.body.momentum(
        DQuat::from_array(end.orientation),
        end.angular_velocity.into(),
    );
    let exact_dl = hit.arm.cross(hit.law.impulse);
    Ok(
        json!({"point_friction":p.point,"pgs":p.pgs,"relaxation":p.relax,"calls":p.calls,"remote_other":p.remote_other,"face":hit.surface,"span_ns":h,"upper":upper.data(),"lower":lower.data(),"momentum_impulse":net.to_array(),"direction":direction,"chosen_impulse":measured.map(|j|j.to_array()),"vector_error_ns":measured.map(|j|j.distance(hit.law.impulse)),"angular_momentum_error":(endl-startl).distance(exact_dl),"wrong_corners_ns":chosen.scalar-chosen.corners[hit.corner]}),
    )
}
pub(super) fn sample(
    i: &Impact,
    g: &Gate,
    mu: f64,
    hit: Crossing,
    follow: &Continued,
) -> crate::Result<Value> {
    let xml = source(i, g, mu);
    let (full, _) = measure_gate(i, &xml, true)?;
    let calls = full["native_intervals"].as_array().unwrap();
    let first = calls
        .iter()
        .find(|s| s["opponent_ns"].as_f64().unwrap() > 0.)
        .ok_or("ceiling received no native impulse")?;
    let lower = first["primary_ns"].as_f64().unwrap();
    let first_end = first["end_ns"].as_u64().unwrap();
    let (h, cold) = response(i, g, &xml, hit, follow)?;
    let isolated = first_end as f64 <= follow.through * 1e9 && first_end as f64 >= hit.at * 1e9;
    Ok(
        json!({"first_gate_start_ns":first["start_ns"],"first_gate_end_ns":first_end,"first_gate_normal_ns":first["opponent_ns"],"simultaneous_native_lower_ns":lower,"first_net_impulse":first["net_foot_impulse"],"first_interval_inside_physical_window": isolated,"matched_horizon_ns":h,"physical_free_after_upper":follow.through>hit.at,"cold":cold}),
    )
}

pub(super) fn response(
    i: &Impact,
    g: &Gate,
    xml: &str,
    hit: Crossing,
    follow: &Continued,
) -> crate::Result<(u64, Vec<Value>)> {
    let cert = follow.through - hit.at;
    let speed = Bounds::new(i.r.body, hit.after()).w * 0.32
        + (hit.after().v - g.velocity)
            .length()
            .max((hit.after().v - i.r.pad_v).length())
        + 1.;
    // A cold solve beginning at impact. Only claimed as an isolated physical
    // impulse when independent post-impact flight covers the chosen horizon.
    let horizon = (0.125 * hit.other_gap / speed)
        .min(0.5e-6)
        .min(if cert > 0. { cert * 0.25 } else { 0.5e-6 });
    let h = (horizon * 1e9).floor() as u64;
    let mut cold = vec![];
    for p in COLD {
        if u64::from(p.calls) <= h {
            cold.push(matched(i, g, xml, hit, h, p)?);
        }
    }
    Ok((h, cold))
}
/// Measure the real roof and pad on a matched approach ending at the third
/// physical event. The control moves the lower box away from that same cut.
pub(super) fn third_controls(
    i: &Impact,
    g: &Gate,
    xml: &str,
    t: f64,
    state: Flight,
    h: u64,
) -> crate::Result<Vec<Value>> {
    let mut out = vec![];
    for p in [
        Profile {
            point: false,
            pgs: 1,
            relax: 16,
            calls: 1,
            remote_other: false,
        },
        Profile {
            point: true,
            pgs: 8,
            relax: 16,
            calls: 1,
            remote_other: false,
        },
        Profile {
            point: true,
            pgs: 8,
            relax: 16,
            calls: 1,
            remote_other: true,
        },
        Profile {
            point: true,
            pgs: 8,
            relax: 16,
            calls: 4,
            remote_other: false,
        },
    ] {
        if u64::from(p.calls) > h {
            continue;
        }
        let mut c = restart(i, g, xml, t, state, p.point, p.remote_other.then_some(0))?;
        let mut backend = c.sim.world_mut().resource_mut::<PhysicsBackend>();
        let native = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
        let before = native.state(c.ids.foot).unwrap();
        let (lower, upper) = added(native, c.ids, c.gate, h, p)?;
        let after = native.state(c.ids.foot).unwrap();
        let net = M
            * (DVec3::from(after.velocity)
                - DVec3::from(before.velocity)
                - i.r.gravity * (h as f64 * 1e-9));
        out.push(json!({"start_ns":t*1e9,"dt_ns":h,"point":p.point,"pgs":p.pgs,"calls":p.calls,"remote_lower":p.remote_other,"normal_upper":upper.scalar,"normal_lower":lower.scalar,"upper_vector":upper.tangent.map(|j|(j+upper.normal).to_array()),"lower_vector":lower.tangent.map(|j|(j+lower.normal).to_array()),"upper_corners":upper.corners,"lower_corners":lower.corners,"momentum_impulse":net.to_array(),"before":body_view(before),"after":body_view(after)}));
    }
    Ok(out)
}
