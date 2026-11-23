// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Apply a one-public-step wrench to a rocking foot while native contact is refined.
use super::*;
use gz_physics::{DynamicBackend, NativeStepGrid};
use gz_physics_multifidelity::RigidContactPatch;

const CALLS: u32 = 8;
const OUTER: Duration = Duration::from_micros(STEP_US);
const FORCE: [f64; 3] = [0., 80., 0.];
const TORQUE: [f64; 3] = [0.4, 0., 0.];
#[derive(Clone, Copy, PartialEq, Eq)]
enum Input {
    SingleNative,
    Public,
    Reissued,
}
/// Measurements from the actual native foot/pad points on a public tick.
#[derive(Default)]
pub(super) struct Solve {
    pub(super) corners: [f64; 4],
    pub(super) normal: f64,
    pub(super) first: Option<u32>,
    pub(super) elapsed: Duration,
    pub(super) steps: u32,
    pub(super) tangent: DVec3,
    pub(super) positive_points: usize,
    /// Sum of normal impulse times absolute post-solve relative normal speed.
    pub(super) loaded_normal_residual: f64,
    pub(super) max_post_closing: f64,
    pub(super) first_end: Option<Duration>,
    pub(super) first_native_normal: f64,
}
impl Solve {
    pub(super) fn append(
        &mut self,
        t: Duration,
        foot: BodyState,
        support: BodyState,
        ids: [BodyId; 2],
        patches: Vec<RigidContactPatch>,
    ) {
        self.steps += 1;
        self.elapsed = t;
        let inverse = DQuat::from_array(foot.orientation).conjugate();
        let mut load = 0.;
        for p in patches {
            let foot_first = (p.a, p.b) == (ids[0], ids[1]);
            if !foot_first && (p.b, p.a) != (ids[0], ids[1]) {
                continue;
            }
            let sign = if foot_first { 1. } else { -1. };
            let normal = sign * DVec3::from(p.normal_on_a);
            for point in p.points {
                let (a, b) = if foot_first {
                    (point.lever_a, point.lever_b)
                } else {
                    (point.lever_b, point.lever_a)
                };
                let lever = DVec3::from(a);
                let r = inverse * lever;
                let j = point.normal_impulse;
                self.corners[2 * usize::from(r.x > 0.) + usize::from(r.y > 0.)] += j;
                self.normal += j;
                load += j;
                if let Some(tangent) = point.tangent_impulse_on_a {
                    self.tangent += sign * DVec3::from(tangent);
                }
                if j > 0. {
                    let v = DVec3::from(foot.velocity)
                        + DVec3::from(foot.angular_velocity).cross(lever);
                    let u = DVec3::from(support.velocity)
                        + DVec3::from(support.angular_velocity).cross(DVec3::from(b));
                    let vn = normal.dot(v - u);
                    self.loaded_normal_residual += j * vn.abs();
                    self.max_post_closing = self.max_post_closing.max(-vn);
                    self.positive_points += 1;
                }
            }
        }
        if self.first.is_none() && load > 0. {
            self.first = Some(self.steps);
            self.first_end = Some(t);
            self.first_native_normal = load;
        }
    }
}

fn experiment(f: Fixture, cut: &ReboundFrame, method: Input) -> Result<Vec<ReboundFrame>> {
    let mut r = restart(f, ADAPTIVE, cut)?;
    let lane = &mut r.case.lane;
    let ids = [lane.foot, lane.pad];
    let upper = lane.upper;
    let mut resource = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let engine = resource
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("missing local engine")?;
    let mut frames = vec![*cut];
    let pulse = |solver: &mut MultiFidelityBackend| solver.add_world_wrench(upper, FORCE, TORQUE);
    let mut speed = r.last_speed;
    for step in 1..=30 {
        let tick = cut.tick + step;
        let next = command(f, tick - 1);
        if next != speed {
            engine.set_world_velocity(ids[1], [0., 0., next], [0.; 3])?;
            speed = next;
        }
        let mut solve = Solve::default();
        match method {
            Input::Public => {
                if step == 1 {
                    pulse(engine)?;
                }
                let pair = engine.observe_contact_interval(OUTER, CALLS, 4, |s| {
                    solve.append(
                        s.elapsed,
                        s.state(ids[0]).expect("known foot"),
                        s.state(ids[1]).expect("known support"),
                        ids,
                        s.contact_patches(),
                    );
                })?;
                let receipt = pair
                    .pairs
                    .iter()
                    .find(|p| (p.a, p.b) == (ids[0].min(ids[1]), ids[0].max(ids[1])));
                let normal = receipt.map_or(0., |p| p.normal_impulse);
                let first = solve.first.map(|n| OUTER / CALLS * n);
                if pair.native_steps != CALLS
                    || (normal - solve.normal).abs() > 1e-12
                    || receipt.and_then(|p| p.first_positive_end) != first
                {
                    return Err("public receipt and point-by-point readback disagree".into());
                }
            }
            Input::SingleNative | Input::Reissued => {
                let mut t = Duration::ZERO;
                for (index, h) in NativeStepGrid::new(OUTER, CALLS)?.intervals().enumerate() {
                    if step == 1 && (method == Input::Reissued || index == 0) {
                        pulse(engine)?;
                    }
                    engine.step(h)?;
                    t += h;
                    solve.append(
                        t,
                        engine.state(ids[0]).ok_or("missing foot")?,
                        engine.state(ids[1]).ok_or("missing support")?,
                        ids,
                        engine.rigid_contact_patches(),
                    );
                }
            }
        }
        if solve.elapsed != OUTER || solve.steps != CALLS {
            return Err("contact interval drifted".into());
        }
        let read = |id| engine.state(id).ok_or("missing interval body");
        let foot = read(ids[0])?;
        let pad = read(ids[1])?;
        frames.push(ReboundFrame {
            tick,
            time_us: tick * STEP_US,
            foot: foot.into(),
            upper: read(upper)?.into(),
            pad: pad.into(),
            normal_impulse_ns: solve.normal,
            corner_ns: solve.corners,
            first_loaded_substep: solve.first,
            gap_m: supported_gap(foot, pad).ok_or("foot left support footprint")?,
            native_steps: CALLS,
            solver_iterations: 4,
            selected: true,
            primary_start: false,
            secondary_start: false,
            weak_start: false,
            tangent_on_foot_ns: None,
        });
    }
    Ok(frames)
}

pub(crate) fn write(out: &Path) -> Result<()> {
    let f = CASES
        .iter()
        .find(|f| f.name == "outer-fast")
        .copied()
        .ok_or("missing outer-fast fixture")?;
    let (donor, _) = run(f, ADAPTIVE)?;
    // Apply a force one tick before primary contact so the airborne impulse can
    // be compared with the input integral independent of contact.
    let cut = &donor[primary(&donor, 1)? - 2];
    fs::create_dir_all(out.join("traces"))?;
    let naive = experiment(f, cut, Input::SingleNative)?;
    let public = experiment(f, cut, Input::Public)?;
    let explicit = experiment(f, cut, Input::Reissued)?;
    let mut max_error = 0.0_f64;
    for (a, b) in public.iter().zip(&explicit) {
        max_error = max_error.max(distance(a, b));
        if !same_state(a, b) {
            return Err(format!(
                "refined input differs from explicit reference at tick {}",
                a.tick
            )
            .into());
        }
    }
    let delivered =
        |frames: &[ReboundFrame]| 2. * (velocity(&frames[1]).y - velocity(&frames[0]).y);
    if (delivered(&public) - OUTER.as_secs_f64() * FORCE[1]).abs() > 1e-6
        || (delivered(&naive) - OUTER.as_secs_f64() * FORCE[1] / f64::from(CALLS)).abs() > 1e-6
    {
        return Err("free-flight input and measured momentum disagree".into());
    }
    for (name, frames) in [
        ("naive", &naive),
        ("public", &public),
        ("explicit", &explicit),
    ] {
        trace(out, name, frames)?;
    }
    let lanes = [
        ("naive", "Only first native interval"),
        ("public", "One public-step command"),
        ("explicit", "Reissued every native interval"),
    ]
    .map(|(name, title)| json!({"file":format!("traces/{name}.json"),"label":name,"name":title}));
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Force duration through a subdivided physics tick","lanes":lanes}),
    )?;
    write_json(
        &out.join("interval.json"),
        &json!({"cut":snapshot(cut),"calls_per_tick":CALLS,"outer_us":STEP_US,
        "force_n":FORCE,"torque_nm":TORQUE,"public_impulse_ns":delivered(&public),"first_only_impulse_ns":delivered(&naive),
        "public_matches_explicit_boundaries":public.len(),"max_public_vs_explicit_com_m":max_error,
        "first_only_vs_public_end_com_m":distance(naive.last().unwrap(),public.last().unwrap())}),
    )
}
