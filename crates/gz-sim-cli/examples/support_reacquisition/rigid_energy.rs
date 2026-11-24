// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate the contact representation, incoming tilt, spin, and inertial frame.
use super::*;
#[path = "hinge_impact.rs"]
pub(crate) mod hinge;
#[path = "analytic_impact.rs"]
pub(crate) mod analytic;
use bevy_ecs::prelude::{Res, ResMut, Resource};
use gz_physics::{BodySpec, DynamicBackend, Inertia, RotationSpec, Shape};
use gz_sim_core::{StepFault, UpdateInfo};

const G: f64 = 9.81;
const M: f64 = 2.;
const I: f64 = 0.05;
const UPPER_HALF: [f64; 3] = [0.05, 0.06, 0.06];
const ARM: DVec3 = DVec3::new(0., 0., 0.07);

#[derive(Clone, Copy, PartialEq, Eq)]
enum Variant {
    Matched,
    Level,
    NoSpin,
    LevelNoSpin,
    Central,
    RelativeFrame,
    ColdTree,
}
impl Variant {
    fn name(self) -> &'static str {
        match self {
            Self::Matched => "matched",
            Self::Level => "level",
            Self::NoSpin => "no_spin",
            Self::LevelNoSpin => "level_no_spin",
            Self::Central => "central",
            Self::RelativeFrame => "relative_frame",
            Self::ColdTree => "cold_tree",
        }
    }
}

struct Mechanics {
    com: DVec3,
    v: DVec3,
    angular_momentum: DVec3,
    translation: f64,
    rotation: f64,
}
fn mechanics(foot: Body, upper: Body) -> Mechanics {
    // Each link has mass 1 kg. Both link-local inertia tensors are diagonal.
    let com = (DVec3::from(foot.position) + DVec3::from(upper.position)) / 2.;
    let v = (DVec3::from(foot.velocity) + DVec3::from(upper.velocity)) / 2.;
    let mut l = DVec3::ZERO;
    let mut rotation = 0.;
    for link in [foot, upper] {
        let w = DVec3::from(link.angular_velocity);
        let q = DQuat::from_array(link.orientation);
        let spin = q * (DVec3::new(0.0201, 0.0201, 0.025) * (q.conjugate() * w));
        let relative = DVec3::from(link.velocity) - v;
        l += (DVec3::from(link.position) - com).cross(relative) + spin;
        rotation += 0.5 * (relative.length_squared() + w.dot(spin));
    }
    Mechanics {
        com,
        v,
        angular_momentum: l,
        translation: M * 0.5 * v.length_squared(),
        rotation,
    }
}

fn cube(half_extents: [f64; 3], b: Body, mass: f64) -> BodySpec {
    BodySpec {
        shape: Shape::Box { half_extents },
        position: b.position,
        velocity: b.velocity,
        mass: Some(mass),
    }
}
fn orientation(body: Body) -> RotationSpec {
    RotationSpec {
        orientation: body.orientation,
        angular_velocity: body.angular_velocity,
    }
}
fn inertia(center: [f64; 3], diag: [f64; 3]) -> Option<Inertia> {
    Some(Inertia {
        center,
        orientation: DQuat::IDENTITY.to_array(),
        tensor: [diag[0], diag[1], diag[2], 0., 0., 0.],
    })
}
#[derive(Copy, Clone)]
struct Bodies {
    foot: BodyId,
    upper: Option<BodyId>,
    pad: BodyId,
}

pub(super) fn spawn_locked_pair(
    native: &mut MultiFidelityBackend,
    foot: Body,
    upper: Body,
) -> Result<[BodyId; 2]> {
    let spawn = |engine: &mut MultiFidelityBackend, link: Body, half_extents| {
        engine.spawn_articulation_link(
            cube(half_extents, link, 1.),
            orientation(link),
            inertia([0.; 3], [0.0201, 0.0201, 0.025]),
        )
    };
    let a = spawn(native, foot, FOOT_HALF)?;
    let b = spawn(native, upper, UPPER_HALF)?;
    for id in [a, b] {
        native.set_self_collision_group(id, Some(2))?;
    }
    let axis = (DQuat::from_array(upper.orientation) * DVec3::X).to_array();
    native.add_limited_revolute(Some(a), b, upper.position, axis, [0., 0.])?;
    native.enable_exact_revolute_locks(a)?;
    Ok([a, b])
}

fn world(s: Sample, variant: Variant) -> Result<(MultiFidelityBackend, Bodies, f64)> {
    let mut native = MultiFidelityBackend::new([0., 0., -G])?;
    let mut pad = s.pad;
    let mut boost = 0.;
    let (foot, upper) = if variant == Variant::ColdTree {
        let [a, b] = spawn_locked_pair(&mut native, s.foot, s.upper)?;
        (a, Some(b))
    } else {
        let m = mechanics(s.foot, s.upper);
        let mut v = m.v;
        let mut w = m.angular_momentum / I;
        let mut shape = s.foot;
        // Level about the same incoming minimum clearance to remove the angular
        // exposure of the other corners without shifting the first geometric arrival.
        if matches!(
            variant,
            Variant::Level | Variant::LevelNoSpin | Variant::Central
        ) {
            let q = DQuat::from_array(s.foot.orientation);
            let heading = q * DVec3::X;
            shape.orientation = DQuat::from_rotation_z(heading.y.atan2(heading.x)).to_array();
            shape.position[2] = pad.position[2] + PAD_HALF[2] + FOOT_HALF[2] + s.vertical_gap_m;
        }
        if matches!(
            variant,
            Variant::NoSpin | Variant::LevelNoSpin | Variant::Central
        ) {
            w = DVec3::ZERO;
        }
        if variant == Variant::Central {
            shape.orientation = DQuat::IDENTITY.to_array();
            shape.position[..2].copy_from_slice(&pad.position[..2]);
            v.x = 0.;
            v.y = 0.;
        }
        if variant == Variant::RelativeFrame {
            boost = pad.velocity[2];
            v.z -= boost;
            pad.velocity[2] -= boost;
        }
        shape.velocity = v.to_array();
        shape.angular_velocity = w.to_array();
        let a = native.spawn_link(
            cube(FOOT_HALF, shape, M),
            orientation(shape),
            inertia(ARM.to_array(), [I; 3]),
        )?;
        (a, None)
    };
    let pad = native.spawn_kinematic_link(cube(PAD_HALF, pad, 1.), orientation(pad), None)?;
    Ok((native, Bodies { foot, upper, pad }, boost))
}

#[derive(Serialize)]
struct Frame {
    time_us: u64,
    dt_us: u64,
    foot: Body,
    upper: Body,
    pad: Body,
    normal_impulse_ns: f64,
    corner_ns: [f64; 4],
    com_m: [f64; 3],
    momentum_ns: [f64; 3],
    angular_momentum_com_nms: [f64; 3],
    kinetic_translation_j: f64,
    kinetic_rotation_j: f64,
    potential_j: f64,
    support_work_j: f64,
    normal_work_j: f64,
    contact_work_j: Option<f64>,
    solved_vertical_impulse_ns: Option<f64>,
    total_vertical_impulse_ns: f64,
    normal_vertical_impulse_ns: f64,
    sole_gap_m: f64,
    upper_clearance_m: f64,
}

impl Frame {
    fn energy(&self) -> f64 {
        self.kinetic_translation_j + self.kinetic_rotation_j + self.potential_j
    }

    fn read(
        native: &MultiFidelityBackend,
        ids: Bodies,
        prev: Option<&Self>,
        time_us: u64,
        dt_us: u64,
    ) -> std::result::Result<Self, PhysicsError> {
        let get = |id| {
            native
                .state(id)
                .map(Body::from)
                .ok_or(PhysicsError::MissingBody)
        };
        let mut foot = get(ids.foot)?;
        let pad = get(ids.pad)?;
        let q = DQuat::from_array(foot.orientation);
        let arm = q * ARM;
        let upper = if let Some(id) = ids.upper {
            get(id)?
        } else {
            // The free body's velocity is reported at its COM. Represent its
            // lower and upper mass centres at their actual swept offset points.
            let v = DVec3::from(foot.velocity);
            let w = DVec3::from(foot.angular_velocity);
            foot.velocity = (v - w.cross(arm)).to_array();
            Body {
                position: (DVec3::from(foot.position) + 2. * arm).to_array(),
                velocity: (v + w.cross(arm)).to_array(),
                ..foot
            }
        };
        let m = mechanics(foot, upper);
        let mut corner_ns = [0.; 4];
        let mut normal_z = 0.;
        let mut solved_z = Some(0.);
        for p in native.rigid_contact_patches() {
            let direct = p.a == ids.foot && p.b == ids.pad;
            if !direct && (p.b != ids.foot || p.a != ids.pad) {
                continue;
            }
            for point in p.points {
                let lever = if direct { point.lever_a } else { point.lever_b };
                let local = q.conjugate() * DVec3::from(lever);
                let k = usize::from(local.x > 0.) * 2 + usize::from(local.y > 0.);
                corner_ns[k] += point.normal_impulse;
                let sign = if direct { 1. } else { -1. };
                let normal = point.normal_impulse * p.normal_on_a[2] * sign;
                normal_z += normal;
                // Grouped free-body friction is a separate patch wrench. For a
                // perfectly horizontal plane its unknown tangent has zero z projection.
                let tangent_z = point
                    .tangent_impulse_on_a
                    .map(|j| sign * j[2])
                    .or_else(|| (p.normal_on_a[0] == 0. && p.normal_on_a[1] == 0.).then_some(0.));
                solved_z = solved_z
                    .zip(tangent_z)
                    .map(|(sum, tangent)| sum + normal + tangent);
            }
        }
        // Under gravity and this one contact, a momentum readback gives total
        // vertical impulse, including any vertical component of friction.
        let pz = M * m.v.z;
        let jz = prev.map_or(0., |f| {
            pz - f.momentum_ns[2] + M * G * (dt_us as f64 * 1e-6)
        });
        let work = prev.map_or(0., |f| f.support_work_j) + pad.velocity[2] * jz;
        let normal_work = prev.map_or(0., |f| f.normal_work_j) + pad.velocity[2] * normal_z;
        let contact_work = prev
            .map_or(Some(0.), |f| f.contact_work_j)
            .zip(solved_z)
            .map(|(old, j)| old + pad.velocity[2] * j);
        let upper_q = DQuat::from_array(upper.orientation);
        let upper_clearance = upper.position[2]
            - (upper_q.conjugate() * DVec3::Z)
                .abs()
                .dot(UPPER_HALF.into())
            - pad.position[2]
            - PAD_HALF[2];
        Ok(Self {
            time_us,
            dt_us,
            foot,
            upper,
            pad,
            normal_impulse_ns: corner_ns.iter().sum(),
            corner_ns,
            com_m: m.com.to_array(),
            momentum_ns: (M * m.v).to_array(),
            angular_momentum_com_nms: m.angular_momentum.to_array(),
            kinetic_translation_j: m.translation,
            kinetic_rotation_j: m.rotation,
            potential_j: M * G * m.com.z,
            support_work_j: work,
            normal_work_j: normal_work,
            contact_work_j: contact_work,
            solved_vertical_impulse_ns: solved_z,
            total_vertical_impulse_ns: jz,
            normal_vertical_impulse_ns: normal_z,
            sole_gap_m: sole_gap(foot, pad),
            upper_clearance_m: upper_clearance,
        })
    }
}

#[derive(Resource)]
struct Recorder {
    ids: Bodies,
    factor: u32,
    frames: Vec<Frame>,
}

fn advance(
    time: Res<UpdateInfo>,
    fault: Res<StepFault>,
    mut backend: ResMut<PhysicsBackend>,
    mut recorder: ResMut<Recorder>,
) {
    if time.dt.is_zero() {
        return;
    }
    let outer = Outer {
        tick: CUT_TICK + time.iteration,
        command: None,
    };
    let n = outer.substeps(recorder.factor);
    let dt = time.dt / n;
    let us = dt.as_micros() as u64;
    let mut t = (outer.tick - 1) * STEP_US;
    for _ in 0..n {
        t += us;
        if let Err(e) = backend.step(dt) {
            fault.report(e);
            return;
        }
        let native = backend
            .downcast_ref::<MultiFidelityBackend>()
            .expect("local native control");
        match Frame::read(native, recorder.ids, recorder.frames.last(), t, us) {
            Ok(frame) => recorder.frames.push(frame),
            Err(e) => {
                fault.report(e);
                return;
            }
        }
    }
}

// A continuous purely vertical, central, perfectly plastic catch. t starts at
// the incoming cut and the pad velocity is constant during this five-ms window.
fn ideal(start: &Frame) -> Value {
    let closing = start.pad.velocity[2] - start.momentum_ns[2] / M;
    let gap = start.sole_gap_m;
    let v = (closing * closing + 2. * G * gap).sqrt();
    let t = 2. * gap / (v + closing);
    let impact = M * v;
    let window = impact + M * G * (0.005 - t);
    json!({"impact_after_cut_s":t,"impact_ns":impact,"window_ns":window,
        "plastic_impact_loss_j":0.5*M*v*v,"window_support_work_j":window*start.pad.velocity[2]})
}

fn run(
    donor: &str,
    s: Sample,
    variant: Variant,
    factor: u32,
    output: Option<&Path>,
) -> Result<Value> {
    let (native, ids, boost) = world(s, variant)?;
    let cut = CUT_TICK * STEP_US;
    let first = Frame::read(&native, ids, None, cut, 0)?;
    let mut sim = Simulation::new(Duration::from_micros(STEP_US))?;
    gz_physics::install_with_step(&mut sim, native, advance)?;
    sim.world_mut().insert_resource(Recorder {
        ids,
        factor,
        frames: vec![first],
    });
    for step in schedule() {
        if let Some(v) = step.command {
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .set_world_velocity(ids.pad, [0., 0., v - boost], [0.; 3])?;
        }
        sim.step_exact(1)?;
    }
    let recorder = sim
        .world_mut()
        .remove_resource::<Recorder>()
        .expect("registered recorder");
    let f = &recorder.frames;
    let a = &f[0];
    let b = &f[(factor as usize) * WINDOW_STEPS];
    let end = f.last().ok_or("absent terminal state")?;
    let incoming = mechanics(s.foot, s.upper);
    let first_load = f
        .iter()
        .position(|r| r.normal_impulse_ns > 0.)
        .ok_or("no rigid landing")?;
    let landing = &f[first_load];
    let mut recontacts = Vec::new();
    let mut free = None;
    for step in &f[first_load + 1..] {
        if step.normal_impulse_ns == 0. && step.sole_gap_m > 1e-5 {
            let (_, peak) = free.get_or_insert((step.time_us, 0.0_f64));
            *peak = peak.max(step.sole_gap_m);
        } else if step.normal_impulse_ns > 0.
            && let Some((since, peak)) = free.take()
        {
            recontacts.push(json!({"time_us":step.time_us,"first_free_us":since,"peak_gap_m":peak,"first_ns":step.normal_impulse_ns}));
        }
    }
    let min_upper = f
        .iter()
        .map(|r| r.upper_clearance_m)
        .fold(f64::INFINITY, f64::min);
    if min_upper < 0.1 || b.time_us != cut + 5_000 || end.time_us != 840_500 {
        return Err("rigid control clearance or timing qualification failed".into());
    }
    let window_j: f64 = f[..=factor as usize * WINDOW_STEPS]
        .iter()
        .map(|r| r.normal_impulse_ns)
        .sum();
    let window_z: f64 = f[..=factor as usize * WINDOW_STEPS]
        .iter()
        .map(|r| r.total_vertical_impulse_ns)
        .sum();
    let write_trace = matches!(
        variant,
        Variant::Matched | Variant::LevelNoSpin | Variant::RelativeFrame | Variant::ColdTree
    );
    let trace = format!("traces/{donor}-{}-{factor}x.json", variant.name());
    if write_trace && let Some(dir) = output {
        let geometry = json!({"foot_half_extents":FOOT_HALF,"upper_half_extents":UPPER_HALF,"pad_half_extents":PAD_HALF});
        write_json(
            &dir.join(&trace),
            &json!({"donor":donor,"variant":variant.name(),"factor":factor,"geometry":geometry,"frames":f}),
        )?;
    }
    let max_balance = f
        .iter()
        .map(|r| (r.total_vertical_impulse_ns - r.normal_vertical_impulse_ns).abs())
        .fold(0., f64::max);
    let max_solved_balance = f.iter().try_fold(0_f64, |old, r| {
        r.solved_vertical_impulse_ns
            .map(|j| old.max((r.total_vertical_impulse_ns - j).abs()))
    });
    Ok(
        json!({"donor":donor,"variant":variant.name(),"factor":factor,"trace":write_trace.then_some(trace),
        "cut_com_m":a.com_m,"cut_min_gap_m":a.sole_gap_m, "cut_p_ns":a.momentum_ns,"cut_h_nms":a.angular_momentum_com_nms,
        "cut_com_offset_from_donor_m":(DVec3::from(a.com_m)-incoming.com).to_array(),
        "cut_p_offset_from_donor_ns":(DVec3::from(a.momentum_ns)-M*incoming.v).to_array(),
        "cut_h_offset_from_donor_nms":(DVec3::from(a.angular_momentum_com_nms)-incoming.angular_momentum).to_array(),
        "cut_mechanical_energy_j":a.energy(),"min_upper_clearance_m":min_upper,
        "first_time_us":landing.time_us,"first_ns":landing.normal_impulse_ns,"first_corners_ns":landing.corner_ns,
        "window_ns":window_j, "stroke_ns":f.iter().map(|r|r.normal_impulse_ns).sum::<f64>(),
        "window_vertical_ns":window_z, "window_work_j":b.support_work_j,"window_normal_work_j":b.normal_work_j,
        "window_contact_work_j":b.contact_work_j,"end_contact_work_j":end.contact_work_j,
        "max_step_vertical_impulse_minus_solved_z_ns":max_solved_balance,
        "window_delta_energy_j":b.energy()-a.energy(),"window_loss_j":b.support_work_j-(b.energy()-a.energy()),
        "window_rotation_j":b.kinetic_rotation_j,"window_com_vz":b.momentum_ns[2]/M,
        "end_work_j":end.support_work_j,"end_loss_j":end.support_work_j-(end.energy()-a.energy()),
        "max_step_vertical_impulse_minus_normal_z_ns":max_balance,"recontacts":recontacts,
        "ideal":(variant==Variant::Central).then(||ideal(a))}),
    )
}

pub(crate) fn write(output: &Path) -> Result<()> {
    fs::create_dir_all(output.join("traces"))?;
    let cut = Cut::new()?;
    let mut runs = Vec::new();
    for (index, &donor) in LABELS.iter().enumerate() {
        let s = cut.cases[index].samples[CUT_TICK as usize];
        for variant in [
            Variant::Matched,
            Variant::Level,
            Variant::NoSpin,
            Variant::LevelNoSpin,
            Variant::Central,
            Variant::RelativeFrame,
            Variant::ColdTree,
        ] {
            for factor in [1, 2, 4] {
                runs.push(run(donor, s, variant, factor, Some(output))?);
            }
        }
    }
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(
            &json!({"cut_us":CUT_TICK*STEP_US,"outer_us":STEP_US,"window_us":5000,"runs":runs}),
        )?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn metric(row: &Value, key: &str) -> f64 {
        row[key].as_f64().unwrap()
    }

    #[test]
    fn cold_free_body_preserves_both_incoming_states_and_has_no_contact_history() -> Result<()> {
        let cut = Cut::new()?;
        for case in &cut.cases {
            let s = case.samples[CUT_TICK as usize];
            let expected = mechanics(s.foot, s.upper);
            let (native, ids, _) = world(s, Variant::Matched)?;
            let initial = Frame::read(&native, ids, None, CUT_TICK * STEP_US, 0)?;
            let historic = case
                .backend()
                .native_contact_warmstart(case.lane.foot, case.lane.pad)?;
            assert_eq!(historic.normal_ns, 0.);
            assert_eq!(historic.points, 4);
            assert_eq!(
                native.native_contact_warmstart(ids.foot, ids.pad)?.points,
                0
            );
            assert!(DVec3::from(initial.com_m).distance(expected.com) < 1e-14);
            assert!(DVec3::from(initial.momentum_ns).distance(M * expected.v) < 1e-14);
            assert!(
                DVec3::from(initial.angular_momentum_com_nms).distance(expected.angular_momentum)
                    < 1e-14
            );
            assert!((initial.kinetic_rotation_j - expected.rotation).abs() < 1e-14);
            assert!((initial.sole_gap_m - s.vertical_gap_m).abs() < 1e-14);
            for (got, donor) in [(initial.foot, s.foot), (initial.upper, s.upper)] {
                assert!(DVec3::from(got.position).distance(donor.position.into()) < 1e-14);
                assert!(DVec3::from(got.velocity).distance(donor.velocity.into()) < 1e-12);
                assert!(
                    DQuat::from_array(got.orientation)
                        .dot(DQuat::from_array(donor.orientation))
                        .abs()
                        > 1. - 1e-14
                );
            }
        }
        Ok(())
    }

    #[test]
    fn changing_inertial_frame_preserves_impulse_and_inferred_loss_through_touchdown() -> Result<()>
    {
        let cut = Cut::new()?;
        for (case, label) in cut.cases.iter().zip(LABELS) {
            let s = case.samples[CUT_TICK as usize];
            for factor in [1, 2, 4] {
                let lab = run(label, s, Variant::Matched, factor, None)?;
                let moving = run(label, s, Variant::RelativeFrame, factor, None)?;
                let j = metric(&lab, "window_vertical_ns");
                for row in [&lab, &moving] {
                    assert!(metric(row, "max_step_vertical_impulse_minus_normal_z_ns") < 1e-12);
                    assert!(
                        (metric(row, "window_work_j") - metric(row, "window_normal_work_j")).abs()
                            < 1e-12
                    );
                    assert_eq!(row["first_time_us"], 712_000);
                }
                let expected_work = s.pad.velocity[2] * j;
                assert!(metric(&moving, "window_work_j").abs() < 1e-12);
                assert!((metric(&lab, "window_work_j") - expected_work).abs() < 1e-12);
                assert!((metric(&moving, "window_vertical_ns") - j).abs() < 1e-8);
                assert!(
                    (metric(&moving, "window_loss_j") - metric(&lab, "window_loss_j")).abs() < 1e-8
                );
            }
        }
        Ok(())
    }

    #[test]
    fn cold_locked_tree_preserves_window_response_and_leveling_changes_it() -> Result<()> {
        let cut = Cut::new()?;
        for (index, label) in LABELS.into_iter().enumerate() {
            let s = cut.cases[index].samples[CUT_TICK as usize];
            for factor in [1, 2, 4] {
                let tree = run(label, s, Variant::ColdTree, factor, None)?;
                assert!(metric(&tree, "max_step_vertical_impulse_minus_solved_z_ns") < 1e-4);
                assert!(
                    (metric(&tree, "window_contact_work_j") - metric(&tree, "window_work_j")).abs()
                        < 1e-4
                );
                let native = simulate(&cut, index, None, factor, &schedule())?;
                let native_sum: f64 = native
                    .frames
                    .iter()
                    .take_while(|f| f.time_us <= CUT_TICK * STEP_US + 5_000)
                    .map(|f| f.normal_impulse_ns)
                    .sum();
                assert!((native_sum - metric(&tree, "window_ns")).abs() < 1e-4);
                let central = run(label, s, Variant::Central, factor, None)?;
                assert!(
                    (metric(&central, "window_contact_work_j") - metric(&central, "window_work_j"))
                        .abs()
                        < 1e-12
                );
                let exact = &central["ideal"];
                let analytic = metric(exact, "window_ns");
                assert!((metric(&central, "window_ns") - analytic).abs() < 0.03);
                let expected_arrival =
                    (CUT_TICK * STEP_US) as f64 + metric(exact, "impact_after_cut_s") * 1e6;
                let measured_arrival = metric(&central, "first_time_us");
                assert!(
                    (0. ..=STEP_US as f64 / f64::from(factor))
                        .contains(&(measured_arrival - expected_arrival))
                );
            }
            let matched = run(label, s, Variant::Matched, 4, None)?;
            let nospin = run(label, s, Variant::NoSpin, 4, None)?;
            let level = run(label, s, Variant::Level, 4, None)?;
            let both = run(label, s, Variant::LevelNoSpin, 4, None)?;
            for row in [&nospin, &level, &both] {
                assert!(
                    (metric(row, "cut_min_gap_m") - metric(&matched, "cut_min_gap_m")).abs()
                        < 1e-14
                );
            }
            assert!((metric(&matched, "window_ns") - metric(&nospin, "window_ns")).abs() < 0.001);
            assert!(metric(&matched, "window_ns") - metric(&level, "window_ns") > 0.35);
            assert!(metric(&nospin, "window_rotation_j") > 0.1);
            assert!(metric(&both, "window_rotation_j") < 0.0002);
        }
        Ok(())
    }
}
