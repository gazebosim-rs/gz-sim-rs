// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Mechanical balance of the two 1 kg links, on each actual contact solver boundary.
use super::*;
use glam::DMat3;

const G: f64 = 9.81;
const M: f64 = 2.;
const I: DVec3 = DVec3::new(0.0201, 0.0201, 0.025);

#[derive(Clone, Copy)]
struct Mechanics {
    com: DVec3,
    p: DVec3,
    h: DVec3,
    com_energy: f64,
    rotation: f64,
    internal: f64,
    link_spin: f64,
    potential: f64,
    total: f64,
}

impl Mechanics {
    fn from_links(links: [Body; 2]) -> Self {
        // Link frames coincide with each link's COM. Each link has mass 1 kg.
        let [v0, v1] = links.map(|b| DVec3::from(b.velocity));
        let [x0, x1] = links.map(|b| DVec3::from(b.position));
        let com = (x0 + x1) / M;
        let p = v0 + v1;
        let v_com = p / M;
        let mut h = DVec3::ZERO;
        let mut inertia = DMat3::ZERO;
        let mut link_spin = 0.;
        let mut k = 0.;
        let components = links.map(|b| {
            let rot = DMat3::from_quat(DQuat::from_array(b.orientation));
            let i = rot * DMat3::from_diagonal(I) * rot.transpose();
            let w = DVec3::from(b.angular_velocity);
            let v = DVec3::from(b.velocity);
            let arm = DVec3::from(b.position) - com;
            let iw = i * w;
            h += iw + arm.cross(v - v_com);
            // Parallel-axis tensor, m=1.
            inertia += i + DMat3::IDENTITY * arm.length_squared()
                - DMat3::from_cols(arm * arm.x, arm * arm.y, arm * arm.z);
            link_spin += 0.5 * w.dot(iw);
            k += 0.5 * (v.length_squared() + w.dot(iw));
            (i, w, v, arm)
        });
        let spin = inertia.inverse() * h;
        let rotation = 0.5 * spin.dot(h);
        let com_energy = 0.5 * M * v_com.length_squared();
        let internal = components
            .into_iter()
            .map(|(i, w, v, r)| {
                let rel = w - spin;
                0.5 * ((v - v_com - spin.cross(r)).length_squared() + rel.dot(i * rel))
            })
            .sum::<f64>();
        let potential = M * G * com.z;
        // This compares the direct body sum with a separate kinetic decomposition.
        debug_assert!((k - com_energy - rotation - internal).abs() < 1e-11);
        Self {
            com,
            p,
            h,
            com_energy,
            rotation,
            internal,
            link_spin,
            potential,
            total: k + potential,
        }
    }
}

#[derive(Default)]
struct Contact {
    normal: f64,
    normal_j: DVec3,
    tangential: DVec3,
    moment: DVec3,
    normal_moment: DVec3,
    corners: [f64; 4],
    corner_vectors: [DVec3; 4],
    axial_friction_moment: DVec3,
    corner_moments: [DVec3; 4],
    lift_work: f64,
    foot_work: f64,
    unavailable_tangents: bool,
}

impl Contact {
    fn read(
        fork: &LandingFork,
        foot_id: BodyId,
        pad_id: BodyId,
        before: Body,
        after: Body,
        pad: Body,
        com: DVec3,
    ) -> Self {
        let mut c = Self::default();
        let local = DQuat::from_array(after.orientation).conjugate();
        let arm = DVec3::from(before.position) - com;
        let midpoint_v = (DVec3::from(before.velocity) + DVec3::from(after.velocity)) * 0.5;
        let midpoint_w =
            (DVec3::from(before.angular_velocity) + DVec3::from(after.angular_velocity)) * 0.5;
        for patch in fork.contact_patches() {
            let foot_a = (patch.a, patch.b) == (foot_id, pad_id);
            if !foot_a && (patch.b, patch.a) != (foot_id, pad_id) {
                continue;
            }
            let sign = if foot_a { 1. } else { -1. };
            let normal = DVec3::from(patch.normal_on_a) * sign;
            for point in patch.points {
                let Some(tangent) = point.tangent_impulse_on_a else {
                    c.unavailable_tangents = true;
                    continue;
                };
                let t = DVec3::from(tangent) * sign;
                let jn = normal * point.normal_impulse;
                let j = jn + t;
                let foot_arm = DVec3::from(if foot_a { point.lever_a } else { point.lever_b });
                let pad_arm = DVec3::from(if foot_a { point.lever_b } else { point.lever_a });
                let xyz = local * foot_arm;
                let quadrant = 2 * usize::from(xyz.x > 0.) + usize::from(xyz.y > 0.);
                // The generalized contact constraint applies only normal and two
                // point tangent rows. The moment includes their actual solver arms.
                let r = arm + foot_arm;
                let moment = r.cross(j);
                c.normal += point.normal_impulse;
                c.normal_j += jn;
                c.tangential += t;
                c.normal_moment += r.cross(jn);
                c.moment += moment;
                c.axial_friction_moment += normal * normal.dot(r.cross(t));
                c.corner_vectors[quadrant] += j;
                c.corners[quadrant] += point.normal_impulse;
                c.corner_moments[quadrant] += moment;
                c.lift_work += j.dot(
                    DVec3::from(pad.velocity) + DVec3::from(pad.angular_velocity).cross(pad_arm),
                );
                c.foot_work += j.dot(midpoint_v + midpoint_w.cross(foot_arm));
            }
        }
        c
    }
    fn add(&mut self, part: &Self) {
        self.normal += part.normal;
        self.normal_j += part.normal_j;
        self.tangential += part.tangential;
        self.moment += part.moment;
        self.axial_friction_moment += part.axial_friction_moment;
        self.normal_moment += part.normal_moment;
        self.lift_work += part.lift_work;
        self.foot_work += part.foot_work;
        for k in 0..4 {
            self.corners[k] += part.corners[k];
            self.corner_vectors[k] += part.corner_vectors[k];
            self.corner_moments[k] += part.corner_moments[k];
        }
        self.unavailable_tangents |= part.unavailable_tangents;
    }
    fn impulse(&self) -> DVec3 {
        self.normal_j + self.tangential
    }
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
    contact_impulse_ns: [f64; 3],
    tangent_impulse_ns: [f64; 3],
    corner_vector_ns: [[f64; 3]; 4],
    corner_angular_nms: [[f64; 3]; 4],
    step_lift_work_j: f64,
    normal_sum_ns: f64,
    contact_sum_ns: [f64; 3],
    normal_vector_sum_ns: [f64; 3],
    tangent_sum_ns: [f64; 3],
    corner_sum_ns: [f64; 4],
    moment_sum_nms: [f64; 3],
    normal_moment_sum_nms: [f64; 3],
    corner_angular_sum_nms: [[f64; 3]; 4],
    corner_vector_sum_ns: [[f64; 3]; 4],
    axial_friction_moment_sum_nms: [f64; 3],
    com_m: [f64; 3],
    momentum_ns: [f64; 3],
    angular_momentum_nms: [f64; 3],
    linear_residual_ns: [f64; 3],
    angular_residual_nms: [f64; 3],
    kinetic_com_j: f64,
    kinetic_vertical_j: f64,
    kinetic_lateral_j: f64,
    kinetic_rotation_j: f64,
    kinetic_internal_j: f64,
    kinetic_link_spin_j: f64,
    potential_j: f64,
    delta_kinetic_com_j: f64,
    delta_kinetic_rotation_j: f64,
    delta_kinetic_internal_j: f64,
    delta_potential_j: f64,
    lift_work_j: f64,
    midpoint_foot_work_j: f64,
    gravity_work_j: f64,
    mechanical_change_j: f64,
    energy_loss_j: f64,
    energy_closure_residual_j: f64,
    sole_gap_m: f64,
}

fn frame(
    time: u64,
    dt: u64,
    bodies: [Body; 3],
    initial: Mechanics,
    cumulative: &Contact,
    contact: &Contact,
) -> Frame {
    let [foot, upper, pad] = bodies;
    let state = Mechanics::from_links([foot, upper]);
    let dp = state.p - initial.p;
    let dh = state.h - initial.h;
    let elapsed = (time - CUT_TICK * STEP_US) as f64 * 1e-6;
    let jg = DVec3::NEG_Z * (G * M * elapsed);
    let de = state.total - initial.total;
    Frame {
        time_us: time,
        dt_us: dt,
        foot,
        upper,
        pad,
        normal_impulse_ns: contact.normal,
        corner_ns: contact.corners,
        contact_impulse_ns: contact.impulse().to_array(),
        tangent_impulse_ns: contact.tangential.to_array(),
        corner_vector_ns: contact.corner_vectors.map(|v| v.to_array()),
        corner_angular_nms: contact.corner_moments.map(|v| v.to_array()),
        step_lift_work_j: contact.lift_work,
        normal_sum_ns: cumulative.normal,
        contact_sum_ns: cumulative.impulse().to_array(),
        normal_vector_sum_ns: cumulative.normal_j.to_array(),
        tangent_sum_ns: cumulative.tangential.to_array(),
        corner_sum_ns: cumulative.corners,
        moment_sum_nms: cumulative.moment.to_array(),
        normal_moment_sum_nms: cumulative.normal_moment.to_array(),
        corner_angular_sum_nms: cumulative.corner_moments.map(|v| v.to_array()),
        corner_vector_sum_ns: cumulative.corner_vectors.map(|v| v.to_array()),
        axial_friction_moment_sum_nms: cumulative.axial_friction_moment.to_array(),
        com_m: state.com.to_array(),
        momentum_ns: state.p.to_array(),
        angular_momentum_nms: state.h.to_array(),
        linear_residual_ns: (dp - jg - cumulative.impulse()).to_array(),
        angular_residual_nms: (dh - cumulative.moment).to_array(),
        kinetic_com_j: state.com_energy,
        kinetic_vertical_j: state.p.z * state.p.z / (2. * M),
        kinetic_lateral_j: state.p.truncate().length_squared() / (2. * M),
        kinetic_rotation_j: state.rotation,
        kinetic_internal_j: state.internal,
        kinetic_link_spin_j: state.link_spin,
        potential_j: state.potential,
        delta_kinetic_com_j: state.com_energy - initial.com_energy,
        delta_kinetic_rotation_j: state.rotation - initial.rotation,
        delta_kinetic_internal_j: state.internal - initial.internal,
        delta_potential_j: state.potential - initial.potential,
        gravity_work_j: initial.potential - state.potential,
        lift_work_j: cumulative.lift_work,
        midpoint_foot_work_j: cumulative.foot_work,
        mechanical_change_j: de,
        energy_loss_j: cumulative.lift_work - de,
        energy_closure_residual_j: de - cumulative.foot_work,
        sole_gap_m: sole_gap(foot, pad),
    }
}

/// Mechanics and native impulses sampled from the same cut in both replay lanes.
pub(super) struct Recorder {
    initial: Mechanics,
    cumulative: Contact,
    frames: Vec<Frame>,
}

impl Recorder {
    pub(super) fn new(fork: &LandingFork) -> Self {
        let bodies = fork.states().map(Body::from);
        let initial = Mechanics::from_links([bodies[0], bodies[1]]);
        let cumulative = Contact::default();
        let frames = vec![frame(
            CUT_TICK * STEP_US,
            0,
            bodies,
            initial,
            &cumulative,
            &Contact::default(),
        )];
        Self {
            initial,
            cumulative,
            frames,
        }
    }

    pub(super) fn observe(&mut self, fork: &LandingFork, lane: &Lane, dt: u64) {
        let [foot, upper, pad] = fork.states().map(Body::from);
        let prev = self.frames.last().expect("incoming state");
        let start = Mechanics::from_links([prev.foot, prev.upper]);
        let contact = Contact::read(fork, lane.foot, lane.pad, prev.foot, foot, pad, start.com);
        let time = prev.time_us + dt;
        self.cumulative.add(&contact);
        self.frames.push(frame(
            time,
            dt,
            [foot, upper, pad],
            self.initial,
            &self.cumulative,
            &contact,
        ));
    }

    pub(super) fn write(&self, path: &Path, window: f64, stroke: f64) -> Result<()> {
        let boundary = self
            .frames
            .iter()
            .find(|f| f.time_us == (CUT_TICK + 10) * STEP_US)
            .ok_or("missing common window")?;
        if (boundary.normal_sum_ns - window).abs() > 1e-11
            || (self.cumulative.normal - stroke).abs() > 1e-11
            || self.cumulative.unavailable_tangents
        {
            return Err(
                "native replay differs from scored impulse or lacks measured friction".into(),
            );
        }
        write_trace(path, &self.frames)
    }
}

fn write_trace(path: &Path, frames: &[Frame]) -> Result<()> {
    #[derive(Serialize)]
    struct Trace<'a> {
        geometry: Value,
        frames: &'a [Frame],
    }
    write_json(
        path,
        &Trace {
            geometry: json!({"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF}),
            frames,
        },
    )
}

fn run(cut: &Cut, receiver: usize, motion: Option<usize>, factor: u32) -> Result<Vec<Frame>> {
    let lane = &cut.cases[receiver].lane;
    let mut fork = cut.fork(receiver, motion)?;
    let [foot, upper, pad] = fork.states().map(Body::from);
    let donor = cut.cases[motion.unwrap_or(receiver)].samples[CUT_TICK as usize];
    if [foot, upper, pad] != [donor.foot, donor.upper, donor.pad] || fork.normal_impulse() != 0. {
        return Err("energy fork differs from airborne donor".into());
    }
    let mut recorder = Recorder::new(&fork);
    for outer in schedule() {
        let n = outer.substeps(factor);
        let dt = STEP_US / u64::from(n);
        fork.step_outer(n, outer.command, |fork| recorder.observe(fork, lane, dt))?;
        if recorder.cumulative.unavailable_tangents {
            return Err("native generalized friction writeback unavailable".into());
        }
        let time = recorder.frames.last().expect("incoming state").time_us;
        if time != outer.tick * STEP_US {
            return Err("actuator clock mismatch".into());
        }
    }
    Ok(recorder.frames)
}

fn secondary(frames: &[Frame]) -> Vec<Value> {
    let mut peak: Option<f64> = None;
    let mut hits = Vec::new();
    let first = frames
        .iter()
        .position(|f| f.normal_impulse_ns > 0.)
        .expect("landing");
    for pair in frames[first..].windows(2) {
        let [prev, f] = pair else { unreachable!() };
        if f.sole_gap_m > 1e-5 && f.normal_impulse_ns == 0. {
            peak = Some(peak.unwrap_or_default().max(f.sole_gap_m));
        } else if f.normal_impulse_ns > 0.
            && let Some(gap) = peak.take()
        {
            hits.push(
                json!({"time_us":f.time_us,"normal_impulse_ns":f.normal_impulse_ns,"peak_gap_m":gap,
                "energy_loss_j":f.energy_loss_j,"com_vz_m_s":f.momentum_ns[2]/M,
                "step_lift_work_j":f.step_lift_work_j,"step_mechanical_change_j":f.mechanical_change_j-prev.mechanical_change_j,
                "step_energy_loss_j":f.energy_loss_j-prev.energy_loss_j, "pre_com_vz_m_s":prev.momentum_ns[2]/M}),
            );
        }
    }
    hits
}

fn study(output: Option<&Path>) -> Result<Value> {
    let cut = Cut::new()?;
    let mut results = Vec::new();
    if let Some(out) = output {
        fs::create_dir_all(out.join("traces"))?;
    }
    for (receiver_index, receiver) in LABELS.into_iter().enumerate() {
        for motion in [None, Some(0), Some(1)] {
            for factor in [1, 2, 4] {
                let frames = run(&cut, receiver_index, motion, factor)?;
                let motion = motion.map_or("native", |i| LABELS[i]);
                let trace = format!("traces/{receiver}-{motion}-{factor}x.json");
                let first = frames
                    .iter()
                    .find(|f| f.time_us == (CUT_TICK + 1) * STEP_US)
                    .ok_or("first boundary missing")?;
                let window = frames
                    .iter()
                    .find(|f| f.time_us == (CUT_TICK + WINDOW_STEPS as u64) * STEP_US)
                    .ok_or("window boundary missing")?;
                let end = frames.last().ok_or("empty trace")?;
                let max_norm = |get: fn(&Frame) -> [f64; 3]| {
                    frames
                        .iter()
                        .map(|f| DVec3::from(get(f)).length())
                        .fold(0., f64::max)
                };
                let max_internal = frames
                    .iter()
                    .map(|f| f.kinetic_internal_j)
                    .fold(0., f64::max);
                let max_corner_closure = frames
                    .iter()
                    .map(|f| (f.corner_ns.iter().sum::<f64>() - f.normal_impulse_ns).abs())
                    .fold(0., f64::max);
                let hits = secondary(&frames);
                results.push(json!({"receiver":receiver,"motion":motion,"factor":factor,"trace":trace,
                    "initial":&frames[0],"first":first,"window":window,"stroke":end,"recontacts":hits,
                    "max_linear_residual_ns":max_norm(|f|f.linear_residual_ns),"max_angular_residual_nms":max_norm(|f|f.angular_residual_nms),
                    "max_internal_kinetic_j":max_internal,"max_corner_closure_ns":max_corner_closure}));
                if let Some(out) = output {
                    write_trace(&out.join(&trace), &frames)?;
                }
            }
        }
    }
    Ok(
        json!({"cut_us":CUT_TICK*STEP_US,"window_ends_us":(CUT_TICK+WINDOW_STEPS as u64)*STEP_US,
        "mass_kg":M,"link_inertia_kg_m2":I.to_array(),"runs":results}),
    )
}

pub(crate) fn write(output: &Path) -> Result<()> {
    let summary = study(Some(output))?;
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn laboratory_mass_properties_match_the_authored_rocker() -> Result<()> {
        let sdf = sdformat::load_str(SDF, "support_reacquisition.sdf", &sdformat::NoResources)?;
        let world = &sdf.worlds[0];
        assert_eq!(world.gravity, [0., 0., -G]);
        let rocker = world
            .models
            .iter()
            .find(|model| model.name == "rocker")
            .unwrap();
        assert_eq!(rocker.links.len(), 2);
        for (name, height) in [("foot", 0.), ("upper", 0.14)] {
            let link = rocker.links.iter().find(|link| link.name == name).unwrap();
            assert_eq!(link.inertial.mass, M / 2.);
            assert_eq!(link.inertial.inertia, [I.x, I.y, I.z, 0., 0., 0.]);
            assert_eq!(link.inertial.pose, sdformat::Pose::default());
            assert!(link.pose.relative_to.is_empty());
            assert_eq!(link.pose.transform.translation, [0., 0., height]);
            assert_eq!(link.pose.transform.rotation, [0., 0., 0., 1.]);
        }
        Ok(())
    }

    #[test]
    fn rotated_offset_links_separate_rigid_spin_and_relative_motion() {
        let orientation = DQuat::from_euler(glam::EulerRot::XYZ, 0.7, -0.2, 1.3);
        let axis = orientation * DVec3::Z;
        let com = DVec3::new(12., -8., 2.5);
        let velocity = DVec3::new(0.7, 0.3, -0.6);
        let angular = DVec3::new(-2.4, 1.3, 0.2);
        for relative_speed in [0., 0.35] {
            let links = [-1., 1.].map(|sign| {
                let offset = sign * 0.07 * axis;
                Body {
                    position: (com + offset).to_array(),
                    velocity: (velocity + angular.cross(offset) + sign * relative_speed * axis)
                        .to_array(),
                    angular_velocity: angular.to_array(),
                    orientation: orientation.to_array(),
                }
            });
            let state = Mechanics::from_links(links);
            let expected_internal = relative_speed * relative_speed;
            assert!((state.com - com).length() < 1e-14);
            assert!((state.p - M * velocity).length() < 1e-14);
            // The parallel-axis theorem gives diag(0.05, 0.05, 0.05).
            assert!((state.h - 0.05 * angular).length() < 1e-14);
            assert!((state.com_energy - velocity.length_squared()).abs() < 1e-14);
            assert!((state.rotation - 0.025 * angular.length_squared()).abs() < 1e-14);
            assert!((state.internal - expected_internal).abs() < 1e-14);
            assert!((state.potential - M * G * com.z).abs() < 1e-14);
            assert!(
                (state.total
                    - state.potential
                    - state.rotation
                    - state.com_energy
                    - expected_internal)
                    .abs()
                    < 1e-14
            );
        }
    }

    #[test]
    fn native_impulses_close_angular_momentum_and_bound_discrete_work_at_matched_cuts() -> Result<()>
    {
        let cut = Cut::new()?;
        for receiver in 0..2 {
            for factor in [1, 2, 4] {
                let native = run(&cut, receiver, None, factor)?;
                let sham = run(&cut, receiver, Some(receiver), factor)?;
                assert_eq!(serde_json::to_vec(&native)?, serde_json::to_vec(&sham)?);
                let other = run(&cut, receiver, Some(1 - receiver), factor)?;
                for frames in [&native, &other] {
                    let window = &frames[WINDOW_STEPS * factor as usize];
                    assert_eq!(window.time_us, 716500);
                    assert!(DVec3::from(window.tangent_sum_ns).length() > 0.02);
                    for f in frames {
                        assert!(DVec3::from(f.linear_residual_ns).length() < 1.5e-4);
                        assert!(DVec3::from(f.angular_residual_nms).length() < 1e-12);
                        assert!(f.energy_closure_residual_j.abs() < 0.01);
                        assert!(f.kinetic_internal_j < 1e-27);
                        assert!(
                            (f.kinetic_lateral_j + f.kinetic_vertical_j - f.kinetic_com_j).abs()
                                < 1e-12
                        );
                        assert!(
                            (f.corner_ns.iter().sum::<f64>() - f.normal_impulse_ns).abs() < 1e-12
                        );
                        let sum = |vectors: [[f64; 3]; 4]| {
                            vectors.into_iter().map(DVec3::from).sum::<DVec3>()
                        };
                        assert!(
                            (sum(f.corner_angular_sum_nms) - DVec3::from(f.moment_sum_nms))
                                .length()
                                < 1e-12
                        );
                        assert!(
                            (sum(f.corner_vector_sum_ns) - DVec3::from(f.contact_sum_ns)).length()
                                < 1e-12
                        );
                    }
                }
                if receiver == 0 && factor == 4 {
                    let window = &native[WINDOW_STEPS * factor as usize];
                    assert!((window.normal_sum_ns - 3.504508).abs() < 1e-5);
                    assert!((window.lift_work_j - 2.102771).abs() < 1e-5);
                    assert!((window.energy_loss_j - 1.975721).abs() < 1e-5);
                }
                if receiver == 0 && factor == 2 {
                    let hits = secondary(&native);
                    assert_eq!(hits.len(), 1);
                    assert_eq!(hits[0]["time_us"], 725000);
                    assert!(hits[0]["step_lift_work_j"].as_f64().unwrap() > 0.01);
                }
            }
        }
        Ok(())
    }
}
