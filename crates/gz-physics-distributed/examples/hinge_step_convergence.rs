// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the floating two-link native solver with the reduced free-space Lagrangian.
#[path = "support/hinge_convergence_plot.rs"]
mod hinge_convergence_plot;
#[path = "support/hinge_timing.rs"]
mod hinge_timing;
use clap::Parser;
use gz_physics::{BodyId, JointId, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointHandle, Name, PhysicsEngine, load_world_with_engine, queue_joint_effort,
    queue_link_world_wrench,
};
use rapier3d_f64::math::{Rotation, Vector};
use serde::Serialize;
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::{self, File},
    path::{Path, PathBuf},
    time::Duration,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_articulation.sdf");
const CUT: u64 = 120;
const END: u64 = CUT + 256;
const NS_PER_MS: u64 = 1_000_000;

#[derive(Parser)]
struct Args {
    /// Real original 1 ms checkpoint-qualification trajectory.
    #[arg(long)]
    source: PathBuf,
    /// Measure repeated ordinary Bevy advances, with fixture construction and oracle checks outside the timer.
    #[arg(long, default_value_t = 0)]
    timing_rounds: usize,
    /// New directory for common-time errors and the independent solution.
    directory: PathBuf,
}
struct Link {
    native: BodyId,
    public: u64,
    mass: f64,
    inertia: f64,
    com: Vector,
    anchor: Vector,
}
struct Rig {
    links: [Link; 2],
    joint: JointId,
    public_joint: u64,
    zero_rotation: Rotation,
    relative_angle: f64,
    center_at_zero: Vector,
}
fn node<'a>(sim: &'a Simulation, n: &str) -> Result<bevy_ecs::world::EntityRef<'a>> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|a| a.0 == n))
        .ok_or_else(|| format!("unknown SDF entity {n}").into())
}
fn setup(dt: u64) -> Result<(Simulation, Rig)> {
    let sim = load_world_with_engine(
        SDF,
        "articulation.sdf",
        "default",
        Duration::from_nanos(dt),
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let sdf = sdformat::load_str(SDF, "articulation.sdf", &sdformat::NoResources)?;
    let model = &sdf.worlds[0].models[0];
    let hinge = &model.joints[0];
    let hinge_name = format!("{}::{}", model.scoped_name, hinge.name);
    let je = node(&sim, &hinge_name)?;
    let joint = je.get::<JointHandle>().unwrap().id();
    let public_joint = je.get::<EntityId>().unwrap().get();
    let mut bindings = Vec::new();
    for link in &model.links {
        let e = node(&sim, &link.scoped_name)?;
        let anchor = sdf.pose_of(&hinge_name, &link.scoped_name)?.translation;
        let [xx, yy, zz, xy, xz, yz] = link.inertial.inertia;
        if [xy, xz, yz] != [0.; 3] || xx != yy || xx != zz {
            return Err("oracle expects isotropic link inertia".into());
        }
        bindings.push(Link {
            native: e.get::<BodyHandle>().unwrap().id(),
            public: e.get::<EntityId>().unwrap().get(),
            mass: link.inertial.mass,
            inertia: xx,
            com: Vector::from(link.inertial.pose.transform.translation),
            anchor: Vector::from(anchor),
        });
    }
    let links: [Link; 2] = bindings
        .try_into()
        .map_err(|_| "fixture requires 2 links")?;
    let phy = sim.world().resource::<PhysicsBackend>();
    let [b, t] = links.each_ref().map(|l| phy.state(l.native).unwrap());
    let zero_rotation = Rotation::from_array(b.orientation);
    let relative = zero_rotation.inverse() * Rotation::from_array(t.orientation);
    let [x, y, z, w] = relative.to_array();
    let relative_angle = 2. * z.atan2(w);
    let centers = [(&b, &links[0]), (&t, &links[1])].map(|(state, link)| {
        Vector::from(state.position) + Rotation::from_array(state.orientation) * link.com
    });
    let m = links[0].mass + links[1].mass;
    let center_at_zero = (centers[0] * links[0].mass + centers[1] * links[1].mass) / m;
    if x.abs() + y.abs() > 1e-12 || links.iter().any(|l| (l.com - l.anchor).z.abs() > 1e-12) {
        return Err("fixture is not planar around the relative z axis".into());
    }
    Ok((
        sim,
        Rig {
            links,
            joint,
            public_joint,
            zero_rotation,
            relative_angle,
            center_at_zero,
        },
    ))
}
/// Initial orbital forces have equal acceleration on each COM, producing zero torque about the total COM.
/// Eliminating free base yaw conserves zero angular momentum in that translating frame.
#[derive(Clone, Copy)]
struct Relative {
    a: f64,
    b: f64,
    mu: f64,
    u: Vector,
    v: Vector,
    initial_phi: f64,
}
impl Relative {
    fn new(rig: &Rig) -> Self {
        let [a, b] = &rig.links;
        let mu = a.mass * b.mass / (a.mass + b.mass);
        let u = a.com - a.anchor;
        let v = b.com - b.anchor;
        Self {
            a: a.inertia + mu * u.length_squared(),
            b: b.inertia + mu * v.length_squared(),
            mu,
            u,
            v,
            initial_phi: rig.relative_angle,
        }
    }
    fn metric(self, q: f64) -> (f64, f64, f64) {
        let v = Rotation::from_rotation_z(self.initial_phi + q) * self.v;
        let c = -self.mu * self.u.dot(v);
        let dc = -self.mu * self.u.dot(Vector::Z.cross(v));
        let d = self.a + 2. * c + self.b;
        let h = self.b + c;
        let j = self.b - h * h / d;
        let dj = (-2. * h / d + 2. * h * h / (d * d)) * dc;
        (j, dj, h / d)
    }
    fn derivative(self, [q, qd, _]: [f64; 3], torque: f64) -> [f64; 3] {
        let (j, dj, h) = self.metric(q);
        [qd, (torque - 0.5 * dj * qd * qd) / j, -h * qd]
    }
    fn integrate(self, state: [f64; 3], torque: f64, dt: f64) -> [f64; 3] {
        let step = |x: [f64; 3], k: [f64; 3], h: f64| std::array::from_fn(|i| x[i] + h * k[i]);
        let a = self.derivative(state, torque);
        let b = self.derivative(step(state, a, 0.5 * dt), torque);
        let c = self.derivative(step(state, b, 0.5 * dt), torque);
        let d = self.derivative(step(state, c, dt), torque);
        std::array::from_fn(|i| state[i] + dt / 6. * (a[i] + 2. * b[i] + 2. * c[i] + d[i]))
    }
}
fn effort_at_ms(time: u64) -> f64 {
    match time {
        0..43 => 4.,
        120 => -0.8125,
        219 => 0.375,
        _ => 0.,
    }
}
fn oracle_path(rig: &Rig, per_ms: u64) -> Vec<[f64; 3]> {
    let relative = Relative::new(rig);
    let mut x = [0.; 3];
    let mut samples = vec![x];
    for t in 0..END {
        for _ in 0..per_ms {
            x = relative.integrate(x, effort_at_ms(t), 0.001 / per_ms as f64);
        }
        samples.push(x);
    }
    samples
}
struct Expected {
    center: [Vector; 2],
    velocity: [Vector; 2],
    angular_velocity: [Vector; 2],
    linear: Vector,
    angular: Vector,
}
fn expected(r: &Rig, time_ms: u64, [q, dq, yaw]: [f64; 3]) -> Expected {
    let rel = Relative::new(r);
    let (_, _, h) = rel.metric(q);
    let m = r.links[0].mass + r.links[1].mass;
    let t = time_ms as f64 * 0.001;
    let accelerated = t.min(0.001);
    let vx = 250. * accelerated;
    let dx = 125. * accelerated.powi(2) + vx * (t - accelerated);
    let cg = r.center_at_zero + Vector::X * dx;
    let vg = Vector::X * vx;
    let axis = r.zero_rotation * Vector::Z;
    let s = r.zero_rotation * Rotation::from_rotation_z(yaw);
    let rb = s * rel.u;
    let rt = s * (Rotation::from_rotation_z(q + r.relative_angle) * rel.v);
    let d = rt - rb;
    let w = [axis * (-h * dq), axis * ((1. - h) * dq)];
    let dd = w[1].cross(rt) - w[0].cross(rb);
    let center = [
        cg - d * (r.links[1].mass / m),
        cg + d * (r.links[0].mass / m),
    ];
    let velocity = [
        vg - dd * (r.links[1].mass / m),
        vg + dd * (r.links[0].mass / m),
    ];
    let linear = vg * m;
    Expected {
        center,
        velocity,
        angular_velocity: w,
        linear,
        angular: r.center_at_zero.cross(linear),
    }
}
#[derive(Debug, Clone, Copy, Default, Serialize)]
struct Errors {
    q_rad: f64,
    qdot_rad_per_s: f64,
    link_com_m: f64,
    link_com_m_per_s: f64,
    link_rad_per_s: f64,
    total_linear_ns: f64,
    total_angular_world_nms: f64,
    total_angular_center_nms: f64,
    post_cut_linear_drift_ns: f64,
    post_cut_angular_drift_nms: f64,
}
impl Errors {
    fn include(&mut self, b: Self) {
        macro_rules! update{($($field:ident),*)=>{$(self.$field=self.$field.max(b.$field);)*}}
        update!(
            q_rad,
            qdot_rad_per_s,
            link_com_m,
            link_com_m_per_s,
            link_rad_per_s,
            total_linear_ns,
            total_angular_world_nms,
            total_angular_center_nms,
            post_cut_linear_drift_ns,
            post_cut_angular_drift_nms
        );
    }
}
fn read_trace(path: &Path) -> Result<Value> {
    Ok(serde_json::from_reader(File::open(path)?)?)
}
fn same_floats<const N: usize>(stored: &Value, native: [f64; N]) -> bool {
    stored.as_array().is_some_and(|values| {
        values.len() == N
            && values
                .iter()
                .zip(native)
                .all(|(j, n)| j.as_f64().is_some_and(|x| x.to_bits() == n.to_bits()))
    })
}
fn advance_ms(sim: &mut Simulation, rig: &Rig, time: u64, per_ms: u64) -> Result<()> {
    for _ in 0..per_ms {
        if time == 0 {
            for link in &rig.links {
                queue_link_world_wrench(sim, link.public, [250. * link.mass, 0., 0.], [0.; 3])?
            }
        }
        let effort = effort_at_ms(time);
        if effort != 0. {
            queue_joint_effort(sim, rig.public_joint, effort)?
        }
        sim.step_exact(1)?;
    }
    Ok(())
}
fn run(dt: u64, dir: &Path, reference: &Value, oracle: &[[f64; 3]]) -> Result<Value> {
    let (mut sim, rig) = setup(dt)?;
    let per_ms = NS_PER_MS / dt;
    let mut max = Errors::default();
    let mut after = Errors::default();
    let mut cut = None;
    let mut data = Vec::new();
    for time in 0..=END {
        if sim.elapsed().as_nanos() != u128::from(time) * u128::from(NS_PER_MS) {
            return Err("clock drift".into());
        }
        let phy = sim.world().resource::<PhysicsBackend>();
        let hinge = phy.joint_state(rig.joint).ok_or("missing native hinge")?;
        let reported = oracle[time as usize];
        let e = expected(&rig, time, reported);
        let mut p = Vector::ZERO;
        let mut l = Vector::ZERO;
        let mut cg = Vector::ZERO;
        let mut err = Errors {
            q_rad: (hinge.position - reported[0]).abs(),
            qdot_rad_per_s: (hinge.velocity - reported[1]).abs(),
            ..Default::default()
        };
        let source =
            (time >= CUT && dt == NS_PER_MS).then(|| &reference["frames"][(time - CUT) as usize]);
        for (i, link) in rig.links.iter().enumerate() {
            let state = phy.state(link.native).ok_or("missing native link")?;
            let center =
                Vector::from(state.position) + Rotation::from_array(state.orientation) * link.com;
            let velocity = Vector::from(state.velocity);
            let spin = Vector::from(state.angular_velocity);
            err.link_com_m = err.link_com_m.max((center - e.center[i]).length());
            err.link_com_m_per_s = err
                .link_com_m_per_s
                .max((velocity - e.velocity[i]).length());
            err.link_rad_per_s = err
                .link_rad_per_s
                .max((spin - e.angular_velocity[i]).length());
            let lp = velocity * link.mass;
            p += lp;
            l += center.cross(lp) + spin * link.inertia;
            cg += center * link.mass;
            if let Some(f) = source {
                let b = &f["links"][i];
                if !same_floats(&b["position"], state.position)
                    || !same_floats(&b["orientation_xyzw"], state.orientation)
                    || !same_floats(&b["com_velocity"], state.velocity)
                    || !same_floats(&b["angular_velocity"], state.angular_velocity)
                    || f["q"].as_f64().map(f64::to_bits) != Some(hinge.position.to_bits())
                    || f["qdot"].as_f64().map(f64::to_bits) != Some(hinge.velocity.to_bits())
                {
                    return Err(format!("1-ms native observation differs from published checkpoint source at {time} ms").into());
                }
            }
        }
        cg /= rig.links[0].mass + rig.links[1].mass;
        err.total_linear_ns = (p - e.linear).length();
        err.total_angular_world_nms = (l - e.angular).length();
        err.total_angular_center_nms = (l - cg.cross(p)).length();
        if time == CUT {
            cut = Some((p, l))
        }
        if let Some((p0, l0)) = cut {
            err.post_cut_linear_drift_ns = (p - p0).length();
            err.post_cut_angular_drift_nms = (l - l0).length();
            after.include(err);
        }
        max.include(err);
        data.push(json!({"time_ms":time,"native_q_rad":hinge.position,"native_qdot_rad_per_s":hinge.velocity,"oracle_q_rad":reported[0],"oracle_qdot_rad_per_s":reported[1],"error":err}));
        if time == END {
            break;
        }
        advance_ms(&mut sim, &rig, time, per_ms)?;
    }
    let f = File::create(dir.join(format!("{dt}-ns.json")))?;
    serde_json::to_writer(f, &data)?;
    Ok(
        json!({"dt_ns":dt,"native_steps":END*per_ms,"native_endpoint_sha256":hinge_timing::endpoint(&sim,&rig)?,"common_times":data.len(),"max_error":max,"from_cut":after,"endpoint":data.last()}),
    )
}
fn main() -> Result<()> {
    let a = Args::parse();
    let source = read_trace(&a.source)?;
    if source["mode"] != "source" || source["native_source_cut_ns"] != CUT * NS_PER_MS {
        return Err("invalid native reference".into());
    }
    let (_, rig) = setup(NS_PER_MS)?;
    let coarse = oracle_path(&rig, 64);
    let reference = oracle_path(&rig, 128);
    let mut diffs = [0_f64; 3];
    for (a, b) in reference.iter().zip(&coarse) {
        for i in 0..3 {
            diffs[i] = diffs[i].max((a[i] - b[i]).abs())
        }
    }
    if diffs.into_iter().any(|x| x > 2e-10) {
        return Err(format!("independent time-refined RK4 paths disagree: {diffs:?}").into());
    }
    // For zero angular momentum about the COM frame, the internal kinetic energy is J(q) qdot²/2.
    // A constant actuator torque performs the exact work torque * (q_end - q_start).
    let reduced = Relative::new(&rig);
    let energy = |x: [f64; 3]| 0.5 * reduced.metric(x[0]).0 * x[1].powi(2);
    let mut work = 0.;
    let mut max_work_residual = 0_f64;
    for (time, step) in reference.windows(2).enumerate() {
        work += effort_at_ms(time as u64) * (step[1][0] - step[0][0]);
        max_work_residual = max_work_residual.max((energy(step[1]) - work).abs());
    }
    if max_work_residual > 2e-11 {
        return Err(
            format!("independent actuator work conservation failed: {max_work_residual}").into(),
        );
    }
    fs::create_dir(&a.directory)?;
    let mut reports = Vec::new();
    for divisor in [1, 2, 4, 8, 16, 32] {
        let record = run(NS_PER_MS / divisor, &a.directory, &source, &reference)?;
        println!("{}", record);
        reports.push(record);
    }
    for pair in reports.windows(2) {
        for (domain, field) in [
            ("max_error", "q_rad"),
            ("max_error", "qdot_rad_per_s"),
            ("max_error", "link_com_m"),
            ("max_error", "link_com_m_per_s"),
            ("from_cut", "post_cut_linear_drift_ns"),
            ("from_cut", "post_cut_angular_drift_nms"),
        ] {
            let ratio =
                pair[0][domain][field].as_f64().unwrap() / pair[1][domain][field].as_f64().unwrap();
            if !(1.9..=2.1).contains(&ratio) {
                return Err(
                    format!("unexpected first-order refinement ratio {ratio} for {field}").into(),
                );
            }
        }
    }
    if a.timing_rounds > 0 {
        hinge_timing::measure(&a.directory, a.timing_rounds, &reports)?;
    }
    hinge_convergence_plot::draw(&a.directory, &reports)?;
    let report = json!({"sdf":"examples/multifidelity_articulation.sdf","duration_ms":END,"cut_ms":CUT,"oracle_rk4_substeps_per_ms":[64,128],"max_oracle_refinement_difference_q_qdot_base_yaw":diffs,"maximum_oracle_work_energy_residual_j":max_work_residual,"native_1ms_exact_source_frames":257,"input_angular_impulses_nms":[0.172,-0.0008125,0.000375],"total_external_impulse_ns":[0.7,0.,0.],"runs":reports});
    serde_json::to_writer_pretty(File::create(a.directory.join("summary.json"))?, &report)?;
    Ok(())
}
