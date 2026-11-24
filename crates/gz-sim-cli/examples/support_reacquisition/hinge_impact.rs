// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent instantaneous plastic impact of an unlocked floating two-link assembly.
//! All inertias are about the individual link COMs in the world frame.
#[path = "hinge_friction.rs"]
pub(crate) mod friction;
use super::*;
use glam::DMat3;
use gz_physics::JointId;
use gz_physics_multifidelity::{ArticulatedContactSite, ContactSolverIterations};

const DT: Duration = Duration::from_micros(500);
const PAD_V: f64 = 0.3;
const TICKS: u64 = 16;

#[derive(Clone, Copy, Debug)]
struct Setup {
    centers: [DVec3; 2],
    q: [DQuat; 2],
    mass: [f64; 2],
    tensors: [DVec3; 2],
    anchor: DVec3,
    axis: DVec3,
    velocity: DVec3,
    corner: usize,
    closing: f64,
}

fn vertices() -> [DVec3; 4] {
    let [x, y, z] = FOOT_HALF;
    [
        DVec3::new(-x, -y, -z),
        DVec3::new(-x, y, -z),
        DVec3::new(x, -y, -z),
        DVec3::new(x, y, -z),
    ]
}
impl Setup {
    fn new(mass: f64, axis: DVec3, tilt: f64, closing: f64) -> Self {
        let q = DQuat::from_euler(glam::EulerRot::ZYX, 0.15, tilt, -0.18);
        let v = vertices().map(|point| q * point);
        let corner = (0..4).min_by(|&a, &b| v[a].z.total_cmp(&v[b].z)).unwrap();
        let foot = DVec3::new(0., 0., -v[corner].z);
        Self {
            centers: [foot, foot + q * DVec3::new(0.08, 0.015, 0.17)],
            q: [q, q * DQuat::from_rotation_y(0.28)],
            mass: [1., mass],
            tensors: [
                DVec3::new(0.017, 0.021, 0.026),
                mass * DVec3::new(0.006, 0.011, 0.012),
            ],
            anchor: foot + q * DVec3::new(-0.02, 0.025, 0.075),
            axis: q * axis.normalize(),
            velocity: DVec3::new(0., 0., PAD_V - closing),
            corner,
            closing,
        }
    }
    fn com(self) -> DVec3 {
        (self.mass[0] * self.centers[0] + self.mass[1] * self.centers[1])
            / (self.mass[0] + self.mass[1])
    }
    fn response(self, impulse: DVec3, locked: bool) -> Mobility {
        let m = self.mass[0] + self.mass[1];
        let r = self.centers.map(|c| c - self.com());
        let g = self.axis.cross(self.centers[1] - self.anchor);
        // Linear velocity induced by a unit hinge speed with total COM fixed.
        let s = [-self.mass[1] / m * g, self.mass[0] / m * g];
        let inertias = std::array::from_fn::<_, 2, _>(|i| {
            let q = DMat3::from_quat(self.q[i]);
            q * DMat3::from_diagonal(self.tensors[i]) * q.transpose()
        });
        let mut composite = inertias[0] + inertias[1];
        let mut cross = inertias[1] * self.axis;
        let mut hinge_mass = self.axis.dot(cross);
        for i in 0..2 {
            let v = r[i];
            let square = DMat3::from_cols(v * v.x, v * v.y, v * v.z);
            composite += self.mass[i] * (v.length_squared() * DMat3::IDENTITY - square);
            cross += self.mass[i] * v.cross(s[i]);
            hinge_mass += self.mass[i] * s[i].length_squared();
        }
        let lever = self.q[0] * vertices()[self.corner];
        let torque = (r[0] + lever).cross(impulse);
        let inverse = composite.inverse();
        let dq = if locked {
            0.
        } else {
            (s[0].dot(impulse) - cross.dot(inverse * torque))
                / (hinge_mass - cross.dot(inverse * cross))
        };
        let w = inverse * (torque - cross * dq);
        let lin = std::array::from_fn::<_, 2, _>(|i| impulse / m + w.cross(r[i]) + s[i] * dq);
        Mobility {
            point: lin[0] + w.cross(lever),
            qdot: dq,
            v: lin,
            w: [w, w + self.axis * dq],
        }
    }
    fn impulse(self, locked: bool) -> Impact {
        let m = self.response(DVec3::Z, locked);
        let k = m.point.z;
        let j = self.closing / k;
        Impact {
            k,
            j,
            qdot: j * m.qdot,
            v: m.v.map(|u| self.velocity + j * u),
            w: m.w.map(|w| j * w),
        }
    }
}
#[derive(Clone, Copy)]
struct Mobility {
    point: DVec3,
    qdot: f64,
    v: [DVec3; 2],
    w: [DVec3; 2],
}
#[derive(Clone, Copy)]
struct Impact {
    k: f64,
    j: f64,
    qdot: f64,
    v: [DVec3; 2],
    w: [DVec3; 2],
}
impl Impact {
    fn json(self) -> Value {
        json!({"effective_inverse_mass_kg_inv":self.k,"impulse_ns":self.j,"joint_rate_rad_s":self.qdot,
            "link_velocity_m_s":self.v.map(|v|v.to_array()),"link_spin_rad_s":self.w.map(|v|v.to_array())})
    }
}
#[derive(Clone, Copy, PartialEq)]
enum Treatment {
    Locked,
    Unlocked,
    Projected,
}
struct Rig {
    native: MultiFidelityBackend,
    ids: [BodyId; 3],
    joint: JointId,
    impulse: f64,
    initial: Value,
    mobility: Option<f64>,
}
fn tree(
    f: Setup,
    states: [Body; 2],
    locked: bool,
    material: Option<gz_physics_multifidelity::IsotropicFriction>,
) -> Result<(MultiFidelityBackend, [BodyId; 2], JointId)> {
    let mut n = MultiFidelityBackend::new([0.; 3])?;
    let (ids, joint) = add_tree(&mut n, f, states, locked, material)?;
    Ok((n, ids, joint))
}
fn add_tree(
    n: &mut MultiFidelityBackend,
    f: Setup,
    states: [Body; 2],
    locked: bool,
    material: Option<gz_physics_multifidelity::IsotropicFriction>,
) -> Result<([BodyId; 2], JointId)> {
    let mut links = Vec::new();
    for (i, b) in states.into_iter().enumerate() {
        let shape = cube(if i == 0 { FOOT_HALF } else { UPPER_HALF }, b, f.mass[i]);
        let props = inertia([0.; 3], f.tensors[i].to_array());
        let id = if let Some(mu) = material {
            n.spawn_material_link(
                shape,
                orientation(b),
                props,
                gz_physics_multifidelity::MaterialLinkMotion::Articulation,
                mu,
            )?
        } else {
            n.spawn_articulation_link(shape, orientation(b), props)?
        };
        n.set_self_collision_group(id, Some(2))?;
        links.push(id);
    }
    let a = links[0];
    let b = links[1];
    let joint = if locked {
        let j =
            n.add_limited_revolute(Some(a), b, f.anchor.to_array(), f.axis.to_array(), [0.; 2])?;
        n.enable_exact_revolute_locks(a)?;
        j
    } else {
        n.add_reduced_revolute(Some(a), b, f.anchor.to_array(), f.axis.to_array())?
    };
    Ok(([a, b], joint))
}
fn native(f: Setup, method: Treatment) -> Result<Rig> {
    let states = std::array::from_fn(|i| Body {
        position: f.centers[i].to_array(),
        orientation: f.q[i].to_array(),
        velocity: f.velocity.to_array(),
        angular_velocity: [0.; 3],
    });
    let (mut n, [a, b], joint) = tree(f, states, method == Treatment::Locked, None)?;
    let pad_body = Body {
        position: [0., 0., -PAD_HALF[2]],
        orientation: DQuat::IDENTITY.to_array(),
        velocity: [0., 0., PAD_V],
        angular_velocity: [0.; 3],
    };
    let initial = json!({"tick":0,"time_us":0,"foot":Body::from(n.state(a).unwrap()),"upper":Body::from(n.state(b).unwrap()),
        "pad":pad_body,"normal_impulse_ns":0.,"corner_ns":([0.;4]),"native_steps":0,"solver_iterations":0});
    let target = f.impulse(false);
    let mut mobility = None;
    let mut impulse = 0.;
    if method != Treatment::Locked {
        let tx = n.prepare_articulated_contacts(&[ArticulatedContactSite {
            body: a,
            local_point: vertices()[f.corner].to_array(),
        }])?;
        let k = tx
            .sample()
            .mobility(0, 0)
            .ok_or("missing contact mobility")?[2][2];
        mobility = Some(k);
        if (target.k - k).abs() > 1e-10 || tx.sample().dofs != 7 {
            return Err(format!(
                "native/independent free hinge mobility disagreement: {k} vs {}",
                target.k
            )
            .into());
        }
        if method == Treatment::Projected {
            impulse = target.j;
            let report = tx.commit(&[[0., 0., target.j]])?;
            if (report.after_velocities[0][2] - PAD_V).abs() > 1e-10 {
                return Err("projected contact velocity is not plastic".into());
            }
            for (i, id) in [a, b].into_iter().enumerate() {
                let state = n.state(id).ok_or("missing projected link")?;
                if DVec3::from(state.velocity).distance(target.v[i]) > 1e-10
                    || DVec3::from(state.angular_velocity).distance(target.w[i]) > 1e-10
                {
                    return Err("independent link impulse responses do not match generalized impulse commit".into());
                }
            }
        }
    }
    // The free-tree transaction models the isolated instantaneous contact.
    // Then add the same finite pad, so subsequent load is always native.
    let pad = n.spawn_ideal_landing_pad(cube(PAD_HALF, pad_body, 1.))?;
    Ok(Rig {
        native: n,
        ids: [a, b, pad],
        joint,
        impulse,
        initial,
        mobility,
    })
}
fn trial(
    f: Setup,
    method: Treatment,
    calls: u32,
    relaxation: usize,
) -> Result<(Value, Vec<Value>)> {
    let Rig {
        mut native,
        ids,
        joint,
        impulse,
        initial,
        mobility,
    } = native(f, method)?;
    let mut frames = vec![initial];
    let mut first = None;
    let mut total = impulse;
    let mut other = 0.;
    let mut min_clearance = f64::INFINITY;
    let target = f.impulse(method == Treatment::Locked);
    for tick in 1..=TICKS {
        let mut corners = [0.; 4];
        if method == Treatment::Projected && tick == 1 {
            corners[f.corner] = impulse;
        }
        native.observe_contact_interval(DT,calls,ContactSolverIterations{temporal:4,pgs:None,relaxation:Some(relaxation)},|s|{
            let foot=s.state(ids[0]).unwrap();
            let upper=s.state(ids[1]).unwrap();
            let q=DQuat::from_array(foot.orientation);
            let pad=s.state(ids[2]).unwrap();
            let pad_top=pad.position[2]+PAD_HALF[2];
            for (i,v) in vertices().into_iter().enumerate(){
                if i!=f.corner{min_clearance=min_clearance.min(foot.position[2]+(q*v).z-pad_top);}
            }
            let uq=DQuat::from_array(upper.orientation);
            min_clearance=min_clearance.min(upper.position[2]-(uq.conjugate()*DVec3::Z).abs().dot(UPPER_HALF.into())-pad_top);
            let mut j=0.;
            for p in s.contact_patches(){
                let foot_first=p.a==ids[0] && p.b==ids[2];
                if !foot_first && !(p.b==ids[0] && p.a==ids[2]){
                    other+=p.points.iter().map(|v|v.normal_impulse).sum::<f64>();
                    continue;
                }
                for v in p.points{
                    let local=q.conjugate()*DVec3::from(if foot_first{v.lever_a}else{v.lever_b});
                    let i=2*usize::from(local.x>0.)+usize::from(local.y>0.);
                    corners[i]+=v.normal_impulse;
                    j+=v.normal_impulse;
                    if i!=f.corner{other+=v.normal_impulse;}
                }
            }
            if j>1e-9 && first.is_none(){
                let qdot=(DVec3::from(upper.angular_velocity)-DVec3::from(foot.angular_velocity)).dot(f.axis);
                first=Some(json!({"native_end_ns":(tick-1)*500_000+s.elapsed.as_nanos() as u64,"impulse_ns":j,"joint_rate_rad_s":qdot,
                    "joint_rate_error_rad_s":qdot-target.qdot,"link_spin_error_rad_s":[DVec3::from(foot.angular_velocity).distance(target.w[0]),DVec3::from(upper.angular_velocity).distance(target.w[1])]}));
            }
            total+=j;
        })?;
        let body = |id| Body::from(native.state(id).expect("registered link"));
        frames.push(json!({"tick":tick,"time_us":tick*500,"foot":body(ids[0]),"upper":body(ids[1]),"pad":body(ids[2]),
            "normal_impulse_ns":corners.iter().sum::<f64>(),"corner_ns":corners,"native_steps":calls,"solver_iterations":4}));
    }
    if other > 1e-10 || min_clearance < 0.002 {
        return Err(format!(
            "additional contact in isolated hinge control: {other}; {min_clearance}"
        )
        .into());
    }
    Ok((
        json!({"method":match method{Treatment::Locked=>"locked",Treatment::Unlocked=>"unlocked",Treatment::Projected=>"projected"},"calls":calls,"relaxation":relaxation,"native_mobility":mobility,
        "projected_impulse_ns":impulse,"first_native_load":first,"total_ns":total,"final_hinge_rate_rad_s":native.joint_state(joint).unwrap().velocity,
        "other_impulse_ns":other,"other_clearance_m":min_clearance}),
        frames,
    ))
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let axes = [DVec3::X, DVec3::new(0.2, 1., 0.25)];
    let mut studies = vec![];
    for mass in [0.4, 1., 2.5] {
        for (axis_index, &axis) in axes.iter().enumerate() {
            for tilt in [-0.25, 0.31] {
                for closing in [0.6, 1.4] {
                    let f = Setup::new(mass, axis, tilt, closing);
                    let mut data = vec![];
                    let show = mass == 1. && axis_index == 1 && tilt > 0. && closing > 1.;
                    for (method, name) in [
                        (Treatment::Locked, "locked"),
                        (Treatment::Unlocked, "unlocked"),
                        (Treatment::Projected, "projected"),
                    ] {
                        for (calls, passes) in [(1, 1), (8, 1), (8, 8), (64, 8)] {
                            let (row, frames) = trial(f, method, calls, passes)?;
                            data.push(row);
                            if show && (calls, passes) == (8, 8) {
                                write_json(
                                    &out.join("traces").join(format!("{name}.json")),
                                    &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":UPPER_HALF,"pad_half_extents":PAD_HALF},"frames":frames}),
                                )?;
                            }
                        }
                    }
                    studies.push(json!({"upper_mass_kg":mass,"axis_world":f.axis.to_array(),"axis_index":axis_index,"tilt_rad":tilt,"closing_m_s":closing,"corner":f.corner,
            "unlocked":f.impulse(false).json(),"locked":f.impulse(true).json(),"trials":data}));
                }
            }
        }
    }
    let lanes = [
        ("locked", "Locked ankle"),
        ("unlocked", "Free ankle"),
        ("projected", "Exact impulse, then free ankle"),
    ]
    .map(|(id, name)| json!({"name":name,"label":id,"file":format!("traces/{id}.json")}));
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Landing with a freely rotating ankle","lanes":lanes}),
    )?;
    write_json(
        &out.join("study.json"),
        &json!({"duration_us":TICKS*500,"cases":studies}),
    )
}
