// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Oblique single-corner impacts of a free ankle with independently computed spatial mobility.
#[path = "registered_corner.rs"]
pub(crate) mod registered;
use super::*;
use gz_physics_multifidelity::{IsotropicFriction, MaterialLinkMotion, PointImpact};
const DECK: DVec3 = DVec3::new(0.23, -0.17, PAD_V);
#[derive(Clone, Copy)]
struct Approach {
    frame: Setup,
    slip: DVec3,
    spin: DVec3,
    hinge: f64,
    mu: f64,
}
impl Approach {
    fn states(self) -> [Body; 2] {
        let f = self.frame;
        let omega = [self.spin, self.spin + self.hinge * f.axis];
        let lever = f.q[0] * vertices()[f.corner];
        let root_v = DECK + self.slip - f.closing * DVec3::Z - omega[0].cross(lever);
        let pivot = root_v + omega[0].cross(f.anchor - f.centers[0]);
        let linear = [root_v, pivot + omega[1].cross(f.centers[1] - f.anchor)];
        std::array::from_fn(|i| Body {
            position: f.centers[i].to_array(),
            orientation: f.q[i].to_array(),
            velocity: linear[i].to_array(),
            angular_velocity: omega[i].to_array(),
        })
    }
    fn matrix(self) -> [[f64; 3]; 3] {
        let cols =
            [DVec3::X, DVec3::Y, DVec3::Z].map(|v| self.frame.response(v, false).point.to_array());
        std::array::from_fn(|i| std::array::from_fn(|j| cols[j][i]))
    }
    fn target(self) -> Result<gz_physics_multifidelity::PointImpactResponse> {
        Ok(PointImpact {
            inverse_mass: self.matrix(),
            relative_velocity: (self.slip - self.frame.closing * DVec3::Z).to_array(),
            friction: self.mu,
        }
        .solve()?)
    }
}
fn run(
    c: Approach,
    calls: u32,
    relax: usize,
    project: bool,
    save: bool,
) -> Result<(Value, Vec<Value>)> {
    let f = c.frame;
    let states = c.states();
    let (mut n, [a, b], joint) = tree(f, states, false, Some(IsotropicFriction::new(1.5)?))?;
    let mut iter = vec![];
    let goal = c.target()?;
    let goal_j = DVec3::from(goal.impulse);
    let dv = f.response(goal_j, false);
    let tx = n.prepare_articulated_contacts(&[ArticulatedContactSite {
        body: a,
        local_point: vertices()[f.corner].to_array(),
    }])?;
    let m = tx
        .sample()
        .mobility(0, 0)
        .ok_or("missing spatial mobility")?;
    let predicted = c.matrix();
    let mobility_error = (0..3)
        .flat_map(|i| (0..3).map(move |j| (m[i][j] - predicted[i][j]).abs()))
        .fold(0., f64::max);
    let rel_before = DVec3::from(tx.sample().velocities[0]) - DECK;
    let incoming_error = rel_before.distance(c.slip - f.closing * DVec3::Z);
    if mobility_error > 1e-10 || incoming_error > 1e-10 {
        return Err(format!(
            "unmatched generalized state or mobility: {mobility_error} {incoming_error}"
        )
        .into());
    }
    let mut applied = DVec3::ZERO;
    let mut exact_work_error = 0.;
    let mut exact_twist_error = 0.0_f64;
    if project {
        let receipt = tx.commit(&[goal.impulse])?;
        applied = goal_j;
        exact_work_error =
            (receipt.kinetic_energy_change - (DECK.dot(applied) - goal.dissipated_energy)).abs();
        let after = DVec3::from(receipt.after_velocities[0]) - DECK;
        exact_twist_error = after.distance(goal.relative_velocity.into());
        for (i, id) in [a, b].into_iter().enumerate() {
            let s = n.state(id).unwrap();
            exact_twist_error = exact_twist_error
                .max(DVec3::from(s.velocity).distance(DVec3::from(states[i].velocity) + dv.v[i]))
                .max(
                    DVec3::from(s.angular_velocity)
                        .distance(DVec3::from(states[i].angular_velocity) + dv.w[i]),
                );
        }
        if exact_work_error > 1e-9 || exact_twist_error > 1e-9 {
            return Err("generalized impulse fails independent twist or work balance".into());
        }
    } else {
        drop(tx);
    }
    let deck = Body {
        position: [0., 0., -PAD_HALF[2]],
        orientation: DQuat::IDENTITY.to_array(),
        velocity: DECK.to_array(),
        angular_velocity: [0.; 3],
    };
    let pad = n.spawn_material_link(
        cube(PAD_HALF, deck, 1.),
        orientation(deck),
        None,
        MaterialLinkMotion::Prescribed,
        IsotropicFriction::new(c.mu)?,
    )?;
    let trace = |tick: u64, sole: Body, top: Body, support: Body, j: f64| {
        let mut corners = [0.; 4];
        corners[f.corner] = j;
        json!({"tick":tick,"time_us":tick*500,"foot":sole,"upper":top,"pad":support,"normal_impulse_ns":j,"corner_ns":corners,"native_steps":if tick==0{0}else{calls},"solver_iterations":4})
    };
    if save {
        iter.push(trace(0, states[0], states[1], deck, 0.));
    }
    let mut cumulative = applied;
    let mut first = None;
    let mut other = 0.;
    let mut unavailable = false;
    let mut clearance = f64::INFINITY;
    let mut balance = 0.0_f64;
    for tick in 1..=if save { 16 } else { 2 } {
        let mut normal = if tick == 1 { applied.z } else { 0. };
        let before = |id| DVec3::from(n.state(id).unwrap().velocity);
        let mut mom = f.mass[0] * before(a) + f.mass[1] * before(b);
        n.observe_contact_interval(DT,calls,ContactSolverIterations{temporal:4,pgs:None,relaxation:Some(relax)},|s|{
            let foot=s.state(a).unwrap();
            let upper=s.state(b).unwrap();
            let support=s.state(pad).unwrap();
            let rot=DQuat::from_array(foot.orientation);
            let pad_z=support.position[2]+PAD_HALF[2];
            for(i,v)in vertices().into_iter().enumerate(){if i!=f.corner{clearance=clearance.min(foot.position[2]+(rot*v).z-pad_z);}}
            let uq=DQuat::from_array(upper.orientation);
            clearance=clearance.min(upper.position[2]-(uq.conjugate()*DVec3::Z).abs().dot(UPPER_HALF.into())-pad_z);
            let mut sample=DVec3::ZERO;
            let mut load=0.;
            for p in s.contact_patches(){
                let forward=p.a==a&&p.b==pad;
                if !forward && !(p.b==a && p.a==pad){other+=p.points.iter().map(|v|v.normal_impulse).sum::<f64>();continue;}
                let sign=if forward{1.}else{-1.};
                for v in p.points{
                    let local=rot.conjugate()*DVec3::from(if forward{v.lever_a}else{v.lever_b});
                    let k=2*usize::from(local.x>0.)+usize::from(local.y>0.);
                    if k!=f.corner{other+=v.normal_impulse;}
                    load+=v.normal_impulse;
                    sample+=sign*(DVec3::from(p.normal_on_a)*v.normal_impulse+DVec3::from(v.tangent_impulse_on_a.unwrap_or_else(||{unavailable=true;[0.;3]})));
                }
            }
            let next=f.mass[0]*DVec3::from(foot.velocity)+f.mass[1]*DVec3::from(upper.velocity);
            balance=balance.max((next-mom).distance(sample));mom=next;
            if load>1e-9 && first.is_none(){
                let slip= DVec3::from(foot.velocity)+DVec3::from(foot.angular_velocity).cross(rot*vertices()[f.corner])-DECK;
                first=Some(json!({"native_end_ns":(tick-1)*500_000+s.elapsed.as_nanos() as u64,"native_world_impulse_ns":sample.to_array(),"impulse_error_ns":(!project).then(||sample.distance(goal_j)),
                    "relative_point_velocity_m_s":slip.to_array(),"slip_error_m_s":slip.distance(goal.relative_velocity.into()),"foot_spin_error_rad_s":DVec3::from(foot.angular_velocity).distance(DVec3::from(states[0].angular_velocity)+dv.w[0])}));
            }
            normal+=load;cumulative+=sample;
        })?;
        if save {
            let state = |id| Body::from(n.state(id).unwrap());
            iter.push(trace(tick, state(a), state(b), state(pad), normal));
        }
    }
    if unavailable || other > 1e-8 || clearance < 1e-3 || (!project && first.is_none()) {
        return Err(format!("single-point qualification failed: unavailable={unavailable} additional_impulse={other} clearance={clearance}").into());
    }
    Ok((
        json!({"calls":calls,"relaxation":relax,"projected":project,"first_native":first,"initial_mobility_error":mobility_error,"initial_point_velocity_error":incoming_error,
        "applied_world_impulse_ns":applied.to_array(),"initial_projection_work_error_j":exact_work_error,"initial_projection_twist_error":exact_twist_error,
        "elapsed_us":if save{8000}else{1000},"total_world_impulse_ns":cumulative.to_array(),"cumulative_error_ns":cumulative.distance(goal_j),
        "max_native_momentum_residual_ns":balance,"minimum_other_clearance_m":clearance,"joint_rate_rad_s":n.joint_state(joint).unwrap().velocity}),
        iter,
    ))
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut data = vec![];
    for mass in [0.4, 1., 2.5] {
        for (axis_i, axis) in [DVec3::X, DVec3::new(0.2, 1., 0.25)]
            .into_iter()
            .enumerate()
        {
            for tilt in [-0.25, 0.31] {
                for closing in [0.6, 1.4] {
                    for dir_i in 0..8 {
                        for (speed_i, ratio) in [0.18, 1.35].into_iter().enumerate() {
                            for mu in [0., 0.2, 0.6, 1.] {
                                let frame = Setup::new(mass, axis, tilt, closing);
                                let angle = (dir_i as f64 + 0.23) * std::f64::consts::FRAC_PI_4;
                                let c = Approach {
                                    frame,
                                    slip: closing
                                        * ratio
                                        * DVec3::new(angle.cos(), angle.sin(), 0.),
                                    spin: DVec3::new(0.35, -0.23, 0.7),
                                    hinge: 0.83,
                                    mu,
                                };
                                let show = mass == 1.
                                    && axis_i == 1
                                    && tilt > 0.
                                    && closing > 1.
                                    && dir_i == 1
                                    && speed_i == 1
                                    && mu == 0.6;
                                let target = c.target()?;
                                let mut paths = vec![];
                                for (calls, r, project, label) in [
                                    (8, 1, false, "ordinary"),
                                    (8, 8, false, "relaxed"),
                                    (64, 8, false, "fine"),
                                    (8, 8, true, "projected"),
                                ] {
                                    let save = show && label != "fine";
                                    let (row, frames) = run(c, calls, r, project, save)?;
                                    paths.push(row);
                                    if save {
                                        write_json(
                                            &out.join("traces").join(format!("{label}.json")),
                                            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":UPPER_HALF,"pad_half_extents":PAD_HALF},"frames":frames}),
                                        )?;
                                    }
                                }
                                data.push(json!({"upper_mass":mass,"axis_index":axis_i,"tilt":tilt,"closing":closing,"slip":c.slip.to_array(),"mu":mu,"direction_index":dir_i,"speed_index":speed_i,"corner":frame.corner,
            "target":{"world_impulse_ns":target.impulse,"relative_after":target.relative_velocity,"sticking":target.sticking,"dissipation_j":target.dissipated_energy},"paths":paths}));
                            }
                        }
                    }
                }
            }
        }
    }
    let lanes = [
        ("ordinary", "Free ankle, one pass"),
        ("relaxed", "Free ankle, eight passes"),
        ("projected", "Spatial impulse then native"),
    ]
    .map(|(id, name)| json!({"name":name,"label":id,"file":format!("traces/{id}.json")}));
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Spatial friction on an unlocked ankle","lanes":lanes,"foot_mass_kg":1.0,"upper_mass_kg":1.0}),
    )?;
    write_json(
        &out.join("study.json"),
        &json!({"format":"free-hinge-spatial-friction-v1","cases":data}),
    )
}
