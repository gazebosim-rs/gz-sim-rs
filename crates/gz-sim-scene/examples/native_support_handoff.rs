// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run the loaded sphere-plane native/GPU handoffs against two independent solvers.
#[path = "support/native_support_drive.rs"]
mod drive;
#[allow(dead_code)]
#[path = "../../gz-physics-gpu/examples/support/sphere_plane/reference.rs"]
mod reference;
use drive::{Drive, END, Event, IDS, KICK, LAUNCH};
use gz_physics_gpu::SpherePlaneLane as Sphere;
use gz_sim_scene::{
    gpu_scene::{
        self, Environment, EnvironmentState, GpuSceneObservations,
        PairSolver::{self, NativeRapier},
    },
    reset_loaded_world,
};
use reference::V;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
struct Control {
    state: Sphere,
    native: Option<reference::Native>,
    kick: bool,
}
impl Control {
    fn owner(&mut self, owner: PairSolver) {
        self.native =
            (owner == NativeRapier).then(|| reference::Native::with_substeps(self.state, 16));
    }
    fn step(&mut self) -> Sphere {
        let mut before = self.state;
        if self.kick {
            before.velocity =
                (V::from(before.velocity) + V::from(KICK) * before.inverse_mass).to_array();
            if let Some(n) = &mut self.native {
                n.impulse_a(KICK);
            }
            self.kick = false;
        }
        if let Some(n) = &mut self.native {
            n.step();
            self.state = n.state;
        } else {
            self.state = before;
            reference::analytical(&mut self.state);
        }
        before
    }
}
fn sphere(e: &Environment) -> Sphere {
    let EnvironmentState::SpherePlane(s) = e.state else {
        unreachable!()
    };
    s
}
fn free_kinetic(s: Sphere, dv: V) -> f64 {
    let m = s.inverse_mass.recip();
    0.5 * m
        * ((V::from(s.velocity) + dv).length_squared()
            + 0.4 * s.radius * s.radius * V::from(s.angular_velocity).length_squared())
}
#[derive(Debug, Default)]
struct Peaks {
    position: f64,
    velocity: f64,
    spin: f64,
    quaternion: f64,
    impulse: f64,
    force_residual: f64,
    torque_residual: f64,
    cone_excess: f64,
    energy_gain: f64,
    weight_error: f64,
    largest_penetration_m: f64,
    largest_impact_resolution_m: f64,
    warm_normal: f64,
    warm_tangent: f64,
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("provide ADAPTER NEW.csv")?;
    let out = args.next().ok_or("provide new trajectory file")?;
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    let mut file = BufWriter::new(File::create_new(out)?);
    writeln!(
        file,
        "tick,id,solver,x,y,z,gap_m,slip_m_s,normal_Ns,tangent_Ns,warm_normal_Ns,warm_tangent_Ns,points,whole_ns"
    )?;
    let mut sim = drive::load(&adapter)?;
    let mut committed: Vec<Vec<Environment>> = vec![];
    for epoch in 0..2 {
        let mut driver = Drive::new(&sim);
        let mut refs = sim
            .world()
            .resource::<GpuSceneObservations>()
            .environments()
            .iter()
            .map(|e| {
                let mut c = Control {
                    state: sphere(e),
                    native: None,
                    kick: false,
                };
                c.owner(gpu_scene::pair_solver(&sim, e.id).unwrap());
                c
            })
            .collect::<Vec<_>>();
        let mut peaks = Peaks::default();
        let mut loaded = [false; 3];
        let mut impact_resolution = [0.; 3];
        let mut supports = [0; 3];
        let mut air = [0; 3];
        let mut moves = 0;
        let mut refusals = 0;
        let mut loaded_native = 0;
        let mut loaded_specialized = 0;
        for tick in 0..=END {
            let started = Instant::now();
            if tick > 0 {
                sim.step_exact(1)?;
            }
            let wall = started.elapsed().as_nanos();
            let frame = sim
                .world()
                .resource::<GpuSceneObservations>()
                .environments()
                .to_vec();
            if epoch == 0 {
                committed.push(frame.clone());
            } else {
                for (a, b) in frame.iter().zip(&committed[tick as usize]) {
                    assert_eq!(a.id, b.id);
                    assert!(
                        a.state.values().iter().map(|v| v.to_bits()).eq(b
                            .state
                            .values()
                            .iter()
                            .map(|v| v.to_bits()))
                    );
                }
            }
            if tick > 0 {
                for (i, e) in frame.iter().enumerate() {
                    let st = sphere(e);
                    let start = refs[i].step();
                    let oracle = refs[i].state;
                    let n = V::from(st.plane_normal);
                    let j = n * st.normal_impulse + V::from(st.tangential_impulse);
                    let expected_j = n * oracle.normal_impulse + V::from(oracle.tangential_impulse);
                    let dv = V::from(st.gravity) * 0.001;
                    let mass = st.inverse_mass.recip();
                    let moment = 0.4 * mass * st.radius * st.radius;
                    let discrepancy = [
                        V::from(st.position).distance(oracle.position.into()),
                        V::from(st.velocity).distance(oracle.velocity.into()),
                        V::from(st.angular_velocity).distance(oracle.angular_velocity.into()),
                        st.orientation
                            .iter()
                            .zip(oracle.orientation)
                            .map(|(a, b)| (a - b).powi(2))
                            .sum::<f64>()
                            .sqrt(),
                        j.distance(expected_j),
                    ];
                    peaks.position = peaks.position.max(discrepancy[0]);
                    peaks.velocity = peaks.velocity.max(discrepancy[1]);
                    peaks.spin = peaks.spin.max(discrepancy[2]);
                    peaks.quaternion = peaks.quaternion.max(discrepancy[3]);
                    peaks.impulse = peaks.impulse.max(discrepancy[4]);
                    assert!(
                        discrepancy
                            .iter()
                            .zip([1e-8, 1e-8, 1e-7, 1e-8, 1e-8])
                            .all(|(x, t)| *x < t),
                        "independent native/event reference epoch {epoch} tick {tick} id {} {discrepancy:?}",
                        e.id
                    );
                    let force =
                        (mass * (V::from(st.velocity) - V::from(start.velocity) - dv) - j).length();
                    let torque = (moment
                        * (V::from(st.angular_velocity) - V::from(start.angular_velocity))
                        - (-st.radius * n).cross(j))
                    .length();
                    let cone =
                        V::from(st.tangential_impulse).length() - st.friction * st.normal_impulse;
                    let energy = free_kinetic(st, V::ZERO) - free_kinetic(start, dv);
                    peaks.force_residual = peaks.force_residual.max(force);
                    peaks.torque_residual = peaks.torque_residual.max(torque);
                    peaks.cone_excess = peaks.cone_excess.max(cone);
                    peaks.energy_gain = peaks.energy_gain.max(energy);
                    // The native friction lever arm can include a discrete substep's penetration.
                    let bound = 2e-9
                        + j.length() * (1e-6 + 0.001 / 16. * V::from(start.velocity).length()) * 4.;
                    assert!(
                        force < 2e-8 && torque <= bound && cone < 2e-8 && energy < 2e-8,
                        "mechanics tick={tick} id={} linear={force} angular={torque}/{bound} cone={cone} kinetic_gain={energy}",
                        e.id
                    );
                    let owner = gpu_scene::pair_solver(&sim, e.id)?;
                    let witness = gpu_scene::native_contact(&sim, e.id)?;
                    let obs = witness.unwrap_or_default();
                    if witness.is_some() {
                        assert!(j.distance(obs.impulse_on_a.into()) < 2e-12);
                        assert!(
                            V::from(obs.impulse_on_a).distance(-V::from(obs.impulse_on_b)) < 1e-15
                        );
                    }
                    let gap = n.dot(V::from(st.position)) - st.radius - st.plane_offset;
                    let surface =
                        V::from(st.velocity) + V::from(st.angular_velocity).cross(-st.radius * n);
                    let slip = (surface - n * n.dot(surface)).length();
                    if st.normal_impulse > 1e-8 && !loaded[i] {
                        // Contact detection is discrete in native Rapier. A falling sphere may travel
                        // up to a full native substep into the halfspace at first detection.
                        impact_resolution[i] = if owner == NativeRapier {
                            n.dot(V::from(start.velocity) + dv).abs() * (0.001 / 16.) + 1e-6 + 1e-7
                        } else {
                            1e-9
                        };
                        peaks.largest_impact_resolution_m =
                            peaks.largest_impact_resolution_m.max(impact_resolution[i]);
                    }
                    loaded[i] = st.normal_impulse > 1e-8;
                    peaks.largest_penetration_m = peaks.largest_penetration_m.max(-gap);
                    if (260..=360).contains(&tick) || (960..=1060).contains(&tick) {
                        let weight = -mass * dv.dot(n);
                        let error = (st.normal_impulse - weight).abs();
                        assert!(
                            gap >= -impact_resolution[i] && gap < 1e-6 && error < 3e-6,
                            "support tick={tick} id={} gap={gap} normal={} expected={weight} impact_bound={}",
                            e.id,
                            st.normal_impulse,
                            impact_resolution[i]
                        );
                        peaks.weight_error = peaks.weight_error.max(error);
                        supports[i] += 1;
                    }
                    if tick > LAUNCH + 45 && tick < LAUNCH + 110 {
                        assert!(gap > 0.075 && j.length() < 2e-9);
                        air[i] += 1;
                    }
                    if tick == 235 {
                        assert!(slip > 0.06 && V::from(st.tangential_impulse).length() > 1e-4);
                        if owner == NativeRapier {
                            assert!(
                                obs.retained_points > 0
                                    && obs.active_solver_contact
                                    && obs.warmstart_normal_impulse > 1e-6
                                    && obs.warmstart_tangent_impulse > 1e-7,
                                "active load without cached normal/tangent response {obs:?}"
                            );
                            loaded_native += 1;
                        } else {
                            loaded_specialized += 1;
                        }
                    }
                    peaks.warm_normal = peaks.warm_normal.max(obs.warmstart_normal_impulse);
                    peaks.warm_tangent = peaks.warm_tangent.max(obs.warmstart_tangent_impulse);
                    if epoch == 0 {
                        writeln!(
                            file,
                            "{tick},{},{owner:?},{},{},{},{gap},{slip},{},{},{},{},{},{wall}",
                            e.id,
                            st.position[0],
                            st.position[1],
                            st.position[2],
                            st.normal_impulse,
                            V::from(st.tangential_impulse).length(),
                            obs.warmstart_normal_impulse,
                            obs.warmstart_tangent_impulse,
                            obs.retained_points
                        )?;
                    }
                }
            }
            for event in driver.boundary(&mut sim)? {
                if epoch == 0 {
                    println!("tick={tick} {event:?}");
                }
                match event {
                    Event::Kick(id) => refs[IDS.iter().position(|v| *v == id).unwrap()].kick = true,
                    Event::Moved(id, owner) => {
                        moves += 1;
                        refs[IDS.iter().position(|v| *v == id).unwrap()].owner(owner);
                    }
                    Event::Refused(id, reason) => {
                        assert!(IDS.contains(&id));
                        assert!(matches!(
                            reason,
                            gpu_scene::NativeHandoffRefusal::ContactHistory
                                | gpu_scene::NativeHandoffRefusal::SweptClearance
                        ));
                        refusals += 1;
                    }
                }
            }
        }
        assert_eq!(moves, 6);
        assert!(refusals >= 9);
        assert_eq!((loaded_native, loaded_specialized), (2, 1));
        assert_eq!(supports, [202; 3]);
        assert_eq!(air, [64; 3]);
        println!(
            "epoch={epoch} exact_transfers={moves} physical_refusals={refusals} supported_ticks={supports:?} separated_flight_ticks={air:?} independent_mechanics={peaks:?}"
        );
        reset_loaded_world(&mut sim, |_| ())?;
    }
    println!("Exact f64 bits for all 1101 committed frames in both complete SDF epochs.");
    Ok(())
}
