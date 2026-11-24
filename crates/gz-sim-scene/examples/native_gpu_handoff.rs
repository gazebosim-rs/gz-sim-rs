// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a real SDF native/GPU round trip against independent native and event solvers.
#[allow(dead_code)]
#[path = "../../gz-physics-gpu/examples/support/sphere_pair/reference.rs"]
mod reference;
#[path = "support/native_pair_drive.rs"]
mod runner;
use glam::DVec3 as V;
use gz_physics_gpu::{EnvironmentState, SpherePairLane as Pair, simulation::BatchTiming};
use gz_sim_scene::{
    gpu_scene::{
        self, GpuSceneObservations,
        PairSolver::{NativeRapier, SpecializedGpu},
    },
    reset_loaded_world,
};
use runner::{Drive, Event, IDS, INPUT};
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn linear(p: &Pair) -> V {
    V::from(p.a.velocity) / p.a.inverse_mass + V::from(p.b.velocity) / p.b.inverse_mass
}
fn angular(p: &Pair) -> V {
    let m = p.a.inverse_mass.recip() + p.b.inverse_mass.recip();
    let center =
        (V::from(p.a.position) / p.a.inverse_mass + V::from(p.b.position) / p.b.inverse_mass) / m;
    [p.a, p.b]
        .into_iter()
        .map(|b| {
            (V::from(b.position) - center).cross(V::from(b.velocity)) / b.inverse_mass
                + V::from(b.angular_velocity) * reference::inertia(&b)
        })
        .sum()
}
fn kinetic(p: &Pair, dv: V) -> f64 {
    [p.a, p.b]
        .iter()
        .map(|b| {
            0.5 * ((V::from(b.velocity) - dv).length_squared() / b.inverse_mass
                + V::from(b.angular_velocity).length_squared() * reference::inertia(b))
        })
        .sum()
}
struct Oracle {
    state: Pair,
    native: Option<reference::Native>,
    action: bool,
}
impl Oracle {
    fn move_to(&mut self, s: gpu_scene::PairSolver) {
        self.native = (s == NativeRapier).then(|| reference::Native::new(self.state));
    }
    fn advance(&mut self) -> (Pair, V) {
        let mut start = self.state;
        if self.action {
            start.a.velocity =
                (V::from(start.a.velocity) + V::from(INPUT) * start.a.inverse_mass).to_array();
        }
        let impulse = if let Some(n) = &mut self.native {
            if self.action {
                n.impulse_a(INPUT);
            }
            n.step();
            self.state = n.state;
            V::from(n.impulse_b.to_array())
        } else {
            self.state = start;
            reference::analytical(&mut self.state);
            V::from(self.state.contact_normal) * self.state.normal_impulse
                + V::from(self.state.tangential_impulse)
        };
        self.action = false;
        (start, impulse)
    }
}
#[derive(Debug, Default)]
struct Max {
    position: f64,
    velocity: f64,
    spin: f64,
    quaternion: f64,
    contact_impulse: f64,
    linear_momentum: f64,
    angular_momentum: f64,
    energy_gain: f64,
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("provide ADAPTER TRACE.csv")?;
    let path = args.next().ok_or("provide new CSV")?;
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    let mut file = BufWriter::new(File::create_new(path)?);
    writeln!(
        file,
        "tick,id,solver_before_barrier,ax,ay,az,bx,by,bz,normal_impulse_Ns,jx,jy,jz,native_bracket_start_s,native_bracket_end_s,retained_points,positive_warmstart_Ns,upload_B,read_B,whole_step_ns"
    )?;
    let mut sim = runner::load(&adapter)?;
    let mut prev = vec![];
    for epoch in 0..2 {
        let mut drive = Drive::new(&sim);
        let mut controls = vec![];
        for e in sim
            .world()
            .resource::<GpuSceneObservations>()
            .environments()
        {
            let EnvironmentState::SpherePair(p) = e.state else {
                unreachable!()
            };
            let mut o = Oracle {
                state: p,
                native: None,
                action: false,
            };
            o.move_to(gpu_scene::pair_solver(&sim, e.id)?);
            controls.push(o);
        }
        let mut peak = Max::default();
        let mut moved = 0;
        let mut refused = 0;
        let mut exact = 0;
        for tick in 0..=700 {
            let t = Instant::now();
            if tick > 0 {
                sim.step_exact(1)?;
            }
            let wall = t.elapsed().as_nanos();
            let states = sim
                .world()
                .resource::<GpuSceneObservations>()
                .environments()
                .to_vec();
            if epoch == 0 {
                prev.push(states.clone());
            } else {
                runner::same_frame_bits(&states, &prev[tick]);
            }
            if tick > 0 {
                let latency = sim.world().resource::<BatchTiming>().last.unwrap();
                for (i, e) in states.iter().enumerate() {
                    let EnvironmentState::SpherePair(p) = e.state else {
                        unreachable!()
                    };
                    let owner = gpu_scene::pair_solver(&sim, e.id)?;
                    let (start, expect_j) = controls[i].advance();
                    let oracle = controls[i].state;
                    for (a, b) in [(p.a, oracle.a), (p.b, oracle.b)] {
                        peak.position = peak
                            .position
                            .max(V::from(a.position).distance(b.position.into()));
                        peak.velocity = peak
                            .velocity
                            .max(V::from(a.velocity).distance(b.velocity.into()));
                        peak.spin = peak
                            .spin
                            .max(V::from(a.angular_velocity).distance(b.angular_velocity.into()));
                        let qerr = a
                            .orientation
                            .iter()
                            .zip(b.orientation)
                            .map(|(x, y)| (x - y).powi(2))
                            .sum::<f64>()
                            .sqrt();
                        peak.quaternion = peak.quaternion.max(qerr);
                    }
                    let native = gpu_scene::native_contact(&sim, e.id)?;
                    let j = V::from(p.contact_normal) * p.normal_impulse
                        + V::from(p.tangential_impulse);
                    peak.contact_impulse = peak.contact_impulse.max(j.distance(expect_j));
                    if let Some(n) = native {
                        assert!(j.distance(n.impulse_on_b.into()) < 2e-14);
                        assert!(p.impact_time < 0.);
                    }
                    let gravity = V::from(p.gravity) * 0.001;
                    let error = (linear(&p)
                        - linear(&start)
                        - gravity * (p.a.inverse_mass.recip() + p.b.inverse_mass.recip()))
                    .length();
                    peak.linear_momentum = peak.linear_momentum.max(error);
                    let dl = angular(&p).distance(angular(&start));
                    peak.angular_momentum = peak.angular_momentum.max(dl);
                    let energy = kinetic(&p, gravity) - kinetic(&start, V::ZERO);
                    peak.energy_gain = peak.energy_gain.max(energy);
                    let relative_speed =
                        V::from(start.a.velocity).distance(start.b.velocity.into());
                    let bound = 2e-10 + j.length() * (relative_speed * 0.001 / 16. + 1e-6) * 4.;
                    assert!(
                        dl <= bound,
                        "angular impulse balance tick={tick} id={} error={dl} bound={bound}",
                        e.id
                    );
                    assert!(
                        error < 2e-10 && energy <= 1e-9,
                        "linear/energy at tick {tick}: {error} / {energy}"
                    );
                    let interval = native
                        .and_then(|n| n.first_impulse_interval)
                        .unwrap_or([-1.; 2]);
                    if epoch == 0 {
                        writeln!(
                            file,
                            "{tick},{},{owner:?},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{wall}",
                            e.id,
                            p.a.position[0],
                            p.a.position[1],
                            p.a.position[2],
                            p.b.position[0],
                            p.b.position[1],
                            p.b.position[2],
                            p.normal_impulse,
                            j.x,
                            j.y,
                            j.z,
                            interval[0],
                            interval[1],
                            native.map_or(0, |n| n.retained_points),
                            native.map_or(0., |n| n.warmstart_normal_impulse),
                            latency.uploaded_bytes,
                            latency.readback_bytes
                        )?;
                    }
                }
            }
            for event in drive.boundary(&mut sim)? {
                let (id, target) = match event {
                    Event::Impulse(id) => {
                        controls[IDS.iter().position(|i| *i == id).unwrap()].action = true;
                        continue;
                    }
                    Event::Moved(id, target) => {
                        moved += 1;
                        (id, target)
                    }
                    Event::ContactRefusal(id) | Event::ClearanceRefusal(id) => {
                        refused += 1;
                        if epoch == 0 {
                            println!("tick={tick} refusal {event:?}");
                        }
                        assert!(IDS.contains(&id));
                        continue;
                    }
                };
                let i = IDS.iter().position(|x| *x == id).unwrap();
                controls[i].move_to(target);
                if epoch == 0 {
                    println!("tick={tick} environment={id} moved={target:?}");
                }
                exact += 1;
            }
        }
        assert!(drive.impact.iter().all(Option::is_some));
        assert_eq!(moved, 8);
        assert!(refused >= 3);
        assert!(
            peak.position < 1e-8
                && peak.velocity < 1e-8
                && peak.spin < 1e-7
                && peak.quaternion < 1e-8
                && peak.contact_impulse < 1e-8,
            "independent native/event controls {peak:?}"
        );
        println!(
            "epoch={epoch} actual impact ticks={:?} exact_barrier_transfers={exact} rejected_active_or_close={refused} maxima={peak:?}",
            drive.impact
        );
        reset_loaded_world(&mut sim, |_| ())?;
        assert_eq!(gpu_scene::pair_solver(&sim, 9)?, NativeRapier);
        assert_eq!(gpu_scene::pair_solver(&sim, 31)?, SpecializedGpu);
    }
    println!(
        "All 701 committed physical frames reproduced bit-for-bit through the loaded SDF reset."
    );
    Ok(())
}
