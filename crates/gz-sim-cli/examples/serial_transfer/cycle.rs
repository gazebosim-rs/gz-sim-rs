// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::receiving_support::SupportReading;
use rigid_tactile::Tactile;

#[derive(Clone, Copy, bevy_ecs::prelude::Resource)]
pub(super) struct Cadence {
    pub(super) control_ms: usize,
    solves: usize,
}
impl Cadence {
    pub(super) const ONE_SOLVE: Self = Self {
        control_ms: 1,
        solves: 1,
    };
    pub(super) fn new(dt: Duration, control_ms: u64) -> Result<Self> {
        let ns = dt.as_nanos();
        let period = control_ms as u128 * 1_000_000;
        if !matches!(control_ms, 1 | 2)
            || !matches!(ns, 500_000 | 1_000_000 | 2_000_000)
            || !period.is_multiple_of(ns)
        {
            return Err(
                "use a 1 or 2 ms control period divisible by a 0.5, 1 or 2 ms SDF physics step"
                    .into(),
            );
        }
        Ok(Self {
            control_ms: control_ms as usize,
            solves: (period / ns) as usize,
        })
    }
}
pub(super) struct Cycle {
    pub(super) cadence: Cadence,
    pub(super) pickup: ReceivingSupport,
    pub(super) receiver: ReceivingSupport,
    pub(super) parcel: Part,
    pub(super) car: Part,
    pub(super) rail: Part,
    pub(super) start_car: f64,
    pub(super) body: BodyId,
    pub(super) pads: [BodyId; 2],
    pub(super) goal: f64,
    pub(super) criteria: SupportCriteria,
    pub(super) trial: Trial,
}
pub(super) struct Reading {
    pub(super) parcel_position: [f64; 3],
    pub(super) carriage_y: f64,
    pub(super) source: SupportReading,
    pub(super) target: SupportReading,
    pub(super) pad: [f64; 2],
    pub(super) tactile: Option<Tactile>,
    pub(super) face_loaded_any: [bool; 2],
    pub(super) stable_face: [bool; 2],
    pub(super) both_pads_loaded: bool,
}
impl Reading {
    // All samples span the same fixed SDF interval. Rails and jaws consume
    // interval means; the support sensor keeps its own per-solve dwell history.
    fn append(&mut self, next: Self, count: usize) {
        let n = count as f64;
        self.parcel_position = next.parcel_position;
        self.carriage_y = next.carriage_y;
        self.both_pads_loaded &= next.both_pads_loaded;
        for (old, new) in self.face_loaded_any.iter_mut().zip(next.face_loaded_any) {
            *old |= new;
        }
        for (old, new) in self.stable_face.iter_mut().zip(next.stable_face) {
            *old &= new;
        }
        for (old, fresh) in [
            (&mut self.source, next.source),
            (&mut self.target, next.target),
        ] {
            let force = old.normal_force_n + (fresh.normal_force_n - old.normal_force_n) / n;
            *old = fresh;
            old.normal_force_n = force;
        }
        for (old, new) in self.pad.iter_mut().zip(next.pad) {
            *old += (new - *old) / n;
        }
        if let (Some(a), Some(b)) = (&mut self.tactile, next.tactile) {
            for (p, q) in a.pads.iter_mut().zip(b.pads) {
                let prior = (n - 1.) * p.normal_force_n;
                let total = prior + q.normal_force_n;
                if total > 0. {
                    p.slip_speed_m_s =
                        (prior * p.slip_speed_m_s + q.normal_force_n * q.slip_speed_m_s) / total;
                }
                p.normal_force_n = total / n;
            }
            for i in 0..3 {
                a.station_ns[i] += b.station_ns[i];
                a.station_up_ns[i] += b.station_up_ns[i];
            }
        }
    }
}
impl Cycle {
    pub(super) fn advance(
        &mut self,
        sim: &mut Simulation,
        tick: usize,
        mut passage: Option<&mut Passage>,
        carry: Option<&carry::Supervisor>,
        mut pull: Option<&mut pull::Measurements>,
    ) -> Result<Reading> {
        let mut result: Option<Reading> = None;
        for i in 1..=self.cadence.solves {
            if self.trial != Trial::CachedObstacle
                && let Some(route) = &mut passage
            {
                route.recheck(sim)?;
            }
            if let Some(pulse) = &mut pull {
                let input_ms = sim.elapsed().as_millis() as usize + 1;
                pulse.before_step(sim, input_ms, self.parcel, self.rail)?;
            }
            sim.step_exact(1)?;
            let parcel_position = self.parcel.pose(sim)?.translation;
            let carriage_y = self.car.pose(sim)?.translation[1];
            let source = self.pickup.sample(sim.world())?;
            let target = self.receiver.sample(sim.world())?;
            let solver = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .ok_or("missing local solver")?;
            let dt = sim.step_duration().as_secs_f64();
            let mut pad = [0.; 2];
            for (n, b) in pad.iter_mut().zip(self.pads) {
                *n = solver
                    .normal_reaction_on(self.body, b)?
                    .ok_or("missing signed impulse")?
                    .magnitude
                    / dt;
            }
            if let Some(route) = &mut passage {
                let travel = carriage_y - self.start_car;
                route.observe(
                    sim,
                    solver,
                    tick,
                    match carry {
                        Some(active) => active.remaining(tick, travel, self.goal),
                        None => (MOVE_START..MOVE_END)
                            .contains(&tick)
                            .then_some((self.goal - travel).max(0.)),
                    },
                    self.trial == Trial::ContactStop,
                    dt,
                )?;
            }
            let tactile = carry
                .filter(|_| tick >= MOVE_START)
                .map(|active| active.read_tactile(sim))
                .transpose()?;
            let stable_face = [source, target].map(|v| {
                v.measurement_available
                    && v.face_loaded
                    && !v.receiver_moving
                    && v.normal_force_n > 0.
                    && v.relative_normal_speed_m_s.abs()
                        <= self.criteria.max_relative_normal_speed_m_s
            });
            let observation = Reading {
                both_pads_loaded: pad.iter().all(|n| *n > 0.1),
                parcel_position,
                carriage_y,
                face_loaded_any: [source.face_loaded, target.face_loaded],
                stable_face,
                source,
                target,
                pad,
                tactile,
            };
            if let Some(prior) = &mut result {
                prior.append(observation, i)
            } else {
                result = Some(observation);
            }
        }
        Ok(result.expect("validated integral physics and control periods"))
    }
}
