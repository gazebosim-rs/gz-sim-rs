// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native point impulses and change in two-link momentum through an asymmetric primary landing.
use super::*;
use gz_physics::DynamicBackend;

#[derive(Clone, Copy, Serialize)]
pub(crate) struct Bodies {
    foot: Body,
    upper: Body,
    pad: Body,
}
impl Bodies {
    pub(crate) fn from_states([foot, upper, pad]: [BodyState;3]) -> Self {
        Self { foot:foot.into(), upper:upper.into(), pad:pad.into() }
    }

    pub(crate) fn com(self) -> DVec3 {
        (DVec3::from(self.foot.position) + DVec3::from(self.upper.position)) * 0.5
    }
    pub(crate) fn momentum(self) -> DVec3 {
        DVec3::from(self.foot.velocity) + DVec3::from(self.upper.velocity)
    }
    pub(crate) fn angular_momentum(self) -> DVec3 {
        let com = self.com();
        let v = self.momentum() * 0.5;
        [self.foot, self.upper]
            .into_iter()
            .map(|b| {
                let q = DQuat::from_array(b.orientation).normalize();
                let spin_body = q.conjugate() * DVec3::from(b.angular_velocity);
                let spin_momentum = q * (DVec3::new(0.0201, 0.0201, 0.025) * spin_body);
                spin_momentum + (DVec3::from(b.position) - com).cross(DVec3::from(b.velocity) - v)
            })
            .sum()
    }
}

#[derive(Serialize)]
struct Point {
    corner: usize,
    world_m: [f64; 3],
    foot_local_m: [f64; 3],
    pad_local_m: [f64; 3],
    foot_solver_arm_world_m: [f64; 3],
    normal_on_foot: [f64; 3],
    normal_ns: f64,
    normal_impulse_ns: [f64; 3],
    tangent_impulse_ns: [f64; 3],
    angular_impulse_nms: [f64; 3],
}
#[derive(Serialize)]
struct Sample {
    tick: u64,
    substep: u32,
    time_ns: u64,
    dt_ns: u64,
    before: Bodies,
    after: Bodies,
    points: Vec<Point>,
    normal_ns: f64,
    corner_ns: [f64; 4],
    normal_impulse_ns: [f64; 3],
    tangent_impulse_ns: [f64; 3],
    angular_impulse_nms: [f64; 3],
    linear_residual_ns: [f64; 3],
    angular_residual_nms: [f64; 3],
}
pub(crate) struct Recorder {
    foot: BodyId,
    upper: BodyId,
    pad: BodyId,
    time_ns: u64,
    samples: Vec<Sample>,
}
impl Recorder {
    pub(crate) fn bodies(
        &self,
        solver: &MultiFidelityBackend,
    ) -> std::result::Result<Bodies, PhysicsError> {
        let read = |id| {
            solver
                .state(id)
                .map(Body::from)
                .ok_or(PhysicsError::MissingBody)
        };
        Ok(Bodies {
            foot: read(self.foot)?,
            upper: read(self.upper)?,
            pad: read(self.pad)?,
        })
    }
    pub(crate) fn observe(
        &mut self,
        solver: &MultiFidelityBackend,
        before: Bodies,
        tick: u64,
        substep: u32,
        dt: Duration,
    ) -> std::result::Result<(), PhysicsError> {
        let after = self.bodies(solver)?;
        let dt_ns = u64::try_from(dt.as_nanos()).map_err(|_| PhysicsError::InvalidValue)?;
        self.time_ns = self
            .time_ns
            .checked_add(dt_ns)
            .ok_or(PhysicsError::InvalidValue)?;
        let mut sample = Sample {
            tick,
            substep,
            time_ns: self.time_ns,
            dt_ns,
            before,
            after,
            points: vec![],
            normal_ns: 0.,
            corner_ns: [0.; 4],
            normal_impulse_ns: [0.; 3],
            tangent_impulse_ns: [0.; 3],
            angular_impulse_nms: [0.; 3],
            linear_residual_ns: [0.; 3],
            angular_residual_nms: [0.; 3],
        };
        let mut normal_j = DVec3::ZERO;
        let mut tangent_j = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        let com = before.com();
        let foot_pos = DVec3::from(before.foot.position);
        let foot_inverse = DQuat::from_array(before.foot.orientation)
            .normalize()
            .conjugate();
        let pad_inverse = DQuat::from_array(before.pad.orientation)
            .normalize()
            .conjugate();
        let quadrant_inverse = DQuat::from_array(after.foot.orientation).conjugate();
        for patch in solver.rigid_contact_patches() {
            let first = (patch.a, patch.b) == (self.foot, self.pad);
            if !first && (patch.b, patch.a) != (self.foot, self.pad) {
                continue;
            }
            let sign = if first { 1. } else { -1. };
            let normal = DVec3::from(patch.normal_on_a) * sign;
            for p in patch.points {
                let tangent =
                    DVec3::from(p.tangent_impulse_on_a.ok_or(PhysicsError::Unsupported {
                        engine: "native landing audit",
                        capability: "per-point tangential impulse",
                    })?) * sign;
                let jn = normal * p.normal_impulse;
                let arm = DVec3::from(if first { p.lever_a } else { p.lever_b });
                // The native Jacobian uses a frozen world-space lever arm. Both body-frame
                // positions below express its reconstructed application point at solve entry.
                let world = foot_pos + arm;
                let local = foot_inverse * arm;
                // Match the existing committed (post-step) quadrant convention exactly.
                let quadrant = quadrant_inverse * arm;
                let corner = 2 * usize::from(quadrant.x > 0.) + usize::from(quadrant.y > 0.);
                let angular = (world - com).cross(jn + tangent);
                normal_j += jn;
                tangent_j += tangent;
                moment += angular;
                sample.normal_ns += p.normal_impulse;
                sample.corner_ns[corner] += p.normal_impulse;
                sample.points.push(Point {
                    corner,
                    world_m: world.to_array(),
                    foot_local_m: local.to_array(),
                    pad_local_m: (pad_inverse * (world - DVec3::from(before.pad.position)))
                        .to_array(),
                    foot_solver_arm_world_m: arm.to_array(),
                    normal_on_foot: normal.to_array(),
                    normal_ns: p.normal_impulse,
                    normal_impulse_ns: jn.to_array(),
                    tangent_impulse_ns: tangent.to_array(),
                    angular_impulse_nms: angular.to_array(),
                });
            }
        }
        sample.normal_impulse_ns = normal_j.to_array();
        sample.tangent_impulse_ns = tangent_j.to_array();
        sample.angular_impulse_nms = moment.to_array();
        let gravity = DVec3::Z * (-2. * 9.81 * dt.as_secs_f64());
        sample.linear_residual_ns =
            (after.momentum() - before.momentum() - gravity - normal_j - tangent_j).to_array();
        sample.angular_residual_nms =
            (after.angular_momentum() - before.angular_momentum() - moment).to_array();
        self.samples.push(sample);
        Ok(())
    }
}

#[derive(Serialize)]
struct Grid {
    substeps: u32,
    first_positive_ns: u64,
    first_positive_outer_tick: u64,
    first_positive_after_unloaded_native_step_ns: Option<u64>,
    first_after_unloaded_outer_tick: Option<u64>,
    first_after_unloaded_ns: Option<u64>,
    onset_normal_ns: f64,
    onset_corner_ns: [f64; 4],
    onset_tangent_impulse_ns: [f64; 3],
    onset_angular_impulse_nms: [f64; 3],
    onset_foot_spin_rad_s: [f64; 3],
    max_linear_residual_ns: f64,
    max_angular_residual_nms: f64,
    samples: Vec<Sample>,
}
fn grid(f: Fixture, cut: &ReboundFrame, end: u64, n: u32) -> Result<(Grid, Vec<ReboundFrame>)> {
    let mut rig = restart(f, ADAPTIVE, cut)?;
    rig.set_mode(Mode::Uniform(Effort::Split(n)));
    let lane = &mut rig.case.lane;
    let recorder = Recorder {
        foot: lane.foot,
        upper: lane.upper,
        pad: lane.pad,
        time_ns: cut.time_us * 1000,
        samples: vec![],
    };
    lane.sim.world_mut().resource_mut::<Step>().point_recorder = Some(Box::new(recorder));
    let mut frames = vec![rig.frame()?];
    while rig.tick < end {
        rig.advance_at(command(f, rig.tick))?;
        frames.push(rig.frame()?);
    }
    let samples = rig
        .case
        .lane
        .sim
        .world_mut()
        .resource_mut::<Step>()
        .point_recorder
        .take()
        .unwrap()
        .samples;
    if samples.len() != (end - cut.tick) as usize * n as usize {
        return Err("native recorder missed a substep".into());
    }
    for (outer, steps) in frames[1..].iter().zip(samples.chunks_exact(n as usize)) {
        let normal: f64 = steps.iter().map(|s| s.normal_ns).sum();
        let corners =
            std::array::from_fn::<_, 4, _>(|k| steps.iter().map(|s| s.corner_ns[k]).sum::<f64>());
        let tangent: DVec3 = steps
            .iter()
            .map(|s| DVec3::from(s.tangent_impulse_ns))
            .sum();
        let last = steps.last().unwrap();
        if (normal - outer.normal_impulse_ns).abs() > 1e-12
            || corners
                .iter()
                .zip(outer.corner_ns)
                .any(|(a, b)| (a - b).abs() > 1e-12)
            || outer
                .tangent_on_foot_ns
                .is_none_or(|x| tangent.distance(DVec3::from(x)) > 1e-12)
            || last.after.foot != outer.foot
            || last.after.upper != outer.upper
            || last.after.pad != outer.pad
            || last.time_ns != outer.time_us * 1000
        {
            return Err(format!(
                "{} native-to-outer mismatch tick {} on grid {n}",
                f.name, outer.tick
            )
            .into());
        }
    }
    let first = samples
        .iter()
        .find(|s| s.normal_ns > 0.)
        .ok_or("no loaded native step")?;
    let first_positive_ns = first.time_ns;
    let native_reload = samples
        .windows(2)
        .find(|pair| {
            pair[0].time_ns > first_positive_ns && pair[0].normal_ns == 0. && pair[1].normal_ns > 0.
        })
        .map(|pair| pair[1].time_ns);
    let tick = first.tick;
    let onset = frames.iter().find(|f| f.tick == tick).unwrap();
    let later = frames
        .windows(2)
        .find(|pair| {
            pair[1].tick > tick && pair[0].normal_impulse_ns == 0. && pair[1].normal_impulse_ns > 0.
        })
        .map(|pair| pair[1].tick);
    let next = samples
        .iter()
        .find(|s| Some(s.tick) == later && s.normal_ns > 0.)
        .map(|s| s.time_ns);
    let onset_sum = |select: fn(&Sample) -> [f64; 3]| {
        samples
            .iter()
            .filter(|s| s.tick == tick)
            .map(|s| DVec3::from(select(s)))
            .sum::<DVec3>()
            .to_array()
    };
    let peak = |select: fn(&Sample) -> [f64; 3]| {
        samples
            .iter()
            .map(|s| DVec3::from(select(s)).length())
            .fold(0., f64::max)
    };
    Ok((
        Grid {
            substeps: n,
            first_positive_ns,
            first_positive_outer_tick: tick,
            first_positive_after_unloaded_native_step_ns: native_reload,
            first_after_unloaded_outer_tick: later,
            first_after_unloaded_ns: next,
            onset_normal_ns: onset.normal_impulse_ns,
            onset_corner_ns: onset.corner_ns,
            onset_tangent_impulse_ns: onset_sum(|s| s.tangent_impulse_ns),
            onset_angular_impulse_nms: onset_sum(|s| s.angular_impulse_nms),
            onset_foot_spin_rad_s: onset.foot.angular_velocity,
            max_linear_residual_ns: peak(|s| s.linear_residual_ns),
            max_angular_residual_nms: peak(|s| s.angular_residual_nms),
            samples,
        },
        frames,
    ))
}

#[derive(Serialize)]
struct Case {
    fixture: Fixture,
    cut_tick: u64,
    end_tick: u64,
    grids: Vec<Grid>,
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut cases = vec![];
    for f in CASES.into_iter().filter(|f| f.name.starts_with("outer-")) {
        let (a, _) = run(f, ADAPTIVE)?;
        let (fine, _) = run(f, FINE)?;
        let major = first_above(&a, &fine, 0.001).ok_or("missing 1 mm crossing")?;
        let mut cut = None;
        for stroke in 0..sweep::DEPART_US.len() {
            let onset = primary(&a, stroke)?.min(primary(&fine, stroke)?);
            if onset > major {
                break;
            }
            cut = Some(onset - 1);
        }
        let cut = a[cut.ok_or("no preceding primary")?];
        let mut grids = vec![];
        for (n, name) in [(4, "four"), (8, "eight"), (16, "sixteen")] {
            let (g, frames) = grid(f, &cut, major as u64, n)?;
            if f.name == "outer-fast" {
                trace(out, name, &frames)?;
            }
            grids.push(g);
        }
        cases.push(Case {
            fixture: f,
            cut_tick: cut.tick,
            end_tick: major as u64,
            grids,
        })
    }
    #[derive(Serialize)]
    struct Study {
        actuator_tick_ns: u64,
        cases: Vec<Case>,
    }
    write_json(
        &out.join("native.json"),
        &Study {
            actuator_tick_ns: STEP_US * 1000,
            cases,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn point_receipts_reconstruct_each_impulse_and_the_recorded_clock() -> Result<()> {
        let f = CASES
            .iter()
            .copied()
            .find(|f| f.name == "outer-fast")
            .unwrap();
        let mut r = start(f, ADAPTIVE)?;
        while r.tick < 938 {
            r.advance_at(command(f, r.tick))?;
        }
        let cut = r.frame()?;
        let (eight, frames8) = grid(f, &cut, 940, 8)?;
        let (sixteen, frames16) = grid(f, &cut, 940, 16)?;
        assert_eq!(eight.first_positive_ns, sixteen.first_positive_ns);
        assert!(frames8[1].normal_impulse_ns > 0. && frames16[1].normal_impulse_ns > 0.);
        for (g, rows) in [(eight, frames8), (sixteen, frames16)] {
            let mut prior = 0;
            for sample in &g.samples {
                if prior > 0 {
                    assert_eq!(sample.time_ns - prior, sample.dt_ns);
                }
                prior = sample.time_ns;
                for p in &sample.points {
                    let q = DQuat::from_array(sample.before.foot.orientation).normalize();
                    let world =
                        DVec3::from(sample.before.foot.position) + q * DVec3::from(p.foot_local_m);
                    assert!(world.distance(DVec3::from(p.world_m)) < 1e-13);
                }
            }
            assert_eq!(
                g.samples.last().unwrap().after.foot.position,
                rows[2].foot.position
            );
            assert!(g.max_linear_residual_ns < 1e-3);
            assert!(g.max_angular_residual_nms < 1e-3);
        }
        Ok(())
    }
}
