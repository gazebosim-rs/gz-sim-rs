// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Direct pinned Rapier controls. The default case is checked against the production
//! multifidelity adapter before changing a single numerical integration parameter.
use super::*;
use rapier3d_f64::prelude::*;

struct Native {
    world: PhysicsWorld,
    ids: Vec<RigidBodyHandle>,
    ground: ColliderHandle,
}
#[derive(Default, Clone, Copy)]
struct PairDetail {
    total: f64,
    solver_points: f64,
    projected: f64,
    tangent_vertical_bound: f64,
}

impl Native {
    fn new(seeds: &[Body], h: Duration, inner: usize) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0., 0., -G);
        world.integration_parameters.dt = h.as_secs_f64();
        world.integration_parameters.contact_recycling = false;
        world.integration_parameters.num_internal_pgs_iterations = inner;
        let ground = world.colliders.insert(
            ColliderBuilder::new(SharedShape::halfspace(Vector::Z))
                .friction(0.5)
                .friction_combine_rule(CoefficientCombineRule::Min)
                .restitution(0.)
                .build(),
        );
        let mut ids = Vec::new();
        for s in seeds {
            let [x, y, z] = s.size.map(|v| v * 0.5);
            let body = RigidBodyBuilder::dynamic()
                .can_sleep(false)
                .pose(Pose::from_parts(
                    Vector::from_array(s.position),
                    Rotation::from_xyzw(
                        s.orientation[0],
                        s.orientation[1],
                        s.orientation[2],
                        s.orientation[3],
                    ),
                ))
                .linvel(Vector::from_array(s.linear))
                .angvel(Vector::from_array(s.angular));
            let collider = ColliderBuilder::cuboid(x, y, z)
                .mass(s.mass)
                .friction(s.friction)
                .friction_combine_rule(CoefficientCombineRule::Min);
            let (id, _) = world.insert(body, collider);
            world.bodies[id].recompute_mass_properties_from_colliders(&world.colliders);
            ids.push(id);
        }
        Self { world, ids, ground }
    }
    fn state(&self, index: usize) -> BodyState {
        let b = &self.world.bodies[self.ids[index]];
        BodyState {
            position: b.translation().to_array(),
            orientation: b.rotation().to_array(),
            velocity: b.linvel().to_array(),
            angular_velocity: b.angvel().to_array(),
        }
    }
    fn contacts(
        &self,
        pairs: &mut Pairs,
        ground: &mut Supports,
        details: &mut [PairDetail],
        first_id: u64,
    ) {
        for p in self
            .world
            .narrow_phase
            .contact_pairs()
            .filter(|p| p.has_any_active_contact())
        {
            let local = |c: ColliderHandle| {
                self.world.colliders[c].parent().unwrap().into_raw_parts().0 as u64
            };
            let total = p.total_impulse_magnitude();
            // Preserve geometric summation order in comparisons with production.
            let selected: f64 = p
                .solver_manifolds()
                .iter()
                .map(|m| {
                    m.points
                        .iter()
                        .enumerate()
                        .filter(|(i, _)| {
                            m.data.solver_contacts.iter().any(|row| {
                                (row.contact_id[0] & !rapier3d_f64::geometry::NEW_CONTACT_BIT)
                                    as usize
                                    == *i
                            })
                        })
                        .map(|(_, point)| point.data.impulse)
                        .sum::<f64>()
                })
                .sum();
            if p.collider1 == self.ground || p.collider2 == self.ground {
                let c = if p.collider1 == self.ground {
                    p.collider2
                } else {
                    p.collider1
                };
                ground.insert(first_id + local(c), selected);
                continue;
            }
            let a = first_id + local(p.collider1);
            let b = first_id + local(p.collider2);
            let key = (a.min(b), a.max(b));
            assert_eq!(key.1, key.0 + 1);

            let d = &mut details[key.0 as usize / 2];
            d.total = total;
            for m in p.solver_manifolds() {
                let nz = m.data.normal.z.abs();
                for sc in &m.data.solver_contacts {
                    let j = (sc.contact_id[0] & !rapier3d_f64::geometry::NEW_CONTACT_BIT) as usize;
                    let v = &m.points[j].data;
                    d.solver_points += v.impulse;
                    d.projected += v.impulse * nz;
                    // The tangent impulse lies in the contact plane. This is
                    // a rigorous bound on its world-Z projection for a static normal.
                    d.tangent_vertical_bound +=
                        v.tangent_impulse.norm() * m.data.normal.x.hypot(m.data.normal.y);
                }
            }
            pairs.insert(key, selected);
        }
    }
}

struct Layout {
    worlds: Vec<Native>,
    statistics: Stats,
    last_points: Vec<PairDetail>,
    reports: Vec<PairDetail>,
    before_window: Vec<f64>,
    pz_after: Vec<f64>,
}
impl Layout {
    fn new(seeds: &[Body], h: Duration, inner: usize, separate: bool) -> Self {
        let cells = seeds.len() / 2;
        let chunk = if separate { 2 } else { seeds.len() };
        Self {
            worlds: seeds
                .chunks_exact(chunk)
                .map(|b| Native::new(b, h, inner))
                .collect(),
            statistics: Stats::new(cells),
            last_points: vec![PairDetail::default(); cells],
            reports: vec![PairDetail::default(); cells],
            before_window: vec![0.; cells],
            pz_after: vec![0.; cells],
        }
    }
    fn step(&mut self, seeds: &[Body], ms: u64, states: &mut [BodyState]) -> (Pairs, Supports) {
        let mut first = 0;
        let mut pairs = Pairs::new();
        let mut floor = Supports::new();
        self.last_points.fill(PairDetail::default());
        for w in &mut self.worlds {
            w.world.step();
            for i in 0..w.ids.len() {
                states[first + i] = w.state(i);
            }
            w.contacts(&mut pairs, &mut floor, &mut self.last_points, first as u64);
            first += w.ids.len();
        }
        self.statistics.observe(seeds, states, &pairs, &floor, ms);
        for (cell, pair) in states.chunks_exact(2).enumerate() {
            let pz = seeds[2 * cell + 1].mass * pair[1].velocity[2];
            if ms == END_MS - WINDOW_MS {
                self.before_window[cell] = pz;
            }
            if ms > END_MS - WINDOW_MS {
                // A foreign or ground contact with an upper body would invalidate
                // the interpretation of its vertical momentum change.
                assert!(!floor.contains_key(&seeds[2 * cell + 1].id));
                let q = &mut self.reports[cell];
                let d = self.last_points[cell];
                q.total += d.total;
                q.solver_points += d.solver_points;
                q.projected += d.projected;
                q.tangent_vertical_bound += d.tangent_vertical_bound;
                self.pz_after[cell] = pz;
            }
        }
        (pairs, floor)
    }
}
fn raw_diff(a: BodyState, b: BodyState) -> f64 {
    norm(std::array::from_fn(|i| a.position[i] - b.position[i]))
}
fn target_probe(target: &[Body], h: Duration, out: &mut BufWriter<File>) -> Result<()> {
    let ms = h.as_millis() as u64;
    let centered: Vec<_> = target
        .iter()
        .map(|b| Body {
            position: [0., 0., b.position[2]],
            ..b.clone()
        })
        .collect();
    for (n, hit) in [(123, true), (124, true), (124, false)] {
        let mut population = centered.clone();
        for j in 0..n {
            population.push(Body {
                id: (2 + j) as u64,
                position: [
                    8. + 8. * (j % 16) as f64,
                    8. * (j / 16) as f64,
                    if hit { 0.64 } else { 10000. },
                ],
                ..centered[0].clone()
            });
        }
        for inner in [1, 2, 4, 8] {
            let mut small = Native::new(&centered, h, inner);
            let mut large = Native::new(&population, h, inner);
            let mut peak = 0f64;
            let mut peak_xy = [0f64; 2];
            let mut first = None;
            for tick in 1..=END_MS / ms {
                small.world.step();
                large.world.step();
                for id in 0..2 {
                    let a = small.state(id);
                    let b = large.state(id);
                    if a != b {
                        first.get_or_insert(ms * tick);
                    }
                    peak = peak.max(raw_diff(a, b));
                    for (m, state) in peak_xy.iter_mut().zip([a, b]) {
                        *m = m.max(state.position[0].hypot(state.position[1]));
                    }
                }
            }
            writeln!(
                out,
                "{ms},{inner},{n},{hit},{},{peak:.12e},{:.12e},{:.12e}",
                first.unwrap_or(0),
                peak_xy[0],
                peak_xy[1]
            )?;
        }
        out.flush()?;
    }
    Ok(())
}

pub(super) fn study(worker: &Path, output: &Path, steps: &[u64]) -> Result<()> {
    // The worker establishes the exact normal SDF admission, body ordering and
    // material coefficients. Native probes below do not edit production settings.
    let sim = load_world_configured(
        &grid::sdf(256),
        "grid.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: Some(Duration::from_millis(5)),
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;
    let seeds = (0..256)
        .map(|i| frame(&sim).body(i).unwrap().1.clone())
        .collect::<Vec<_>>();
    drop(sim);
    for pair in seeds.chunks_exact(2) {
        assert_eq!((pair[0].mass, pair[1].mass), (1., 2.));
        assert_eq!(pair[0].position[..2], pair[1].position[..2]);
    }
    let mut table = BufWriter::new(File::create(output.join("native-options.csv"))?);
    let mut closure = BufWriter::new(File::create(output.join("native-loads.csv"))?);
    let mut threshold = BufWriter::new(File::create(output.join("threshold.csv"))?);
    writeln!(
        table,
        "step_ms,inner_pgs,layout,first_difference_ms,peak_difference_m,peak_horizontal_m,peak_tilt_rad,late_z_err_m,settled_from_ms,late_mean_kinetic_J_stack,peak_penetration_m,vertical_momentum_residual_Ns,late_floor_error_N,late_selected_pair_error_N,layout_measurement_s"
    )?;
    writeln!(
        closure,
        "step_ms,inner_pgs,layout,cell,all_retained_normal_N,selected_point_normal_N,projected_normal_N,tangent_vertical_bound_N,vertical_load_from_momentum_N"
    )?;
    writeln!(
        threshold,
        "step_ms,inner_pgs,remote_boxes,remotes_contact,first_difference_ms,peak_difference_m,isolated_max_xy_m,loaded_max_xy_m"
    )?;
    let mut curves = Vec::new();
    for &hms in steps {
        let h = Duration::from_millis(hms);
        for inner in [1, 2, 4, 8] {
            let start = std::time::Instant::now();
            let mut layouts = [
                Layout::new(&seeds, h, inner, true),
                Layout::new(&seeds, h, inner, false),
            ];
            let mut production = (inner == 1).then(|| Control::new(&seeds, h)).transpose()?;
            let mut states = [
                vec![state(&seeds[0]); seeds.len()],
                vec![state(&seeds[0]); seeds.len()],
            ];
            let mut peak = 0f64;
            let mut elapsed = [Duration::ZERO; 2];
            let mut first = None;
            for tick in 1..=END_MS / hms {
                let ms = tick * hms;
                let timer = std::time::Instant::now();
                layouts[0].step(&seeds, ms, &mut states[0]);
                elapsed[0] += timer.elapsed();
                let timer = std::time::Instant::now();
                let (pairs, floor) = layouts[1].step(&seeds, ms, &mut states[1]);
                elapsed[1] += timer.elapsed();
                if let Some(w) = &mut production {
                    w.step(h)?;
                    for (id, raw) in states[1].iter().enumerate() {
                        assert_eq!(
                            raw,
                            &w.state(id),
                            "raw native baseline dt {hms} tick {tick}, body {id}"
                        );
                    }
                    let mut p = Pairs::new();
                    let mut g = Supports::new();
                    w.contacts(&mut p, &mut g);
                    assert_eq!(p, pairs);
                    assert_eq!(g, floor);
                }
                for (&a, &b) in states[0].iter().zip(&states[1]) {
                    if a != b {
                        first.get_or_insert(ms);
                    }
                    peak = peak.max(raw_diff(a, b));
                }
            }
            let wall = start.elapsed().as_secs_f64();
            curves.push((
                hms,
                inner,
                peak,
                layouts[0].statistics.peak_xy,
                layouts[1].statistics.peak_xy,
            ));
            for ((layout, world), solve_time) in ["component", "monolithic"]
                .into_iter()
                .zip(layouts)
                .zip(elapsed)
            {
                let s = &world.statistics;
                writeln!(
                    table,
                    "{hms},{inner},{layout},{},{peak:.12e},{:.12e},{:.12e},{:.12e},{},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.6}",
                    first.unwrap_or(0),
                    s.peak_xy,
                    s.peak_tilt,
                    s.late_z,
                    s.last_unsettled_ms + hms,
                    s.late_ke,
                    s.peak_floor_penetration,
                    s.peak_cell_momentum_gap,
                    Stats::load_error(&s.late_floor, 3. * G),
                    Stats::load_error(&s.late_pair, 2. * G),
                    solve_time.as_secs_f64()
                )?;
                let inv = 1000. / WINDOW_MS as f64;
                for (i, q) in world.reports.iter().enumerate() {
                    let net = 2. * G + (world.pz_after[i] - world.before_window[i]) * inv;
                    writeln!(
                        closure,
                        "{hms},{inner},{layout},{i},{:.12e},{:.12e},{:.12e},{:.12e},{net:.12e}",
                        q.total * inv,
                        q.solver_points * inv,
                        q.projected * inv,
                        q.tangent_vertical_bound * inv
                    )?;
                }
            }
            table.flush()?;
            closure.flush()?;
            eprintln!("native {hms} ms, {inner} PGS, peak {peak} m in {wall:.2}s");
        }
        target_probe(&seeds[..2], h, &mut threshold)?;
    }
    plot::options(&curves, &output.join("refinement.svg"))?;
    Ok(())
}
