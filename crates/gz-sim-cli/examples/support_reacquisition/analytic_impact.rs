// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Perfectly plastic frictionless impacts of a locked foot with spherical total inertia.
use super::*;
use gz_physics::NativeStepGrid;

#[path = "impact_certification.rs"]
pub(crate) mod certification;
#[path = "analytic_cost.rs"]
pub(crate) mod cost;
#[path = "impact_response.rs"]
pub(crate) mod response;
#[path = "sdf_box_features.rs"]
pub(crate) mod sdf_features;
const OUTER: Duration = Duration::from_micros(500);
const END_TICKS: u64 = 8;
const ARRIVAL_NS: u64 = 2_187_500;
const PAD_SPEED: f64 = 0.30;
const GRIDS: [u32; 7] = [1, 4, 8, 16, 32, 64, 128];

#[derive(Clone, Copy, Serialize)]
struct Fixture {
    label: &'static str,
    roll_rad: f64,
    pitch_rad: f64,
    relative_closing_m_s: f64,
}
impl Fixture {
    fn level(self) -> bool {
        self.roll_rad == 0. && self.pitch_rad == 0.
    }
}
const FIXTURES: [Fixture; 6] = [
    Fixture {
        label: "level-slow",
        roll_rad: 0.,
        pitch_rad: 0.,
        relative_closing_m_s: 0.6,
    },
    Fixture {
        label: "level-fast",
        roll_rad: 0.,
        pitch_rad: 0.,
        relative_closing_m_s: 1.4,
    },
    Fixture {
        label: "corner-a-slow",
        roll_rad: 0.19,
        pitch_rad: -0.24,
        relative_closing_m_s: 0.6,
    },
    Fixture {
        label: "corner-a-fast",
        roll_rad: 0.19,
        pitch_rad: -0.24,
        relative_closing_m_s: 1.4,
    },
    Fixture {
        label: "corner-b-slow",
        roll_rad: -0.32,
        pitch_rad: 0.14,
        relative_closing_m_s: 0.6,
    },
    Fixture {
        label: "corner-b-fast",
        roll_rad: -0.32,
        pitch_rad: 0.14,
        relative_closing_m_s: 1.4,
    },
];
fn corners() -> [DVec3; 4] {
    let [x, y, z] = FOOT_HALF;
    [
        DVec3::new(-x, -y, -z),
        DVec3::new(-x, y, -z),
        DVec3::new(x, -y, -z),
        DVec3::new(x, y, -z),
    ]
}
#[derive(Clone, Copy)]
struct Ideal {
    fixture: Fixture,
    orientation: DQuat,
    initial_com: DVec3,
    incoming_velocity: DVec3,
    outgoing_velocity: DVec3,
    spin: DVec3,
    impulse: f64,
    corner: Option<usize>,
}
#[derive(Serialize, Clone, Copy, PartialEq)]
struct State {
    foot: Body,
    upper: Body,
    pad: Body,
}
impl State {
    fn com(self) -> DVec3 {
        (DVec3::from(self.foot.position) + DVec3::from(self.upper.position)) * 0.5
    }
    fn velocity(self) -> DVec3 {
        (DVec3::from(self.foot.velocity) + DVec3::from(self.upper.velocity)) * 0.5
    }
}
impl Ideal {
    fn new(f: Fixture) -> Self {
        // Nonzero yaw exercises world/body axes for the tilted cases. Keep the
        // level validation aligned with the collider's symmetry axes.
        let yaw = if f.level() { 0. } else { 0.17 };
        let q = DQuat::from_euler(glam::EulerRot::ZYX, yaw, f.pitch_rad, f.roll_rad);
        let arms = corners().map(|p| q * (p - ARM));
        let lowest = (0..4)
            .min_by(|&a, &b| arms[a].z.total_cmp(&arms[b].z))
            .unwrap();
        let corner = (!f.level()).then_some(lowest);
        // On a level face the COM projects strictly inside the supporting rectangle,
        // so the impulse resultant through that projection gives a no-spin solution.
        let r = if f.level() {
            -DVec3::Z * (FOOT_HALF[2] + ARM.z)
        } else {
            arms[lowest]
        };
        let rxn = r.cross(DVec3::Z);
        // Parallel-axis sum of the two specified 1 kg inertias = 0.05 * identity.
        let effective_inverse = 1. / M + rxn.length_squared() / I;
        let impulse = f.relative_closing_m_s / effective_inverse;
        let v = DVec3::new(0.13, -0.09, PAD_SPEED - f.relative_closing_m_s);
        let initial_com = DVec3::new(
            0.,
            0.,
            -arms[lowest].z + f.relative_closing_m_s * ARRIVAL_NS as f64 * 1e-9,
        );
        Self {
            fixture: f,
            orientation: q,
            initial_com,
            incoming_velocity: v,
            outgoing_velocity: v + DVec3::Z * impulse / M,
            spin: rxn * impulse / I,
            impulse,
            corner,
        }
    }
    fn state(self, ns: u64) -> State {
        let t = ns as f64 * 1e-9;
        let after = ns >= ARRIVAL_NS;
        let free = ns.min(ARRIVAL_NS) as f64 * 1e-9;
        let elapsed = ns.saturating_sub(ARRIVAL_NS) as f64 * 1e-9;
        let com =
            self.initial_com + self.incoming_velocity * free + self.outgoing_velocity * elapsed;
        // Torque-free spherical inertia keeps world angular velocity constant.
        let q = DQuat::from_scaled_axis(self.spin * elapsed) * self.orientation;
        let w = if after { self.spin } else { DVec3::ZERO };
        let v = if after {
            self.outgoing_velocity
        } else {
            self.incoming_velocity
        };
        let body = |arm: DVec3| Body {
            position: (com + arm).to_array(),
            orientation: q.to_array(),
            velocity: (v + w.cross(arm)).to_array(),
            angular_velocity: w.to_array(),
        };
        State {
            foot: body(q * (-ARM)),
            upper: body(q * ARM),
            pad: Body {
                position: [0., 0., -PAD_HALF[2] + PAD_SPEED * t],
                orientation: DQuat::IDENTITY.to_array(),
                velocity: [0., 0., PAD_SPEED],
                angular_velocity: [0.; 3],
            },
        }
    }
}

fn fresh(ideal: Ideal, tree: bool, passes: usize) -> Result<(MultiFidelityBackend, Bodies)> {
    fresh_at(ideal.state(0), tree, passes)
}
fn fresh_at(s: State, tree: bool, passes: usize) -> Result<(MultiFidelityBackend, Bodies)> {
    fresh_in_order(s, tree, passes, false)
}
fn fresh_in_order(
    s: State,
    tree: bool,
    passes: usize,
    pad_first: bool,
) -> Result<(MultiFidelityBackend, Bodies)> {
    let mut native = MultiFidelityBackend::new([0.; 3])?;
    native.set_contact_pgs_passes(passes)?;
    if native.contact_pgs_passes() != passes {
        return Err("incorrect fixed-time PGS budget".into());
    }
    let pad_spec = cube(PAD_HALF, s.pad, 1.);
    let first = pad_first
        .then(|| native.spawn_ideal_landing_surface(pad_spec, orientation(s.pad)))
        .transpose()?;
    let (foot, upper) = if tree {
        let [a, b] = spawn_locked_pair(&mut native, s.foot, s.upper)?;
        (a, Some(b))
    } else {
        let rigid = Body {
            velocity: s.velocity().to_array(),
            ..s.foot
        };
        let a = native.spawn_link(
            cube(FOOT_HALF, rigid, M),
            orientation(s.foot),
            inertia(ARM.to_array(), [I; 3]),
        )?;
        (a, None)
    };
    let pad = match first {
        Some(id) => id,
        None => native.spawn_ideal_landing_surface(pad_spec, orientation(s.pad))?,
    };
    Ok((native, Bodies { foot, upper, pad }))
}
fn state(native: &MultiFidelityBackend, ids: Bodies) -> std::result::Result<State, PhysicsError> {
    let get = |id| {
        native
            .state(id)
            .map(Body::from)
            .ok_or(PhysicsError::MissingBody)
    };
    let mut foot = get(ids.foot)?;
    let upper = if let Some(id) = ids.upper {
        get(id)?
    } else {
        let v = DVec3::from(foot.velocity);
        let w = DVec3::from(foot.angular_velocity);
        let arm = DQuat::from_array(foot.orientation).normalize() * ARM;
        foot.velocity = (v - w.cross(arm)).to_array();
        Body {
            position: (DVec3::from(foot.position) + arm * 2.).to_array(),
            velocity: (v + w.cross(arm)).to_array(),
            ..foot
        }
    };
    Ok(State {
        foot,
        upper,
        pad: get(ids.pad)?,
    })
}
#[derive(Clone, Serialize)]
struct Snapshot {
    #[serde(flatten)]
    state: State,
    tick: u64,
    time_us: u64,
    native_steps: u32,
    selected: bool,
    secondary_start: bool,
    normal_impulse_ns: f64,
    corner_ns: [f64; 4],
}
fn geometry(state: State, corner: Option<usize>) -> (f64, f64, f64) {
    let q = DQuat::from_array(state.foot.orientation).normalize();
    let pad = DVec3::from(state.pad.position);
    let mut other = f64::INFINITY;
    let mut margin = f64::INFINITY;
    for (i, offset) in corners().into_iter().enumerate() {
        let p = DVec3::from(state.foot.position) + q * offset - pad;
        if corner != Some(i) {
            other = other.min(p.z - PAD_HALF[2]);
        }
        margin = margin
            .min(PAD_HALF[0] - p.x.abs())
            .min(PAD_HALF[1] - p.y.abs());
    }
    let upper = state.upper;
    let uq = DQuat::from_array(upper.orientation).normalize();
    let height = upper.position[2]
        - pad.z
        - PAD_HALF[2]
        - (uq.conjugate() * DVec3::Z).abs().dot(UPPER_HALF.into());
    (other, margin, height)
}

#[derive(Serialize)]
struct Receipt {
    time_ns: u64,
    corner_ns: [f64; 4],
    normal_ns: f64,
    velocity: [f64; 3],
    spin_rad_s: [f64; 3],
}
#[derive(Resource)]
struct Driver {
    ids: Bodies,
    n: u32,
    iterations: usize,
    passes: usize,
    ideal: Ideal,
    time_ns: u64,
    frames: Vec<Snapshot>,
    loaded: Vec<Receipt>,
    total: f64,
    unexpected: f64,
    max_tangent: f64,
    max_horizontal_speed_change: f64,
    min_other_gap: f64,
    min_margin: f64,
    min_upper: f64,
    min_exact_other_gap: f64,
    min_exact_margin: f64,
}
impl Driver {
    fn new(ids: Bodies, n: u32, ideal: Ideal, start: State) -> Self {
        let (g, m, h) = geometry(start, ideal.corner);
        Self {
            ids,
            n,
            iterations: 4,
            passes: 1,
            ideal,
            time_ns: 0,
            frames: vec![Snapshot {
                state: start,
                time_us: 0,
                tick: 0,
                native_steps: 0,
                selected: false,
                secondary_start: false,
                normal_impulse_ns: 0.,
                corner_ns: [0.; 4],
            }],
            loaded: vec![],
            total: 0.,
            unexpected: 0.,
            max_tangent: 0.,
            max_horizontal_speed_change: 0.,
            min_other_gap: g,
            min_margin: m,
            min_upper: h,
            min_exact_other_gap: g,
            min_exact_margin: m,
        }
    }
    fn advance(
        &mut self,
        native: &mut MultiFidelityBackend,
        dt: Duration,
    ) -> std::result::Result<(), PhysicsError> {
        let mut outer_corners = [0.; 4];
        for interval in NativeStepGrid::new(dt, self.n)?.intervals() {
            if self.iterations == 4 {
                native.step(interval)?;
            } else {
                native.step_landing_with_iterations(interval, self.iterations)?;
            }
            self.time_ns += interval.as_nanos() as u64;
            let s = state(native, self.ids)?;
            let inverse = DQuat::from_array(s.foot.orientation)
                .normalize()
                .conjugate();
            let (gap, margin, height) = geometry(s, self.ideal.corner);
            self.min_other_gap = self.min_other_gap.min(gap);
            self.min_margin = self.min_margin.min(margin);
            self.min_upper = self.min_upper.min(height);
            let (g, m, _) = geometry(self.ideal.state(self.time_ns), self.ideal.corner);
            self.min_exact_other_gap = self.min_exact_other_gap.min(g);
            self.min_exact_margin = self.min_exact_margin.min(m);
            self.max_horizontal_speed_change = self.max_horizontal_speed_change.max(
                (s.velocity() - self.ideal.incoming_velocity)
                    .truncate()
                    .length(),
            );
            let mut delivered = [0.; 4];
            for patch in native.rigid_contact_patches() {
                let direct = (patch.a, patch.b) == (self.ids.foot, self.ids.pad);
                if !direct && (patch.b, patch.a) != (self.ids.foot, self.ids.pad) {
                    if patch.points.iter().any(|p| p.normal_impulse > 0.) {
                        return Err(PhysicsError::InvalidValue);
                    }
                    continue;
                }
                for point in patch.points {
                    if let Some(j) = point.tangent_impulse_on_a {
                        self.max_tangent = self.max_tangent.max(DVec3::from(j).length());
                    }
                    let arm = DVec3::from(if direct { point.lever_a } else { point.lever_b });
                    // Rapier reports COM-relative lever arms; for the matched rigid
                    // collider shift the point back to its geometric foot frame.
                    let local = inverse * arm
                        + if self.ids.upper.is_none() {
                            ARM
                        } else {
                            DVec3::ZERO
                        };
                    let corner = 2 * usize::from(local.x > 0.) + usize::from(local.y > 0.);
                    delivered[corner] += point.normal_impulse;
                }
            }
            let j: f64 = delivered.iter().sum();
            if let Some(k) = self.ideal.corner {
                self.unexpected += delivered
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| *i != k)
                    .map(|(_, v)| v)
                    .sum::<f64>();
            }
            for (dst, src) in outer_corners.iter_mut().zip(delivered) {
                *dst += src;
            }
            self.total += j;
            if j > 0. {
                self.loaded.push(Receipt {
                    time_ns: self.time_ns,
                    corner_ns: delivered,
                    normal_ns: j,
                    velocity: s.velocity().to_array(),
                    spin_rad_s: s.foot.angular_velocity,
                });
            }
        }
        self.frames.push(Snapshot {
            state: state(native, self.ids)?,
            tick: self.frames.len() as u64,
            time_us: self.time_ns / 1000,
            native_steps: self.n,
            selected: false,
            secondary_start: false,
            normal_impulse_ns: outer_corners.iter().sum(),
            corner_ns: outer_corners,
        });
        Ok(())
    }
}
fn step(
    t: Res<UpdateInfo>,
    fault: Res<StepFault>,
    mut backend: ResMut<PhysicsBackend>,
    mut driver: ResMut<Driver>,
) {
    if !t.dt.is_zero()
        && let Err(e) = driver.advance(
            backend.downcast_mut::<MultiFidelityBackend>().unwrap(),
            t.dt,
        )
    {
        fault.report(e);
    }
}
fn run(ideal: Ideal, tree: bool, n: u32, iterations: usize, passes: usize) -> Result<Driver> {
    let (native, ids) = fresh(ideal, tree, passes)?;
    let start = state(&native, ids)?;
    let mut sim = Simulation::new(OUTER)?;
    gz_physics::install_with_step(&mut sim, native, step)?;
    let mut driver = Driver::new(ids, n, ideal, start);
    driver.iterations = iterations;
    driver.passes = passes;
    sim.world_mut().insert_resource(driver);
    sim.step_exact(END_TICKS)?;
    let d = sim.world_mut().remove_resource::<Driver>().unwrap();
    if d.loaded.is_empty()
        || d.time_ns != END_TICKS * 500_000
        || d.max_tangent > 1e-12
        || d.min_margin <= 0.
        || d.min_exact_margin <= 0.
        || d.min_upper <= 0.
        || ideal.corner.is_some()
            && (d.unexpected > 1e-10 || d.min_other_gap < 0.005 || d.min_exact_other_gap < 0.005)
    {
        return Err(format!("invalid analytic qualification {} tree={tree} grid={n}: loads={} off-corner={} other gap={} tangent={} margin={} upper={}",ideal.fixture.label,d.loaded.len(),d.unexpected,d.min_other_gap,d.max_tangent,d.min_margin,d.min_upper).into());
    }
    Ok(d)
}
fn metrics(d: &Driver) -> Value {
    let end = d.frames.last().unwrap().state;
    let target = d.ideal.state(d.time_ns);
    json!({"representation":if d.ids.upper.is_some(){"locked_pair"}else{"single_rigid"},"substeps":d.n,"iterations":d.iterations,"fixed_time_pgs_passes":d.passes,
        "normal_impulse_ns":d.total,"normal_error_ns":d.total-d.ideal.impulse,"first_positive_time_ns":d.loaded[0].time_ns,
        "last_positive_time_ns":d.loaded.last().unwrap().time_ns,"final_velocity_m_s":end.velocity().to_array(),
        "final_angular_velocity_rad_s":end.foot.angular_velocity,"final_com_m":end.com().to_array(),
        "velocity_error_m_s":end.velocity().distance(target.velocity()),
        "angular_velocity_error_rad_s":DVec3::from(end.foot.angular_velocity).distance(d.ideal.spin),
        "com_error_m":end.com().distance(target.com()),"off_corner_impulse_ns":d.unexpected,
        "min_other_corner_gap_m":d.ideal.corner.map(|_|d.min_other_gap),"min_analytic_other_corner_gap_m":d.ideal.corner.map(|_|d.min_exact_other_gap),
        "min_footprint_margin_m":d.min_margin,"min_upper_clearance_m":d.min_upper,
        "max_reported_per_point_tangent_ns":d.max_tangent,"max_horizontal_com_velocity_change_m_s":d.max_horizontal_speed_change,
        "native_receipts":d.loaded})
}
fn trace(out: &Path, name: &str, frames: &[Snapshot]) -> Result<()> {
    #[derive(Serialize)]
    struct Trace<'a> {
        geometry: Value,
        frames: &'a [Snapshot],
    }
    write_json(
        &out.join("traces").join(format!("{name}.json")),
        &Trace {
            geometry: json!({"foot_half_extents":FOOT_HALF,
        "pad_half_extents":PAD_HALF,"upper_half_extents":UPPER_HALF}),
            frames,
        },
    )
}
fn save_visual(out: &Path, tree: bool, ideal: Ideal, data: &Driver) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    trace(out, if tree { "locked" } else { "rigid" }, &data.frames)?;
    if !tree {
        return Ok(());
    }
    let frames = (0..=END_TICKS)
        .map(|tick| {
            let time_ns = tick * 500_000;
            let hit = tick > 0 && ARRIVAL_NS > (tick - 1) * 500_000 && ARRIVAL_NS <= time_ns;
            let normal = if hit { ideal.impulse } else { 0. };
            // A level face has many admissible pressure partitions. Equal loads
            // give one symmetric realization for the four visual dots.
            let mut corner_ns = [normal * 0.25; 4];
            if let Some(k) = ideal.corner {
                corner_ns = [0.; 4];
                corner_ns[k] = normal;
            }
            Snapshot {
                state: ideal.state(time_ns),
                tick,
                time_us: time_ns / 1000,
                native_steps: 0,
                selected: false,
                secondary_start: false,
                normal_impulse_ns: normal,
                corner_ns,
            }
        })
        .collect::<Vec<_>>();
    trace(out, "analytic", &frames)?;
    let title = if ideal.corner.is_some() {
        "Perfectly plastic oblique corner impact"
    } else {
        "Perfectly plastic level face impact"
    };
    write_json(
        &out.join("comparison.json"),
        &json!({"title":title,"lanes":[
        {"label":"locked","name":"Locked foot, 8 substeps","file":"traces/locked.json"},
        {"label":"rigid","name":"Inertia-matched single body, 8 substeps","file":"traces/rigid.json"},
        {"label":"analytic","name":"Analytic impact","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    let mut cases = vec![];
    for fixture in FIXTURES {
        let ideal = Ideal::new(fixture);
        let mut trials = vec![];
        let mut iteration_controls = vec![];
        let mut fixed_pgs_controls = vec![];
        for tree in [true, false] {
            for n in GRIDS {
                let data = run(ideal, tree, n, 4, 1)?;
                if n == 8 {
                    match fixture.label {
                        "corner-a-fast" => save_visual(out, tree, ideal, &data)?,
                        "level-fast" => save_visual(&out.join("face"), tree, ideal, &data)?,
                        _ => {}
                    }
                }
                trials.push(metrics(&data));
            }
            for passes in [2, 4, 8, 16, 32] {
                let result = run(ideal, tree, 8, 4, passes)?;
                fixed_pgs_controls.push(metrics(&result));
                if fixture.label == "level-fast" && tree && passes == 8 {
                    let path = out.join("face-pgs");
                    fs::create_dir_all(path.join("traces"))?;
                    trace(&path, "pgs", &result.frames)?;
                    write_json(
                        &path.join("comparison.json"),
                        &json!({"title":"Eight fixed-time constraint sweeps on a level face",
                        "lanes":[{"label":"locked","name":"Locked foot, default 1 PGS sweep","file":"../face/traces/locked.json"},
                        {"label":"pgs","name":"Locked foot, 8 PGS sweeps","file":"traces/pgs.json"},
                        {"label":"analytic","name":"Analytic impact","file":"../face/traces/analytic.json","analytical":true}]}),
                    )?;
                }
            }
            for iterations in [1, 2, 8, 16, 32] {
                iteration_controls.push(metrics(&run(ideal, tree, 8, iterations, 1)?));
            }
        }
        cases.push(json!({"fixture":fixture,"ideal":{"first_time_ns":ARRIVAL_NS,"normal_impulse_ns":ideal.impulse,"corner":ideal.corner,
            "outgoing_velocity_m_s":ideal.outgoing_velocity.to_array(),"outgoing_angular_velocity_rad_s":ideal.spin.to_array(),
            "initial":ideal.state(0),"at_impact":ideal.state(ARRIVAL_NS),"at_end":ideal.state(END_TICKS*500_000)},"trials":trials,"iteration_controls":iteration_controls,"fixed_pgs_controls":fixed_pgs_controls}));
    }
    write_json(
        &out.join("analytic.json"),
        &json!({"mass_kg":M,"com_inertia_kg_m2":I,"gravity_m_s2":[0.,0.,0.],
        "pad_speed_m_s":PAD_SPEED,"actuator_tick_ns":500000,"end_ns":END_TICKS*500000,"cases":cases}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closed_form_impact_satisfies_impulse_and_contact_velocity() {
        for f in FIXTURES {
            let ideal = Ideal::new(f);
            let s = ideal.state(ARRIVAL_NS);
            let arms = corners().map(|p| ideal.orientation * (p - ARM));
            assert!(
                s.velocity()
                    .distance(ideal.incoming_velocity + DVec3::Z * ideal.impulse / M)
                    < 1e-12
            );
            for (i, r) in arms.into_iter().enumerate() {
                if ideal.corner.is_none_or(|k| k == i) {
                    assert!(((s.velocity() + ideal.spin.cross(r)).z - PAD_SPEED).abs() < 1e-12);
                    assert!(
                        ((s.com() + r - DVec3::from(s.pad.position)).z - PAD_HALF[2]).abs() < 1e-12
                    );
                }
                if ideal.corner == Some(i) {
                    assert!(ideal.spin.distance(r.cross(DVec3::Z) * ideal.impulse / I) < 1e-12);
                    assert!(geometry(s, Some(i)).0 > 0.01);
                }
            }
        }
    }

    #[test]
    fn frictionless_native_rigid_reproduces_face_and_both_isolated_corners() {
        for f in [FIXTURES[1], FIXTURES[3], FIXTURES[5]] {
            let ideal = Ideal::new(f);
            let d = run(ideal, false, 8, 4, 1).unwrap();
            assert!((d.total - ideal.impulse).abs() < 1e-6);
            let measured = DVec3::from(d.frames.last().unwrap().state.foot.angular_velocity);
            assert!(measured.distance(ideal.spin) < 1e-6);
            assert_eq!(d.unexpected, 0.);
            let tree = run(ideal, true, 8, 4, 1).unwrap();
            assert_eq!(tree.unexpected, 0.);
            assert_eq!(tree.time_ns, 4_000_000);
        }
    }
}
