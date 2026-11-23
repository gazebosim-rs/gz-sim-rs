// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed-form support-edge, sticking and sliding impulses of a level foot.
#[path = "planar_impact/arrival.rs"]
mod arrival;
#[path = "planar_impact/census_report.rs"]
mod census_report;
#[path = "planar_impact/edge_cost.rs"]
mod edge_cost;
#[path = "planar_impact/edge_load.rs"]
mod edge_load;
#[path = "planar_impact/projected.rs"]
mod projected;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, Inertia, RotationSpec, Shape};
use gz_physics_multifidelity::{
    Bearing, ContactSolverIterations, IsotropicFriction, MaterialLinkMotion as Motion,
    MultiFidelityBackend, PlanarImpactResponse, PlanarPatchImpact,
};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Duration,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const FOOT: [f64; 3] = [0.18, 0.10, 0.06];
const TOP: [f64; 3] = [0.05, 0.06, 0.06];
const PAD: [f64; 3] = [0.33, 0.18, 0.06];
const HEIGHT: f64 = 0.13;
const DT: Duration = Duration::from_micros(500);
const FIELD: usize = 25 * 17 * 13 * 2 * 2;
const FORMAT: &str = "planar-coulomb-impact-v1";
#[derive(Parser)]
struct Args {
    output: PathBuf,
    /// Write complete, restartable census batches.
    #[arg(long, group = "phase")]
    census: bool,
    /// Check the full physical lattice and summarize signed and absolute errors.
    #[arg(long, group = "phase")]
    summarize: bool,
    /// Export three native traces at this physical census index.
    #[arg(long, group = "phase")]
    replay: Option<usize>,
    /// Sweep fractional arrival times on a fixed public clock.
    #[arg(long, group = "phase")]
    arrival: bool,
    /// Measure whole public-tick cost with the actual predictor and native interval.
    #[arg(long, group = "phase")]
    arrival_cost: bool,
    /// Follow a landing just before the actuator boundary into the next tick.
    #[arg(long, group = "phase")]
    arrival_carry: bool,
    /// Register a symmetric sole patch and check a consecutive slice of fractional landings.
    #[arg(long, group = "phase")]
    projected_patch: bool,
    /// Follow sustained contact on a translating deck corner after its instantaneous patch load.
    #[arg(long, group = "phase")]
    edge_load: bool,
    /// Qualify an admitted continuous deck-edge reaction against the constrained path.
    #[arg(long, group = "phase")]
    edge_follow: bool,
    /// Follow the material point held by static friction on a deck corner.
    #[arg(long, requires = "edge_follow")]
    edge_pivot: bool,
    /// Constant longitudinal world gravity gx gz (m/s²) for the edge-follow experiment.
    #[arg(
        long,
        num_args = 2,
        allow_hyphen_values = true,
        requires = "edge_follow"
    )]
    edge_gravity: Vec<f64>,
    /// Measure Bevy deck-edge impact and ongoing reaction cost with static obstacles.
    #[arg(long, group = "phase")]
    edge_cost: bool,
    /// Summarize the complete arrival lattice.
    #[arg(long, group = "phase")]
    arrival_summary: bool,
    /// Re-run and visualize a fractional-arrival case.
    #[arg(long, group = "phase")]
    arrival_replay: Option<usize>,
    /// Start at this consecutive fixture index.
    #[arg(long, default_value_t = 0)]
    first: usize,
    /// Number of fixture indices to run.
    #[arg(long, default_value_t = 32)]
    count: usize,
}
#[derive(Clone, Copy, Serialize, Deserialize)]
struct Case {
    offset: f64,
    mu: f64,
    slip: f64,
    closing: f64,
    pad_first: bool,
}
impl Case {
    fn analytic(self) -> Result<PlanarImpactResponse> {
        Ok(PlanarPatchImpact {
            mass: 2.,
            pitch_inertia: 0.05,
            com_height: HEIGHT,
            supported_x: [
                (-FOOT[0]).max(self.offset - PAD[0]),
                FOOT[0].min(self.offset + PAD[0]),
            ],
            tangent_speed: self.slip,
            closing_speed: self.closing,
            friction: self.mu,
        }
        .solve()?)
    }
    fn at(index: usize) -> Self {
        // Coprime stride permutes every physical combination exactly once.
        let mut k = index * 13007 % FIELD;
        let pad_first = k % 2 == 1;
        k /= 2;
        let closing = [0.65, 1.4][k % 2];
        k /= 2;
        let speed_index = k % 13;
        k /= 13;
        let mu = (k % 17) as f64 / 16.;
        k /= 17;
        let offset = (k as f64 - 12.) * 0.038;
        // Sample absolute approach ratios and both sides of the Coulomb transition.
        let ratios = if mu == 0. {
            [
                -2.3, -1.3, -0.7, -0.4, -0.2, -0.04, 0., 0.04, 0.2, 0.4, 0.7, 1.3, 2.3,
            ]
        } else {
            [
                -2.3,
                -1.3,
                -0.7,
                -mu * 1.01,
                -mu,
                -mu * 0.99,
                0.,
                mu * 0.99,
                mu,
                mu * 1.01,
                0.7,
                1.3,
                2.3,
            ]
        };
        let ratio = ratios[speed_index];
        Self {
            offset,
            mu,
            slip: closing * ratio,
            closing,
            pad_first,
        }
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
enum Model {
    Locked,
    RigidPoint,
    RigidGrouped,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
struct Profile {
    model: Model,
    calls: u32,
    relax: usize,
}
const PROFILES: [Profile; 12] = [
    profile(Model::Locked, 8, 1),
    profile(Model::Locked, 8, 8),
    profile(Model::Locked, 8, 32),
    profile(Model::Locked, 64, 1),
    profile(Model::Locked, 64, 8),
    profile(Model::Locked, 512, 8),
    profile(Model::RigidPoint, 8, 1),
    profile(Model::RigidPoint, 8, 8),
    profile(Model::RigidPoint, 64, 8),
    profile(Model::RigidPoint, 512, 8),
    profile(Model::RigidGrouped, 8, 1),
    profile(Model::RigidGrouped, 64, 8),
];
const fn profile(model: Model, calls: u32, relax: usize) -> Profile {
    Profile {
        model,
        calls,
        relax,
    }
}
fn vec_body(p: [f64; 3], v: [f64; 3]) -> BodyState {
    BodyState {
        position: p,
        orientation: [0., 0., 0., 1.],
        velocity: v,
        angular_velocity: [0.; 3],
    }
}
fn as_json(s: BodyState) -> Value {
    json!({"position":s.position,"orientation":s.orientation,"velocity":s.velocity,"angular_velocity":s.angular_velocity})
}
fn cube(
    n: &mut MultiFidelityBackend,
    body: BodyState,
    half: [f64; 3],
    inertia: (f64, [f64; 3], [f64; 3]),
    motion: Motion,
    mu: f64,
) -> Result<BodyId> {
    let (mass, center, diag) = inertia;
    let i = Inertia {
        center,
        orientation: [0., 0., 0., 1.],
        tensor: [diag[0], diag[1], diag[2], 0., 0., 0.],
    };
    Ok(n.spawn_material_link(
        BodySpec {
            position: body.position,
            velocity: body.velocity,
            mass: Some(mass),
            shape: Shape::Box { half_extents: half },
        },
        RotationSpec {
            orientation: body.orientation,
            angular_velocity: body.angular_velocity,
        },
        Some(i),
        motion,
        IsotropicFriction::new(mu)?,
    )?)
}
struct Rig {
    native: MultiFidelityBackend,
    foot: BodyId,
    upper: Option<BodyId>,
    pad: BodyId,
    velocity: DVec3,
}
impl Rig {
    fn new(c: Case, m: Model) -> Result<Self> {
        Self::arriving(c, m, Duration::ZERO)
    }
    fn arriving(c: Case, m: Model, touch: Duration) -> Result<Self> {
        Self::build(c, m, touch, DVec3::ZERO)
    }
    fn build(c: Case, m: Model, touch: Duration, gravity: DVec3) -> Result<Self> {
        if !touch.is_zero() && gravity != DVec3::ZERO {
            return Err("this matched gravity study starts at the instant of contact".into());
        }
        let flight = touch.as_secs_f64();
        let mut native = if m == Model::RigidPoint {
            MultiFidelityBackend::new_per_point_coulomb(gravity.to_array())?
        } else {
            MultiFidelityBackend::new(gravity.to_array())?
        };
        // Common translation tests both forward and reversed slip in the pad frame.
        let pad_speed = DVec3::new(0.31, 0., 0.25);
        let velocity = pad_speed + DVec3::new(c.slip, 0., -c.closing);
        let support = |n: &mut MultiFidelityBackend| {
            cube(
                n,
                vec_body(
                    [c.offset + c.slip * flight, 0., -PAD[2]],
                    pad_speed.to_array(),
                ),
                PAD,
                (1., [0.; 3], [1.; 3]),
                Motion::Prescribed,
                c.mu,
            )
        };
        let before = if c.pad_first {
            Some(support(&mut native)?)
        } else {
            None
        };
        let lower = vec_body([0., 0., FOOT[2] + c.closing * flight], velocity.to_array());
        let (foot, upper) = if m == Model::Locked {
            let i = [0.0201, 0.0201, 0.025];
            let a = cube(
                &mut native,
                lower,
                FOOT,
                (1., [0.; 3], i),
                Motion::Articulation,
                1.2,
            )?;
            let top = vec_body([0., 0., 0.20 + c.closing * flight], velocity.to_array());
            let b = cube(
                &mut native,
                top,
                TOP,
                (1., [0.; 3], i),
                Motion::Articulation,
                1.2,
            )?;
            for id in [a, b] {
                native.set_self_collision_group(id, Some(2))?;
            }
            native.add_limited_revolute(Some(a), b, top.position, [1., 0., 0.], [0.; 2])?;
            native.enable_exact_revolute_locks(a)?;
            (a, Some(b))
        } else {
            (
                cube(
                    &mut native,
                    lower,
                    FOOT,
                    (2., [0., 0., 0.07], [0.05; 3]),
                    Motion::Ordinary,
                    1.2,
                )?,
                None,
            )
        };
        let pad = if let Some(id) = before {
            id
        } else {
            support(&mut native)?
        };
        Ok(Self {
            native,
            foot,
            upper,
            pad,
            velocity,
        })
    }
}
fn links(mut f: BodyState, upper: Option<BodyState>) -> [BodyState; 2] {
    if let Some(u) = upper {
        return [f, u];
    }
    let h = DQuat::from_array(f.orientation) * DVec3::new(0., 0., 0.07);
    let w = DVec3::from(f.angular_velocity);
    let v = DVec3::from(f.velocity);
    f.velocity = (v - w.cross(h)).to_array();
    let mut top = f;
    top.position = (DVec3::from(f.position) + 2. * h).to_array();
    top.velocity = (v + w.cross(h)).to_array();
    [f, top]
}
fn velocity(b: [BodyState; 2]) -> DVec3 {
    (DVec3::from(b[0].velocity) + DVec3::from(b[1].velocity)) * 0.5
}
fn error(impulse: DVec3, spin: DVec3, target: PlanarImpactResponse) -> Value {
    let goal = DVec3::new(target.impulse[0], 0., target.impulse[1]);
    json!({"impulse_ns":impulse.to_array(),"spin_rad_s":spin.to_array(),"impulse_error_ns":impulse.distance(goal),
        "spin_error_rad_s":spin.distance(DVec3::Y*target.pitch_rate)})
}
fn trial(c: Case, p: Profile, ticks: u64, capture: bool) -> Result<(Value, Vec<Value>)> {
    let target = c.analytic()?;
    let Rig {
        mut native,
        foot,
        upper,
        pad,
        velocity: incoming,
    } = Rig::new(c, p.model)?;
    let mut first = None;
    let mut onset = None;
    let mut frames = vec![];
    let mut max_balance = 0.0_f64;
    let mut first_end = None;
    let mut reference_normal = 0.;
    let trace = |b: [BodyState; 2], s: BodyState, tick: u64, j: f64, corners: [f64; 4]| json!({"tick":tick,"time_us":tick*500,"foot":as_json(b[0]),"upper":as_json(b[1]),"pad":as_json(s),"native_steps":if tick==0{0}else{p.calls},"solver_iterations":4,"normal_impulse_ns":j,"corner_ns":corners});
    if capture {
        frames.push(trace(
            links(
                native.state(foot).unwrap(),
                upper.and_then(|id| native.state(id)),
            ),
            native.state(pad).unwrap(),
            0,
            0.,
            [0.; 4],
        ));
    }
    let mut end = None;
    for tick in 1..=ticks {
        let mut nsum = 0.;
        let mut corners = [0.; 4];
        let mut unknown_pair = 0.;
        let mut pre = velocity(links(
            native.state(foot).unwrap(),
            upper.and_then(|id| native.state(id)),
        ));
        let pair = native.observe_contact_interval(
            DT,
            p.calls,
            ContactSolverIterations {
                temporal: 4,
                pgs: None,
                relaxation: Some(p.relax),
            },
            |s| {
                let raw = s.state(foot).unwrap();
                let b = links(raw, upper.and_then(|id| s.state(id)));
                let v = velocity(b);
                let mut j = 0.;
                let mut native_impulse = DVec3::ZERO;
                let mut full = true;
                for patch in s.contact_patches() {
                    let first_body = patch.a == foot && patch.b == pad;
                    if !first_body && !(patch.a == pad && patch.b == foot) {
                        unknown_pair += patch.points.iter().map(|q| q.normal_impulse).sum::<f64>();
                        continue;
                    }
                    let sign = if first_body { 1. } else { -1. };
                    let n = sign * DVec3::from(patch.normal_on_a);
                    for q in patch.points {
                        let r = DVec3::from(if first_body { q.lever_a } else { q.lever_b });
                        let local = DQuat::from_array(raw.orientation).conjugate() * r;
                        corners[2 * usize::from(local.x > 0.) + usize::from(local.y > 0.)] +=
                            q.normal_impulse;
                        j += q.normal_impulse;
                        native_impulse += n * q.normal_impulse;
                        if let Some(t) = q.tangent_impulse_on_a {
                            native_impulse += sign * DVec3::from(t);
                        } else {
                            full = false;
                        }
                    }
                }
                if full {
                    max_balance = max_balance.max(native_impulse.distance(2. * (v - pre)));
                }
                pre = v;
                nsum += j;
                if j > 0. && first.is_none() {
                    first = Some(error(
                        2. * (v - incoming),
                        DVec3::from(b[0].angular_velocity),
                        target,
                    ));
                    first_end = Some((tick - 1) * 500_000 + s.elapsed.as_nanos() as u64);
                }
            },
        )?;
        let observed: f64 = pair.pairs.iter().map(|n| n.normal_impulse).sum();
        if (observed - nsum).abs() > 1e-9 || unknown_pair > 1e-9 {
            return Err("impulse on an unmodeled pair or interval receipt mismatch".into());
        }
        reference_normal += nsum;
        let b = links(
            native.state(foot).unwrap(),
            upper.and_then(|id| native.state(id)),
        );
        let r = error(
            2. * (velocity(b) - incoming),
            DVec3::from(b[0].angular_velocity),
            target,
        );
        if tick == 1 {
            onset = Some(r.clone());
        }
        end = Some(r);
        if capture {
            frames.push(trace(b, native.state(pad).unwrap(), tick, nsum, corners));
        }
    }
    if first.is_none() {
        return Err("positive closing speed produced no native reaction".into());
    }
    Ok((
        json!({"profile":p,"first_end_ns":first_end,"first":first,"at_500us":onset,"end":end,"sum_normal_ns":reference_normal,"max_point_momentum_residual_ns":max_balance}),
        frames,
    ))
}
fn describe(c: Case) -> Result<Value> {
    let t = c.analytic()?;
    Ok(
        json!({"mode":match t.bearing{Bearing::Face=>"face",Bearing::LowerXEdge=>"lower_x_edge",Bearing::UpperXEdge=>"upper_x_edge"},
        "sticking":t.sticking,"cop_x_m":t.center_of_pressure,"impulse_ns":t.impulse,"velocity_m_s":t.com_velocity,"pitch_rad_s":t.pitch_rate,"dissipation_j":t.dissipated_energy}),
    )
}
fn write_json(path: &Path, v: &Value) -> Result<()> {
    let mut w = BufWriter::new(fs::File::create(path)?);
    serde_json::to_writer(&mut w, v)?;
    w.flush()?;
    Ok(())
}
fn study(out: &Path) -> Result<()> {
    let mut cases = vec![];
    for (offset, slip, mu, label) in [
        (0.265, 1.8, 0.55, "face"),
        (0.40, -2.4, 0.45, "edge"),
        (0., 0.25, 0.6, "stick"),
        (-0.40, 0.12, 0.75, "edge_stick"),
    ] {
        for reverse in [false, true] {
            let c = Case {
                offset,
                mu,
                slip,
                closing: 1.4,
                pad_first: reverse,
            };
            let mut results = vec![];
            let mut lanes = vec![];
            let dir = out.join(label);
            fs::create_dir_all(dir.join("traces"))?;
            for p in PROFILES {
                let lane = match (p.model, p.calls, p.relax) {
                    (Model::Locked, 8, 1) => Some(("default", "Locked, 1 relaxation pass")),
                    (Model::Locked, 8, 32) => Some(("relaxed", "Locked, 32 relaxation passes")),
                    (Model::RigidPoint, 8, 8) => {
                        Some(("rigid", "Rigid equivalent, point friction"))
                    }
                    _ => None,
                };
                let keep = !reverse && lane.is_some();
                let (row, frames) = trial(c, p, 16, keep)?;
                results.push(row);
                if let Some((id, name)) = lane.filter(|_| keep) {
                    write_json(
                        &dir.join("traces").join(format!("{id}.json")),
                        &json!({"geometry":{"foot_half_extents":FOOT,"upper_half_extents":TOP,"pad_half_extents":PAD},"frames":frames}),
                    )?;
                    lanes.push(json!({"name":name,"label":id,"file":format!("traces/{id}.json")}));
                }
            }
            if !reverse {
                write_json(
                    &dir.join("comparison.json"),
                    &json!({"title":format!("Planar frictional impact, {label}"),"lanes":lanes}),
                )?;
            }
            cases.push(json!({"label":label,"fixture":c,"analytic":describe(c)?,"paths":results}));
        }
    }
    write_json(
        &out.join("study.json"),
        &json!({"format":FORMAT,"cases":cases}),
    )
}
fn census(out: &Path, first: usize, count: usize) -> Result<()> {
    let end = first
        .checked_add(count)
        .filter(|&v| v <= FIELD && count > 0)
        .ok_or("census range exceeds 22,100 fixtures")?;
    let dir = out.join("batches");
    fs::create_dir_all(&dir)?;
    for begin in (first..end).step_by(16) {
        let stop = (begin + 16).min(end);
        let file = dir.join(format!("batch-{begin:05}-{stop:05}.json"));
        if file.exists() {
            let old: Value = serde_json::from_reader(fs::File::open(&file)?)?;
            if old["format"] != FORMAT
                || old["first"] != begin
                || old["cases"]
                    .as_array()
                    .is_none_or(|r| r.len() != stop - begin)
            {
                return Err("invalid census batch".into());
            }
            continue;
        }
        let mut cases = vec![];
        for idx in begin..stop {
            let c = Case::at(idx);
            let paths = PROFILES
                .iter()
                .map(|&p| trial(c, p, 2, false).map(|r| r.0))
                .collect::<Result<Vec<_>>>()?;
            cases.push(json!({"index":idx,"fixture":c,"analytic":describe(c)?,"paths":paths}));
        }
        let temp = file.with_extension(format!("partial-{}", std::process::id()));
        write_json(
            &temp,
            &json!({"format":FORMAT,"first":begin,"end":stop,"cases":cases}),
        )?;
        fs::rename(temp, file)?;
        if (begin - first).is_multiple_of(256) {
            eprintln!("completed planar fixtures through {stop} of {end}");
        }
    }
    Ok(())
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir_all(&args.output)?;
    if args.edge_cost {
        edge_cost::run(&args.output)
    } else if args.edge_load || args.edge_follow {
        let gravity = match args.edge_gravity.as_slice() {
            [] => DVec3::ZERO,
            [gx, gz] => DVec3::new(*gx, 0., *gz),
            _ => unreachable!(),
        };
        edge_load::study(&args.output, args.edge_follow, gravity, args.edge_pivot)
    } else if args.projected_patch {
        projected::study(&args.output, args.first, args.count)
    } else if args.arrival_carry {
        arrival::carry::study(&args.output)
    } else if args.arrival_cost {
        arrival::cost(&args.output)
    } else if args.arrival {
        arrival::campaign(&args.output, args.first, args.count)
    } else if args.arrival_summary {
        arrival::summary(&args.output)
    } else if let Some(i) = args.arrival_replay {
        arrival::replay(&args.output, i)
    } else if args.summarize {
        census_report::summarize(&args.output)
    } else if let Some(i) = args.replay {
        census_report::replay(&args.output, i)
    } else if args.census {
        census(&args.output, args.first, args.count)
    } else {
        study(&args.output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn permutation_covers_all_physical_grid_indices() {
        let unique = (0..FIELD)
            .map(|i| i * 13007 % FIELD)
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(unique.len(), FIELD);
    }
}
