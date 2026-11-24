// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Model sensitivity after a finite moving edge lands flat on the deck.
//! Integrals use material arclength on the undeformed edge in the translating
//! deck frame. The incoming rigid donor has no initial layer precompression.
use super::{Result, array, num, read, write};
use glam::{DQuat, DVec3};
use gz_physics::ElasticLineFoundation;
use serde_json::{Value, json};
use std::{fs, path::Path};

#[path = "corner_preload.rs"]
pub(super) mod corner_preload;
#[path = "free_landing.rs"]
pub(super) mod free_landing;

const GRAVITY: DVec3 = DVec3::new(0., 0., -9.81);
const MASS: f64 = 2.;
const INERTIA: f64 = 0.05;
const MU: f64 = 0.5;
const CLOCK_S: f64 = 0.0005;

fn vector(v: &Value) -> Result<DVec3> {
    let p = DVec3::from_array(serde_json::from_value(v.clone())?);
    p.is_finite()
        .then_some(p)
        .ok_or_else(|| "invalid reference vector".into())
}

#[derive(Clone, Copy)]
struct State {
    center: DVec3,
    orientation: DQuat,
    velocity: DVec3,
    spin: DVec3,
}

impl State {
    fn kinetic(self) -> f64 {
        0.5 * (MASS * self.velocity.length_squared() + INERTIA * self.spin.length_squared())
    }

    fn advance(self, stage: Self, load: &Load, dt: f64) -> Self {
        self.advance_with_gravity(stage, load, dt, GRAVITY)
    }

    fn advance_with_gravity(self, stage: Self, load: &Load, dt: f64, gravity: DVec3) -> Self {
        Self {
            center: self.center + dt * stage.velocity,
            orientation: (DQuat::from_scaled_axis(dt * stage.spin) * self.orientation).normalize(),
            velocity: self.velocity + dt * (gravity + load.force / MASS),
            spin: self.spin + dt * load.torque / INERTIA,
        }
    }
}

#[derive(Clone, Copy)]
enum Layers {
    Uniform,
    StifferDeck,
    GradedFoot,
    SwappedStifferDeck,
}
impl Layers {
    fn name(self) -> &'static str {
        match self {
            Self::Uniform => "Uniform layers",
            Self::StifferDeck => "Stiffer deck",
            Self::GradedFoot => "Graded foot",
            Self::SwappedStifferDeck => "Swapped layers control",
        }
    }

    // Fraction along the complete material edge (before clipping to the deck).
    fn layers(self) -> ([f64; 2], f64) {
        match self {
            Self::Uniform => ([2e7, 2e7], 2e7),
            Self::StifferDeck => ([2e7, 2e7], 8e7),
            Self::GradedFoot => ([1e7, 3e7], 2e7),
            Self::SwappedStifferDeck => ([8e7, 8e7], 2e7),
        }
    }

    fn foundation(self, material_fraction: f64) -> ElasticLineFoundation {
        let ([a, b], deck) = self.layers();
        ElasticLineFoundation::new(a + (b - a) * material_fraction, deck)
            .expect("positive finite layers")
    }
}

struct Geometry {
    edge_local: [DVec3; 2], // Relative to assembly center of mass.
    other_vertices: Vec<DVec3>,
    foot_local: DVec3,
    deck_center: DVec3,
    deck_half: DVec3,
    height: f64,
}

struct Segment {
    start_arm: DVec3,
    delta: DVec3,
    lo: f64,
    hi: f64,
    lo_rate: f64,
    hi_rate: f64,
}

impl Geometry {
    fn segment(&self, state: State) -> Result<Segment> {
        let [a, b] = self.edge_local.map(|v| state.orientation * v);
        let delta = b - a;
        let (mut lo, mut hi, mut lo_rate, mut hi_rate) = (0., 1., 0., 0.);
        // Clip the material interval to the two deck side slabs. Rates account
        // for the advected spring energy at a moving overlap boundary.
        for axis in 0..2 {
            let origin = (state.center + a - self.deck_center)[axis];
            let extent = self.deck_half[axis];
            if delta[axis].abs() < 1e-14 {
                if origin.abs() > extent {
                    return Err("edge left deck footprint".into());
                }
                continue;
            }
            let limits = [-extent, extent].map(|limit| (limit - origin) / delta[axis]);
            let rate =
                |s: f64| -(state.velocity + state.spin.cross(a + s * delta))[axis] / delta[axis];
            let lower = limits[0].min(limits[1]);
            let upper = limits[0].max(limits[1]);
            if lower > lo {
                lo = lower;
                lo_rate = rate(lo);
            }
            if upper < hi {
                hi = upper;
                hi_rate = rate(hi);
            }
        }
        if lo >= hi {
            return Err("no finite edge above deck".into());
        }
        Ok(Segment {
            start_arm: a,
            delta,
            lo,
            hi,
            lo_rate,
            hi_rate,
        })
    }

    fn from_reference(doc: &Value, donor: &Value, line: &Value, state: State) -> Result<Self> {
        let endpoints = [
            vector(&line["line_points_m"][0])?,
            vector(&line["line_points_m"][1])?,
        ];
        let half = vector(&doc["geometry"]["foot_half_extents"])?;
        let reverse = state.orientation.conjugate();
        let foot = vector(&donor["foot"])?;
        let foot_local = reverse * (foot - state.center);
        let local_axis = reverse * (endpoints[1] - endpoints[0]).normalize();
        let axis = (0..3)
            .find(|&i| local_axis[i].abs() > 1. - 1e-10)
            .ok_or("line is not a material edge")?;
        let mut edge = reverse * (endpoints[1] - foot);
        for i in (0..3).filter(|i| *i != axis) {
            if (edge[i].abs() - half[i]).abs() > 1e-9 {
                return Err("line misses the material edge".into());
            }
            edge[i] = half[i].copysign(edge[i]);
        }
        let edge_local = [-1., 1.].map(|sign| {
            let mut p = edge;
            p[axis] = sign * half[axis].copysign(local_axis[axis]);
            p + foot_local
        });
        let mut other_vertices = Vec::new();
        for ix in [-1., 1.] {
            for iy in [-1., 1.] {
                for iz in [-1., 1.] {
                    let p = DVec3::new(ix, iy, iz) * half + foot_local;
                    if edge_local.iter().all(|s| p.distance(*s) > 1e-10) {
                        other_vertices.push(p);
                    }
                }
            }
        }
        let pad = &doc["frames"][0]["pad"];
        let pad_q: [f64; 4] = serde_json::from_value(pad["orientation"].clone())?;
        if DQuat::from_array(pad_q).dot(DQuat::IDENTITY).abs() < 1. - 1e-12
            || vector(&line["normal"])?.distance(DVec3::Z) > 1e-12
            || vector(&doc["drive"])?.truncate().length() > 1e-12
        {
            return Err(
                "experiment requires an unrotated horizontal deck with vertical drive".into(),
            );
        }
        let geometry = Self {
            edge_local,
            other_vertices,
            foot_local,
            deck_center: vector(&pad["position"])?,
            deck_half: vector(&doc["geometry"]["pad_half_extents"])?,
            height: endpoints[0].z,
        };
        let s = geometry.segment(state)?;
        for (u, p) in [s.lo, s.hi].into_iter().zip(endpoints) {
            if p.distance(state.center + s.start_arm + u * s.delta) > 1e-9 {
                return Err("clipped edge disagrees with incoming donor".into());
            }
        }
        Ok(geometry)
    }
}

#[derive(Default)]
struct Load {
    force: DVec3,
    torque: DVec3,
    elastic_energy: f64,
    friction_power: f64,
    boundary_power: f64,
    min_slip: Option<f64>,
    max_compression: f64,
    end_overlap: [f64; 2],
    min_other_gap: f64,
    center_of_pressure: f64,
    samples: Vec<Value>,
}

fn load(
    state: State,
    geometry: &Geometry,
    layers: Layers,
    cells: usize,
    display: bool,
) -> Result<Load> {
    sample_load(state, geometry, layers, cells, display, false)
}

fn sample_load(
    state: State,
    geometry: &Geometry,
    layers: Layers,
    cells: usize,
    display: bool,
    resolve_loaded_region: bool,
) -> Result<Load> {
    let seg = geometry.segment(state)?;
    let length = seg.delta.length();
    let site = |s: f64| state.center + seg.start_arm + s * seg.delta;
    let energy_at = |s: f64| {
        layers
            .foundation(s)
            .sample(geometry.height - site(s).z)
            .expect("finite geometry")
            .energy_j_m
    };
    let min_other_gap = geometry
        .other_vertices
        .iter()
        .map(|v| (state.center + state.orientation * *v).z - geometry.height)
        .fold(f64::INFINITY, f64::min);
    if min_other_gap <= 0. {
        return Err("another foot feature would load the deck".into());
    }
    let mut out = Load {
        boundary_power: length
            * (energy_at(seg.hi) * seg.hi_rate - energy_at(seg.lo) * seg.lo_rate),
        min_other_gap,
        ..Default::default()
    };
    // Tangent velocity is affine along the material line. Check its minimum
    // over the entire clipped interval, including between quadrature sites.
    let [a, b] = [seg.lo, seg.hi].map(|u| geometry.height - site(u).z);
    out.end_overlap = [a, b];
    let max_overlap = a.max(b).max(0.);
    out.max_compression = max_overlap;
    let (active_lo, active_hi) = if resolve_loaded_region {
        match (a > 0., b > 0.) {
            (false, false) => return Ok(out),
            (true, true) => (seg.lo, seg.hi),
            _ => {
                // Signed overlap is affine in the material coordinate. Split at
                // its zero so that even a sub-cell strip carries its proper load.
                let root = (seg.lo + (seg.hi - seg.lo) * (a / (a - b))).clamp(seg.lo, seg.hi);
                if a > 0. {
                    (seg.lo, root)
                } else {
                    (root, seg.hi)
                }
            }
        }
    } else {
        (seg.lo, seg.hi)
    };
    let width = (active_hi - active_lo) * length / cells as f64;
    if max_overlap > 0. {
        let slip_origin =
            (state.velocity + state.spin.cross(seg.start_arm)).reject_from_normalized(DVec3::Z);
        let slip_delta = state.spin.cross(seg.delta).reject_from_normalized(DVec3::Z);
        let nearest = if slip_delta.length_squared() > 0. {
            (-slip_origin.dot(slip_delta) / slip_delta.length_squared()).clamp(active_lo, active_hi)
        } else {
            active_lo
        };
        let speed = (slip_origin + nearest * slip_delta).length();
        if speed < 1e-8 {
            return Err("sliding law reached a sticking point".into());
        }
        out.min_slip = Some(speed);
    }
    for i in 0..cells {
        let cell_fraction = (i as f64 + 0.5) / cells as f64;
        let u = active_lo + (active_hi - active_lo) * cell_fraction;
        let fraction = if resolve_loaded_region {
            (u - seg.lo) / (seg.hi - seg.lo)
        } else {
            cell_fraction
        };
        let p = site(u);
        let extension = geometry.height - p.z;
        let response = layers
            .foundation(u)
            .sample(extension)
            .ok_or("nonfinite spring")?;
        let normal = width * response.line_load_n_m;
        let r = seg.start_arm + u * seg.delta;
        let slip = (state.velocity + state.spin.cross(r)).reject_from_normalized(DVec3::Z);
        let shear = -MU * normal * slip.normalize_or_zero();
        let force = normal * DVec3::Z + shear;
        out.force += force;
        out.torque += r.cross(force);
        out.elastic_energy += width * response.energy_j_m;
        out.friction_power += shear.dot(slip);
        out.center_of_pressure += fraction * normal;
        if display {
            out.samples.push(json!({"fraction":fraction,"material_fraction":u,"weight_m":width,"site_m":p.to_array(),"line_load_n_m":response.line_load_n_m,"foot_compression_m":response.foot_compression_m,"deck_compression_m":response.deck_compression_m}));
        }
    }
    Ok(out)
}

struct Run {
    frames: Vec<Value>,
    com: Vec<DVec3>,
    impulses: Vec<DVec3>,
    loads: Vec<f64>,
    checks: Value,
}

#[allow(clippy::too_many_arguments)]
fn integrate(
    initial: State,
    geometry: &Geometry,
    layers: Layers,
    start: f64,
    horizon: f64,
    cells: usize,
    substeps: u32,
    display: bool,
) -> Result<Run> {
    let first_clock = (start / CLOCK_S).floor() as usize;
    let clocks = std::iter::once(start)
        .chain(
            (first_clock + 1..=(horizon / CLOCK_S).round() as usize)
                .map(|tick| tick as f64 * CLOCK_S),
        )
        .collect::<Vec<_>>();
    integrate_at(
        initial, geometry, layers, &clocks, cells, substeps, display, false,
    )
}

#[allow(clippy::too_many_arguments)]
fn integrate_at(
    initial: State,
    geometry: &Geometry,
    layers: Layers,
    clocks: &[f64],
    cells: usize,
    substeps: u32,
    display: bool,
    resolve_loaded_region: bool,
) -> Result<Run> {
    let start = *clocks.first().ok_or("missing integration clocks")?;
    if clocks.windows(2).any(|p| p[1] <= p[0]) {
        return Err("unordered integration clocks".into());
    }
    let loads_at = |s, display| {
        if resolve_loaded_region {
            sample_load(s, geometry, layers, cells, display, true)
        } else {
            load(s, geometry, layers, cells, display)
        }
    };
    let (mut s, mut now) = (initial, start);
    let (mut impulses, mut com, mut loads, mut frames) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let (mut j, mut torque, mut friction_work, mut edge_work) = (DVec3::ZERO, DVec3::ZERO, 0., 0.);
    let (mut peak_force, mut max_compression, mut min_slip, mut min_other_gap, mut worst_energy) =
        (0f64, 0f64, f64::INFINITY, f64::INFINITY, 0f64);
    let initial_energy = loads_at(s, false)?.elastic_energy;
    let mut switches = corner_preload::EdgeLoadHistory::default();
    for (index, &goal) in clocks.iter().enumerate() {
        if index > 0 {
            let dt = (goal - now) / f64::from(substeps);
            for step in 0..substeps {
                let a = loads_at(s, false)?;
                if resolve_loaded_region {
                    switches.observe(now + f64::from(step) * dt, a.end_overlap, s);
                }
                let mid = s.advance(s, &a, dt * 0.5);
                let b = loads_at(mid, false)?;
                s = s.advance(mid, &b, dt);
                j += dt * b.force;
                torque += dt * b.torque;
                friction_work += dt * b.friction_power;
                edge_work += dt * b.boundary_power;
                peak_force = peak_force.max(b.force.z);
                max_compression = max_compression.max(b.max_compression);
                if let Some(v) = b.min_slip {
                    min_slip = min_slip.min(v);
                }
                min_other_gap = min_other_gap.min(b.min_other_gap);
            }
            now = goal;
        }
        let l = loads_at(s, display)?;
        if resolve_loaded_region {
            switches.observe(now, l.end_overlap, s);
        }
        let energy = s.kinetic() - initial.kinetic() + l.elastic_energy
            - initial_energy
            - MASS * GRAVITY.dot(s.center - initial.center)
            - friction_work
            - edge_work;
        worst_energy = worst_energy.max(energy.abs());
        if display {
            frames.push(json!({"original_time_s":now,"com_m":s.center.to_array(),"foot_m":(s.center+s.orientation*geometry.foot_local).to_array(),"orientation_xyzw":s.orientation.to_array(),"v_m_s":s.velocity.to_array(),"spin_rad_s":s.spin.to_array(),"support_frame_j_ns":j.to_array(),"normal_force_n":l.force.z,"strain_energy_j":l.elastic_energy,"center_of_pressure_fraction":if l.force.z>0.{Some(l.center_of_pressure/l.force.z)}else{None},"pressure":l.samples}));
            if resolve_loaded_region {
                let seg = geometry.segment(s)?;
                let points = [seg.lo, seg.hi].map(|u| s.center + seg.start_arm + u * seg.delta);
                let row = frames.last_mut().expect("just pushed frame");
                row["edge_endpoints_m"] = json!(points.map(|p| p.to_array()));
                row["endpoint_overlaps_m"] = json!(points.map(|p| geometry.height - p.z));
                row["clipped_material_fractions"] = json!([seg.lo, seg.hi]);
            }
        }
        com.push(s.center);
        impulses.push(j);
        loads.push(l.force.z);
    }
    let linear = j.distance(MASS * (s.velocity - initial.velocity - (now - start) * GRAVITY));
    let angular = torque.distance(INERTIA * (s.spin - initial.spin));
    if linear > 1e-11
        || angular > 1e-11
        || !worst_energy.is_finite()
        || friction_work >= 0.
        || !min_slip.is_finite()
    {
        return Err("compliant integration failed its force, torque or dissipation checks".into());
    }
    let mut result = Run {
        frames,
        com,
        impulses,
        loads,
        checks: json!({"cells":cells,"substeps_per_clock":substeps,"peak_force_n":peak_force,"max_compression_m":max_compression,"min_loaded_slip_m_s":min_slip,"min_other_vertex_gap_m":min_other_gap,"max_frame_energy_balance_j":worst_energy,"friction_work_j":friction_work,"advected_boundary_energy_j":edge_work,"linear_balance_ns":linear,"angular_balance_nms":angular}),
    };
    if resolve_loaded_region {
        result.checks["edge_contact_history"] = switches.finish(geometry, s, now)?;
    }
    Ok(result)
}

fn differences(a: &Run, b: &Run) -> Value {
    let distance = |a: &[DVec3], b: &[DVec3]| {
        a.iter()
            .zip(b)
            .map(|(x, y)| x.distance(*y))
            .fold(0., f64::max)
    };
    json!({"max_com_delta_m":distance(&a.com,&b.com),"max_impulse_delta_ns":distance(&a.impulses,&b.impulses),"max_force_delta_n":a.loads.iter().zip(&b.loads).map(|(a,b)|(a-b).abs()).fold(0.,f64::max)})
}

pub(super) fn study(reference: &Path, output: &Path) -> Result<()> {
    let doc = read(reference)?;
    if doc["case"] != 80 || doc["stroke"] != 2 {
        return Err("the material experiment is parameterized for fixture 80, stroke 2".into());
    }
    let row = array(&doc["references"])?
        .last()
        .ok_or("missing matched donor")?;
    let donor = &row["fine"];
    if donor["end"] != "unproved_line" {
        return Err("requires the state before the line receives an impulse".into());
    }
    let state = State {
        center: vector(&donor["com"])?,
        orientation: DQuat::from_array(serde_json::from_value(donor["orientation"].clone())?),
        velocity: vector(&donor["velocity"])?,
        spin: vector(&donor["omega"])?,
    };
    let geometry = Geometry::from_reference(&doc, donor, &row["unresolved_line"], state)?;
    let start = num(&donor["elapsed"])?;
    let horizon = num(&doc["conditional_pressure_models"]["horizon_s"])?;
    if !(start < horizon && horizon <= 0.0135 + 1e-12) {
        return Err("unsupported horizon".into());
    }
    let mut profiles = Vec::new();
    let mut swap_control = Value::Null;
    for layers in [Layers::Uniform, Layers::StifferDeck, Layers::GradedFoot] {
        let fine = integrate(state, &geometry, layers, start, horizon, 128, 256, true)?;
        let spatial = integrate(state, &geometry, layers, start, horizon, 64, 256, false)?;
        let temporal = integrate(state, &geometry, layers, start, horizon, 128, 128, false)?;
        let coarsest_space = integrate(state, &geometry, layers, start, horizon, 32, 256, false)?;
        let coarsest_time = integrate(state, &geometry, layers, start, horizon, 128, 64, false)?;
        if matches!(layers, Layers::StifferDeck) {
            let swap = integrate(
                state,
                &geometry,
                Layers::SwappedStifferDeck,
                start,
                horizon,
                128,
                256,
                true,
            )?;
            let mut max_swapped_deformation = 0f64;
            let mut max_pressure_delta = 0f64;
            for (direct, reversed) in fine.frames.iter().zip(&swap.frames) {
                for (a, b) in array(&direct["pressure"])?
                    .iter()
                    .zip(array(&reversed["pressure"])?)
                {
                    max_pressure_delta = max_pressure_delta
                        .max((num(&a["line_load_n_m"])? - num(&b["line_load_n_m"])?).abs());
                    for (first, second) in [
                        ("foot_compression_m", "deck_compression_m"),
                        ("deck_compression_m", "foot_compression_m"),
                    ] {
                        max_swapped_deformation =
                            max_swapped_deformation.max((num(&a[first])? - num(&b[second])?).abs());
                    }
                }
            }
            if max_swapped_deformation > 1e-15 || max_pressure_delta > 1e-7 {
                return Err("swapped layers do not reproduce series response".into());
            }
            swap_control = json!({"direct":"Stiffer deck","swapped_foot_n_m2":8e7,"swapped_deck_n_m2":2e7,"max_swapped_deformation_difference_m":max_swapped_deformation,"max_line_load_difference_n_m":max_pressure_delta,"trajectory":differences(&fine,&swap)});
        }
        let space_change = differences(&spatial, &fine);
        let time_change = differences(&temporal, &fine);
        if num(&space_change["max_com_delta_m"])? > 5e-9
            || num(&time_change["max_com_delta_m"])? > 1e-9
            || num(&space_change["max_impulse_delta_ns"])? > 5e-6
            || num(&time_change["max_impulse_delta_ns"])? > 1e-6
            || num(&space_change["max_force_delta_n"])? > 0.002
            || num(&time_change["max_force_delta_n"])? > 0.002
            || num(&fine.checks["max_frame_energy_balance_j"])? > 1e-9
        {
            return Err(format!(
                "{} failed fixture 80 mesh or energy agreement",
                layers.name()
            )
            .into());
        }
        let (foot, deck) = layers.layers();
        profiles.push(json!({"name":layers.name(),"foot_endpoint_stiffness_n_m2":foot,"deck_stiffness_n_m2":deck,
            "spatial_refinement":space_change,"time_refinement":time_change,
            "coarsest_to_coarse_space":differences(&coarsest_space,&spatial),"coarsest_to_coarse_time":differences(&coarsest_time,&temporal),"coarse_space_checks":spatial.checks,"coarse_time_checks":temporal.checks,"checks":fine.checks,"frames":fine.frames}));
    }
    fs::create_dir_all(output)?;
    write(
        &output.join("reference.json"),
        &json!({"case":80,"stroke":2,"start_s":start,"horizon_s":horizon,"normal":[0.,0.,1.],"public_step_s":CLOCK_S,"mass_kg":MASS,"inertia_kg_m2":INERTIA,"sliding_friction":MU,"coordinate_frame":"original support-translating frame","initial_condition":"uncompressed at rigid line arrival; no impulsive velocity reset","material_model":"independent undamped linear layer pairs, force per metre of undeformed material edge; local sliding Coulomb friction","calibration":"prescribed sensitivity inputs; neither stiffness nor pre-impact strain has been measured","swapped_layers_control":swap_control,"geometry_scope":"A compliant ridge on the selected edge only. The adjoining sole area has no contact width in this model.","initial_preload":"Prior rigid corner load is not carried as elastic strain. This is a model launch from a matched incoming pose and velocity.","profiles":profiles}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn static_ridge(za: f64, zb: f64, deck_half: f64) -> (State, Geometry) {
        (
            State {
                center: DVec3::ZERO,
                orientation: DQuat::IDENTITY,
                velocity: DVec3::X,
                spin: DVec3::ZERO,
            },
            Geometry {
                edge_local: [DVec3::new(-1., 0., za), DVec3::new(1., 0., zb)],
                other_vertices: vec![DVec3::Z],
                foot_local: DVec3::ZERO,
                deck_center: DVec3::ZERO,
                deck_half: DVec3::splat(deck_half),
                height: 0.,
            },
        )
    }

    #[test]
    fn flat_clipped_ridge_load_integrates_over_the_deck_only_and_releases() -> Result<()> {
        let overlap = 1e-4;
        let (mut state, g) = static_ridge(-overlap, -overlap, 0.5);
        let loaded = load(state, &g, Layers::Uniform, 64, true)?;
        let expected_force = 1e7 * overlap;
        assert!((loaded.force.z - expected_force).abs() < 1e-10);
        assert!((loaded.force.x + MU * expected_force).abs() < 1e-10);
        assert!((loaded.elastic_energy - 0.5 * expected_force * overlap).abs() < 1e-14);
        assert!(
            loaded
                .samples
                .iter()
                .all(|p| p["site_m"][0].as_f64().unwrap().abs() < 0.5)
        );
        state.center.z = 2. * overlap;
        let open = load(state, &g, Layers::Uniform, 64, false)?;
        assert_eq!(open.force, DVec3::ZERO);
        assert_eq!(open.elastic_energy, 0.);
        Ok(())
    }

    #[test]
    fn tilted_unilateral_ridge_recovers_triangular_load_and_center_of_pressure() -> Result<()> {
        let overlap = 1e-4;
        let (state, g) = static_ridge(-overlap, overlap, 2.);
        let loaded = load(state, &g, Layers::Uniform, 128, true)?;
        let length = g.edge_local[0].distance(g.edge_local[1]);
        assert!((loaded.force.z - 1e7 * overlap * length / 4.).abs() < 1e-10);
        assert!((loaded.center_of_pressure / loaded.force.z - 1. / 6.).abs() < 3e-5);
        assert!(
            loaded.samples[64..]
                .iter()
                .all(|p| p["line_load_n_m"] == 0.)
        );
        Ok(())
    }

    #[test]
    fn corner_narrower_than_a_quadrature_cell_carries_force_on_loaded_region() -> Result<()> {
        let (mut state, g) = static_ridge(0.9999, -0.0001, 2.);
        // The zero-slip site is at the midpoint, where the ridge is open.
        state.velocity = DVec3::ZERO;
        state.spin = DVec3::Z;
        let l = sample_load(state, &g, Layers::Uniform, 8, true, true)?;
        let length = g.edge_local[0].distance(g.edge_local[1]);
        let expected = 0.5 * 1e7 * 1e-4 * 1e-4 * length;
        assert!((l.force.z - expected).abs() < 1e-12);
        assert!(
            l.samples
                .iter()
                .all(|p| p["fraction"].as_f64().unwrap() > 0.9999)
        );
        assert!(l.min_slip.unwrap() > 0.999);
        Ok(())
    }

    #[test]
    fn slip_that_vanishes_between_pressure_samples_is_excluded() {
        let (mut state, g) = static_ridge(-1e-4, -1e-4, 2.);
        state.velocity = DVec3::ZERO;
        state.spin = DVec3::Z;
        assert!(load(state, &g, Layers::Uniform, 64, false).is_err());
    }
}
