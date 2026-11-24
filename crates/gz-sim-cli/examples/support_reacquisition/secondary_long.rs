// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Propagate alternative second impacts from matched cold physical cuts in the
//! laboratory frame. All later actuator commands land on the original grid.
use super::super::finite::load::transfer::second::{LaboratoryCut, laboratory};
use super::*;
use gz_physics::{BodyId, PhysicsBackend};
#[path = "secondary_interval_view.rs"]
pub(crate) mod interval_view;
#[path = "physical_secondary_long.rs"]
pub(crate) mod physical;
#[path = "secondary_landing_policy.rs"]
pub(crate) mod policy;
#[path="bound_point_cost.rs"]
pub(crate) mod cost;
#[path="contact_bin_observer.rs"]
pub(super) mod bins;
#[path="internal_transfer_long.rs"]
pub(crate) mod long_transfer;
const FORMAT: &str = "cold-secondary-three-strokes-v1";
const MODES: [(Profile, bool); 6] = [
    (p("native8_then512", b(8, 1), b(512, 1)), false),
    (p("endpoint8_then512", b(8, 1), b(512, 1)), true),
    (p("native8r8_then512", b(8, 8), b(512, 1)), false),
    (p("native4096_then512", b(4096, 1), b(512, 1)), false),
    (p("continuous4096", b(4096, 1), b(4096, 1)), false),
    (p("continuous8192", b(8192, 1), b(8192, 1)), false),
];
fn source(k: usize) -> usize {
    let j = 17 * k % 50;
    let target = 10 * (j / 2) + 4 + j % 2;
    (0..250).find(|v| v * 117 % 250 == target).unwrap()
}
struct Branch {
    initial: Value,
    ticks: Vec<Tick>,
    impulse: Value,
    first_world_impulse: DVec3,
    vertices: Vec<Value>,
    transfers:Vec<Value>,
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum Action {
    Native,
    Endpoint,
    Retime,
    Follow,
    FollowRetimed,
    Stick,
    StickRetimed,
    Both,
    BothRetimed,
    Transfer,
    TransferRetimed,
    Spatial,
    SpatialRetimed,
    Launch,
    LaunchRetimed,
}
fn path(
    f: Fixture,
    original: &ReboundFrame,
    donor: &LaboratoryCut,
    mode: Profile,
    action: Action,
    until: u64,
) -> Result<Branch> {
    path_with_samples(f, original, donor, mode, action, until, None)
}
fn path_with_samples(
    f: Fixture,
    original: &ReboundFrame,
    donor: &LaboratoryCut,
    mode: Profile,
    action: Action,
    until: u64,
    samples: Option<&mut Vec<Value>>,
) -> Result<Branch> {
    if until <= original.tick || until > DURATION {
        return Err("invalid post-impact horizon".into());
    }
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        &donor.xml,
        "secondary-laboratory.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let body = |name: &str| {
        sim.world()
            .iter_entities()
            .find_map(|e| {
                if e.get::<Name>()?.0 == name {
                    Some(e.get::<BodyHandle>()?.id())
                } else {
                    None
                }
            })
            .ok_or("missing laboratory body")
    };
    let (foot, upper, pad) = (
        body("default::rocker::foot")?,
        body("default::rocker::upper")?,
        body("default::lift::pad")?,
    );
    let mut res = sim.world_mut().resource_mut::<PhysicsBackend>();
    let native = res
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("requires native backend")?;
    let state = |id: BodyId| native.state(id).ok_or("missing laboratory link");
    let (a, u, d) = (state(foot)?, state(upper)?, state(pad)?);
    let center = 0.5 * (DVec3::from(a.position) + DVec3::from(u.position));
    let velocity = 0.5 * (DVec3::from(a.velocity) + DVec3::from(u.velocity));
    if center.distance(donor.end.center) > 1e-12
        || DVec3::from(a.position).distance(donor.end.moving.center) > 1e-12
        || velocity.distance(donor.end.velocity) > 1e-12
        || DVec3::from(a.angular_velocity).distance(donor.end.angular_velocity) > 1e-12
        || DVec3::from(d.position).distance(donor.deck.center) > 1e-12
        || DVec3::from(d.velocity).distance(DVec3::Z * command(f, original.tick)) > 1e-12
    {
        return Err("the laboratory secondary reload changed a measured state".into());
    }
    let initial = json!({"tick":original.tick,"time_us":original.tick*STEP_US+(donor.clock_offset.as_nanos()/1000)as u64,"time_ns":original.tick*STEP_US*1000+donor.clock_offset.as_nanos() as u64,
        "foot":Body::from(a),"upper":Body::from(u),"pad":Body::from(d),"native_steps":0,"normal_impulse_ns":0.});
    let h = DT - donor.clock_offset;
    let mut speed = command(f, original.tick);
    let pair = [(foot, pad)];
    let params = ContactSolverIterations {
        temporal: 4,
        pgs: None,
        relaxation: Some(mode.onset.r),
    };
    let actions=gz_physics_multifidelity::BoxContactActions {
                    finite_segment_impacts: &pair,
                    retime_finite_transfer: if matches!(action,Action::Retime|Action::FollowRetimed|Action::StickRetimed|Action::BothRetimed|Action::TransferRetimed|Action::SpatialRetimed|Action::LaunchRetimed) { &pair } else { &[] },
                    certify_segment_friction_work: &pair,
                    sticking_points: if matches!(action,Action::Stick|Action::StickRetimed|Action::Both|Action::BothRetimed){&pair}else{&[]},
                    sliding_skew_edges: if matches!(action,Action::Spatial|Action::SpatialRetimed|Action::Launch|Action::LaunchRetimed){&pair}else{&[]},
                    slip_launches: if matches!(action,Action::Launch|Action::LaunchRetimed){&pair}else{&[]},
                    point_transfers: if matches!(action,Action::Transfer|Action::TransferRetimed|Action::Spatial|Action::SpatialRetimed|Action::Launch|Action::LaunchRetimed){&pair}else{&[]},
                    sliding_vertices: if matches!(action,Action::Follow|Action::FollowRetimed|Action::Both|Action::BothRetimed){&pair}else{&[]},
        ..Default::default()
    };
    let (receipt,observed_sum)=if let Some(rows)=samples {
        if !matches!(action,Action::Native|Action::Endpoint|Action::Both|Action::Transfer|Action::Spatial|Action::Launch){return Err("retimed observation bins require a common native clock".into())}
        let (r,j)=bins::run(native,gz_physics_multifidelity::ContactSegment{duration:h,native_steps:mode.onset.n,iterations:params},[foot,upper,pad],&initial,(action!=Action::Native).then_some(actions),rows)?;
        (r,Some(j))
    }else if action==Action::Native{
        (native.step_contact_interval(h,mode.onset.n,params)?,None)
    }else{
        (native.advance_box_contacts(h,mode.onset.n,params,actions,None)?.interval,None)
    };
    if receipt.native_steps != mode.onset.n {
        return Err("native remainder call budget mismatch".into());
    }
    let mut vertices=Vec::with_capacity(receipt.sliding_vertices.len());
    for s in receipt.sliding_vertices {
        if (s.foot,s.support)!=(foot,pad) || s.start+s.duration>h || s.duration.is_zero() {
            return Err("sliding vertex receipt outside registered interval".into());
        }
        vertices.push(json!({"start_ns":s.start.as_nanos() as u64,"duration_ns":s.duration.as_nanos() as u64,"normal":s.normal,
        "from_m":s.point_start,"to_m":s.point_end,"impulse_ns":s.impulse_on_foot,"angular_impulse_nms":s.angular_impulse_about_com,
        "friction_work_j":s.friction_work,"minimum_slip_m_s":s.minimum_slip_speed,"minimum_inward_m_s2":s.minimum_inward_acceleration,"subdivisions":s.subdivisions}));
    }
    let growth=|g:gz_physics::SlipGrowth,final_velocity:[f64;3]|json!({"direction":g.direction.to_array(),"end_material_velocity_m_s":final_velocity,"initial_m_s":g.initial_component,"acceleration_m_s2":g.acceleration});
    for s in receipt.sliding_skew_edges {
        if (s.foot,s.support)!=(foot,pad) || s.start+s.duration>h || s.duration.is_zero() {
            return Err("skew sliding receipt outside registered interval".into());
        }
        vertices.push(json!({"start_ns":s.start.as_nanos() as u64,"duration_ns":s.duration.as_nanos() as u64,"normal":s.normal,"sliding_edge":true,"feature":format!("{:?}",s.feature),
        "normal_impulse_ns":s.normal_impulse,"normal_impulse_vector_ns":s.normal_impulse_on_foot,
        "from_m":s.point_start,"to_m":s.point_end,"impulse_ns":s.impulse_on_foot,"angular_impulse_nms":s.angular_impulse_about_com,
        "friction_work_j":s.friction_work,"minimum_slip_m_s":s.minimum_slip_speed,"minimum_inward_m_s2":s.minimum_inward_acceleration,"subdivisions":s.subdivisions}));
        if let Some(g)=s.slip_growth { vertices.last_mut().unwrap()["slip_growth"]=growth(g,s.end_material_velocity); }
    }
    for s in receipt.sticking_points {
        if (s.foot,s.support)!=(foot,pad) || s.start+s.duration>h || s.duration.is_zero() {
            return Err("sticking-point span outside registered interval".into());
        }
        vertices.push(json!({"start_ns":s.start.as_nanos() as u64,"duration_ns":s.duration.as_nanos() as u64,"normal":s.normal_start,
        "normal_end":s.normal_end,"from_m":s.point_start,"to_m":s.point_end,"impulse_ns":s.impulse_on_foot,"angular_impulse_nms":s.angular_impulse_about_com,
        "normal_impulse_ns":s.normal_impulse,"normal_impulse_vector_ns":s.normal_impulse_on_foot,"feature":format!("{:?}",s.feature),
        "minimum_normal_n":s.minimum_normal_force,"minimum_cone_margin_n":s.minimum_cone_margin,"subdivisions":s.subdivisions}));
    }
    vertices.sort_by_key(|v|v["start_ns"].as_u64().unwrap());
    let mut transfers=vec![];
    for s in receipt.point_transfers {
        if (s.foot,s.support)!=(foot,pad) || s.start+s.duration>h || s.duration.is_zero() {
            return Err("internal transfer outside registered interval".into());
        }
        let pieces=s.intervals.iter().map(|a|{let mut out=json!({"offset_s":a.offset,"duration_s":a.duration,"feature":format!("{:?}",a.feature),"sticking":a.sticking,
            "j_ns":a.impulse_on_foot,"normal_ns":a.normal_impulse,"normal_vector_ns":a.normal_impulse_on_foot,"angular_nms":a.angular_impulse_about_com,"work_j":a.friction_work,
            "impact":a.impact.map(|e|json!({"bracket_s":e.local_bracket,"point_m":e.point,"normal":e.normal,"j_ns":e.impulse_on_foot,"angular_nms":e.angular_impulse_about_com,
            "opening_m_s":e.opposite_opening_speed,"stick":e.sticking,"lost_j":e.lost_energy,"work_boxes":e.work_boxes}))});if let Some(g)=a.slip_growth{out["slip_growth"]=growth(g,a.end_material_velocity);}out}).collect::<Vec<_>>();
        transfers.push(json!({"start_ns":s.start.as_nanos()as u64,"duration_ns":s.duration.as_nanos()as u64,"j_ns":s.impulse_on_foot,"normal_ns":s.normal_impulse,
        "normal_vector_ns":s.normal_impulse_on_foot,"pieces":pieces}));
    }
    let mut impulse = Value::Null;
    for hit in receipt.finite_point_impacts {
        let h = hit.resultant;
        let cert = hit.segment.ok_or("wrong impact type")?;
        if h.elapsed != Duration::ZERO || !impulse.is_null() {
            return Err("uncertified second onset".into());
        }
        impulse = json!({"j_ns":h.impulse_on_foot,"point_m":h.point,"normal":h.normal,"work_boxes":cert.work_boxes,"stick":h.sticking,"opposite_after":cert.opposite_after,"energy_j":h.dissipated_energy});
        if let Some((limit, dt)) = hit.next_feature_boundary {
            impulse["retimed"] = json!({"limit":format!("{limit:?}"),"dt_ns":dt.as_nanos() as u64});
        }
    }
    let mut first_world_impulse = DVec3::ZERO;
    let mut normal = 0.;
    let mut tangent = DVec3::ZERO;
    let mut first = None;
    let mut extra = 0.;
    for z in receipt.pairs {
        if (z.a, z.b) == (foot.min(pad), foot.max(pad)) {
            normal += z.normal_impulse;
            first_world_impulse += if foot == z.a { 1. } else { -1. }
                * (DVec3::from(z.normal_impulse_on_a)
                    + DVec3::from(
                        z.tangent_impulse_on_a
                            .ok_or("unavailable signed reaction")?,
                    ));
            tangent += if z.a == foot { 1. } else { -1. }
                * DVec3::from(z.tangent_impulse_on_a.ok_or("missing public friction")?);
            first = z.first_positive_end.map(|dt| dt + donor.clock_offset);
        } else {
            extra += z.normal_impulse
        }
    }
    if observed_sum.is_some_and(|j| j.distance(first_world_impulse) > 2e-10) {
        return Err("native observed sites do not reproduce signed interval impulse".into());
    }
    let foot_state = native.state(foot).ok_or("foot lost")?;
    let pad_state = native.state(pad).ok_or("pad lost")?;
    let mut ticks = vec![Tick {
        foot: Body::from(foot_state),
        upper: Body::from(native.state(upper).ok_or("upper lost")?),
        pad: Body::from(pad_state),
        tick: original.tick + 1,
        normal,
        extra,
        tangent,
        first,
        gap: supported_gap(foot_state, pad_state),
        calls: receipt.native_steps,
    }];
    let mut unexpected = vec![];
    for tick in original.tick + 2..=until {
        let next = command(f, tick - 1);
        ticks.push(propagate_one(
            native,
            [foot, upper, pad],
            tick,
            mode.suffix,
            (speed != next).then_some(next),
            None,
            &mut unexpected,
        )?);
        speed = next;
    }
    if !unexpected.is_empty() {
        return Err("unexpected later projection".into());
    }
    Ok(Branch {
        initial,
        ticks,
        impulse,
        first_world_impulse,
        vertices,
        transfers,
    })
}
struct Study {
    record: Value,
    branches: Vec<Branch>,
}
fn run(k: usize) -> Result<Study> {
    let f = fixture(source(k));
    let cut = survey::cuts(f)?.0[0].ok_or("no primary landing")?;
    let mut record = json!({"format":FORMAT,"case":k,"source":source(k),"fixture":f,"cut":snapshot(&cut),"cut_tick":cut.tick,"end_tick":DURATION});
    let Some(donor) = laboratory(f, &cut)? else {
        return Ok(Study {
            record,
            branches: vec![],
        });
    };
    record["offset_ns"] = json!(donor.clock_offset.as_nanos() as u64);
    record["geometric_time_s"] = json!(donor.geometric_time_s);
    record["prehistory"] = donor.original.clone();
    let mut branches = vec![];
    for (mode, project) in MODES {
        branches.push(path(
            f,
            &cut,
            &donor,
            mode,
            if project {
                Action::Endpoint
            } else {
                Action::Native
            },
            DURATION,
        )?)
    }
    let mut paths = vec![];
    for (i, r) in branches.iter().enumerate() {
        if r.initial != branches[0].initial {
            return Err("cold branches do not share all incoming states".into());
        }
        let ts = &r.ticks;
        let metrics = [0, 4, 5]
            .map(|ref_i| compare(ts, &branches[ref_i].ticks))
            .into_iter()
            .collect::<Result<Vec<_>>>()?;
        paths.push(json!({"profile":MODES[i].0.label,"native_calls":ts.iter().map(|t|u64::from(t.calls)).sum::<u64>(),"outcomes":events(ts),
            "first":ts[0].as_json(),"end":ts.last().unwrap().as_json(),"impulse":r.impulse,"comparisons":metrics}));
    }
    record["initial"] = branches[0].initial.clone();
    record["paths"] = json!(paths);
    Ok(Study { record, branches })
}
pub(crate) fn survey(dir: &Path, first: usize, count: usize) -> Result<()> {
    let end = first
        .checked_add(count)
        .filter(|n| count > 0 && *n <= 50)
        .ok_or("invalid nominal case range")?;
    fs::create_dir_all(dir.join("cases"))?;
    for k in first..end {
        let file = dir.join("cases").join(format!("case-{k:03}.json"));
        if file.exists() {
            let d: Value = serde_json::from_reader(fs::File::open(&file)?)?;
            if d["case"] != k || d["format"] != FORMAT || d["source"] != source(k) {
                return Err("incomplete secondary record".into());
            }
            continue;
        }
        let started = std::time::Instant::now();
        let study = run(k)?;
        let tmp = file.with_extension(format!("partial-{}", std::process::id()));
        write_json(&tmp, &study.record)?;
        fs::rename(tmp, file)?;
        eprintln!(
            "finished second-impact case {k} in {:.1}s",
            started.elapsed().as_secs_f64()
        );
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize) -> Result<()> {
    let d: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:03}.json")),
    )?)?;
    let study = run(k)?;
    let mut reproduced: Value = serde_json::from_slice(&serde_json::to_vec(&study.record)?)?;
    // Pressure-cell counts describe the proof algorithm. The proof may contract
    // its cells more efficiently across revisions while the committed physics,
    // load-bearing endpoint, affected coordinates and complete history stay exact.
    let before = d["paths"][1]["impulse"]["work_boxes"].as_u64();
    let after = reproduced["paths"][1]["impulse"]["work_boxes"].as_u64();
    if let (Some(old @ 1..=128), Some(1..=128)) = (before, after) {
        reproduced["paths"][1]["impulse"]["work_boxes"] = json!(old);
    }
    if reproduced != d {
        return Err("cold replay differs from archive".into());
    }
    let folder = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(folder.join("traces"))?;
    let mut times = std::collections::BTreeSet::new();
    let cut = study.record["cut_tick"]
        .as_u64()
        .ok_or("missing original clock")?;
    times.extend(cut + 1..=cut + 10);
    times.insert(DURATION);
    for i in [0, 1, 5] {
        let ts = &study.branches.get(i).ok_or("missing landing")?.ticks;
        for stroke in events(ts)["strokes"]
            .as_array()
            .ok_or("missing stroke clocks")?
        {
            if let Some(t) = stroke["first_native_end_ns"].as_u64() {
                let at = t.div_ceil(STEP_US * 1000);
                times.extend(at.saturating_sub(4).max(cut + 1)..=(at + 8).min(DURATION));
            }
        }
    }
    let mut lanes = vec![];
    for (i, label) in [
        (0, "Ordinary second landing then 512"),
        (1, "Certified second landing then 512"),
        (5, "Continuous 8,192 native calls"),
    ] {
        let b = study.branches.get(i).ok_or("no secondary landing")?;
        let mut frames = vec![b.initial.clone()];
        frames.extend(
            b.ticks
                .iter()
                .filter(|v| v.tick % 16 == 0 || times.contains(&v.tick))
                .map(Tick::as_json),
        );
        let id = MODES[i].0.label;
        let file = format!("traces/{id}.json");
        write_json(
            &folder.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"file":file,"label":id,"name":label}));
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"The secondary contact through three actuator strokes","pressure_cells":{"recorded":before,"reproduced":after},"pair_totals_only":true,"sampled_ticks":true,"lanes":lanes}),
    )
}
