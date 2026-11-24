// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Identify the pressure-bearing endpoint of the next incident line when
//! momentum and the Coulomb cone force the other endpoint to open.
#[path = "after_secondary.rs"]
pub(crate) mod after;
#[path = "line_closure_study.rs"]
pub(crate) mod closure;
#[path = "secondary_cost.rs"]
pub(crate) mod cost;
#[path = "sticking_line_study.rs"]
pub(crate) mod line;
#[path = "line_wrench_study.rs"]
pub(crate) mod wrench;
use super::*;
use gz_physics::{FiniteArcEnd, FiniteContactSegment, FiniteFeatureArc, FiniteFeatureTrack};
const SOURCE: &str = include_str!("../../../../examples/multifidelity_finite_primary.sdf");
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer) fn substituted(source: &str, name: &str, value: &str) -> Result<String> {
    let l = format!("<{name}>");
    let r = format!("</{name}>");
    let (a, rest) = source.split_once(&l).ok_or("missing fixture field")?;
    let (_, b) = rest.split_once(&r).ok_or("invalid fixture field")?;
    Ok(format!("{a}{l}{value}{r}{b}"))
}
fn vector(x: DVec3) -> String {
    format!("{:.17e} {:.17e} {:.17e}", x.x, x.y, x.z)
}
fn pose(b: FlightBox) -> String {
    let (yaw, pitch, roll) = b.orientation.to_euler(glam::EulerRot::ZYX);
    format!(
        "{} {}",
        vector(b.center),
        vector(DVec3::new(roll, pitch, yaw))
    )
}
fn sdf(end: gz_physics::FiniteArcResult, support: FlightBox) -> Result<String> {
    boundary_xml(
        end.moving,
        end.velocity,
        end.angular_velocity,
        support,
        DVec3::ZERO,
    )
}
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer) fn boundary_xml(
    moving: FlightBox,
    v: DVec3,
    w: DVec3,
    support: FlightBox,
    deck_speed: DVec3,
) -> Result<String> {
    let (rocker, lift) = SOURCE
        .split_once("<model name=\"lift\">")
        .ok_or("missing deck")?;
    let rocker = substituted(rocker, "pose", &pose(moving))?;
    // The authored model twist is referenced to the aggregate COM.
    let rocker = substituted(&rocker, "linear_velocity", &vector(v))?;
    let rocker = substituted(&rocker, "angular_velocity", &vector(w))?;
    let lift = substituted(lift, "pose", &pose(support))?;
    let lift = substituted(
        &lift,
        "linear_velocity",
        &if deck_speed == DVec3::ZERO {
            "0 0 0".into()
        } else {
            vector(deck_speed)
        },
    )?;
    Ok(format!("{rocker}<model name=\"lift\">{lift}"))
}
fn cold(
    xml: &str,
    h: Duration,
    relax: usize,
    expected: gz_physics::FiniteArcResult,
) -> Result<Value> {
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        xml,
        "transfer.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let id = |name: &str| -> Result<BodyId> {
        sim.world()
            .iter_entities()
            .find_map(|e| {
                (e.get::<Name>()?.0 == name)
                    .then(|| e.get::<BodyHandle>().map(|h| h.id()))
                    .flatten()
            })
            .ok_or_else(|| format!("missing {name}").into())
    };
    let (foot, upper, pad) = (
        id("default::rocker::foot")?,
        id("default::rocker::upper")?,
        id("default::lift::pad")?,
    );
    let mut resource = sim.world_mut().resource_mut::<PhysicsBackend>();
    let e = resource
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("expected native local physics")?;
    let read = |e: &MultiFidelityBackend| -> Result<(DVec3, DVec3)> {
        let a = e.state(foot).ok_or("missing lower link")?;
        let b = e.state(upper).ok_or("missing upper link")?;
        Ok((
            0.5 * (DVec3::from(a.velocity) + DVec3::from(b.velocity)),
            a.angular_velocity.into(),
        ))
    };
    let (v0, w0) = read(e)?;
    let shoe = e.state(foot).ok_or("missing lower link")?;
    if v0.distance(expected.velocity) > 2e-12
        || w0.distance(expected.angular_velocity) > 2e-12
        || DVec3::from(shoe.position).distance(expected.moving.center) > 2e-12
        || (DQuat::from_array(shoe.orientation) * expected.moving.orientation.inverse())
            .xyz()
            .length()
            > 2e-12
    {
        return Err("cold continuation changed its matched incoming pose or twist".into());
    }

    let mut sites = vec![];
    let interval=e.observe_contact_interval(h,1,ContactSolverIterations{temporal:4,pgs:Some(1),relaxation:Some(relax)},|s|{
        for p in s.contact_patches(){if(p.a,p.b)==(foot.min(pad),foot.max(pad)){
            for site in p.points.into_iter().filter(|x|x.normal_impulse>0.){
                let lever=if p.a==foot{site.lever_a}else{site.lever_b};
                let j=site.tangent_impulse_on_a.map(|t|if p.a==foot{1.}else{-1.}*(DVec3::from(t)+site.normal_impulse*DVec3::from(p.normal_on_a)));
                sites.push(json!({"lever":lever,"normal_ns":site.normal_impulse,"j_ns":j.map(|v|v.to_array())}));
            }
        }}
    })?;
    let mut total = DVec3::ZERO;
    let mut normal = 0.;
    for p in interval.pairs {
        if (p.a, p.b) == (foot.min(pad), foot.max(pad)) {
            let sign = if p.a == foot { 1. } else { -1. };
            normal += p.normal_impulse;
            total += sign
                * (DVec3::from(p.normal_impulse_on_a)
                    + DVec3::from(p.tangent_impulse_on_a.ok_or("unknown tangential load")?));
        } else if p.normal_impulse > 0. {
            return Err("competing cold load".into());
        }
    }
    let (v, w) = read(e)?;
    Ok(
        json!({"duration_ns":h.as_nanos()as u64,"relaxation":relax,"before_v":v0.to_array(),"before_w":w0.to_array(),"after_v":v.to_array(),"after_w":w.to_array(),"j_ns":total.to_array(),"normal_ns":normal,"sites":sites}),
    )
}
fn registered_xml(xml: &str, calls: usize) -> Result<String> {
    substituted(
        &xml.replace("project_finite_point", "project_segment_endpoint"),
        "native_steps",
        &calls.to_string(),
    )
}
fn with_work(xml: String) -> String {
    xml.replace(
        "</pair>",
        "<certify_segment_friction_work>true</certify_segment_friction_work></pair>",
    )
}
fn registered_path(xml: &str, calls: usize) -> Result<Value> {
    registered_measured(xml, calls, None)
}
fn registered_measured(xml: &str, calls: usize, clock: Option<&mut u64>) -> Result<Value> {
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        &registered_xml(xml, calls)?,
        "secondary.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let start = clock.as_ref().map(|_| std::time::Instant::now());
    sim.step_exact(1)?;
    if let (Some(out), Some(start)) = (clock, start) {
        *out = start.elapsed().as_nanos().try_into()?
    }
    let Some(r) = gz_sim_scene::primary_contact::last_refinement(&sim) else {
        return Ok(json!({"calls":calls, "scheduled":false}));
    };
    if r.interval.native_steps != calls as u32 || r.interval.finite_point_impacts.len() > 1 {
        return Err("invalid secondary interval usage".into());
    }
    let projected = r.interval.finite_point_impacts.first().map(|h| -> Result<Value>{
        let a=h.resultant;
        let b=h.segment.ok_or("unexpected non-segment projection")?;
        let mut j=json!({"time_ns":a.elapsed.as_nanos() as u64,"feature":format!("{:?}",b.feature),
            "end":b.loaded_end,"points":b.points,"point":a.point,"normal":a.normal,"j_ns":a.impulse_on_foot,
            "before":a.before_relative_velocity,"after":a.after_relative_velocity,
            "opposite_before":b.opposite_before,"opposite_after":b.opposite_after,
            "opening_bound_m_s":b.opening_lower_bound,"work_loss_j":a.dissipated_energy,"sticking":a.sticking});
        if xml.contains("<certify_segment_friction_work>"){j["work_boxes"]=json!(b.work_boxes)}
        Ok(j)
    }).transpose()?;
    Ok(
        json!({"calls":calls,"scheduled":true,"projected":projected,"pair_load_ns":r.requested_pairs[0].normal_impulse,
        "first_end_ns":r.requested_pairs[0].first_positive_end.map(|t|t.as_nanos() as u64)}),
    )
}
type Secondary = (gz_physics::FiniteArcResult, FiniteContactSegment);
fn locate(f: FiniteBoxFlight, precise: bool) -> Result<(Value, Option<Secondary>)> {
    let geo = geometry(f)?;
    let mut result = json!({"geometry":geo});
    let Some(d) = departure(f, &geo)? else {
        return Ok((result, None));
    };
    let Some(arc) = FiniteFeatureArc::new(
        d.after,
        d.site,
        MATERIAL,
        geo["sticking"].as_bool().unwrap(),
        d.allowance + 1e-12,
    )?
    else {
        return Ok((result, None));
    };
    let end = arc.measure_to(
        (Duration::from_micros(STEP_US) - d.hit.latest).as_secs_f64(),
        128,
        if precise { 1e-17 } else { 1e-13 },
    )?;
    result["arc_end"] = record(end, d.after);
    let FiniteArcEnd::Feature(limit) = end.end else {
        return Ok((result, None));
    };
    let track = FiniteFeatureTrack::new(d.after, d.site)?;
    let Some(seg) = track.incident_segment(end.moving, end.site, limit, 1e-10) else {
        return Ok((result, None));
    };
    Ok((result, Some((end, seg))))
}
fn measure(f: FiniteBoxFlight, registered: bool, precise: bool, work: bool) -> Result<Value> {
    let (mut result, hit) = locate(f, precise)?;
    let Some((end, seg)) = hit else {
        return Ok(result);
    };
    let FiniteContactSegment {
        moving_points: p,
        support_points: q,
        normal: n,
    } = seg;
    result["segment"] = json!({"points_m":p.map(|a|a.to_array()),"support_m":q.map(|a|a.to_array()),"normal":n.to_array()});
    let motion = p.map(|x| end.velocity + end.angular_velocity.cross(x - end.center));
    result["velocity_before"] = json!(motion.map(|x| x.to_array()));
    let xml = sdf(end, f.support)?;
    let xml = if work { with_work(xml) } else { xml };
    if registered {
        let mut rows = vec![];
        for n in [2, 8, 64] {
            rows.push(registered_path(&xml, n)?);
        }
        result["registered"] = json!(rows);
    }
    let Some(hit) =
        seg.single_loaded_endpoint(end.center, end.velocity, end.angular_velocity, MATERIAL)?
    else {
        return Ok(result);
    };
    let post_v = end.velocity + hit.impulse / MATERIAL.mass;
    let post_w = end.angular_velocity
        + (p[hit.loaded_end] - end.center).cross(hit.impulse) / MATERIAL.inertia;
    result["impact"] = json!({"loaded_end":hit.loaded_end,"impulse_ns":hit.impulse.to_array(),"velocity_after":hit.velocity_after.map(|v|v.to_array()),"com_velocity_after":post_v.to_array(),"spin_after":post_w.to_array(),"sticking":hit.sticking,"opening_bound_m_s":hit.opening_lower_bound,"work_loss_j":hit.lost_energy});
    if registered {
        return Ok(result);
    }
    let mut runs = vec![];
    for (ns, r) in [(1, 1), (1, 8), (128, 1), (128, 8), (2048, 1), (2048, 8)] {
        runs.push(cold(&xml, Duration::from_nanos(ns), r, end)?);
    }
    result["native"] = json!(runs);
    Ok(result)
}
fn collect(f: Fixture, registered: bool, precise: bool, work: bool) -> Result<Value> {
    let (cuts, disjoint) = survey::cuts(f)?;
    let mut trials = vec![];
    for (i, cut) in cuts.into_iter().enumerate() {
        if let Some(c) = cut {
            let (flight, defect) = kinematics(f, &c)?;
            if defect > 1e-8 {
                return Err("invalid locked incoming state".into());
            }
            trials.push(
                json!({"cut":snapshot(&c),"stroke":i+1,"response":measure(flight,registered,precise,work)?}),
            );
        }
    }
    Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
}
pub(crate) fn write(
    dir: &Path,
    first: usize,
    count: usize,
    registered: bool,
    precise: bool,
    work: bool,
) -> Result<()> {
    survey::write_cases(
        dir,
        first,
        count,
        if work {
            "finite-second-registered-work-v1"
        } else if precise {
            "finite-second-registered-tight-v1"
        } else if registered {
            "finite-second-registered-v1"
        } else {
            "finite-second-contact-active-site-v2"
        },
        |f| collect(f, registered, precise, work),
    )
}

/// Each movie frame is one actual native interval. Projection is allowed at
/// the initial cold line only. All later commands use the same uniform clock.
fn trace(xml: &str, calls: u32, relax: usize, project: bool) -> Result<Value> {
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        xml,
        "secondary.sdf",
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
            .ok_or("missing secondary body")
    };
    let (foot, upper, pad) = (
        body("default::rocker::foot")?,
        body("default::rocker::upper")?,
        body("default::lift::pad")?,
    );
    let mut res = sim.world_mut().resource_mut::<PhysicsBackend>();
    let e = res
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("expected native backend")?;
    let pair = [(foot, pad)];
    let params = ContactSolverIterations {
        temporal: 4,
        pgs: Some(1),
        relaxation: Some(relax),
    };
    let mut frames = vec![];
    let mut projected = None;
    let mut load = DVec3::ZERO;
    for tick in 0..=26 {
        let mut normal = 0.;
        let mut vector = DVec3::ZERO;
        let h = Duration::from_micros(STEP_US);
        if tick != 0 {
            let r = if tick == 1 && project {
                e.advance_box_contacts(
                    h,
                    calls,
                    params,
                    gz_physics_multifidelity::BoxContactActions {
                        finite_segment_impacts: &pair,
                        certify_segment_friction_work: if xml
                            .contains("<certify_segment_friction_work>")
                        {
                            &pair
                        } else {
                            &[]
                        },
                        ..Default::default()
                    },
                    None,
                )?
                .interval
            } else {
                e.step_contact_interval(h, calls, params)?
            };
            for q in r.pairs {
                if (q.a, q.b) == (foot.min(pad), foot.max(pad)) {
                    normal += q.normal_impulse;
                    vector += if foot == q.a { 1. } else { -1. }
                        * (DVec3::from(q.normal_impulse_on_a)
                            + DVec3::from(q.tangent_impulse_on_a.ok_or("unavailable friction")?));
                } else if q.normal_impulse > 0. {
                    return Err("competing positive reaction".into());
                }
            }
            for hit in r.finite_point_impacts {
                let a = hit.resultant;
                let b = hit.segment.ok_or("missing endpoint certificate")?;
                if projected.is_some() || a.elapsed != Duration::ZERO || tick != 1 {
                    return Err("unexpected secondary impulse".into());
                }
                projected = Some(
                    json!({"j_ns":a.impulse_on_foot,"other_after":b.opposite_after,"point":a.point,"normal":a.normal,"sticking":a.sticking}),
                )
            }
        }
        load += vector;
        frames.push(json!({"tick":tick,"time_us":tick*STEP_US,"foot":Body::from(e.state(foot).ok_or("lost foot")?),
            "upper":Body::from(e.state(upper).ok_or("lost upper")?),"pad":Body::from(e.state(pad).ok_or("lost pad")?),
            "normal_impulse_ns":normal,"world_reaction_ns":vector.to_array(),"cumulative_reaction_ns":load.to_array(),
            "native_steps":if tick==0{0}else{calls},"solver_iterations":4}));
    }
    Ok(
        json!({"geometry":{"foot_half_extents":FOOT_HALF,"pad_half_extents":PAD_HALF,"upper_half_extents":[0.05,0.06,0.06]},
        "projected":projected,"frames":frames}),
    )
}
pub(crate) fn replay(dir: &Path, k: usize, stroke: usize) -> Result<()> {
    if k >= 1250 || !(1..=3).contains(&stroke) {
        return Err("invalid secondary replay selection".into());
    }
    let saved: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:04}.json")),
    )?)?;
    let f = survey::fixture(k);
    let cut = survey::cuts(f)?.0[stroke - 1].ok_or("missing donor")?;
    let (flight, _) = kinematics(f, &cut)?;
    let work = saved["format"] == "finite-second-registered-work-v1";
    let precise = work || saved["format"] == "finite-second-registered-tight-v1";
    let (_, hit) = locate(flight, precise)?;
    let (end, _) = hit.ok_or("no incident segment")?;
    let xml = registered_xml(&sdf(end, flight.support)?, 8)?;
    let xml = if work { with_work(xml) } else { xml };
    let trial = saved["result"]["trials"]
        .as_array()
        .ok_or("missing trials")?
        .iter()
        .find(|t| t["stroke"] == stroke)
        .ok_or("missing stroke")?;
    let target = &trial["response"]["registered"][1];
    let folder = dir.join(format!("case-{k:04}-stroke-{stroke}"));
    fs::create_dir_all(folder.join("traces"))?;
    fs::write(folder.join("secondary.sdf"),xml.replace("Matched first outer-support landing, fixture 0. The skew edges of the foot and deck meet during the first public tick.",&format!("Matched secondary contact, fixture {k}, stroke {stroke}. The deck frame is at rest.")))?;
    let mut lanes = vec![];
    for (id, label, n, r, project) in [
        ("native8", "Eight ordinary native calls", 8, 1, false),
        (
            "endpoint8",
            "Certified endpoint, then eight native calls",
            8,
            1,
            true,
        ),
        ("fine4096", "Continuous 4,096 native calls", 4096, 1, false),
    ] {
        let v = trace(&xml, n, r, project)?;
        if project {
            let h = &target["projected"];
            let row: Value = serde_json::from_slice(&serde_json::to_vec(&v["projected"])?)?;
            if row["j_ns"] != h["j_ns"]
                || (target["scheduled"] == true
                    && v["frames"][1]["normal_impulse_ns"] != target["pair_load_ns"])
            {
                return Err(
                    "direct native interval disagrees with the authored scene receipt".into(),
                );
            }
        }
        let file = format!("traces/{id}.json");
        write_json(&folder.join(&file), &v)?;
        lanes.push(json!({"label":id,"name":label,"file":file}));
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"Load at the newly incident finite segment","pair_totals_only":true,"lanes":lanes}),
    )
}

/// Put the cold secondary cut back into the original laboratory frame. Only
/// the unfinished public interval is quantized to the nanosecond actuator
/// clock. Pose and twist are evaluated at the measured physical event.
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer) struct LaboratoryCut {
    pub xml: String,
    pub clock_offset: Duration,
    pub geometric_time_s: f64,
    pub end: gz_physics::FiniteArcResult,
    pub deck: FlightBox,
    pub relative_end: gz_physics::FiniteArcResult,
    pub line: FiniteContactSegment,
    pub relative_flight: FiniteBoxFlight,
    pub original: Value,
}
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer) fn laboratory(
    f: Fixture,
    cut: &ReboundFrame,
) -> Result<Option<LaboratoryCut>> {
    let (g, defect) = kinematics(f, cut)?;
    if defect > 1e-8 {
        return Err("invalid laboratory donor twist".into());
    }
    let (r, hit) = locate(g, true)?;
    let Some((mut end, line)) = hit else {
        return Ok(None);
    };
    let primary_ns = r["geometry"]["hi_ns"]
        .as_u64()
        .ok_or("missing first crossing clock")?;
    let time = primary_ns as f64 * 1e-9 + end.bracket[0];
    let clock_offset = Duration::from_nanos((time * 1e9).round() as u64);
    if clock_offset >= Duration::from_micros(STEP_US) {
        return Ok(None);
    }
    let relative_end = end;
    let speed = command(f, cut.tick) * DVec3::Z;
    end.center += speed * time;
    end.moving.center += speed * time;
    end.velocity += speed;
    let mut deck = g.support;
    deck.center += speed * time;
    let cold = sdf(end, deck)?;
    let (foot, lift) = cold
        .split_once("<model name=\"lift\">")
        .ok_or("missing translating support")?;
    let xml = with_work(registered_xml(
        &format!(
            "{foot}<model name=\"lift\">{}",
            substituted(lift, "linear_velocity", &vector(speed))?
        ),
        8,
    )?);
    Ok(Some(LaboratoryCut {
        xml,
        clock_offset,
        geometric_time_s: time,
        end,
        deck,
        relative_end,
        line,
        relative_flight: g,
        original: r,
    }))
}
