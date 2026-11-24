// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Apply the existing registered material advances to the same warm and rebuilt native returns.
use super::*;
#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Native,
    Point,
    Retime,
    Bound,
    Launch,
    Subdivide,
    Directed,
    RetimedDirected,
}
impl Mode {
    fn name(self) -> &'static str {
        match self {
            Self::Native => "native",
            Self::Point => "point",
            Self::Retime => "retimed",
            Self::Bound => "bound",
            Self::Launch => "launch",
            Self::Subdivide => "subdivided",
            Self::Directed => "directed",
            Self::RetimedDirected => "retimed_directed",
        }
    }
}
struct Advance {
    world: MultiFidelityBackend,
    tick: Tick,
    report: Value,
}
fn advance(
    parent: &MultiFidelityBackend,
    ids: [BodyId; 3],
    incoming: &Tick,
    f: Fixture,
    graph: Graph,
    grid: u32,
    policy: (Mode, bool),
) -> Result<Advance> {
    let (mode, certify) = policy;
    let [foot, _, pad] = ids;
    let mut w = parent.copy_airborne_landing(foot, pad, graph.cold())?;
    let speed = command(f, incoming.tick);
    if w.state(pad).ok_or("missing pad")?.velocity[2] != speed {
        w.set_world_velocity(pad, [0., 0., speed], [0.; 3])?;
    }
    let initial = Bodies::from_states(states(&w, ids)?);
    let pair = [(foot, pad)];
    let it = ContactSolverIterations {
        temporal: 4,
        pgs: None,
        relaxation: Some(8),
    };
    let subdivide = matches!(
        mode,
        Mode::Subdivide | Mode::Directed | Mode::RetimedDirected
    );
    let directed = matches!(mode, Mode::Directed | Mode::RetimedDirected);
    let bound = matches!(mode, Mode::Bound | Mode::Launch) || subdivide;
    let receipt = if mode == Mode::Native {
        w.step_contact_interval(DT, grid, it)?
    } else {
        w.advance_box_contacts(
            DT,
            grid,
            it,
            BoxContactActions {
                finite_point_impacts: &pair,
                certify_face_vertices: if certify { &pair } else { &[] },
                retime_finite_transfer: if matches!(mode, Mode::Retime | Mode::RetimedDirected) {
                    &pair
                } else {
                    &[]
                },
                sliding_vertices: if bound { &pair } else { &[] },
                sticking_points: if bound { &pair } else { &[] },
                point_transfers: if bound { &pair } else { &[] },
                sliding_skew_edges: if bound { &pair } else { &[] },
                subdivide_point_bounds: if subdivide { &pair } else { &[] },
                target_point_bounds: if directed { &pair } else { &[] },
                slip_launches: if mode == Mode::Launch || subdivide {
                    &pair
                } else {
                    &[]
                },
                ..Default::default()
            },
            None,
        )?
        .interval
    };
    if receipt.native_steps != grid {
        return Err("registered return changed native call budget".into());
    }
    let mut j = DVec3::ZERO;
    let mut z = 0.;
    let mut tan = DVec3::ZERO;
    let mut first = None;
    for r in &receipt.pairs {
        if (r.a, r.b) != (foot.min(pad), foot.max(pad)) {
            if r.normal_impulse > 0. {
                return Err("third body paid".into());
            }
            continue;
        }
        let sign = if r.a == foot { 1. } else { -1. };
        tan = sign
            * DVec3::from(
                r.tangent_impulse_on_a
                    .ok_or("missing signed registered friction")?,
            );
        j = sign * DVec3::from(r.normal_impulse_on_a) + tan;
        z = r.normal_impulse;
        first = r.first_positive_end;
    }
    let landed = states(&w, ids)?;
    let final_body = Bodies::from_states(landed);
    let balance = (final_body.momentum() - initial.momentum()
        + DVec3::Z * (2. * 9.81 * DT.as_secs_f64())
        - j)
        .length();
    let impacts=receipt.finite_point_impacts.iter().map(|s|{
        let r=s.resultant;
        json!({"time_ns":r.elapsed.as_nanos() as u64,"feature":format!("{:?}",s.feature),"j_ns":r.impulse_on_foot,"point":r.point,"normal":r.normal,"stick":r.sticking,"loss_j":r.dissipated_energy,"next_boundary_ns":s.next_feature_boundary.map(|a|a.1.as_nanos() as u64)})
    }).collect::<Vec<_>>();
    let mut spans = vec![];
    for (label, list) in [
        ("vertex", &receipt.sliding_vertices),
        ("skew", &receipt.sliding_skew_edges),
    ] {
        for s in list {
            spans.push(json!({"kind":label,"start_ns":s.start.as_nanos() as u64,"duration_ns":s.duration.as_nanos() as u64,"j_ns":s.impulse_on_foot,"normal_ns":s.normal_impulse}));
        }
    }
    for s in &receipt.sticking_points {
        spans.push(json!({"kind":"stick","start_ns":s.start.as_nanos() as u64,"duration_ns":s.duration.as_nanos() as u64,"j_ns":s.impulse_on_foot,"normal_ns":s.normal_impulse}));
    }
    for s in &receipt.point_transfers {
        spans.push(json!({"kind":"transfer","start_ns":s.start.as_nanos() as u64,"duration_ns":s.duration.as_nanos() as u64,"j_ns":s.impulse_on_foot,"normal_ns":s.normal_impulse,
    "branches":s.intervals.iter().map(|p|json!({"time_s":p.offset,"span_s":p.duration,"feature":format!("{:?}",p.feature),"stick":p.sticking,"slip_growth":p.slip_growth.is_some(),"jump_ns":p.impact.map(|i|i.impulse_on_foot)})).collect::<Vec<_>>()}));
    }
    spans.sort_by_key(|s| s["start_ns"].as_u64());
    let tick = Tick {
        tick: incoming.tick + 1,
        foot: landed[0].into(),
        upper: landed[1].into(),
        pad: landed[2].into(),
        normal: z,
        calls: grid,
        extra: 0.,
        tangent: tan,
        first,
        gap: supported_gap(landed[0], landed[2]),
    };
    let report = json!({"graph":graph.name(),"mode":mode.name(),"calls":grid,"passes":8,"world_impulse_ns":j.to_array(),"end_com_m":tick.com().to_array(),"end_spin_rad_s":tick.foot.angular_velocity,"normal_ns":z,"normal_momentum_residual_ns":balance,"first_ns":first.map(|d|d.as_nanos() as u64),"impacts":impacts,"sustained":spans});
    Ok(Advance {
        world: w,
        tick,
        report,
    })
}
fn run(dir: &Path, id: usize, replay: bool, certify: bool, event_bounds: bool) -> Result<Value> {
    let studies = dir.parent().ok_or("missing studies")?;
    let field = load(&studies.join(format!(
        "multifidelity-secondary-natural/cases/case-{id:03}.json"
    )))?;
    let full = load(&studies.join(format!(
        "multifidelity-secondary-material-response/cases/case-{id:03}.json"
    )))?;
    let tick = full["cut_tick"].as_u64().ok_or("no source cut")?;
    if field["incoming"] != full["incoming"] {
        return Err("material reference mismatch".into());
    }
    let f = fixture(id);
    let cut = survey::cuts(f)?.0[0].ok_or("no primary")?;
    let mut rows = vec![];
    let mut lanes = vec![];
    let dest = dir.join(format!("case-{id:03}"));
    let prior = if replay {
        fs::create_dir_all(dest.join("traces"))?;
        Some(load(&dir.join(format!("cases/case-{id:03}.json")))?)
    } else {
        None
    };
    let control_archive = if replay && certify && !event_bounds {
        Some(load(&studies.join(format!(
            "multifidelity-secondary-face-controls/cases/case-{id:03}.json"
        )))?)
    } else {
        None
    };
    observe_path_until(
        f,
        &cut,
        PATHS[6],
        ProjectionSpan::Native,
        tick,
        |parent, ids, incoming| {
            if incoming.tick != tick {
                return Ok(());
            }
            let state = parent.airborne_restart_state(ids[0], ids[2])?;
            if motion(&state) != full["incoming"]["motion"]
                || cache(&state) != full["incoming"]["cache"]
            {
                return Err("lost material donor".into());
            }
            for grid in [8, 64] {
                for graph in [Graph::Retained, Graph::Cold] {
                    let modes = if event_bounds {
                        [
                            Mode::Native,
                            Mode::Launch,
                            Mode::Subdivide,
                            Mode::Directed,
                            Mode::RetimedDirected,
                        ]
                    } else {
                        [
                            Mode::Native,
                            Mode::Point,
                            Mode::Retime,
                            Mode::Bound,
                            Mode::Launch,
                        ]
                    };
                    for m in modes {
                        let display = if event_bounds {
                            matches!(m, Mode::Native | Mode::Subdivide | Mode::Directed)
                        } else {
                            matches!(m, Mode::Native | Mode::Point | Mode::Launch)
                        };
                        if replay && (grid != 8 || graph.cold() || !display) {
                            continue;
                        }
                        let old_registered = replay && certify && m == Mode::Point;
                        let actual_mode = if old_registered { Mode::Launch } else { m };
                        let mut a = advance(
                            parent,
                            ids,
                            incoming,
                            f,
                            graph,
                            grid,
                            (actual_mode, certify && !old_registered),
                        )?;
                        if m == Mode::Native {
                            let old = field["treatments"]
                                .as_array()
                                .unwrap()
                                .iter()
                                .find(|r| {
                                    r["graph"] == graph.name()
                                        && r["calls"] == grid
                                        && r["passes"] == 8
                                })
                                .ok_or("missing native control")?;
                            for k in [
                                "world_impulse_ns",
                                "end_com_m",
                                "end_spin_rad_s",
                                "normal_ns",
                                "first_ns",
                            ] {
                                if old[k] != a.report[k] {
                                    let difference = |old: &Value, actual: &Value| -> Option<f64> {
                                        let x: DVec3 = DVec3::from_array(
                                            serde_json::from_value(old.clone()).ok()?,
                                        );
                                        let y: DVec3 = DVec3::from_array(
                                            serde_json::from_value(actual.clone()).ok()?,
                                        );
                                        Some(x.distance(y))
                                    };
                                    if !difference(&old[k], &a.report[k]).is_some_and(|d| d < 1e-12)
                                    {
                                        return Err(
                                            format!("control {k} changed at {id}/{grid}").into()
                                        );
                                    }
                                }
                            }
                        }
                        if let Some(src) = &prior {
                            let source = if old_registered {
                                control_archive.as_ref().unwrap()
                            } else {
                                src
                            };
                            if !source["rows"].as_array().unwrap().contains(&a.report) {
                                return Err(
                                    "projection replay does not reproduce archived result".into()
                                );
                            }
                            let mut frames = vec![incoming.as_json(), a.tick.as_json()];
                            for t in tick + 2..=tick + 12 {
                                let speed = command(f, t - 1);
                                let cmd = (speed != a.tick.pad.velocity[2]).then_some(speed);
                                a.tick = propagate_one(
                                    &mut a.world,
                                    ids,
                                    t,
                                    b(4096, 1),
                                    cmd,
                                    None,
                                    &mut vec![],
                                )?;
                                frames.push(a.tick.as_json());
                            }
                            let label = if old_registered { "previous" } else { m.name() };
                            let name = match m {
                                Mode::Native => "8 native contact advances",
                                Mode::Subdivide => "Subdivided material contact",
                                Mode::Directed => "Event-directed material contact",
                                Mode::Point if old_registered => {
                                    "Original registered point and transfers"
                                }
                                Mode::Point => "Registered finite point",
                                Mode::Launch if certify => {
                                    "Certified vertex and material transfers"
                                }
                                Mode::Launch => "Registered point and material transfers",
                                _ => unreachable!(),
                            };
                            let file = format!("traces/{label}.json");
                            write_json(
                                &dest.join(&file),
                                &json!({"frames":frames,"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF}}),
                            )?;
                            lanes.push(json!({"label":label,"name":name,"file":file}));
                        }
                        rows.push(a.report);
                    }
                }
            }
            Ok(())
        },
    )?;
    if replay {
        if lanes.len() != 3 {
            return Err("missing replay".into());
        }
        write_json(
            &dest.join("comparison.json"),
            &json!({"pair_totals_only":true,"title":"Registered material contact at an original second landing","lanes":lanes,"capture_ticks":{"second_landing":tick+1,"six_ms":tick+12}}),
        )?;
    } else if rows.len() != 20 {
        return Err("incomplete native material crossing".into());
    }
    Ok(
        json!({"format":"matched-secondary-material-policy-v1","certify_vertices":certify,"event_bounds":event_bounds,"case":id,"slot":field["slot"],"held":field["held"],"fixture":field["fixture"],"cut_tick":tick,"incoming":field["incoming"],"rows":rows}),
    )
}
pub(crate) fn field(
    dir: &Path,
    start: usize,
    count: usize,
    certify: bool,
    event_bounds: bool,
) -> Result<()> {
    if count == 0 || start.checked_add(count).is_none_or(|s| s > 50) {
        return Err("invalid policy coordinates".into());
    }
    let d = dir.join("cases");
    fs::create_dir_all(&d)?;
    for s in start..start + count {
        let (id, _, _) = cohort::coordinate(s)?;
        let p = d.join(format!("case-{id:03}.json"));
        if p.exists() {
            let v = load(&p)?;
            if v["format"] != "matched-secondary-material-policy-v1"
                || v["slot"] != s
                || v["certify_vertices"].as_bool().unwrap_or(false) != certify
                || v["event_bounds"].as_bool().unwrap_or(false) != event_bounds
            {
                return Err("unexpected material policy format".into());
            }
            continue;
        }
        let result = run(dir, id, false, certify, event_bounds)?;
        let tmp = p.with_extension(format!("{}.pending", std::process::id()));
        write_json(&tmp, &result)?;
        fs::rename(tmp, p)?;
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, id: usize, certify: bool, event_bounds: bool) -> Result<()> {
    run(dir, id, true, certify, event_bounds)?;
    Ok(())
}

/// Rebuild the two-link initial twists and the actual deck command using ordinary SDF plugins.
pub(crate) fn authored(root: &Path, selected: usize, event: bool, skew: bool, fractional:bool, retime:bool) -> Result<()> {
    use crate::touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::second::boundary_xml;
    use gz_physics::FlightBox;
    use gz_sim_scene::{load_world_with_sdf_step,primary_contact::last_refinement};
    let sibling = root.parent().ok_or("missing studies")?;
    let dir = sibling.join("multifidelity-secondary-material-response/cases");
    fs::create_dir_all(root.join("traces"))?;
    let mut evidence = vec![];
    let mut lanes = vec![];
    for slot in 0..50 {
        let (id, _, held) = cohort::coordinate(slot)?;
        let source = load(&dir.join(format!("case-{id:03}.json")))?;
        let links = &source["incoming"]["motion"]["links"];
        let v = |x: &Value| -> Result<DVec3> {
            Ok(DVec3::from_array(serde_json::from_value(x.clone())?))
        };
        let read_box = |i: usize, half: [f64; 3]| -> Result<FlightBox> {
            Ok(FlightBox {
                center: v(&links[i]["position"])?,
                orientation: DQuat::from_array(serde_json::from_value(
                    links[i]["orientation"].clone(),
                )?),
                half_extent: half.into(),
            })
        };
        let foot = read_box(0, FOOT_HALF)?;
        let support = read_box(2, PAD_HALF)?;
        let cvel = (v(&links[0]["velocity"])? + v(&links[1]["velocity"])?) * 0.5;
        let spin = v(&links[0]["angular_velocity"])?;
        let drive = v(&source["pad_command_m_s"])?;
        let xml=boundary_xml(foot,cvel,spin,support,drive)?.replace("Matched first outer-support landing, fixture 0. The skew edges of the foot and deck meet during the first public tick.","Measured second-landing cut with a translating support.").replace("<relaxation_iterations>1","<relaxation_iterations>8");
        let feature = "<project_finite_point>true</project_finite_point>";
        let projection = format!(
            "{feature}<follow_sliding_vertex>true</follow_sliding_vertex><follow_sticking_point>true</follow_sticking_point><resolve_point_transfers>true</resolve_point_transfers><follow_sliding_skew_edge>true</follow_sliding_skew_edge><follow_skew_slip_launch>true</follow_skew_slip_launch>"
        );
        let mut rows = vec![];
        let certified = format!("{projection}<certify_face_vertex>true</certify_face_vertex>");
        let subdivided =
            format!("{certified}<subdivide_point_bounds>true</subdivide_point_bounds>");
        let profiles = if retime {
            let directed=format!("{subdivided}<target_point_bounds>true</target_point_bounds>");
            let full=format!("{directed}<resolve_fractional_points>true</resolve_fractional_points>");
            [("integer","Bounded single-point integer landing",format!("{directed}<certify_skew_edge>true</certify_skew_edge>")),
            ("fractional","Complete fractional material landing",full.clone()),
            ("retimed","Retiming an unqualified subsequent span",format!("{full}<retime_finite_transfer>true</retime_finite_transfer>"))]
        } else if fractional {
            let directed=format!("{subdivided}<target_point_bounds>true</target_point_bounds>");
            [("native","Ordinary authored nomination",String::new()),
             ("integer","Bounded single-point integer landing",format!("{directed}<certify_skew_edge>true</certify_skew_edge>")),
             ("fractional","Complete fractional material landing",format!("{directed}<resolve_fractional_points>true</resolve_fractional_points>"))]
        } else if skew {
            let directed=format!("{subdivided}<target_point_bounds>true</target_point_bounds>");
            [("native","Ordinary authored nomination", String::new()),
             ("directed","Event-directed finite point",directed.clone()),
             ("skew","Skew geometry and outgoing material bound",format!("{directed}<certify_skew_edge>true</certify_skew_edge>"))]
        } else if event {
            [
                ("launch", "Registered material transfers", certified),
                (
                    "subdivided",
                    "Subdivided material contact",
                    subdivided.clone(),
                ),
                (
                    "directed",
                    "Event-directed material contact",
                    format!("{subdivided}<target_point_bounds>true</target_point_bounds>"),
                ),
            ]
        } else {
            [
                ("native", "Ordinary authored nomination", String::new()),
                ("point", "Registered point and transfers", projection),
                ("certified", "Certified vertex and transfers", certified),
            ]
        };
        for (label, name, options) in profiles {
            let sdf = xml.replace(feature, &options);
            if fractional {
                let dir=root.join(format!("worlds/case-{id:03}")); fs::create_dir_all(&dir)?;
                fs::write(dir.join(format!("{label}.sdf")),&sdf)?;
            }
            let mut lane = Lane::from_sim(load_world_with_sdf_step(
                &sdf,
                "secondary_face.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )?)?;
            let initial = lane.sample()?;
            for (i, b) in [initial.foot, initial.upper, initial.pad]
                .into_iter()
                .enumerate()
            {
                let expected = if i == 2 {
                    drive
                } else {
                    v(&links[i]["velocity"])?
                };
                if (DQuat::from_array(b.orientation)
                    * DQuat::from_array(serde_json::from_value(links[i]["orientation"].clone())?)
                        .inverse())
                .xyz()
                .length()
                    > 1e-11
                    || DVec3::from(b.position).distance(v(&links[i]["position"])?) > 1e-12
                    || DVec3::from(b.velocity).distance(expected) > 1e-11
                    || DVec3::from(b.angular_velocity).distance(v(&links[i]["angular_velocity"])?)
                        > 1e-11
                {
                    return Err(
                        format!("SDF changed original state for source {id}, link {i}").into(),
                    );
                }
            }
            let mut frames = vec![];
            for step in 0..=if id == selected { 12 } else { 1 } {
                if step > 0 {
                    lane.sim.step_exact(1)?;
                }
                let s = lane.sample()?;
                let obs = last_refinement(&lane.sim).filter(|r| r.iteration == step);
                let calls = if step == 0 {
                    0
                } else {
                    obs.map_or(1, |r| r.interval.native_steps)
                };
                let normal =
                    obs.map_or(s.normal_impulse_ns, |r| r.requested_pairs[0].normal_impulse);
                if step == 1 {
                    let measured=obs.map(|obs|{
                        let p=&obs.requested_pairs[0];
                        let j=DVec3::from(p.normal_impulse_on_foot)+DVec3::from(p.tangent_impulse_on_foot.unwrap());
                        let mut initial=obs.interval.finite_point_impacts.iter().map(|i|json!({"time_ns":i.resultant.elapsed.as_nanos() as u64,"j_ns":i.resultant.impulse_on_foot,"feature":format!("{:?}",i.feature)})).collect::<Vec<_>>();
                        let mut later=Vec::new();
                        let mut fractional=Vec::new();
                        for r in &obs.interval.fractional_point_paths{
                            let first=&r.events[0];
                            initial.push(json!({"time_ns":first.time_s*1e9,"j_ns":first.impulse_on_foot,"feature":format!("{:?}",r.first_feature)}));
                            later.extend(r.events[1..].iter().map(|e|json!({"time_ns":e.time_s*1e9,"j_ns":e.impulse_on_foot})));
                            fractional.push(json!({"times_ns":r.events.iter().map(|s|s.time_s*1e9).collect::<Vec<_>>(),"first_bracket_ns":r.first_bracket_s.map(|x|x*1e9),"native_boundary_ns":r.native_boundary.as_nanos() as u64,"next_boundary_ns":r.next_native_boundary.map(|h|h.as_nanos() as u64),"continued_boundaries_ns":r.continued_boundaries.iter().map(|t|t.as_nanos() as u64).collect::<Vec<_>>(),
                                "total_ns":r.impulse_on_foot,"normal_vector_ns":r.normal_impulse_on_foot,"normal_ns":r.normal_impulse,"angular_nms":r.angular_impulse_about_com,"sustained_work_j":r.sustained_friction_work,"end_feature":format!("{:?}",r.end_feature)}));
                        }
                        later.extend(obs.interval.point_transfers.iter().flat_map(|r|r.intervals.iter().filter_map(move|s|s.impact.map(|i|json!({"time_ns":r.start.as_secs_f64()*1e9+1e9*(s.offset+s.duration),"j_ns":i.impulse_on_foot})))));
                        json!({"world_impulse_ns":j.to_array(),"internal_jumps":later,"fractional":fractional,"transfer_spans":obs.interval.point_transfers.len(),"first_ns":p.first_positive_end.map(|t|t.as_nanos() as u64),"impacts":initial})
                    });
                    rows.push(json!({"lane":label,"calls":calls,"normal_ns":normal,"refined":measured,"foot":s.foot,"upper":s.upper,"pad":s.pad}));
                }
                if id == selected {
                    frames.push(json!({"tick":step,"time_us":step*500,"native_steps":calls,"normal_impulse_ns":normal,"foot":s.foot,"upper":s.upper,"pad":s.pad}));
                }
            }
            if id == selected {
                fs::write(root.join(format!("{label}.sdf")), &sdf)?;
                let file = format!("traces/{label}.json");
                write_json(
                    &root.join(&file),
                    &json!({"frames":frames,"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF}}),
                )?;
                lanes.push(json!({"label":label,"name":name,"file":file}));
            }
        }
        evidence.push(
            json!({"case":id,"slot":slot,"held":held,"cut_tick":source["cut_tick"],"rows":rows}),
        );
    }
    if lanes.len() != 3 {
        return Err("selected source outside SDF sample".into());
    }
    write_json(
        &root.join("study.json"),
        &json!({"selected":selected,"cases":evidence}),
    )?;
    write_json(
        &root.join("comparison.json"),
        &json!({"pair_totals_only":true,"lanes":lanes,"capture_ticks":{"material_impact":1,"six_ms":12},"title":if retime{"Changing the next native boundary after a fractional response"}else if fractional{"All material reactions before the integer native clock"}else if skew{"Finite edge geometry and the response before the integer clock"}else if event{"Three changes of loaded vertex in an authored landing"}else{"A certified face vertex on the returning support in authored SDF"}}),
    )?;
    Ok(())
}

/// Reconstruct the exact authored incoming world outside the timed region.
/// Each timed unobserved public step must reproduce its full reference receipt
/// and all three terminal body states from the archived matching SDF.
pub(crate) fn authored_cost(root:&Path,repeat:u8)->Result<()>{
    use gz_sim_scene::{load_world_with_sdf_step,primary_contact::last_refinement};
    use std::time::Instant;
    if !(1..=2).contains(&repeat){return Err("use repeat one or two".into());}
    let src=load(&root.join("study.json"))?;
    let out=root.join(format!("cost/repeat-{repeat}"));
    fs::create_dir_all(&out)?;
    let clk=||{let t=rustix::time::clock_gettime(rustix::time::ClockId::ThreadCPUTime);t.tv_sec as u64*1_000_000_000+t.tv_nsec as u64};
    let donors=src["cases"].as_array().ok_or("missing SDF sources")?;
    for k in 0..donors.len(){
        let case=&donors[if repeat==1{k}else{donors.len()-1-k}];
        let id=case["case"].as_u64().ok_or("missing SDF number")?;
        let presets=case["rows"].as_array().ok_or("missing SDF profiles")?;
        let mut controls=Vec::new();let mut files=Vec::new();
        for r in presets{
            let label=r["lane"].as_str().ok_or("missing SDF label")?;
            let xml=fs::read_to_string(root.join(format!("worlds/case-{id:03}/{label}.sdf")))?;
            let mut baseline=Lane::from_sim(load_world_with_sdf_step(&xml,"secondary_face.sdf","default",&sdformat::NoResources,PhysicsEngine::MultiFidelity)?)?;
            baseline.sim.step_exact(1)?;let s=baseline.sample()?;let obs=last_refinement(&baseline.sim).cloned();
            if json!(s.foot)!=r["foot"] || json!(s.upper)!=r["upper"] || json!(s.pad)!=r["pad"] ||
                obs.as_ref().map_or(s.normal_impulse_ns,|v|v.requested_pairs[0].normal_impulse)!=r["normal_ns"].as_f64().ok_or("invalid archived pair impulse")? {return Err("timed SDF changed archive".into());}
            controls.push((s,obs));files.push(xml);
        }
        let mut wall=vec![Vec::new();presets.len()];let mut cpu=wall.clone();
        for round in 0..36{for j in 0..presets.len(){
            let pos=(k+round+j)%presets.len();let arm=if repeat==1{pos}else{presets.len()-1-pos};
            let mut lane=Lane::from_sim(load_world_with_sdf_step(&files[arm],"secondary_face.sdf","default",&sdformat::NoResources,PhysicsEngine::MultiFidelity)?)?;
            let start_cpu=clk();let start=Instant::now();lane.sim.step_exact(1)?;
            let ns=u64::try_from(start.elapsed().as_nanos())?;let cpu_ns=clk()-start_cpu;
            if lane.sample()?!=controls[arm].0 || last_refinement(&lane.sim)!=controls[arm].1.as_ref(){return Err("timed SDF state or signed receipt changed".into());}
            if round>=4 {wall[arm].push(ns);cpu[arm].push(cpu_ns);}
        }}
        let cells:Vec<_>=presets.iter().enumerate().map(|(i,r)|json!({"lane":r["lane"],"calls":r["calls"],"normal_ns":r["normal_ns"],"wall_ns":wall[i],"thread_ns":cpu[i]})).collect();
        write_json(&out.join(format!("case-{id:03}.json")),&json!({"case":id,"slot":case["slot"],"held":case["held"],"repeat":repeat,"rows":cells}))?;
    }
    Ok(())
}
