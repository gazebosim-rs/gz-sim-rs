// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualification through the ordinary SDF loader, Bevy stepping and signed pair receipts.
use super::transaction::{Frame, fixture};
use super::*;
const AUTHORED: &str = include_str!("../../../../../../examples/multifidelity_sliding_edge.sdf");
pub(super) fn xml(i: Input, frame: Frame, free: bool, gravity: DVec3, policy: &str) -> String {
    let mut src = fixture(i, frame, free).replace(
        "0 0 -9.81</gravity>",
        &format!(
            "{:.17} {:.17} {:.17}</gravity>",
            gravity.x, gravity.y, gravity.z
        ),
    );
    if policy != "ordinary" {
        let a = AUTHORED
            .find("    <plugin name=\"gz::sim::systems::PrimaryContactRefinement\"")
            .unwrap();
        let b = a + AUTHORED[a..].find("</plugin>").unwrap() + "</plugin>".len();
        let p = if policy == "edge" {
            AUTHORED[a..b].replace("project_sliding_edge", "project_edge")
        } else if policy == "mixed" {
            AUTHORED[a..b].replace("project_sliding_edge", "project_mixed_edge")
        } else if policy == "coupled" {
            AUTHORED[a..b].to_string()
        } else {
            AUTHORED[a..b].replace("<project_sliding_edge>true</project_sliding_edge>", "")
        };
        src = src.replace(
            "    <model name=\"rocker\">",
            &format!("{p}\n    <model name=\"rocker\">"),
        );
    }
    src
}
fn input_for_flight(mu: f64, yaw: f64, flight: f64, g: DVec3) -> Input {
    let mut i = make(mu, yaw, 0.18, [0.4, 0.8], false);
    i.edge.spin = DVec3::ZERO;
    i.edge.velocity = DQuat::from_rotation_z(yaw) * DVec3::new(3.3, -1.7, -1.27);
    i.seed.state.v = i.edge.velocity;
    i.seed.state.w = DVec3::ZERO;
    i.seed.state.c.z -= i.edge.velocity.z * flight + 0.5 * g.z * flight * flight;
    i
}
fn exact(i: Input, frame: Frame, free: bool) -> Result<Value> {
    let mut e = i.edge;
    if free {
        let ax = i.seed.state.q * DVec3::X;
        e.inertia -= DMat3::from_cols(ax * ax.x, ax * ax.y, ax * ax.z) * 0.0201;
    }
    let physical = e.solve();
    let source = xml(
        i,
        frame,
        free,
        frame.q * DVec3::new(0., 0., -9.81),
        "coupled",
    );
    let mut s = load(&source, true)?;
    s.step_exact(1)?;
    let event = last_refinement(&s).and_then(|h| h.interval.sliding_edge_impacts.first());
    match (physical, event) {
        (Err(reason), None) => Ok(
            json!({"selected":last_refinement(&s).is_some(),"admitted":false,"decline":format!("{reason:?}")}),
        ),
        (Ok(law), Some(hit)) => {
            let mut err = 0f64;
            let mut gap = 0f64;
            let mut world = vec![];
            for p in hit.sites {
                let trial = DVec3::from(p.impulse_on_foot);
                let other = law.impulse.map(|j| trial.distance(frame.q * j));
                err = err.max(other[0].min(other[1]));
                gap = gap.max(p.gap.abs());
                world.push(p.impulse_on_foot);
                if !p.elapsed.is_zero() {
                    return Err("authored cold tangency shifted inside public tick".into());
                }
            }
            let receipt = &last_refinement(&s).unwrap().requested_pairs[0];
            if err > 1e-11 || receipt.first_positive_end != Some(Duration::ZERO) {
                return Err(format!("authored edge at t0 violated point law {err:e}").into());
            }
            Ok(
                json!({"admitted":true,"native_calls":last_refinement(&s).unwrap().interval.native_steps,"site_error_ns":err,"gap_m":gap,"energy_change_j":hit.kinetic_energy_change,"point_impulses_ns":world,
                "first_positive_ns":receipt.first_positive_end.unwrap().as_nanos() as u64,"integrated_normal_ns":receipt.normal_impulse,"last_native_normal_ns":receipt.last_native_normal_impulse}),
            )
        }
        _ => Err("Bevy edge classification disagrees with the spatial oracle".into()),
    }
}
fn flight(i: Input, frame: Frame, g: DVec3, time: f64) -> Result<Value> {
    let source = xml(i, frame, false, frame.q * g, "coupled");
    let mut s = load(&source, true)?;
    let ticks = ((time / DT).ceil() as u64).max(1);
    let mut previous_reaction = 0.;
    let mut onset = None;
    for tick in 1..=ticks {
        s.step_exact(1)?;
        let ev = last_refinement(&s).filter(|r| r.iteration == tick);
        if let Some(h) = ev.and_then(|r| r.interval.sliding_edge_impacts.first()) {
            if onset.is_some() {
                return Err("authored edge generated more than one jump".into());
            }
            let event = ev.unwrap();
            let at = event.start_time + h.sites[0].elapsed;
            let mut e = i.edge;
            e.velocity += g * at.as_secs_f64();
            let law = e
                .solve()
                .map_err(|e| format!("flight oracle declined: {e:?}"))?;
            let center = i.seed.state.c
                + i.edge.velocity * at.as_secs_f64()
                + 0.5 * g * at.as_secs_f64().powi(2)
                + DVec3::from(i.seed.pad.position)
                + PAD
                + DVec3::from(i.seed.pad.velocity) * at.as_secs_f64();
            let mut errors = [0.; 2];
            let mut impulses = [[0.; 3]; 2];
            let mut after = [[0.; 3]; 2];
            for site in h.sites {
                let arm = frame.q.conjugate() * (DVec3::from(site.point) - frame.shift) - center;
                let k = usize::from(arm.distance(e.arm[1]) < arm.distance(e.arm[0]));
                let j = DVec3::from(site.impulse_on_foot);
                if arm.distance(e.arm[k]) > 1e-11 || site.elapsed != h.sites[0].elapsed {
                    return Err("authored edge lost its common boundary".into());
                }
                errors[k] = j.distance(frame.q * law.impulse[k]);
                impulses[k] = j.to_array();
                after[k] = site.after_relative_velocity;
            }
            if errors.iter().any(|x| *x > 1e-10)
                || (at.as_secs_f64() - time).abs() > 1.001e-9
                || previous_reaction != 0.
            {
                return Err(format!("arrival residual {time:e} at {at:?} order {errors:?} preceding {previous_reaction:e}").into());
            }
            onset = Some(
                json!({"tick":tick,"known_ns":time*1e9,"actual_ns":at.as_nanos() as u64,"local_ns":h.sites[0].elapsed.as_nanos() as u64,"completed_public_ns":s.elapsed().as_nanos() as u64,"errors_ns":errors,"impulses_ns":impulses,"outgoing_point_velocity":after,
             "native_calls":event.interval.native_steps,"preceding_reaction_ns":previous_reaction,"interval_normal_ns":event.requested_pairs[0].normal_impulse,"last_native_normal_ns":event.requested_pairs[0].last_native_normal_impulse}),
            );
        } else {
            previous_reaction += if let Some(r) = ev {
                r.interval
                    .pairs
                    .iter()
                    .map(|p| p.normal_impulse)
                    .sum::<f64>()
            } else {
                engine(&s)
                    .rigid_contact_observations()
                    .iter()
                    .map(|p| p.normal_impulse)
                    .sum()
            };
        }
    }
    onset.ok_or_else(|| format!("missed authored flight at {time:e} in {ticks} ticks").into())
}
fn record(out: &Path) -> Result<()> {
    let t = 0.00213819765;
    let g = DVec3::new(0.35, -0.23, -9.81);
    let i = input_for_flight(0.2, 0.50, t, g);
    record_flight(out, i, g, false)
}
pub(super) fn record_flight(out: &Path, i: Input, g: DVec3, mixed: bool) -> Result<()> {
    let frame = Frame {
        q: DQuat::IDENTITY,
        shift: DVec3::ZERO,
        spin: 0.,
    };
    let mut all = vec![];
    for label in ["ordinary", "refined", "coupled"] {
        let policy = if mixed && label == "coupled" {
            "mixed"
        } else {
            label
        };
        let src = xml(i, frame, false, g, policy);
        let mut s = load(&src, true)?;
        let r = ids(&s)?;
        let capture = std::sync::Arc::new(std::sync::Mutex::new([0f64; 4]));
        if label != "ordinary" {
            let dest = capture.clone();
            gz_sim_scene::primary_contact::set_refined_observer(&mut s, move |_, sub| {
                let q = DQuat::from_array(sub.state(r.foot).unwrap().orientation);
                let (_, _, points) = moments(&sub.contact_patches(), &r, q, false).unwrap();
                let mut c = dest.lock().unwrap();
                for k in 0..4 {
                    c[k] += points[k];
                }
            })?;
        }
        let mut frames = vec![];
        let mut events = vec![];
        for tick in 0..=48 {
            let mut native = 0;
            let mut loads = [0.; 4];
            if tick > 0 {
                *capture.lock().unwrap() = [0.; 4];
                s.step_exact(1)?;
                let backend = engine(&s);
                let q = DQuat::from_array(backend.state(r.foot).unwrap().orientation);
                if let Some(rec) = last_refinement(&s).filter(|e| e.iteration == tick) {
                    native = rec.interval.native_steps;
                    loads = *capture.lock().unwrap();
                    let mut site_data = vec![];
                    for h in &rec.interval.sliding_edge_impacts {
                        for p in h.sites {
                            // The registered sole's local x vertex order is
                            // fixed. Use its immutable contact-time orientation.
                            let j = DVec3::from(p.impulse_on_foot);
                            let at = (rec.start_time + p.elapsed).as_secs_f64();
                            let center = i.seed.state.c
                                + (i.edge.velocity + DVec3::from(i.seed.pad.velocity)) * at
                                + 0.5 * g * at * at
                                + DVec3::from(i.seed.pad.position)
                                + PAD;
                            let arm = i.seed.state.q.conjugate() * (DVec3::from(p.point) - center);
                            let k = usize::from(arm.x > 0.) * 2 + usize::from(arm.y > 0.);
                            loads[k] += j.dot(DVec3::from(p.normal));
                            site_data.push(json!({"site":k,"at_ns":p.elapsed.as_nanos()as u64,"j":p.impulse_on_foot,"post":p.after_relative_velocity}));
                        }
                    }
                    for h in &rec.interval.mixed_edge_impacts {
                        let at = (rec.start_time + h.elapsed).as_secs_f64();
                        let center = i.seed.state.c
                            + (i.edge.velocity + DVec3::from(i.seed.pad.velocity)) * at
                            + 0.5 * g * at * at
                            + DVec3::from(i.seed.pad.position)
                            + PAD;
                        for (idx, point) in h.points.iter().enumerate() {
                            let arm = i.seed.state.q.conjugate() * (DVec3::from(*point) - center);
                            let k = usize::from(arm.x > 0.) * 2 + usize::from(arm.y > 0.);
                            loads[k] += h.impulses.normal[idx];
                            site_data.push(json!({"site":k,"at_ns":h.elapsed.as_nanos()as u64,"normal_ns":h.impulses.normal[idx],"post":h.after_relative_velocity[idx],"mode":format!("{:?}",h.mode),"first_longitudinal_range_ns":h.impulses.first_longitudinal_range,"longitudinal_total_ns":h.impulses.longitudinal_total}));
                        }
                    }
                    let normal = rec.requested_pairs[0].normal_impulse;
                    if (normal - loads.iter().sum::<f64>()).abs() > 1e-10 {
                        return Err(
                            "native and projected edge loads do not sum to pair receipt".into()
                        );
                    }
                    events.push(json!({"tick":tick,"native_calls":native,"jump":site_data,"native_last_ns":rec.requested_pairs[0].last_native_normal_impulse,"interval_normal_ns":normal}));
                } else {
                    native = 1;
                    loads = moments(&backend.rigid_contact_patches(), &r, q, false)?.2;
                }
            }
            let (f, u, p) = departure::ingest(engine(&s), &r);
            frames.push(json!({"tick":tick,"time_us":tick*500,"native_steps":native,"solver_iterations":4,"selected":native>1,"weak_start":false,"secondary_start":false,"normal_impulse_ns":loads.iter().sum::<f64>(),"corner_ns":loads,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"gap_m":f.position[2]-p.position[2]-0.06-q_up(f).abs().dot(DVec3::new(0.18,0.1,0.06))}));
        }
        if label == "coupled" {
            fs::write(out.join("edge.sdf"), &src)?;
        }
        fs::write(
            out.join(format!("traces/{label}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":frames}),
            )?,
        )?;
        all.push(json!({"label":label,"events":events,"first_loaded":frames[5]}));
    }
    let lanes = [
        ("ordinary", "Ordinary Bevy solve"),
        ("refined", "Eight native calls"),
        (
            "coupled",
            if mixed {
                "Registered mixed edge"
            } else {
                "Registered sliding edge"
            },
        ),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":if mixed {"Fractional landing with one sticking vertex"}else{"Authored sliding edge at a fractional landing"},"capture_ticks":{"first_landing":5},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&all)?)?;
    Ok(())
}
pub(in super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut tangent = vec![];
    let mut flights = vec![];
    for (family, load, rev, roll) in [
        ("outer-heavy", [0.4, 0.8], false, 0.18),
        ("inner-heavy", [0.75, 0.45], true, 0.18),
        ("other-edge", [0.65, 0.55], false, -0.18),
        ("reverse-slide", [0.5, 0.7], true, -0.18),
    ] {
        for mu in [0.08, 0.2, 0.35, 0.5] {
            for yaw in [0., 27., 68., 112.] {
                let i = make(mu, f64::to_radians(yaw), roll, load, rev);
                for free in [false, true] {
                    for (name, frame) in [
                        (
                            "base",
                            Frame {
                                q: DQuat::IDENTITY,
                                shift: DVec3::ZERO,
                                spin: 0.,
                            },
                        ),
                        (
                            "inclined",
                            Frame {
                                q: DQuat::from_euler(glam::EulerRot::ZYX, 0.34, -0.27, 0.42),
                                shift: DVec3::new(2.3, -0.8, 0.6),
                                spin: 0.,
                            },
                        ),
                        (
                            "spinning-pad",
                            Frame {
                                q: DQuat::from_euler(glam::EulerRot::ZYX, -0.4, 0.34, -0.18),
                                shift: DVec3::new(-1.1, 1.8, -0.2),
                                spin: 1.3,
                            },
                        ),
                    ] {
                        tangent.push(json!({"case":family,"mu":mu,"yaw_deg":yaw,"free_child":free,"frame":name,"result":exact(i,frame,free).map_err(|e|format!("{family}, µ={mu}, yaw={yaw}, free={free}, {name}: {e}"))?}));
                    }
                }
            }
        }
    }
    for ns in [
        0.25, 1.25, 62500.37, 249999.4, 499999.34, 500001.61, 638197.61, 2138197.72,
    ] {
        for g in [DVec3::ZERO, DVec3::new(0.35, -0.23, -9.81)] {
            for yaw in [0., 0.7] {
                for mu in [0.08, 0.2] {
                    for frame in [
                        Frame {
                            q: DQuat::IDENTITY,
                            shift: DVec3::ZERO,
                            spin: 0.,
                        },
                        Frame {
                            q: DQuat::from_euler(glam::EulerRot::ZYX, 0.3, -0.21, 0.16),
                            shift: DVec3::new(-0.5, 0.8, 0.2),
                            spin: 0.,
                        },
                    ] {
                        let t = ns * 1e-9;
                        let value =
                            flight(input_for_flight(mu, yaw, t, g), frame, g, t).map_err(|e| {
                                format!("µ={mu}, yaw={yaw}, g={g:?}, q={:?}: {e}", frame.q)
                            })?;
                        flights.push(json!({"yaw":yaw,"mu":mu,"gravity":g.to_array(),"frame":frame.q.to_array(),"result":value}));
                    }
                }
            }
        }
    }
    fs::write(out.join("tangent.json"), serde_json::to_vec(&tangent)?)?;
    fs::write(out.join("flights.json"), serde_json::to_vec(&flights)?)?;
    record(out)
}
