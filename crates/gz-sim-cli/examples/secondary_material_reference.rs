// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A rigid-material reference for the actual archived, warm second-landing cuts.
use glam::{DQuat, DVec3};
use gz_physics::{
    FiniteArcEnd, FiniteBoxArrival, FiniteBoxFlight, FiniteFeatureArc, FiniteFeatureTrack,
    FinitePointSite, FlightBox, PointImpact, SphericalPointMaterial,
};
use serde_json::{Value, json};
use std::{collections::BTreeSet, error::Error, fs, path::Path, time::Duration};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const MAT: SphericalPointMaterial = SphericalPointMaterial {
    mass: 2.,
    inertia: 0.05,
    friction: 0.5,
};
const H: [f64; 3] = [0.18, 0.1, 0.06];
const P: [f64; 3] = [0.33, 0.18, 0.06];
fn v(x: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(x.clone())?))
}
fn b(x: &Value, half: [f64; 3]) -> Result<FlightBox> {
    Ok(FlightBox {
        center: v(&x["position"])?,
        orientation: DQuat::from_array(serde_json::from_value(x["orientation"].clone())?)
            .normalize(),
        half_extent: half.into(),
    })
}
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn write(p: &Path, row: &Value) -> Result<()> {
    fs::write(p, serde_json::to_vec(row)?)?;
    Ok(())
}
fn flight(d: &Value) -> Result<(FiniteBoxFlight, DVec3)> {
    let m = &d["incoming"]["motion"];
    let [a, u, p] = [&m["links"][0], &m["links"][1], &m["links"][2]];
    let foot = b(a, H)?;
    let upper = b(u, [0.05, 0.06, 0.06])?;
    let com = (foot.center + upper.center) * 0.5;
    let speed = (v(&a["velocity"])? + v(&u["velocity"])?) * 0.5;
    let spin = v(&a["angular_velocity"])?;
    let deck = v(&d["ordinary_onset"][0]["first_positive"]["before"][2]["velocity"])?;
    let defect = (foot.orientation * DVec3::Z * 0.14)
        .distance(upper.center - foot.center)
        .max(spin.distance(v(&u["angular_velocity"])?))
        .max((speed + spin.cross(foot.center - com)).distance(v(&a["velocity"])?))
        .max((1. - foot.orientation.dot(upper.orientation).abs()).abs());
    if defect > 1e-9 || v(&p["angular_velocity"])?.length() > 0. {
        return Err("recorded second landing is not the qualified locked body".into());
    }
    Ok((
        FiniteBoxFlight {
            moving: foot,
            support: b(p, P)?,
            center_of_mass: com,
            relative_velocity: speed - deck,
            angular_velocity: spin,
            acceleration: v(&m["gravity"])?,
        },
        deck,
    ))
}
// Independently enclose the subnanosecond zero using the maximum of the 15 signed supporting-plane gaps.
fn max_gap(a: FlightBox, b: FlightBox) -> f64 {
    let basis = [DVec3::X, DVec3::Y, DVec3::Z];
    let x = basis.map(|v| a.orientation * v);
    let y = basis.map(|v| b.orientation * v);
    x.into_iter()
        .chain(y)
        .chain(x.into_iter().flat_map(|v| y.map(|w| v.cross(w))))
        .filter_map(|u| {
            let n = u.try_normalize()?;
            if u.length_squared() < 1e-20 {
                return None;
            }
            Some(
                (a.center - b.center).dot(n).abs()
                    - a.half_extent.dot((a.orientation.conjugate() * n).abs())
                    - b.half_extent.dot((b.orientation.conjugate() * n).abs()),
            )
        })
        .fold(f64::NEG_INFINITY, f64::max)
}
#[derive(Clone, Copy)]
struct Onset {
    t: f64,
    flight: FiniteBoxFlight,
    site: FinitePointSite,
    impulse: DVec3,
    stuck: bool,
    loss: f64,
}
fn onset(f: FiniteBoxFlight) -> Result<(Value, Option<Onset>)> {
    let hit = match f.first_crossing(Duration::from_micros(500))? {
        FiniteBoxArrival::Crossing(q) => q,
        other => return Ok((json!({"status":format!("{other:?}")}), None)),
    };
    let mut l = hit.earliest.as_secs_f64();
    let mut r = hit.latest.as_secs_f64();
    if max_gap(f.at(l), f.support) < -2e-12 || max_gap(f.at(r), f.support) > 2e-12 {
        return Err("independent material bracket has wrong endpoint signs".into());
    }
    for _ in 0..38 {
        let q = (l + r) * 0.5;
        if max_gap(f.at(q), f.support) > 0. {
            l = q
        } else {
            r = q
        }
    }
    let t = (l + r) * 0.5;
    let at = FiniteBoxFlight {
        moving: f.at(t),
        center_of_mass: f.com_at(t),
        relative_velocity: f.relative_velocity + t * f.acceleration,
        ..f
    };
    let mut meta = json!({"status":"crossing","integer_bracket_ns":[hit.earliest.as_nanos() as u64,hit.latest.as_nanos() as u64],"time_s":t,"geometry_gap_m":max_gap(at.moving,at.support),"coarse_feature":format!("{:?}",hit.feature),"coarse_unique":hit.points.is_some()});
    let Some((feature, n, [p, pad], gap)) = at.contact_geometry(3e-12) else {
        meta["status"] = json!("unresolved_material_point");
        return Ok((meta, None));
    };
    let arm = p - at.center_of_mass;
    let before = at.relative_velocity + at.angular_velocity.cross(arm);
    let x = n.any_orthonormal_vector();
    let axes = [x, n.cross(x), n];
    let mobility = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            axes[i].dot(axes[j] / MAT.mass + arm.cross(axes[j]).cross(arm) / MAT.inertia)
        })
    });
    let law = PointImpact {
        inverse_mass: mobility,
        relative_velocity: axes.map(|a| a.dot(before)),
        friction: MAT.friction,
    };
    let Ok(s) = law.solve() else {
        meta["status"] = json!("unresolved_contact_law");
        return Ok((meta, None));
    };
    let j = axes
        .into_iter()
        .zip(s.impulse)
        .map(|(a, v)| a * v)
        .sum::<DVec3>();
    let after = FiniteBoxFlight {
        relative_velocity: at.relative_velocity + j / MAT.mass,
        angular_velocity: at.angular_velocity + arm.cross(j) / MAT.inertia,
        ..at
    };
    let site = FinitePointSite {
        point: p,
        normal: n,
        feature,
    };
    let load = after.first_point_load(site, MAT, s.sticking)?;
    let upper = FlightBox {
        center: 2. * at.center_of_mass - at.moving.center,
        orientation: at.moving.orientation,
        half_extent: DVec3::new(0.05, 0.06, 0.06),
    };
    meta["status"] = json!("unique");
    meta["feature"] = json!(format!("{feature:?}"));
    meta["site"] = json!({"point":p.to_array(),"support_point":pad.to_array(),"normal":n.to_array(),"signed_gap_m":gap,"com":at.center_of_mass.to_array(),"upper_separation_m":max_gap(upper,at.support)});
    meta["response"] = json!({"impulse_ns":j.to_array(),"angular_impulse_nms":arm.cross(j).to_array(),"sticking":s.sticking,"loss_j":s.dissipated_energy,"before_relative_point_m_s":before.to_array(),"after_relative_point_m_s":(after.relative_velocity+after.angular_velocity.cross(arm)).to_array(),"outgoing_mode":load.map(|v|format!("{:?}",v.mode))});
    Ok((
        meta,
        Some(Onset {
            t,
            flight: after,
            site,
            impulse: j,
            stuck: s.sticking,
            loss: s.dissipated_energy,
        }),
    ))
}
#[derive(Clone, Copy)]
struct End {
    f: FiniteBoxFlight,
    j: DVec3,
    jn: f64,
    jv: DVec3,
    l: DVec3,
    work: f64,
}
fn save_end(s: End, f: FiniteBoxFlight, t: f64, deck: DVec3) -> Value {
    let d = deck * t;
    let body = |at: DVec3| json!({"position":(at+d).to_array(),"orientation":s.f.moving.orientation.to_array(),"velocity":(s.f.relative_velocity+deck+s.f.angular_velocity.cross(at-s.f.center_of_mass)).to_array(),"angular_velocity":s.f.angular_velocity.to_array()});
    let p = s.f.support;
    json!({"total_impulse_ns":s.j.to_array(),"total_normal_ns":s.jn,"normal_vector_ns":s.jv.to_array(),"angular_impulse_nms":s.l.to_array(),"contact_work_j":s.work,
           "com_m":(s.f.center_of_mass+d).to_array(),"velocity_m_s":(s.f.relative_velocity+deck).to_array(),"spin_rad_s":s.f.angular_velocity.to_array(),
           "linear_residual_ns":(MAT.mass*(s.f.relative_velocity-f.relative_velocity-t*f.acceleration)-s.j).length(),
           "angular_residual_nms":(MAT.inertia*(s.f.angular_velocity-f.angular_velocity)-s.l).length(),
           "foot":body(s.f.moving.center),"upper":body(2.*s.f.center_of_mass-s.f.moving.center),"pad":{"position":(p.center+d).to_array(),"orientation":p.orientation.to_array(),"velocity":deck.to_array(),"angular_velocity":[0.,0.,0.]}})
}
fn trace(f: FiniteBoxFlight, o: Onset, deck: DVec3, t: f64, steps: u32) -> Result<Value> {
    if t < o.t {
        let motion = FiniteBoxFlight {
            moving: f.at(t),
            center_of_mass: f.com_at(t),
            relative_velocity: f.relative_velocity + t * f.acceleration,
            ..f
        };
        let end = End {
            f: motion,
            j: DVec3::ZERO,
            jn: 0.,
            jv: DVec3::ZERO,
            l: DVec3::ZERO,
            work: 0.,
        };
        return Ok(json!({"status":"flight","end":save_end(end,f,t,deck),"events":[]}));
    }
    let jn = o.impulse.dot(o.site.normal);
    let r = o.site.point - o.flight.center_of_mass;
    let mut end = End {
        f: o.flight,
        j: o.impulse,
        jn,
        jv: jn * o.site.normal,
        l: r.cross(o.impulse),
        work: -o.loss,
    };
    let Some(mut arc) = FiniteFeatureArc::new(o.flight, o.site, MAT, o.stuck, 1e-11)? else {
        return Ok(json!({"status":"unresolved_outgoing_load"}));
    };
    let mut time = o.t;
    let mut track = FiniteFeatureTrack::new(end.f, o.site)?;
    let mut events = vec![];
    for _ in 0..24 {
        let a = arc.measure_to(t - time, steps, 2e-17)?;
        end.j += a.impulse;
        end.jn += a.normal_impulse;
        end.jv += a.normal_impulse_vector;
        end.l += a.angular_impulse;
        end.work += a.friction_work;
        end.f = FiniteBoxFlight {
            moving: a.moving,
            center_of_mass: a.center,
            relative_velocity: a.velocity,
            angular_velocity: a.angular_velocity,
            ..end.f
        };
        match a.end {
            FiniteArcEnd::Horizon => {
                return Ok(
                    json!({"status":"complete","events":events,"end":save_end(end,f,t,deck)}),
                );
            }
            FiniteArcEnd::Feature(limit) => {
                let old = time;
                time += a.bracket[0];
                if time <= old + 1e-16 || time >= t {
                    return Ok(json!({"status":"indistinguishable_events","events":events}));
                }
                let Some(line) = track.incident_segment(a.moving, a.site, limit, 1e-10) else {
                    return Ok(
                        json!({"status":"unresolved_neighbor","time_s":time,"limit":format!("{limit:?}"),"feature":format!("{:?}",a.site.site.feature),"margins":a.site.margins().iter().map(|m|json!({"limit":format!("{:?}",m.limit),"gap_m":m.distance,"speed_m_s":m.speed})).collect::<Vec<_>>(),"events":events}),
                    );
                };
                let Some((h, boxes)) = line.spherical_endpoint_with_work(
                    a.center,
                    a.velocity,
                    a.angular_velocity,
                    MAT,
                    128,
                )?
                else {
                    return Ok(json!({"status":"unresolved_line_law","events":events}));
                };
                let dj = h.impulse;
                let dl = (line.moving_points[h.loaded_end] - a.center).cross(dj);
                end.j += dj;
                end.jn += dj.dot(line.normal);
                end.jv += dj.dot(line.normal) * line.normal;
                end.l += dl;
                end.work -= h.lost_energy;
                end.f.relative_velocity += dj / MAT.mass;
                end.f.angular_velocity += dl / MAT.inertia;
                events.push(json!({"time_s":time,"remaining_s":t-time,"impulse_ns":dj.to_array(),"normal":line.normal.to_array(),"work_boxes":boxes,"loaded_end":h.loaded_end}));
                let Some((next_track, _)) =
                    FiniteFeatureTrack::outgoing_endpoint(end.f, line, h.loaded_end)
                else {
                    return Ok(json!({"status":"unresolved_outgoing_feature","events":events}));
                };
                let Some(next) = FiniteFeatureArc::after_line_endpoint(
                    end.f,
                    line,
                    h.loaded_end,
                    MAT,
                    h.sticking,
                    1e-10,
                )?
                else {
                    return Ok(json!({"status":"unresolved_line_departure","events":events}));
                };
                arc = next;
                track = next_track;
            }
            mode => return Ok(json!({"status":format!("{mode:?}"),"events":events})),
        }
    }
    Ok(json!({"status":"event_limit","events":events}))
}
#[path = "support_reacquisition/secondary_quantized.rs"]
mod quantized;
fn main() -> Result<()> {
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() == 3 && args[0] == "--quantized-transfers" {
        return quantized::run(Path::new(&args[1]), Path::new(&args[2]));
    }
    if args.len() != 2 {
        return Err("secondary_material_reference ARTIFACTS OUTPUT".into());
    }
    let studies = Path::new(&args[0]);
    let out = Path::new(&args[1]).join("cases");
    fs::create_dir_all(&out)?;
    let base = studies.join("multifidelity-secondary-return-features/cases");
    for e in fs::read_dir(&base)? {
        let e = e?.path();
        if e.extension().is_none_or(|x| x != "json") {
            continue;
        }
        let d = read(&e)?;
        let (f, deck) = flight(&d)?;
        let (event, o) = onset(f)?;
        let mut result = json!({"case":d["case"],"slot":d["slot"],"held":d["held"],"cut_tick":d["cut_tick"],"incoming":d["incoming"],"pad_command_m_s":deck.to_array(),"material":event});
        let name = e.file_name().unwrap();
        if let Some(o) = o {
            let mut endpoints = (0..=8u64).map(|k| k * 62_500).collect::<BTreeSet<_>>();
            let natural = read(
                &studies
                    .join("multifidelity-secondary-natural/cases")
                    .join(name),
            )?;
            for r in [
                &d["ordinary_onset"],
                &d["treatments"],
                &natural["treatments"],
            ] {
                for t in r.as_array().ok_or("missing native cases")? {
                    if t["first_positive"].is_object() {
                        endpoints.insert(
                            t["first_ns"]
                                .as_u64()
                                .ok_or("missing time of native reaction")?,
                        );
                    }
                }
            }
            let mut targets = vec![];
            for ns in endpoints {
                let t = ns as f64 * 1e-9;
                let coarse = trace(f, o, deck, t, 32)?;
                let fine = trace(f, o, deck, t, 128)?;
                let diff = if coarse["end"].is_object() && fine["end"].is_object() {
                    Some(["total_impulse_ns", "com_m", "spin_rad_s"].map(|k| {
                        v(&coarse["end"][k])
                            .unwrap()
                            .distance(v(&fine["end"][k]).unwrap())
                    }))
                } else {
                    None
                };
                let accepted = diff.is_some_and(|d| d[0] < 1e-6 && d[1] < 1e-8 && d[2] < 1e-6)
                    && coarse["events"].as_array().map(Vec::len)
                        == fine["events"].as_array().map(Vec::len);
                targets.push(json!({"time_ns":ns,"coarse_status":coarse["status"],"model":fine,"mesh_difference":diff,"qualified":accepted}));
            }
            result["targets"] = json!(targets);
        }
        write(&out.join(name), &result)?;
    }
    Ok(())
}
