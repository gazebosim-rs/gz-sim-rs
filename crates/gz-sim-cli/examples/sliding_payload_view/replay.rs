use super::{Body, Result, Sample, Trace, soft_replay::Frame};
use bevy_ecs::hierarchy::ChildOf;
use glam::{DQuat, DVec3};
use gz_sim_scene::{
    LocalPose, Name, PhysicsEngine, VisualGeometry, VisualMaterial, load_world_with_sdf_step,
};
use serde_json::{Value, json};
use std::{path::Path, time::Duration};

const SDF: &str = include_str!("../../../../examples/multifidelity_sliding_payload.sdf");
const NAMES: [&str; 3] = ["arm::base", "arm::tip", "ram::head"];
// Orient the physical floor toward the viewer's back preset. Physical X is screen-right.
pub(super) const VIEW: DQuat = DQuat::from_xyzw(-0.5, 0.5, 0.5, -0.5);
pub(super) struct Visual {
    name: String,
    body: usize,
    offset: DVec3,
    rotation: DQuat,
    shape: Value,
    color: [f64; 4],
}

pub(super) fn visuals() -> Result<Vec<Visual>> {
    let sim = load_world_with_sdf_step(
        SDF,
        "sliding_payload.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut visuals = Vec::new();
    for entity in sim.world().iter_entities() {
        let Some(geometry) = entity.get::<VisualGeometry>() else {
            continue;
        };
        let parent = entity.get::<ChildOf>().ok_or("unattached visual")?.parent();
        let link = &sim
            .world()
            .get::<Name>(parent)
            .ok_or("missing link name")?
            .0;
        if link == "default::floor::ground" {
            continue;
        }
        let body = NAMES
            .iter()
            .position(|body| link.strip_prefix("default::") == Some(*body))
            .ok_or("unexpected visual owner")?;
        let name = entity
            .get::<Name>()
            .ok_or("missing visual name")?
            .0
            .strip_prefix("default::")
            .ok_or("unexpected world")?
            .replace("::", "_");
        let pose = entity.get::<LocalPose>().ok_or("missing local pose")?.0;
        let sdformat::Geometry::Box { size } = geometry.0.as_ref() else {
            return Err("expected authored cuboid".into());
        };
        let color = entity
            .get::<VisualMaterial>()
            .ok_or("missing material")?
            .0
            .diffuse
            .map(f64::from);
        visuals.push(Visual {
            name,
            body,
            offset: pose.translation.into(),
            rotation: DQuat::from_array(pose.rotation),
            shape: json!({"kind":"box","size":size}),
            color,
        });
    }
    visuals.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(visuals)
}

pub(super) fn add_bodies(
    frame: &mut Frame,
    lane: &str,
    bodies: &[Body; 3],
    shift: f64,
    visuals: &[Visual],
) -> Result<()> {
    for visual in visuals {
        let body = &bodies[visual.body];
        if body.name != NAMES[visual.body] {
            return Err("recorded body order disagrees with SDF".into());
        }
        let q = DQuat::from_array(body.orientation);
        frame.add(
            &format!("{lane}_{}", visual.name),
            VIEW * (DVec3::from(body.position) + q * visual.offset) + DVec3::Y * shift,
            VIEW * q * visual.rotation,
            visual.shape.clone(),
            visual.color,
        );
    }
    Ok(())
}
fn events(samples: &[Sample]) -> Result<(u64, u64)> {
    let switched = samples
        .iter()
        .find(|s| s.tier == "dynamic")
        .ok_or("automatic arm never switched")?
        .tick;
    let contact = samples
        .iter()
        .find(|s| s.automatic_impulse_ns > 0.)
        .ok_or("missing automatic impact")?
        .tick;
    if switched > contact {
        return Err("arm switched after impact".into());
    }
    Ok((switched, contact))
}

pub(super) fn export(trace: &Trace, output: &Path) -> Result<()> {
    let (promote, contact) = events(&trace.samples)?;
    let reference_contact = trace
        .samples
        .iter()
        .find(|s| s.reference_impulse_ns > 0.)
        .ok_or("missing reference contact")?
        .tick;
    let closest_pass = trace
        .samples
        .iter()
        .min_by(|a, b| {
            let offset =
                |s: &Sample| (s.near_miss[2].position[0] - s.near_miss[1].position[0]).abs();
            offset(a).total_cmp(&offset(b))
        })
        .ok_or("missing near-miss samples")?
        .tick;
    let visuals = visuals()?;
    let mut xmin = f64::INFINITY;
    let mut xmax = f64::NEG_INFINITY;
    let mut ymin = f64::INFINITY;
    let mut ymax = f64::NEG_INFINITY;
    for s in &trace.samples {
        for body in s.automatic.iter().chain(&s.reference).chain(&s.near_miss) {
            xmin = xmin.min(body.position[0]);
            xmax = xmax.max(body.position[0]);
            ymin = ymin.min(body.position[1]);
            ymax = ymax.max(body.position[1]);
        }
    }
    let width = xmax - xmin + 0.65;
    let spacing = width + 0.12;
    let height = ymax - ymin + 0.64;
    let floor_center = DVec3::new((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, -0.1);
    let mut retained = Vec::new();
    let mut frames = Vec::new();
    let miss_switch = trace
        .samples
        .iter()
        .find(|s| s.near_miss_tier == "dynamic")
        .map(|s| s.tick);
    let mut markers = vec![
        (promote, "hit_switch_to_dynamic"),
        (contact, "hit_payload_at_first_impulse"),
    ];
    if let Some(tick) = miss_switch {
        markers.push((tick, "miss_switch_to_dynamic"));
    }
    markers.sort_unstable();
    let points = [promote, contact, reference_contact, closest_pass];
    for (index, sample) in trace.samples.iter().enumerate() {
        if sample.tick != index as u64 || trace.dt_s != 0.001 {
            return Err("expected contiguous 1 ms samples".into());
        }
        if index % 15 != 0
            && index + 1 != trace.samples.len()
            && !points.iter().any(|tick| sample.tick.abs_diff(*tick) <= 2)
        {
            continue;
        }
        let mut frame = Frame::default();
        for (lane, bodies, shift) in [
            ("hit", &sample.automatic, spacing),
            ("reference", &sample.reference, 0.),
            ("miss", &sample.near_miss, -spacing),
        ] {
            let translate = |p: DVec3| VIEW * p + DVec3::Y * shift;
            frame.add(
                &format!("{lane}_floor"),
                translate(floor_center),
                VIEW,
                json!({"kind":"box","size":[width,height,0.2]}),
                [0.30, 0.36, 0.41, 1.],
            );
            add_bodies(&mut frame, lane, bodies, shift, &visuals)?;
        }
        // Append markers by first appearance so each archived object keeps its ID.
        for &(tick, name) in markers.iter().take_while(|(tick, _)| *tick <= sample.tick) {
            let (center, shift, color) = match name {
                "hit_switch_to_dynamic" => {
                    (sample.automatic[1].position, spacing, [0.31, 1., 0.30, 1.])
                }
                "hit_payload_at_first_impulse" => (
                    trace.samples[tick as usize].automatic[2].position,
                    spacing,
                    [1., 0.80, 0.22, 1.],
                ),
                _ => (sample.near_miss[1].position, -spacing, [0.31, 1., 0.30, 1.]),
            };
            let p = VIEW * (DVec3::from(center) + DVec3::Z * 0.24) + DVec3::Y * shift;
            frame.edge(
                name,
                p - DVec3::X * 0.025,
                p + DVec3::X * 0.025,
                0.05,
                0.08,
                color,
            );
        }
        retained.push(sample.tick);
        frames.push(frame.finish(Duration::from_millis(sample.tick), frames.len() + 1));
    }
    gz_sim_cli::export_archive(output, Default::default(), frames)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "milliseconds_since_launch":retained,"settle_s":trace.settle_s,"promotion_ms":promote,"first_impulse_ms":contact,
            "reference_first_impulse_ms":reference_contact,"closest_pass_ms":closest_pass,
            "spacing_m":spacing,"clipped_display_floor_m":[width,height,0.2],"display_rotation_xyzw":VIEW.to_array(),
            "authored_visuals_per_lane":visuals.len()
        }))?
    );
    Ok(())
}
