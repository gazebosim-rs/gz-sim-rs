// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare two real static receiving surfaces to the matched infinite-plane reference.
use super::*;
const COLS: usize = 5;
#[derive(Clone, Copy, PartialEq, Eq)]
enum Support {
    Plane,
    Receiver,
    Miss,
}
impl Support {
    fn label(self) -> &'static str {
        match self {
            Self::Plane => "plane",
            Self::Receiver => "receiver",
            Self::Miss => "miss",
        }
    }
    fn scene(self, base: &str) -> Result<String> {
        if self == Self::Plane {
            return Ok(base.to_owned());
        }
        let plane = r#"<plane point="0 0 0.12" normal="0 0 1"/>"#;
        let target = "<pose>0 .32 .06 0 0 0</pose>";
        if base.matches(plane).count() != 1 || base.matches(target).count() != 1 {
            return Err("folded sheet benches differ from the qualified fixture".into());
        }
        let boxes = concat!(
            r#"<box link="default::source::body" coupling="one_way"/>"#,
            r#"<box link="default::target::body" coupling="one_way"/>"#
        );
        let mut scene = base.replace(plane, boxes);
        if self == Self::Receiver {
            scene = scene.replace(target, "<pose>.32 1.47 .06 0 0 0</pose>");
        }
        Ok(scene)
    }
}
fn branch(mode: Support, base: &str, out: &Path) -> Result<Value> {
    let scene = mode.scene(base)?;
    let mut g = Grasp::pickup(&scene, COLS)?;
    g.acquire(MAX_BLOCKS)?;
    let baseline = g
        .trace
        .last()
        .expect("acquire steps and verifies a supported grip")
        .clone();
    let anchor = g.frames.len() - 1;
    let mut supports = Vec::new();
    for model in ["source", "target"] {
        let link = entity_id(&g.recording.sim, &format!("default::{model}::body"))?;
        let pose = state(&g.recording.sim, link)?;
        supports.push((
            DVec3::from(pose.position),
            DQuat::from_array(pose.orientation),
            arm_cable::half_size(&g.recording.sdf, model, "body")?,
        ));
    }
    let momentum = |points: &[Particle]| -> DVec3 {
        points
            .iter()
            .map(|p| DVec3::from(p.velocity) / p.inverse_mass)
            .sum()
    };
    let mut previous = momentum(arm_sheet::cloth(&g.recording.sim)?.particles());
    let mut observations = vec![];
    let mut marks = serde_json::Map::new();
    marks.insert("held".into(), json!(anchor));
    let mut impact = None;
    let mut below = None;
    for tick in 1..=400 {
        let (phase, force) = pulse(tick, 1.25);
        g.step(phase, force)?;
        let clot = arm_sheet::cloth(&g.recording.sim)?;
        let j = clot.last_boundary_impulses();
        let source = j[2].linear;
        let receiver = j.get(3).map_or([0.; 3], |r| r.linear);
        let pts = clot.particles();
        let bounds = std::array::from_fn::<_, 3, _>(|i| {
            pts.iter()
                .map(|p| p.position[i])
                .fold([f64::INFINITY, f64::NEG_INFINITY], |[a, b], x| {
                    [a.min(x), b.max(x)]
                })
        });
        let horizontal: Vec<f64> = supports
            .iter()
            .map(|&(center, rotation, half)| {
                pts.iter()
                    .map(|p| {
                        let local = rotation.conjugate() * (DVec3::from(p.position) - center);
                        let gap = (local.abs() - half).max(DVec3::ZERO);
                        gap.x.hypot(gap.y) - arm_sheet::RADIUS
                    })
                    .fold(f64::INFINITY, f64::min)
            })
            .collect();
        let next = momentum(pts);
        let impulses: DVec3 = j.iter().map(|r| DVec3::from(r.linear)).sum();
        let balance = next - previous + impulses - DT * DVec3::new(0., force, -g.weight_n);
        previous = next;
        observations.push(json!({"tick":tick,"time_ns":g.recording.sim.elapsed().as_nanos(),
            "source_or_plane_impulse_ns":source,"receiver_impulse_ns":receiver,"node_bounds_m":bounds,
            "horizontal_skin_clearance_m":horizontal,"momentum_balance_ns":balance.to_array()}));
        if mode == Support::Plane && -source[2] > 1e-12
            || mode != Support::Plane && -receiver[2] > 1e-12
        {
            impact = Some(tick);
        }
        if mode == Support::Miss && bounds[2][1] + arm_sheet::RADIUS < arm_sheet::BENCH - 0.01 {
            below = Some(tick);
        }
        if impact.is_some() || below.is_some() {
            if !tick.is_multiple_of(STRIDE) {
                arm_sheet::capture(&mut g.recording, &mut g.frames)?;
            }
            marks.insert(
                if impact.is_some() {
                    "touchdown"
                } else {
                    "below_benches"
                }
                .into(),
                json!(g.frames.len() - 1),
            );
            break;
        }
        if tick.is_multiple_of(PERIOD) {
            g.feedback();
        }
    }
    let last = observations
        .last()
        .expect("the first public tick was measured");
    let outcome = json!({"mode":mode.label(),"pickup_time_ns":baseline.time_ns,"pickup_jaw_up_n":baseline.jaw_up_n,"pickup_bench_up_n":baseline.bench_up_n,"pickup_clearance_m":baseline.min_bench_clearance_m,
        "touchdown_tick":impact,"passed_below_tick":below,"terminal":last,"weight_n":g.weight_n});
    let dir = out.join(mode.label());
    marks.insert("start".into(), json!(0));
    save_with_captures(
        g.recording,
        g.frames,
        &g.trace,
        &scene,
        &dir,
        COLS,
        ("perspective", json!(marks)),
    )?;
    write_observations(&dir, &observations, &outcome)?;
    if (mode == Support::Miss && below.is_none()) || (mode != Support::Miss && impact.is_none()) {
        return Err(format!("{} did not reach the measured endpoint", mode.label()).into());
    }
    Ok(outcome)
}
fn write_observations(dir: &Path, frames: &[Value], outcome: &Value) -> Result<()> {
    write_json(&dir.join("bench-trace.json"), &json!(frames))?;
    write_json(&dir.join("outcome.json"), outcome)
}
pub(crate) fn run(out: &Path) -> Result<()> {
    let base = closing_sheet_scene::scene(COLS)?;
    let mut outcomes = Vec::new();
    for kind in [Support::Receiver, Support::Miss, Support::Plane] {
        outcomes.push(branch(kind, &base, out)?);
    }
    write_json(&out.join("comparison.json"), &json!(outcomes))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn validates_the_authored_plane_and_receiver_before_running_the_trial() {
        let d = tempfile::tempdir().unwrap();
        assert!(
            branch(Support::Receiver, "<sdf/>", d.path())
                .unwrap_err()
                .to_string()
                .contains("qualified fixture")
        );
        assert!(
            branch(
                Support::Miss,
                r#"<plane point="0 0 0.12" normal="0 0 1"/>"#,
                d.path()
            )
            .is_err()
        );
        assert!(branch(Support::Plane, "malformed SDF", d.path()).is_err());
        assert_eq!(d.path().read_dir().unwrap().count(), 0);
    }
    #[test]
    fn both_observation_files_must_be_writable() {
        let d = tempfile::tempdir().unwrap();
        let missing = d.path().join("absent");
        assert!(write_observations(&missing, &[], &Value::Null).is_err());
        fs::create_dir(d.path().join("outcome.json")).unwrap();
        assert!(write_observations(d.path(), &[], &Value::Null).is_err());
    }
}
