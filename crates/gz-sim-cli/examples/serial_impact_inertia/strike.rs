// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::{DQuat, DVec3};
use serde_json::{Value, json};

#[derive(Serialize, Clone)]
pub(super) struct Instant {
    end_ns: u64,
    states: Vec<State>,
    jaw_impulse_ns: f64,
    accumulated_ns: f64,
}

#[derive(Resource, Serialize)]
pub(super) struct Audit {
    samples: Vec<Instant>,
    pulses: Vec<pulses::JawPulse>,
    native_calls: u32,
}

impl Audit {
    fn verify(&self, target: &Sample, committed: &[State]) -> Result<()> {
        if committed != target.bodies {
            return Err("observing contact changed the committed state of the robot".into());
        }
        let impulse = self.samples.last().unwrap().accumulated_ns;
        if (impulse - target.impact.left_ns - target.impact.right_ns).abs() > 1e-12 {
            return Err("observed and committed jaw impulses differ".into());
        }
        Ok(())
    }

    fn save(&mut self, sample: &Instant) {
        if self
            .samples
            .last()
            .is_none_or(|s| s.end_ns != sample.end_ns)
        {
            self.samples.push(sample.clone());
        }
    }
}

// Observe a separately initialized copy of the real first tick. Refuse to export
// when its publicly committed body states or total reaction differ from the source.
pub(super) fn audit(mut trial: Trial) -> Result<Audit> {
    trial.ticks = 1;
    let replay = trial.replay.take();
    let expected = run(trial.clone())?;
    let target = &expected.samples[0];
    let xml = fixture(&trial);
    let (mut sim, bodies, _) = load_robot(&trial, &xml)?;
    let ids = Contacts {
        left: bodies[3],
        right: bodies[4],
        parcel: bodies[5],
    };
    let start = Instant {
        end_ns: 0,
        states: states(&sim, &bodies),
        jaw_impulse_ns: 0.,
        accumulated_ns: 0.,
    };
    let calls = target.native_steps;
    let solver = expected.first_solver;
    let early = expected.first_early_schedule;
    let observed_bodies = bodies.clone();
    sim.initialize_schedules()?;
    assert_eq!(
        sim.remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?,
        1
    );
    sim.world_mut().insert_resource(Audit {
        samples: vec![start.clone()],
        pulses: vec![],
        native_calls: calls,
    });
    sim.add_systems(
        Phase::PhysicsStep,
        move |clock: Res<UpdateInfo>,
              mut backend: ResMut<PhysicsBackend>,
              mut data: ResMut<Audit>,
              fault: Res<StepFault>| {
            let native = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
            let schedule = ContactSegment {
                duration: clock.dt,
                native_steps: calls,
                iterations: solver.unwrap_or_else(|| native.ordinary_contact_iterations()),
            };
            let mut previous = start.clone();
            if let Err(e) = native.observe_contact_schedule(
                clock.dt,
                early
                    .as_ref()
                    .map_or(std::slice::from_ref(&schedule), |s| s.as_slice()),
                &[],
                |cut| {
                    let patches = cut.contact_patches();
                    pulses::record(&mut data.pulses, &patches, ids, cut.elapsed, cut.duration);
                    let impulse = patches
                        .iter()
                        .filter(|p| {
                            ids.counterpart(p.a, p.b)
                                .is_some_and(|(peer, _)| peer == ids.left || peer == ids.right)
                        })
                        .flat_map(|p| &p.points)
                        .map(|p| p.normal_impulse)
                        .sum::<f64>();
                    let sample = Instant {
                        end_ns: cut.elapsed.as_nanos() as u64,
                        states: observed_bodies
                            .iter()
                            .map(|id| State::from(cut.state(*id).unwrap()))
                            .collect(),
                        jaw_impulse_ns: impulse,
                        accumulated_ns: previous.accumulated_ns + impulse,
                    };
                    // Preserve both sides of every transition, and one context frame per
                    // 10 microseconds for long flight or sustained contact. No interpolation.
                    if (impulse > 0.) != (previous.jaw_impulse_ns > 0.) {
                        data.save(&previous);
                        data.save(&sample);
                    } else if sample.end_ns / 10_000 != previous.end_ns / 10_000
                        || cut.elapsed == clock.dt
                    {
                        data.save(&sample);
                    }
                    previous = sample;
                },
            ) {
                fault.report(e);
            }
        },
    );
    sim.step_exact(1)?;
    let data = sim.world_mut().remove_resource::<Audit>().unwrap();
    data.verify(target, &states(&sim, &bodies))?;
    if let Some(path) = replay {
        // The gauge exists only in the playback scene.
        let display_xml = xml.replace("</world>", r#"<model name="jaw_impulse_gauge"><static>true</static><link name="readout"><collision name="stand"><geometry><sphere><radius>0.001</radius></sphere></geometry></collision>
          <visual name="scale"><pose>-0.19 -0.12 0.59 0 0 0</pose><geometry><box><size>0.003 0.003 0.10</size></box></geometry><material><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
          <visual name="indicator"><pose>-0.19 -0.12 0.54 0 0 0</pose><geometry><sphere><radius>0.006</radius></sphere></geometry><material><diffuse>0.1 0.95 0.95 1</diffuse></material></visual>
          </link></model></world>"#);
        let sdf = sdformat::load_str(&display_xml, "serial-impact.sdf", &sdformat::NoResources)?;
        let (display, _, _) = load_robot(&trial, &display_xml)?;
        let frames = frames(
            &data,
            &gz_sim_cli::primitive_frame(&display, &sdf, "default", 1)?,
        )?;
        gz_sim_cli::export_archive(&path, Default::default(), frames)?;
    }
    Ok(data)
}

fn frames(data: &Audit, template: &(String, String)) -> Result<Vec<(String, String)>> {
    let initial: Value = serde_json::from_str(&template.0)?;
    let catalog: Value = serde_json::from_str(&template.1)?;
    let mut links = LINKS
        .map(|(name, _, _)| format!("default::gripper::{name}::"))
        .to_vec();
    links.push("default::parcel::box::".into());
    let mut result = Vec::with_capacity(data.samples.len());
    for (index, sample) in data.samples.iter().enumerate() {
        let mut frame = initial.clone();
        let mut catalog = catalog.clone();
        frame["sequence"] = json!(index + 1);
        frame["nanoseconds"] = json!(sample.end_ns);
        catalog["scene_sequence"] = json!(index + 1);
        for visual in frame["visuals"].as_array_mut().unwrap() {
            let name = visual["name"].as_str().unwrap();
            if name == "default::jaw_impulse_gauge::readout::indicator" {
                visual["translation"][2] = json!(0.54 + sample.accumulated_ns * 2.0);
                continue;
            }
            let Some(i) = links.iter().position(|prefix| name.starts_with(prefix)) else {
                continue;
            };
            let old = &data.samples[0].states[i];
            let new = &sample.states[i];
            let delta = DQuat::from_array(new.q) * DQuat::from_array(old.q).conjugate();
            let position =
                DVec3::from_array(serde_json::from_value(visual["translation"].clone())?);
            let rotation = DQuat::from_array(serde_json::from_value(visual["rotation"].clone())?);
            visual["translation"] = json!(
                (DVec3::from_array(new.xyz) + delta * (position - DVec3::from_array(old.xyz)))
                    .to_array()
            );
            visual["rotation"] = json!((delta * rotation).to_array());
        }
        result.push((
            serde_json::to_string(&frame)?,
            serde_json::to_string(&catalog)?,
        ));
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_matches_endpoint_visuals_and_brackets_the_native_load() {
        let mut trial = Args::try_parse_from([
            "study",
            "--ticks",
            "1",
            "--automatic",
            "--substeps",
            "64",
            "--impact-iterations",
            "8",
            "--impact-pgs",
            "8",
            "--guide-hz",
            "100000",
            "--divisor",
            "1.75",
            "--gap-um",
            "50",
        ])
        .unwrap()
        .trial;
        let reference = run(trial.clone()).unwrap();
        let folder = tempfile::tempdir().unwrap();
        let path = folder.path().join("capture");
        trial.replay = Some(path.clone());
        let mut observed = audit(trial.clone()).unwrap();
        assert_eq!(observed.pulses.len(), 1);
        let events = &observed.samples;
        let on = events.iter().position(|s| s.jaw_impulse_ns > 0.).unwrap();
        assert_eq!(events[on - 1].end_ns as u128, observed.pulses[0].start_ns);
        assert_eq!(events[on].end_ns - events[on - 1].end_ns, 15_625);
        assert!(events.windows(2).all(|w| w[0].end_ns < w[1].end_ns));
        let raw = std::fs::read_to_string(path.join("frames.jsonl")).unwrap();
        let decoded = raw
            .lines()
            .skip(1)
            .map(serde_json::from_str::<Value>)
            .collect::<std::result::Result<Vec<_>, _>>()
            .unwrap()
            .into_iter()
            .filter(|r| r["kind"] == "frame")
            .map(|r| serde_json::from_str(r["scene"].as_str().unwrap()))
            .collect::<std::result::Result<Vec<Value>, _>>()
            .unwrap();
        let xml = fixture(&trial);
        let sdf = sdformat::load_str(&xml, "serial-impact.sdf", &sdformat::NoResources).unwrap();
        let (mut ordinary, _, _) = load_robot(&trial, &xml).unwrap();
        ordinary.step_exact(1).unwrap();
        let end: Value = serde_json::from_str(
            &gz_sim_cli::primitive_frame(&ordinary, &sdf, "default", 1)
                .unwrap()
                .0,
        )
        .unwrap();
        let drawn = decoded.last().unwrap()["visuals"].as_array().unwrap();
        for b in end["visuals"].as_array().unwrap() {
            let a = drawn.iter().find(|a| a["name"] == b["name"]).unwrap();
            for k in ["translation", "rotation"] {
                for (u, v) in a[k]
                    .as_array()
                    .unwrap()
                    .iter()
                    .zip(b[k].as_array().unwrap())
                {
                    assert!((u.as_f64().unwrap() - v.as_f64().unwrap()).abs() < 1e-12);
                }
            }
        }
        let height = |i: usize| {
            decoded[i]["visuals"]
                .as_array()
                .unwrap()
                .iter()
                .find(|v| v["name"] == "default::jaw_impulse_gauge::readout::indicator")
                .unwrap()["translation"][2]
                .as_f64()
                .unwrap()
        };
        assert_eq!(height(on - 1), 0.54);
        assert_eq!(height(on), 0.54 + 2. * events[on].accumulated_ns);
        observed.samples.last_mut().unwrap().accumulated_ns += 0.001;
        assert!(
            observed
                .verify(&reference.samples[0], &reference.samples[0].bodies)
                .is_err()
        );
        assert!(observed.verify(&reference.samples[0], &[]).is_err());
    }

    #[test]
    fn invalid_visual_data_and_an_unwritable_recording_are_reported() {
        let mut trial = Args::try_parse_from(["study", "--miss"]).unwrap().trial;
        let observed = audit(trial.clone()).unwrap();
        for (scene, catalog) in [
            ("{", "{}"),
            ("{}", "{"),
            (
                r#"{"visuals":[{"name":"default::parcel::box::visual","translation":[1,2]}]}"#,
                "{}",
            ),
            (
                r#"{"visuals":[{"name":"default::parcel::box::visual","translation":[1,2,3],"rotation":[0]}]}"#,
                "{}",
            ),
        ] {
            assert!(frames(&observed, &(scene.into(), catalog.into())).is_err());
        }
        let occupied = tempfile::NamedTempFile::new().unwrap();
        trial.replay = Some(occupied.path().to_owned());
        assert!(audit(trial.clone()).is_err());
        trial.guide_hz = Some(-1.);
        assert!(audit(trial).is_err());
    }

    #[test]
    fn an_unloaded_first_tick_has_zero_pulses_and_ends_on_the_public_clock() {
        let trial = Args::try_parse_from(["study", "--miss", "--right", "--substeps", "20"])
            .unwrap()
            .trial;
        let observed = audit(trial).unwrap();
        assert!(observed.pulses.is_empty());
        assert_eq!(observed.samples.last().unwrap().end_ns, 1_000_000);
    }
}
