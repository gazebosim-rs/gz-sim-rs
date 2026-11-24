// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step};

fn source(entries: Option<&str>) -> String {
    let plugin=entries.map(|body|format!(r#"<plugin filename="gz-sim-scheduled-link-force-system" name="gz::sim::systems::ScheduledLinkForce">{body}</plugin>"#)).unwrap_or_default();
    format!(
        r#"<sdf version="1.12"><world name="test"><gravity>0 0 0</gravity><physics name="fixed" type="ignored"><max_step_size>0.001</max_step_size></physics>{plugin}
    <model name="box"><link name="link"><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name="shape"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></world></sdf>"#
    )
}
fn interval(begin: u64, end: u64, force: &str) -> String {
    format!(
        r#"<force link="test::box::link" begin_tick="{begin}" end_tick="{end}" xyz="{force}"/>"#
    )
}
fn load(xml: &str) -> Result<gz_sim_core::Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "scheduled.sdf",
        "test",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}

#[test]
fn half_open_world_forces_sum_and_repeat_after_reset() -> Result<(), Box<dyn std::error::Error>> {
    let mut native = load(&source(None))?;
    let both = interval(7, 17, "14 -2 1") + &interval(12, 20, "-4 5 -1");
    let mut scheduled = load(&source(Some(&both)))?;
    let id = native
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == "test::box::link")
                .then(|| e.get::<BodyHandle>().map(|v| v.id()))
                .flatten()
        })
        .unwrap();
    for _ in 0..2 {
        for t in 0..30 {
            let mut f = [0.; 3];
            if (7..17).contains(&t) {
                f = [14., -2., 1.];
            }
            if (12..20).contains(&t) {
                for (a, b) in f.iter_mut().zip([-4., 5., -1.]) {
                    *a += b;
                }
            }
            if f != [0.; 3] {
                native
                    .world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_world_wrench(id, f, [0.; 3])?;
            }
            native.step_exact(1)?;
            scheduled.step_exact(1)?;
            assert_eq!(
                native.world().resource::<PhysicsBackend>().state(id),
                scheduled.world().resource::<PhysicsBackend>().state(id),
                "accepted interval {t}"
            );
        }
        let v = scheduled
            .world()
            .resource::<PhysicsBackend>()
            .state(id)
            .unwrap()
            .velocity;
        for (actual, expected) in v.into_iter().zip([0.054, 0.01, 0.001]) {
            assert!((actual - expected).abs() < 1e-12, "{actual} vs {expected}");
        }
        gz_sim_scene::reset_loaded_world(&mut scheduled, |_| ())?;
        gz_sim_scene::reset_loaded_world(&mut native, |_| ())?;
    }
    Ok(())
}
#[test]
fn malformed_or_overflowing_schedule_is_rejected_at_load() {
    let valid = interval(1, 2, "1 0 0");
    for entries in [
        "".to_owned(),
        interval(3, 3, "1 0 0"),
        interval(2, 1, "1 0 0"),
        interval(1, 2, "NaN 0 0"),
        interval(1, 2, "1 0"),
        valid.replace("test::box::link", "missing"),
        valid.replace("begin_tick=", "unexpected="),
        interval(1, 5, "1e308 0 0") + &interval(2, 6, "1e308 0 0"),
    ] {
        assert!(load(&source(Some(&entries))).is_err(), "accepted {entries}");
    }
    assert!(
        load(&source(Some(&valid)).replace(
            "<link name=\"link\">",
            "<static>true</static><link name=\"link\">"
        ))
        .is_err()
    );
}
