// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/inclined_sheet.rs"]
mod sheet;
#[allow(dead_code)]
#[path = "../examples/support/slip_transition.rs"]
mod transition;
use gz_physics::BodyState;
use gz_physics_xpbd::Particle;
use gz_sim_scene::{
    deformable::articulated_impact::ArticulatedSupportInterval, reset_loaded_world,
};
use sheet::Incline;
use std::time::Duration;
#[derive(Default)]
struct History {
    loads: Vec<ArticulatedSupportInterval>,
    sample: Vec<([BodyState; 4], Vec<Particle>)>,
}
fn collect(g: &mut Incline) -> History {
    let mut h = History::default();
    while g.sim.elapsed() < Duration::from_millis(300) {
        g.step().unwrap();
        h.loads.extend_from_slice(g.material().support_this_step());
        if g.sim.elapsed().as_nanos().is_multiple_of(4_000_000) {
            h.sample
                .push((g.states(), g.material().particles().to_vec()));
        }
    }
    h
}
fn compare(a: &History, b: &History) {
    assert_eq!(a.loads.len(), b.loads.len());
    assert_eq!(a.sample.len(), 75);
    for (i, (r, s)) in a.loads.iter().zip(&b.loads).enumerate() {
        assert!(r == s, "physical loaded interval {i} differs");
    }
    assert_eq!(a.sample, b.sample);
}
#[test]
fn independent_publication_clocks_preserve_every_warm_native_material_state_and_physical_receipt() {
    for mass in [0., 0.002] {
        let source = transition::Run {
            speed: -0.42,
            mu: 0.15,
            width: 250000,
            transition: Some(2000),
            yaw: 0.,
            extra_mass: mass,
        }
        .sdf();
        let mut reference = None;
        for width in [1000000, 250000, 500000, 2000000, 4000000] {
            let xml = source.replace(
                "<max_step_size>0.001</max_step_size>",
                &format!("<max_step_size>{}</max_step_size>", width as f64 * 1e-9),
            );
            let mut g = Incline::new(&xml).unwrap();
            let actual = collect(&mut g);
            if let Some(r) = &reference {
                compare(r, &actual);
            } else {
                reference = Some(actual);
            }
            if width == 250000 {
                reset_loaded_world(&mut g.sim, |_| ()).unwrap();
                assert!(g.material().support_this_step().is_empty());
                compare(reference.as_ref().unwrap(), &collect(&mut g));
            }
        }
        let h = reference.unwrap();
        println!(
            "Extra {mass} kg, five public clocks plus reset, {} identical complete contact reports and 75 identical native and particle samples per run.",
            h.loads.len()
        );
    }
}
