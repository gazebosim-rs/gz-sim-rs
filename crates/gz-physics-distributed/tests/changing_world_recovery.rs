// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/changing_worlds.rs"]
mod fixture;
#[path = "../examples/support/changing_recovery.rs"]
mod trial;
use std::path::Path;

#[test]
fn alternating_loads_survive_handoff_retry_worker_death_and_restart() -> trial::Result<()> {
    let dir = tempfile::tempdir()?;
    let groups = fixture::seeds(fixture::Layout::Adjacent);
    let r = trial::qualify(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        &dir.path().join("native.gzpool"),
        &groups,
        trial::CHANGING,
        |_| Ok(()),
    )?;
    assert_eq!(
        r.exact_body_states,
        r.ticks as usize * groups.iter().map(Vec::len).sum::<usize>()
    );
    assert_eq!(
        r.loaded_pairs_at_events,
        [[0, 24], [0, 0], [24, 0], [0, 24]]
    );
    assert!(r.resumed_ticks >= 100 && r.changes_after_restore >= 3);
    Ok(())
}

#[test]
fn migrated_pad_lands_on_replacement_and_restores_its_loaded_contacts() -> trial::Result<()> {
    for layout in [fixture::Layout::Adjacent, fixture::Layout::Separated] {
        let dir = tempfile::tempdir()?;
        let groups = fixture::seeds(layout);
        let mut landing = false;
        let r = trial::qualify(
            Path::new(env!("CARGO_BIN_EXE_physics_worker")),
            &dir.path().join("native.gzpool"),
            &groups,
            trial::LANDING,
            |f| {
                if f.tick == trial::LANDING.replace + 1 {
                    assert_eq!(f.loaded_pairs, [24, 0]);
                    landing = true;
                }
                Ok(())
            },
        )?;
        assert!(landing);
        assert_eq!(r.exact_body_states, 21_700);
        assert_eq!(r.loaded_pairs_at_events, [[24, 0], [0, 0], [0, 0], [24, 0]]);
        assert_eq!(r.migrant_ids.len(), 25);
        assert_eq!(r.resumed_ticks, 125);
    }
    Ok(())
}

#[test]
fn loaded_pad_moves_to_a_worker_already_stepping_the_other_pad() -> trial::Result<()> {
    let dir = tempfile::tempdir()?;
    let groups = fixture::seeds(fixture::Layout::Separated);
    let pads: Vec<_> = groups
        .iter()
        .filter(|g| g.len() == 25)
        .map(|g| g[0].id)
        .collect();
    let report = trial::qualify_scenario(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        &dir.path().join("native.gzpool"),
        &groups,
        trial::Scenario {
            schedule: trial::LANDING,
            placement: fixture::Placement::Split,
            helper_limit: Some(0),
            split_wire: None,
        },
        |frame| {
            let owners: Vec<_> = pads
                .iter()
                .map(|id| {
                    let i = frame.bodies.binary_search_by_key(id, |b| b.id).unwrap();
                    frame.owners[i]
                })
                .collect();
            assert_eq!(owners, [u8::from(frame.tick >= trial::LANDING.migrate), 1]);
            if frame.tick == trial::LANDING.replace + 1 {
                assert_eq!(frame.loaded_pairs, [24, 0]);
            }
            Ok(())
        },
    )?;
    let usage = report.helpers.as_ref().expect("tracked explicit cap");
    assert_eq!(usage.through_handoff, [(0, 145)].into());
    assert_eq!(usage.after_handoff_through_restore, [(0, 80)].into());
    assert_eq!(usage.after_restore, [(0, 125)].into());
    assert_eq!(
        report.loaded_pairs_at_events,
        [[24, 0], [0, 0], [0, 0], [24, 0]]
    );
    assert_eq!(
        (
            report.exact_body_states,
            report.exact_pair_receipts,
            report.exact_floor_receipts
        ),
        (21700, 10056, 2800)
    );
    Ok(())
}
