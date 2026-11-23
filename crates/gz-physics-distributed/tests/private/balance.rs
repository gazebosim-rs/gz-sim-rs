// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{
    physics::{Body, Snapshot, fixture},
    protocol::Worker,
};

struct Local {
    worker: Worker,
    drop_activation: bool,
    seed: Vec<Body>,
    id: u8,
}
impl Peer for Local {
    fn exchange(&mut self, r: &Request) -> Result<Reply, TransportError> {
        let reply = self.worker.handle(r.clone())?;
        if self.drop_activation
            && matches!(
                r.command,
                Command::Activate { .. } | Command::ActivateWarm { .. }
            )
        {
            self.drop_activation = false;
            return Err(TransportError::Unavailable(
                "lost load handoff activation".into(),
            ));
        }
        Ok(reply)
    }
    fn restart(&mut self) -> io::Result<()> {
        self.worker = Worker::new(
            31,
            self.id,
            Snapshot {
                bodies: self.seed.clone(),
                ..Default::default()
            },
        )
        .unwrap();
        Ok(())
    }
}
fn world(seeds: [Vec<Body>; 2]) -> Coordinator<Local> {
    let mut index = 0_u8;
    let peers = seeds.map(|seed| {
        let id = index;
        index += 1;
        let worker = Worker::new(
            31,
            id,
            Snapshot {
                bodies: seed.clone(),
                ..Default::default()
            },
        )
        .unwrap();
        Local {
            worker,
            id,
            seed,
            drop_activation: false,
        }
    });
    let mut c = Coordinator::new(31, peers, 128).unwrap();
    c.advance().unwrap();
    c
}
fn single(id: u64, x: f64) -> Body {
    Body {
        id,
        position: [0., x, 0.],
        linear: [0.; 3],
        ..fixture()[1].clone()
    }
}
fn sample(c: &Coordinator<Local>) -> LoadSample {
    LoadSample {
        window: LoadWindow {
            run: 31,
            serial: c.serial,
            start: c.frame.tick - 1,
            span: 1,
        },
        rows: c.frame.workers.each_ref().map(|w| {
            w.components
                .iter()
                .map(|c| ComponentWork {
                    members: c.ids.clone(),
                    elapsed_ns: 100,
                })
                .collect()
        }),
    }
}
#[test]
fn a_measured_ownership_cut_moves_toward_the_lighter_side_and_recovers_lost_activation() {
    for heavy in [0, 1] {
        let mut seeds = [Vec::new(), Vec::new()];
        seeds[heavy] = (1..=5).map(|id| single(id, id as f64 * 16.)).collect();
        seeds[1 - heavy].push(single(6, 96.));
        let mut c = world(seeds);
        let report = sample(&c);
        let p = BalancePolicy::default();
        let plan = c.plan_rebalance(&report, p).unwrap().unwrap();
        assert_eq!(
            (plan.axis, plan.source, plan.destination),
            (1, heavy as u8, (1 - heavy) as u8)
        );
        assert_eq!(plan.members, [4]);
        assert_eq!(plan.before_ns[heavy], 500);
        assert_eq!(plan.estimated_after_ns[heavy], 400);
        assert!(!plan.warm);
        c.peers[1 - heavy].drop_activation = true;
        let old = c.frame.clone();
        assert!(matches!(
            c.rebalance_one(&report, p),
            Err(Error::Transport(_))
        ));
        assert_eq!(c.frame, old);
        c.recover((1 - heavy) as u8).unwrap();
        assert_eq!(c.rebalance_one(&report, p).unwrap(), Some(plan));
        assert!(matches!(
            c.rebalance_one(&report, p),
            Err(Error::Invalid(_))
        ));
        assert_eq!(c.frame.body(4).unwrap().0, (1 - heavy) as u8);
        assert_eq!(c.frame.body(4).unwrap().1, old.body(4).unwrap().1);
        assert_eq!(c.frame.workers[1 - heavy].generations[&4], 1);
    }
}
#[test]
fn zero_gain_thresholds_locked_inputs_stale_reports_and_overflow_never_move_a_body() {
    let mut c = world([
        vec![single(1, 0.), single(2, 16.), single(3, 32.)],
        vec![single(4, 48.)],
    ]);
    let report = sample(&c);
    assert!(
        c.plan_rebalance(
            &report,
            BalancePolicy {
                minimum_savings_ns: 101,
                minimum_savings_per_mille: 0
            }
        )
        .unwrap()
        .is_none()
    );
    assert!(
        c.plan_rebalance(
            &report,
            BalancePolicy {
                minimum_savings_ns: 0,
                minimum_savings_per_mille: 334
            }
        )
        .unwrap()
        .is_none()
    );
    assert!(
        c.plan_rebalance(
            &report,
            BalancePolicy {
                minimum_savings_ns: 0,
                minimum_savings_per_mille: 1001
            }
        )
        .is_err()
    );
    let p = BalancePolicy::default();
    let mut bad = report.clone();
    bad.rows[0][0].members[0] = 99;
    assert!(c.plan_rebalance(&bad, p).is_err());
    bad = report.clone();
    bad.rows[0][0].elapsed_ns = u64::MAX;
    assert!(matches!(c.plan_rebalance(&bad, p), Err(Error::Capacity)));
    c.queue_impulse(3, [0.0; 3]).unwrap();
    // Either spatial orientation may win, but accepted inputs pin their owner.
    if let Some(m) = c.plan_rebalance(&report, p).unwrap() {
        assert!(!m.members.contains(&3));
    }
    c.queue_impulse(1, [0.; 3]).unwrap();
    c.queue_impulse(2, [0.; 3]).unwrap();
    assert!(c.plan_rebalance(&report, p).unwrap().is_none());
    c.advance().unwrap();
    assert!(c.plan_rebalance(&report, p).is_err());
    let balanced = world([vec![single(1, 0.)], vec![single(2, 16.)]]);
    assert!(
        balanced
            .plan_rebalance(&sample(&balanced), p)
            .unwrap()
            .is_none()
    );
}

#[test]
fn prefix_and_suffix_cuts_match_exhaustive_assignments_with_tombstones_and_pinned_units() {
    let mut rng = 0xff24731e3b9u64;
    let mut draw = |limit: u64| {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        (rng >> 24) % limit
    };
    let mut winners = 0;
    for trial in 0..1300 {
        let n = 4 + draw(53) as usize;
        let mut units = Vec::new();
        let mut before = [0u64; 2];
        for i in 0..n {
            let owner = usize::from(draw(7) < (trial % 6 + 1) as u64);
            let cost = draw(10000);
            let members = 1 + draw(8) as usize;
            before[owner] += cost;
            units.push(Unit {
                owner,
                position: std::array::from_fn(|_| draw(9) as f64),
                cost,
                movable: draw(30) != 0,
                members,
                first_id: (8 * i) as u64,
                new_ids: draw((members + 1) as u64) as usize,
            });
        }
        let policy = BalancePolicy {
            minimum_savings_ns: draw(3000),
            minimum_savings_per_mille: draw(120) as u16,
        };
        let options = units
            .iter()
            .map(|u| projected_peak(u, before, policy).unwrap())
            .collect::<Vec<_>>();
        let available = [draw(100) as usize, draw(100) as usize];
        let mut best = (before[0].max(before[1]), 0, 0, 0, 0);
        let mut expected = None;
        for axis in 0..3 {
            let mut order = (0..n).collect::<Vec<_>>();
            order.sort_by(|&i, &j| {
                units[i].position[axis]
                    .total_cmp(&units[j].position[axis])
                    .then(units[i].first_id.cmp(&units[j].first_id))
            });
            for cut in 1..n {
                if units[order[cut - 1]].position[axis] == units[order[cut]].position[axis] {
                    continue;
                }
                for left in 0..2 {
                    let mut load = [0u64; 2];
                    let mut used = [0; 2];
                    let mut moves = 0;
                    let mut allowed = true;
                    let mut best_move = None;
                    for (rank, &i) in order.iter().enumerate() {
                        let dest = if rank < cut { left } else { 1 - left };
                        let u = &units[i];
                        load[dest] += u.cost;
                        if dest != u.owner {
                            allowed &= u.movable;
                            used[dest] += u.new_ids;
                            moves += u.members;
                            if let Some(peak) = options[i] {
                                let candidate = (peak, u.members, u.first_id, i);
                                best_move = Some(
                                    best_move.map_or(candidate, |previous| candidate.min(previous)),
                                );
                            }
                        }
                    }
                    let key = (load[0].max(load[1]), moves, axis, cut, left);
                    if allowed
                        && used[0] <= available[0]
                        && used[1] <= available[1]
                        && key < best
                        && let Some((_, _, _, i)) = best_move
                    {
                        best = key;
                        expected = Some((axis, i));
                    }
                }
            }
        }
        assert_eq!(
            best_cut(&units, &options, available, before),
            expected,
            "trial {trial}"
        );
        winners += usize::from(expected.is_some());
    }
    assert!(winners > 100);
}
