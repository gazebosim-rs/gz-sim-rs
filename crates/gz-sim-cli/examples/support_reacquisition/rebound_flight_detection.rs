// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Causal deck-top load forecasts observed beside, and outside, the native step.
use super::*;
use rebound_gate::projected_clearance_and_speed;

const RELEASE_M: f64 = 10e-6;
const NAMES: [&str; 6] = [
    "installed",
    "overlap_and_closing",
    "released_no_floor",
    "released_0.02",
    "released_0.04",
    "released_0.08",
];
#[derive(Serialize)]
struct Tick {
    tick: u64,
    parked: bool,
    previous_normal_ns: f64,
    incoming_gap_m: f64,
    flight_peak_m: f64,
    projected_gap_m: Option<f64>,
    penetrating_closing_m_s: Option<f64>,
    prior_prediction: [bool; 6],
    // A separate diagnostic from the same incoming state, holding the drive
    // fixed for two ticks. Index zero (the installed gate) has no such mode.
    two_tick_geometric_prediction: [bool; 6],
    secondary_start: bool,
    normal_ns: f64,
}

fn predictions(
    installed: bool,
    prior_loaded: bool,
    released: bool,
    closing: Option<f64>,
) -> [bool; 6] {
    let reaches = closing.is_some_and(|s| s > 0.);
    [
        installed,
        reaches,
        reaches && released,
        released && closing.is_some_and(|s| s >= 0.02),
        released && closing.is_some_and(|s| s >= 0.04),
        released && closing.is_some_and(|s| s >= 0.08),
    ]
    .map(|p| p && !prior_loaded)
}

fn trace(deck: Deck, switched: bool) -> Result<Vec<Tick>> {
    let budget = Budget {
        onset: Effort::Split(4),
        follow: Effort::Iterations(16),
        follow_ticks: 1,
    };
    let mut rig = Rig::new(deck, switched, Mode::Budget(budget))?;
    let mut shadow = ReboundGate::default();
    let mut peak = 0.0_f64;
    let mut seen_load = false;
    let mut rows = Vec::new();
    let mut pre = rig.frame()?;
    for tick in 1..=sweep::END_US / STEP_US {
        // Same commanded lift velocity PhysicsPrepare will apply before this
        // solve. The observer consumes only prior state and the published input.
        let (cycle, speed) = sweep::command((tick - 1) * STEP_US, 0);
        let foot = state(pre.foot);
        let mut pad = state(pre.pad);
        pad.velocity[2] = speed;
        let dt = STEP_US as f64 * 1e-6;
        let admission = shadow.admit(foot, pad, pre.normal_impulse_ns, 2, dt);
        let was_loaded = pre.normal_impulse_ns > 0.;
        if was_loaded {
            seen_load = true;
            peak = 0.;
        } else {
            peak = peak.max(pre.gap_m);
        }
        let endpoint = projected_clearance_and_speed(foot, pad, dt);
        let projected_gap = endpoint.map(|(g, _)| g);
        let closing = endpoint.and_then(|(_, s)| s);
        let secondary_start = admission == Admission::SecondaryStart;
        let installed = secondary_start || admission == Admission::Primary;
        let forecasts = predictions(
            installed,
            was_loaded,
            seen_load && peak > RELEASE_M,
            closing,
        );
        let second_horizon = projected_clearance_and_speed(foot, pad, 2. * dt).and_then(|(_, s)| s);
        let two_tick_geometric_prediction = predictions(
            false,
            was_loaded,
            seen_load && peak > RELEASE_M,
            second_horizon,
        );
        let parked = tick <= sweep::DEPART_US[0] / STEP_US
            || tick > sweep::DEPART_US[cycle] / STEP_US + STROKES[cycle].start_park();
        rig.advance()?;
        let f = rig.frame()?;
        if f.secondary_start != secondary_start {
            return Err("pre-solve shadow gate disagreed with live admission".into());
        }
        rows.push(Tick {
            tick,
            parked,
            previous_normal_ns: pre.normal_impulse_ns,
            incoming_gap_m: pre.gap_m,
            flight_peak_m: peak,
            projected_gap_m: projected_gap,
            penetrating_closing_m_s: closing,
            prior_prediction: forecasts,
            two_tick_geometric_prediction,
            secondary_start,
            normal_ns: f.normal_impulse_ns,
        });
        pre = f;
    }
    Ok(rows)
}

fn confusion(rows: &[Tick], policy: usize, accept: impl Fn(&Tick) -> bool) -> Value {
    let mut tp = 0;
    let mut fp = 0;
    let mut tn = 0;
    let mut missed = Vec::new();
    let mut false_ticks = Vec::new();
    let mut missed_normal = 0.;
    let mut reached_one_tick_late = Vec::new();
    for r in rows
        .iter()
        .filter(|r| r.previous_normal_ns == 0. && accept(r))
    {
        match (r.prior_prediction[policy], r.normal_ns > 0.) {
            (true, true) => tp += 1,
            (true, false) => {
                fp += 1;
                false_ticks.push(r.tick);
            }
            (false, true) => {
                missed.push(r.tick);
                if r.two_tick_geometric_prediction[policy] {
                    reached_one_tick_late.push(r.tick);
                }
                missed_normal += r.normal_ns;
            }
            (false, false) => tn += 1,
        }
    }
    json!({"true_positive":tp,"false_positive":fp,"true_negative":tn,"false_negative":missed.len(),
        "misses_that_project_contact_at_2dt_ticks":(policy!=0).then_some(reached_one_tick_late),
        "missed_first_impulse_ns":missed_normal,"false_alarm_ticks":false_ticks,"missed_ticks":missed})
}

// One unloading episode ends with its first positive native reaction. These
// offsets score an early warning even if the predictor also fires on impact.
fn episodes(rows: &[Tick]) -> Vec<Value> {
    let mut begin = 0;
    let mut events = Vec::new();
    for (i, r) in rows.iter().enumerate() {
        if r.previous_normal_ns > 0. {
            begin = i + 1;
            continue;
        }
        if r.normal_ns <= 0. {
            continue;
        }
        let offsets=(0..NAMES.len()).map(|p| {
            let first=rows[begin..=i].iter().find(|s|s.prior_prediction[p]).map(|s|s.tick as i64-r.tick as i64);
            json!({"policy":NAMES[p],"first_alert_offset_ticks":first,"hit_on_load_tick":r.prior_prediction[p],
                "miss_projects_contact_one_tick_late":(p!=0).then_some(first.is_none() && r.two_tick_geometric_prediction[p])})
        }).collect::<Vec<_>>();
        events.push(json!({"tick":r.tick,"unloaded_episode_start_tick":rows[begin].tick,"peak_gap_m":r.flight_peak_m,
            "incoming_gap_m":r.incoming_gap_m,"projected_gap_m":r.projected_gap_m,"penetrating_closing_m_s":r.penetrating_closing_m_s,
            "first_normal_ns":r.normal_ns,"parked":r.parked,"offsets":offsets}));
    }
    events
}

fn report(rows: &[Tick]) -> Value {
    let policies=NAMES.iter().enumerate().map(|(p,name)|json!({"policy":name,
        "all_previously_unloaded":confusion(rows,p,|_|true),
        "parked_previously_unloaded":confusion(rows,p,|r|r.parked),
        "released_flights":confusion(rows,p,|r|r.flight_peak_m>RELEASE_M),
        "submillimetre_released_flights":confusion(rows,p,|r|r.flight_peak_m>RELEASE_M && r.flight_peak_m<0.001)})).collect::<Vec<_>>();
    json!({"outer_ticks":rows.len(),"parked_outer_ticks":rows.iter().filter(|r|r.parked).count(),
        "previously_unloaded_ticks":rows.iter().filter(|r|r.previous_normal_ns==0.).count(),
        "policies":policies,"episodes":episodes(rows)})
}

pub(super) fn write(out: &Path) -> Result<()> {
    let mut runs = Vec::new();
    for deck in [Deck::Flat, Deck::Edge] {
        for switched in [false, true] {
            let rows = trace(deck, switched)?;
            let summary = report(&rows);
            runs.push(json!({"deck":deck.label(),"switched":switched,"summary":summary,
                "unloaded_ticks":rows.iter().filter(|r|r.previous_normal_ns==0.).collect::<Vec<_>>()}));
        }
    }
    write_json(
        &out.join("flight-detection.json"),
        &json!({"outer_us":STEP_US,"witness_m":RELEASE_M,"predictions_in_order":NAMES,"runs":runs}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn forecasts_require_upcoming_crossing_and_speed_floor_and_never_retrigger_a_loaded_step() {
        assert_eq!(
            predictions(false, false, true, Some(0.03)),
            [false, true, true, true, false, false]
        );
        assert_eq!(
            predictions(false, false, false, Some(0.03)),
            [false, true, false, false, false, false]
        );
        assert_eq!(predictions(false, false, true, None), [false; 6]);
        assert_eq!(predictions(false, false, true, Some(-0.01)), [false; 6]);
        assert_eq!(predictions(true, true, true, Some(1.)), [false; 6]);
    }

    #[test]
    fn edge_weak_return_is_detected_prospectively_after_its_unloaded_prefix() -> Result<()> {
        let rows = trace(Deck::Edge, false)?;
        let event = &rows[1485 - 1];
        assert_eq!(event.previous_normal_ns, 0.);
        assert!(event.incoming_gap_m > 0. && event.flight_peak_m > RELEASE_M);
        assert!(event.normal_ns > 0. && !event.prior_prediction[0] && event.prior_prediction[2]);
        assert!(!event.secondary_start);
        let entry = episodes(&rows)
            .into_iter()
            .find(|e| e["tick"] == 1485)
            .unwrap();
        assert!(
            entry["offsets"][2]["first_alert_offset_ticks"]
                .as_i64()
                .unwrap()
                <= 0
        );
        Ok(())
    }
}
