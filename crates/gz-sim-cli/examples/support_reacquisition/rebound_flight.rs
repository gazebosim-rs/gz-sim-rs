// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ballistic COM references after the last loaded rebound tick on two finite supports.
use super::*;

const G: DVec3 = DVec3::new(0., 0., -9.81);
const DT: f64 = STEP_US as f64 * 1e-6;
const EFFORTS: [(&str, Effort); 7] = [
    ("coarse", Effort::Standard),
    ("split2", Effort::Split(2)),
    ("fine", Effort::Split(4)),
    ("split8", Effort::Split(8)),
    ("iter1", Effort::Iterations(1)),
    ("iter8", Effort::Iterations(8)),
    ("iter16", Effort::Iterations(16)),
];
const SELECTED: Budget = Budget {
    onset: Effort::Split(4),
    follow: Effort::Iterations(16),
    follow_ticks: 1,
};

fn com_velocity(f: &ReboundFrame) -> DVec3 {
    (DVec3::from(f.foot.velocity) + DVec3::from(f.upper.velocity)) * 0.5
}

struct Flight {
    rig: Rig,
    donor: &'static str,
    name: &'static str,
    frames: Vec<ReboundFrame>,
    first_loaded: Option<usize>,
}

impl Flight {
    fn new(
        t: Target,
        cut: &ReboundFrame,
        donor: &'static str,
        prepare: Mode,
        name: &'static str,
        effort: Effort,
    ) -> Result<Self> {
        let mut rig = prefix(t)?;
        rig.set_mode(prepare);
        while rig.tick < cut.tick {
            rig.advance()?;
        }
        let entry = rig.frame()?;
        if !cut_matches(&entry, cut)
            || entry.normal_impulse_ns <= 0.
            || entry.native_steps != cut.native_steps
            || entry.solver_iterations != cut.solver_iterations
        {
            return Err(
                "post-impact cut differs from the donor pose, twist or last solved contact receipt"
                    .into(),
            );
        }
        rig.set_mode(Mode::Uniform(effort));
        Ok(Self {
            rig,
            donor,
            name,
            frames: vec![entry],
            first_loaded: None,
        })
    }

    fn advance(&mut self) -> Result<()> {
        self.rig.advance()?;
        let f = self.rig.frame()?;
        if f.normal_impulse_ns > 0. {
            if f.first_loaded_substep.is_none() {
                return Err("loaded return lacks native boundary receipt".into());
            }
            self.first_loaded.get_or_insert(self.frames.len());
        } else if f.first_loaded_substep.is_some() || f.corner_ns.iter().any(|&j| j != 0.) {
            return Err("unloaded flight has a solved contact receipt".into());
        }
        self.frames.push(f);
        Ok(())
    }
    fn clean_count(&self) -> usize {
        self.first_loaded.unwrap_or(self.frames.len()) - 1
    }

    fn result(&self, common: usize) -> Value {
        let cut = &self.frames[0];
        let p0 = com(cut);
        let v0 = com_velocity(cut);
        let snap = |i: usize| {
            let f = &self.frames[i];
            let elapsed = i as f64 * DT;
            let p = p0 + v0 * elapsed + G * (0.5 * elapsed * elapsed);
            let v = v0 + G * elapsed;
            let dp = com(f) - p;
            let dv = com_velocity(f) - v;
            let h = DT / (f.native_steps as usize * f.solver_iterations) as f64;
            let bias = G * (0.5 * elapsed * h);
            json!({"tick":f.tick,"elapsed_us":(f.tick-cut.tick)*STEP_US,
                "native_steps":f.native_steps,"iterations_per_step":f.solver_iterations,"substep_iteration_s":h,
                "computed_com_m":com(f).to_array(),"ballistic_com_m":p.to_array(),"position_error_m":dp.to_array(),
                "computed_velocity_m_s":com_velocity(f).to_array(),"ballistic_velocity_m_s":v.to_array(),"velocity_error_m_s":dv.to_array(),
                "simple_euler_bias_m":bias.to_array(),"position_minus_simple_euler_m":(dp-bias).to_array(),
                "normal_ns":f.normal_impulse_ns,"corner_ns":f.corner_ns,"supported_gap_m":f.gap_m})
        };
        let first=self.first_loaded.map(|i| {
            let f=&self.frames[i];
            let ordinal=f.first_loaded_substep.expect("positive native receipt");
            let native_ns=STEP_US*1000/u64::from(f.native_steps);
            json!({"tick":f.tick,"start_us":f.time_us-STEP_US,"native_steps":f.native_steps,"iterations_per_step":f.solver_iterations,
                "first_loaded_substep":ordinal,"first_loaded_native_end_ns":(f.time_us-STEP_US)*1000+u64::from(ordinal)*native_ns,
                "previous_normal_ns":self.frames[i-1].normal_impulse_ns,"previous_supported_gap_m":self.frames[i-1].gap_m,
                "outer_ns":f.normal_impulse_ns,"outer_corner_ns":f.corner_ns})
        });
        json!({"donor":self.donor,"effort":self.name,"cut_tick":cut.tick,
            "clean_ticks":self.clean_count(),"common_endpoint":snap(common),"own_clean_endpoint":snap(self.clean_count()),
            "clean_samples":(1..=self.clean_count()).map(snap).collect::<Vec<_>>(),"first_contact":first})
    }
}

fn experiment(deck: Deck, out: Option<&Path>) -> Result<Value> {
    let t = target(deck, false, 2)?.ok_or("missing training or partial-support rebound")?;
    let fine_mode = Mode::Fixed { onset: t.onset };
    let prior_sel = continuation(t, Mode::Budget(SELECTED))?;
    let prior_fine = continuation(t, fine_mode)?;
    let cut_tick = match deck {
        Deck::Flat => t.onset + 1,
        Deck::Edge => t.onset,
    };
    let at = |tick: u64| (tick - t.cut.tick - 1) as usize;
    let si = &prior_sel[at(cut_tick)];
    let fi = &prior_fine[at(cut_tick)];
    let donors = [
        ("selected", Mode::Budget(SELECTED), si),
        ("fine", fine_mode, fi),
    ];
    let mut flights = Vec::new();
    for (donor, mode, cut) in donors {
        for (name, e) in EFFORTS {
            flights.push(Flight::new(t, cut, donor, mode, name, e)?);
        }
    }
    // Retain the common clock across lanes so the rendered first returning
    // contacts use measured poses even when one grid loads before another.
    let prior_end = t.onset + WINDOW - 1;
    let end = moving_end(t.stroke);
    let mut tick = cut_tick;
    while tick < end && (flights.iter().any(|f| f.first_loaded.is_none()) || tick < prior_end) {
        for f in &mut flights {
            f.advance()?;
        }
        tick += 1;
    }
    let get = |donor, name| {
        flights
            .iter()
            .find(|r| r.donor == donor && r.name == name)
            .expect("declared donor and uniform grid")
    };
    let sc = get("selected", "coarse");
    let sf = get("selected", "fine");
    let fc = get("fine", "coarse");
    let ff = get("fine", "fine");
    let common = flights
        .iter()
        .map(Flight::clean_count)
        .min()
        .expect("uniform grid set");
    let later = usize::try_from(prior_end - cut_tick)?;
    // Use the original outer endpoint while the canonical paths are all still unloaded.
    if [sc, sf, fc, ff].iter().any(|x| x.clean_count() < later)
        || [&prior_sel, &prior_fine].iter().any(|v| {
            v[at(cut_tick) + 1..=at(prior_end)]
                .iter()
                .any(|f| f.normal_impulse_ns != 0.)
        })
    {
        return Err("canonical path loaded before the prior five-millisecond endpoint".into());
    }
    let field = |state: fn(&ReboundFrame) -> DVec3| {
        let a = state(&prior_sel[at(prior_end)]);
        let b = state(&prior_fine[at(prior_end)]);
        let sc = state(&sc.frames[later]);
        let sf = state(&sf.frames[later]);
        let fc = state(&fc.frames[later]);
        let ff = state(&ff.frames[later]);
        json!({"original_selected_minus_fine":(a-b).to_array(),
            "selected_schedule_minus_uniform_coarse":(a-sc).to_array(),
            "same_selected_cut_coarse_minus_fine":(sc-sf).to_array(),
            "both_fine_selected_cut_minus_fine_cut":(sf-ff).to_array(),
            "uniform_fine_minus_original_fine":(ff-b).to_array(),
            "same_fine_cut_coarse_minus_fine":(fc-ff).to_array(),
            "both_coarse_selected_cut_minus_fine_cut":(sc-fc).to_array(),
            "decomposition_roundoff":((a-b)-((a-sc)+(sc-sf)+(sf-ff)+(ff-b))).to_array()})
    };
    if let Some(path) = out {
        fs::create_dir_all(path.join("traces"))?;
        let stop = sc
            .first_loaded
            .ok_or("coarse lane did not return")?
            .max(sf.first_loaded.ok_or("fine lane did not return")?);
        for (name, run) in [("coarse", sc), ("fine", sf)] {
            let frames = run.frames[..=stop]
                .iter()
                .map(|f| f.with_effort())
                .collect::<Vec<_>>();
            write_json(
                &path.join(format!("traces/{name}.json")),
                &json!({"geometry":{"foot_half_extents":FOOT_HALF,
                "upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"cut_tick":cut_tick,"first_loaded_tick":run.first_loaded.map(|i|run.frames[i].tick),
                "policy":name,"frames":frames}),
            )?;
        }
    }
    let max_pad_difference = flights
        .iter()
        .flat_map(|f| f.frames.iter().zip(&sc.frames))
        .map(|(f, g)| DVec3::from(f.pad.position).distance(DVec3::from(g.pad.position)))
        .fold(0., f64::max);
    if max_pad_difference > 1e-12
        || flights.iter().flat_map(|f| f.frames.iter()).any(|f| {
            f.time_us != f.tick * STEP_US
                || f.pad.velocity != [0., 0., sweep::command((f.tick - 1) * STEP_US, 0).1]
        })
    {
        return Err("lift clock or motion diverged during the matched flight".into());
    }
    Ok(
        json!({"deck":deck.label(),"cut_tick":cut_tick,"common_clean_ticks":common,
        "prior_window_end_tick":prior_end,"max_pad_position_difference_m":max_pad_difference,
        "cut_selected":si.with_effort(),"cut_fine":fi.with_effort(),
        "results":flights.iter().map(|f|f.result(common)).collect::<Vec<_>>(),
        "reconciliation":{"position_m":field(com),"velocity_m_s":field(com_velocity),
        "cut_position_delta_m":(com(si)-com(fi)).to_array(),"cut_velocity_delta_m_s":(com_velocity(si)-com_velocity(fi)).to_array()}}),
    )
}

pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    let flat = experiment(Deck::Flat, Some(out))?;
    let edge = experiment(Deck::Edge, None)?;
    write_json(
        &out.join("flight.json"),
        &json!({"outer_us":STEP_US,"gravity_m_s2":G.to_array(),"cases":[flat,edge]}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn clean_flat_and_edge_flights_reconstruct_the_prior_window_and_resume_contact() -> Result<()> {
        for deck in [Deck::Flat, Deck::Edge] {
            let r = experiment(deck, None)?;
            assert_eq!(r["results"].as_array().unwrap().len(), 14);
            assert!(r["common_clean_ticks"].as_u64().unwrap() >= 1);
            for b in r["results"].as_array().unwrap() {
                assert!(b["first_contact"]["outer_ns"].as_f64().unwrap() > 0.);
                assert!(b["clean_ticks"].as_u64().unwrap() >= 1);
                let endpoint = &b["own_clean_endpoint"];
                assert_eq!(endpoint["normal_ns"], 0.);
                assert!(endpoint["velocity_error_m_s"][2].as_f64().unwrap().abs() < 1e-4);
            }
            for donor in ["selected", "fine"] {
                let find = |name: &str| {
                    r["results"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .find(|x| x["donor"] == donor && x["effort"] == name)
                        .unwrap()
                };
                let zerr = |name: &str| {
                    find(name)["common_endpoint"]["position_error_m"][2]
                        .as_f64()
                        .unwrap()
                };
                for (a, b) in [("coarse", "split2"), ("split2", "fine"), ("fine", "split8")] {
                    assert!((zerr(a) / zerr(b) - 2.).abs() < 1e-5);
                }
                for (a, b) in [("split2", "iter8"), ("fine", "iter16")] {
                    for (x, y) in find(a)["clean_samples"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .zip(find(b)["clean_samples"].as_array().unwrap())
                    {
                        assert_eq!(x["computed_com_m"], y["computed_com_m"]);
                        assert_eq!(x["computed_velocity_m_s"], y["computed_velocity_m_s"]);
                    }
                }
            }
            let reconciliation = &r["reconciliation"]["position_m"];
            assert!(
                reconciliation["decomposition_roundoff"][2]
                    .as_f64()
                    .unwrap()
                    .abs()
                    < 1e-15
            );
            if matches!(deck, Deck::Flat) {
                assert!(
                    (reconciliation["original_selected_minus_fine"][2]
                        .as_f64()
                        .unwrap()
                        + 1.8174741808034411e-6)
                        .abs()
                        < 1e-12
                );
            }
        }
        Ok(())
    }
}
