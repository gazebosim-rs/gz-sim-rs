// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Remove the support in an isolated native world at a committed rebound boundary.
use super::*;
use gz_physics::{BodySpec, DynamicBackend, RotationSpec, Shape};

const G: DVec3 = DVec3::new(0., 0., -9.81);
const OUTER: Duration = Duration::from_micros(STEP_US);
const SELECTED: Budget = Budget {
    onset: Effort::Split(4),
    follow: Effort::Iterations(16),
    follow_ticks: 1,
};

#[derive(Clone, Copy)]
struct Motion {
    x: DVec3,
    v: DVec3,
}
impl Motion {
    fn pair(a: Body, b: Body) -> Self {
        Self {
            x: (DVec3::from(a.position) + DVec3::from(b.position)) * 0.5,
            v: (DVec3::from(a.velocity) + DVec3::from(b.velocity)) * 0.5,
        }
    }
    fn free(self, g: DVec3, t: f64) -> Self {
        Self {
            x: self.x + self.v * t + g * (0.5 * t * t),
            v: self.v + g * t,
        }
    }
}
fn read(system: &MultiFidelityBackend, ids: [BodyId; 2]) -> Result<Motion> {
    let [a, b] = ids.map(|id| {
        system
            .state(id)
            .map(Body::from)
            .ok_or("native flight body lost")
    });
    Ok(Motion::pair(a?, b?))
}

fn at_cut(t: Target, end: u64) -> Result<Rig> {
    let mut rig = prefix(t)?;
    rig.set_mode(Mode::Budget(SELECTED));
    while rig.tick < end {
        rig.advance()?;
    }
    Ok(rig)
}

struct Run {
    points: Vec<Motion>,
    initial: Motion,
    max_speed_error: f64,
}
fn evolve(
    system: &mut MultiFidelityBackend,
    ids: [BodyId; 2],
    g: DVec3,
    effort: Effort,
    count: usize,
) -> Result<Run> {
    effort.validate_grid(OUTER)?;
    let initial = read(system, ids)?;
    let mut points = Vec::with_capacity(count);
    let mut max_speed_error = 0.0_f64;
    for i in 1..=count {
        for substep in 1..=effort.native_steps() {
            effort.solve(system, effort.duration_at(OUTER, substep))?;
            // The narrow phase can retain an inactive pair for the adjacent links.
            for pair in system.rigid_contact_observations() {
                if !ids.contains(&pair.a)
                    || !ids.contains(&pair.b)
                    || pair.active
                    || pair.normal_impulse != 0.
                {
                    return Err(
                        format!("unexpected contact in unsupported control: {pair:?}").into(),
                    );
                }
            }
        }
        let m = read(system, ids)?;
        max_speed_error =
            max_speed_error.max(m.v.distance(initial.free(g, OUTER.as_secs_f64() * i as f64).v));
        points.push(m);
    }
    Ok(Run {
        points,
        initial,
        max_speed_error,
    })
}

#[derive(Clone, Copy, Debug)]
enum Model {
    NativeWithoutDeck,
    Rebuilt,
    SpinFree,
    SingleBody,
}
fn reduced(
    cut: ReboundFrame,
    g: DVec3,
    model: Model,
) -> Result<(MultiFidelityBackend, [BodyId; 2])> {
    let mut system = MultiFidelityBackend::new(g.to_array())?;
    let mut foot = cut.foot;
    let mut upper = cut.upper;
    let m = Motion::pair(foot, upper);
    let ids = match model {
        Model::SingleBody => {
            let id = system.spawn_link(
                BodySpec {
                    mass: Some(2.),
                    shape: Shape::Box {
                        half_extents: FOOT_HALF,
                    },
                    position: m.x.to_array(),
                    velocity: m.v.to_array(),
                },
                RotationSpec::default(),
                None,
            )?;
            // Using the same center twice gives this 2 kg control its exact COM.
            [id, id]
        }
        Model::Rebuilt | Model::SpinFree => {
            if matches!(model, Model::SpinFree) {
                for link in [&mut foot, &mut upper] {
                    link.velocity = m.v.to_array();
                    link.angular_velocity = [0.; 3];
                }
            }
            rigid_energy::spawn_locked_pair(&mut system, foot, upper)?
        }
        Model::NativeWithoutDeck => return Err("native control requires its live cut".into()),
    };
    Ok((system, ids))
}
fn native(t: Target, tick: u64, cut: ReboundFrame, effort: Effort, count: usize) -> Result<Run> {
    let mut r = at_cut(t, tick)?;
    if !cut_matches(&r.frame()?, &cut) {
        return Err("different incoming native flight cut".into());
    }
    let lane = &mut r.case.lane;
    let ids = [lane.foot, lane.upper];
    let mut resource = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let system = resource
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("missing native backend")?;
    let before = ids.map(|id| system.state(id).map(Body::from));
    if !system.remove(lane.pad)
        || system.state(lane.pad).is_some()
        || before != ids.map(|id| system.state(id).map(Body::from))
    {
        return Err("removing the deck altered the incoming foot or left the deck present".into());
    }
    evolve(system, ids, G, effort, count)
}
fn with_deck(t: Target, tick: u64, effort: Effort, count: usize) -> Result<Vec<Motion>> {
    let mut r = at_cut(t, tick)?;
    r.set_mode(Mode::Uniform(effort));
    let mut points = Vec::new();
    for _ in 0..count {
        r.advance()?;
        let f = r.frame()?;
        if f.normal_impulse_ns > 0. {
            break;
        }
        points.push(Motion::pair(f.foot, f.upper));
    }
    Ok(points)
}

fn record(deck: Deck) -> Result<Value> {
    let t = target(deck, false, 2)?.ok_or("missing secondary landing")?;
    let (tick, count) = match deck {
        Deck::Flat => (t.onset + 1, 8),
        Deck::Edge => (t.onset, 15),
    };
    let cut = at_cut(t, tick)?.frame()?;
    if cut.normal_impulse_ns <= 0. {
        return Err("expected a loaded incoming boundary".into());
    }
    let nominal = Motion::pair(cut.foot, cut.upper);
    let mut rows = Vec::new();
    for effort in [
        Effort::Iterations(1),
        Effort::Standard,
        Effort::Split(2),
        Effort::Iterations(8),
        Effort::Split(4),
        Effort::Iterations(16),
        Effort::Split(8),
    ] {
        let supported = with_deck(t, tick, effort, count)?;
        for (model, g) in [
            (Model::NativeWithoutDeck, G),
            (Model::Rebuilt, G),
            (Model::SpinFree, G),
            (Model::SingleBody, G),
            (Model::Rebuilt, DVec3::ZERO),
            (Model::SpinFree, DVec3::ZERO),
        ] {
            let run = if matches!(model, Model::NativeWithoutDeck) {
                native(t, tick, cut, effort, count)?
            } else {
                let (mut system, ids) = reduced(cut, g, model)?;
                evolve(&mut system, ids, g, effort, count)?
            };
            let last = run.points.last().ok_or("missing endpoint")?;
            let dt = OUTER.as_secs_f64();
            let span = dt * count as f64;
            let exact = run.initial.free(g, span);
            let bias = g
                * (0.5 * span * dt
                    / (f64::from(effort.native_steps()) * effort.iterations() as f64));
            let delta = last.x - exact.x;
            let native_comparison = matches!(model, Model::NativeWithoutDeck).then(|| {
                let worst = run
                    .points
                    .iter()
                    .zip(&supported)
                    .map(|(a, b)| a.x.distance(b.x))
                    .fold(0., f64::max);
                json!({"ticks_before_deck_load":supported.len(),"max_com_difference_m":worst})
            });
            rows.push(json!({"model":format!("{model:?}"),"gravity_m_s2":g.to_array(),
                "native_calls_per_tick":effort.native_steps(),"iterations_per_call":effort.iterations(),
                "initial_com_difference_m":nominal.x.distance(run.initial.x),"initial_velocity_difference_m_s":nominal.v.distance(run.initial.v),
                "endpoint_error_m":delta.to_array(),"endpoint_velocity_error_m_s":(last.v-exact.v).to_array(),
                "predicted_semi_implicit_error_m":bias.to_array(),"error_minus_prediction_m":(delta-bias).to_array(),
                "max_speed_error_m_s":run.max_speed_error,"deck_comparison":native_comparison}));
        }
    }
    Ok(json!({"deck":deck.label(),"cut_tick":tick,"outer_ticks":count,"rows":rows}))
}
pub(in super::super) fn write(out: &Path) -> Result<()> {
    write_json(
        &out.join("no-deck.json"),
        &json!({"flat":record(Deck::Flat)?,"edge":record(Deck::Edge)?}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn no_spin_matches_discrete_gravity_and_spinning_residual_survives_in_zero_gravity()
    -> Result<()> {
        let t = target(Deck::Flat, false, 2)?.ok_or("flat rebound")?;
        let cut = at_cut(t, t.onset + 1)?.frame()?;
        let n = 8;
        let span = n as f64 * OUTER.as_secs_f64();
        for effort in [Effort::Standard, Effort::Split(4)] {
            let h = OUTER.as_secs_f64()
                / (f64::from(effort.native_steps()) * effort.iterations() as f64);
            let bias = G * (span * h * 0.5);
            let error = |model, g| -> Result<DVec3> {
                let (mut s, ids) = reduced(cut, g, model)?;
                let run = evolve(&mut s, ids, g, effort, n)?;
                Ok(run.points[n - 1].x - run.initial.free(g, span).x)
            };
            assert!(error(Model::SingleBody, G)?.distance(bias) < 1e-12);
            assert!(error(Model::SpinFree, G)?.distance(bias) < 1e-12);
            let residual = error(Model::Rebuilt, G)? - bias;
            assert!(residual.z > 1e-9);
            assert!(residual.distance(error(Model::Rebuilt, DVec3::ZERO)?) < 1e-12);
        }
        Ok(())
    }
    #[test]
    fn removing_the_actual_support_preserves_the_cut_and_first_unloaded_trajectory() -> Result<()> {
        let t = target(Deck::Flat, false, 2)?.ok_or("flat rebound")?;
        let end = t.onset + 1;
        let cut = at_cut(t, end)?.frame()?;
        let free = native(t, end, cut, Effort::Standard, 8)?;
        let with = with_deck(t, end, Effort::Standard, 8)?;
        assert_eq!(with.len(), 8);
        for (a, b) in free.points.iter().zip(with) {
            assert!(a.x.distance(b.x) < 1e-12);
            assert!(a.v.distance(b.v) < 1e-12);
        }
        Ok(())
    }
}
