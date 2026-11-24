// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_multifidelity::CoatedAnkle;

/// Distinct native joint, planted material and public clock.
#[derive(Clone, Copy, Debug)]
pub struct AnkleStep {
    pub time: UpdateInfo,
    pub foot: u64,
    pub shank: u64,
    pub panel: PanelStep,
    pub edge_margin_m: f64,
    pub shank_margin_m: f64,
}
#[derive(Resource)]
pub(super) struct Ankle {
    links: [(u64, BodyId); 3],
    material: CoatedAnkle,
    last: Option<AnkleStep>,
}
pub fn ankle_step(sim: &Simulation) -> Option<&AnkleStep> {
    sim.world().get_resource::<Ankle>()?.last.as_ref()
}
pub(crate) fn wrench_body(world: &World, entity: Entity, id: u64) -> Option<BodyId> {
    let ankle = world.get_resource::<Ankle>()?;
    let (_, body) = ankle.links[..2].iter().find(|(link, _)| *link == id)?;
    (world.get::<BodyHandle>(entity)?.id() == *body
        && crate::initial::matches_physics_owner(world)
        && crate::initial::owns_loaded_body(world, id, *body))
    .then_some(*body)
}
pub(super) fn retire(world: &mut World, removed: &[u64]) {
    if let Some(mut a) = world.get_resource_mut::<Ankle>()
        && a.links.iter().any(|p| removed.contains(&p.0))
    {
        a.last = None;
    }
}
pub(super) fn install(
    sim: &mut Simulation,
    backend: PhysicsBackend,
    links: [(u64, BodyId); 3],
    material: SoleMaterial,
    substeps: u32,
    depth: u8,
) -> Result<(), LoadError> {
    let [foot, shank, floor] = links.map(|p| p.1);
    let coating = CoatedAnkle {
        foot,
        shank,
        floor,
        substeps,
        depth,
        material,
    };
    backend
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or(PhysicsError::MissingBody)?
        .validate_coated_ankle(coating, sim.step_duration())?;
    gz_physics::install_with_step(sim, backend, advance)?;
    sim.world_mut().insert_resource(Ankle {
        links,
        material: coating,
        last: None,
    });
    sim.add_systems(
        Phase::Reset,
        (|mut c: ResMut<Ankle>| c.last = None).after(PhysicsReset),
    );
    Ok(())
}
fn advance(
    time: Res<UpdateInfo>,
    ids: EntityLookup,
    bodies: Query<&BodyHandle>,
    mut ankle: ResMut<Ankle>,
    mut backend: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    if time.dt.is_zero() {
        return;
    }
    let result = (|| -> Result<_, PhysicsError> {
        let native = backend
            .downcast_mut::<MultiFidelityBackend>()
            .ok_or(PhysicsError::MissingBody)?;
        let mut complete = true;
        for (id, expected) in ankle.links {
            match ids.get(id) {
                Some(e) if bodies.get(e).is_ok_and(|h| h.id() == expected) => {}
                None if native.state(expected).is_none() => complete = false,
                _ => return Err(PhysicsError::MissingBody),
            }
        }
        if complete {
            let reaction = native.step_coated_ankle(ankle.material, time.dt)?;
            let [foot, shank, floor] = ankle.links.map(|p| p.0);
            ankle.last = Some(AnkleStep {
                time: *time,
                foot,
                shank,
                panel: PanelStep {
                    support: floor,
                    deck: reaction.deck,
                    step: reaction.lower,
                    support_work_j: 0.,
                    angular_velocity: DVec3::ZERO,
                    spin_work_j: 0.,
                },
                edge_margin_m: reaction.edge_margin_m,
                shank_margin_m: reaction.shank_margin_m,
            });
        } else {
            native.step(time.dt)?;
            ankle.last = None;
        }
        Ok(())
    })();
    if let Err(error) = result {
        fault.report(error);
    }
}
