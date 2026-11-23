// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Own native and material subintervals for the first freely approaching contact.
use super::*;
use gz_physics_multifidelity::ArticulatedSweep;
use gz_physics_xpbd::articulated_impact::{SphereMotion, SweptSphere};

fn advance_interval(
    world: &mut World,
    name: &str,
    dt: Duration,
    free_start: Option<Duration>,
) -> Result<(), MaterialImpactError> {
    if dt.is_zero() {
        return Ok(());
    }
    world.resource_mut::<PhysicsBackend>().step(dt)?;
    let mut soft = world.resource_mut::<Deformables>();
    let Deformables {
        initial, current, ..
    } = &mut *soft;
    let material = current.get_mut(name).ok_or(MaterialImpactError::Scene)?;
    if let Some(start) = free_start {
        let end = start.checked_add(dt).ok_or(InputError::InvalidValue)?;
        material.advance_articulated_flight_from(initial[name].particles(), start, end)?;
    } else {
        material.step(dt, &[])?;
    }
    Ok(())
}
fn tick(world: &mut World, info: UpdateInfo) -> Result<(), MaterialImpactError> {
    if !initial::matches_physics_owner(world) {
        return Err(InputError::InvalidPhysics.into());
    }
    let soft = world.resource::<Deformables>();
    let event = soft
        .automatic_impact
        .as_ref()
        .ok_or(MaterialImpactError::Scene)?
        .clone();
    let consumed = soft.automatic_consumed;
    let call = if consumed {
        None
    } else {
        let bodies = event
            .links
            .iter()
            .map(|&id| loaded_link(world, id))
            .collect::<Result<Vec<_>, _>>()?;
        let native = world
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or(InputError::InvalidPhysics)?;
        let motion = match native.articulated_sweep(&bodies, info.dt)? {
            ArticulatedSweep::Translation { velocity } => SphereMotion::Translate { velocity },
            ArticulatedSweep::Revolution {
                pivot,
                axis,
                angular_speed,
            } => SphereMotion::Revolve {
                pivot,
                axis,
                angular_speed,
            },
        };
        let shapes = bodies
            .into_iter()
            .map(|id| {
                let (center, radius) = native.articulated_sphere_geometry(id)?;
                Ok(SweptSphere {
                    sphere: ImpactSphere { center, radius },
                    motion,
                })
            })
            .collect::<Result<Vec<_>, PhysicsError>>()?;
        let body = &soft.current[&event.material];
        if event.sustained
            && info.start_time.is_zero()
            && let SphereMotion::Translate { velocity } = motion
        {
            let geometry = shapes.iter().map(|s| s.sphere).collect::<Vec<_>>();
            let gap = body.articulated_separations(&geometry)?.minimum();
            if (-1e-12..=1e-12).contains(&gap) {
                let mut seed = body.clone();
                let patch = seed.sphere_support_witnesses(&geometry, 1e-12)?;
                // Tangentially moving material can already be in sustained
                // contact at time zero. An incoming normal still requires
                // the ordinary impact solver and its separate restitution.
                let tangent = patch.sites().iter().all(|s| {
                    let v = s
                        .feature
                        .vertices()
                        .iter()
                        .zip(s.weights)
                        .map(|(&i, w)| w * DVec3::from(body.particles()[i].velocity))
                        .sum::<DVec3>()
                        - DVec3::from(velocity);
                    v.dot(DVec3::from(s.normal)).abs() <= 64. * f64::EPSILON * (1. + v.length())
                });
                if tangent && !patch.sites().is_empty() {
                    let width = event.returning_step.ok_or(MaterialImpactError::Scene)?;
                    return super::recurrent::advance(world, info, &event, None, true, width);
                }
            }
        }
        if event.returning_step.is_some() {
            body.first_swept_approach_cut(&shapes, info.dt)?
        } else {
            body.first_swept_sphere_impact(&shapes, info.dt)?
        }
    };
    if let Some(width) = event.returning_step {
        return super::recurrent::advance(world, info, &event, call, consumed, width);
    }
    let receipt = if let Some(cut) = call {
        let head = info
            .start_time
            .checked_add(cut.head())
            .ok_or(InputError::InvalidValue)?;
        advance_interval(world, &event.material, cut.head(), Some(info.start_time))?;
        let result = resolve_world(
            world,
            head,
            info.epoch,
            &event.material,
            &event.links,
            event.restitution,
            Some((&cut, info.start_time)),
        )?
        .ok_or(MaterialImpactError::MissingIncoming)?;
        advance_interval(world, &event.material, info.dt - cut.head(), None)?;
        Some(result)
    } else {
        advance_interval(
            world,
            &event.material,
            info.dt,
            (!consumed).then_some(info.start_time),
        )?;
        None
    };
    let mut soft = world.resource_mut::<Deformables>();
    soft.publish(info.sim_time, info.epoch);
    if let Some(receipt) = receipt {
        soft.samples
            .get_mut(&event.material)
            .expect("registered material sample")
            .record_impact(receipt);
        soft.automatic_consumed = true;
    }
    Ok(())
}
pub(crate) fn advance(world: &mut World) {
    let info = *world.resource::<UpdateInfo>();
    if !info.dt.is_zero()
        && let Err(e) = tick(world, info)
    {
        world.resource::<StepFault>().report(e);
    }
}
