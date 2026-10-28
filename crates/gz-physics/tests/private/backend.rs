// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::Shape;

/// Records schedule calls and fails on request; it simulates no physics.
#[derive(Default)]
struct Recorder {
    owner: Arc<()>,
    steps: Vec<Duration>,
    resets: u32,
    fail: bool,
}

impl DynamicBackend for Recorder {
    fn engine(&self) -> &'static str {
        "recorder"
    }
    fn spawn_link(
        &mut self,
        _: BodySpec,
        _: RotationSpec,
        _: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        Err(PhysicsError::Unsupported {
            engine: self.engine(),
            capability: "bodies",
        })
    }
    fn is_dynamic(&self, _: BodyId) -> Option<bool> {
        None
    }
    fn state(&self, _: BodyId) -> Option<BodyState> {
        None
    }
    fn remove(&mut self, _: BodyId) -> bool {
        false
    }
    fn set_world_velocity(
        &mut self,
        _: BodyId,
        _: [f64; 3],
        _: [f64; 3],
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::MissingBody)
    }
    fn add_world_wrench(
        &mut self,
        _: BodyId,
        _: [f64; 3],
        _: [f64; 3],
    ) -> Result<(), PhysicsError> {
        Err(PhysicsError::MissingBody)
    }
    fn apply_impulse(&mut self, _: BodyId, _: [f64; 3], _: [f64; 3]) -> Result<(), PhysicsError> {
        Err(PhysicsError::MissingBody)
    }
    fn add_world_revolute(
        &mut self,
        _: BodyId,
        _: [f64; 3],
        _: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        Err(PhysicsError::MissingBody)
    }
    fn joint_child(&self, _: JointId) -> Option<BodyId> {
        None
    }
    fn joint_state(&self, _: JointId) -> Option<JointState> {
        None
    }
    fn remove_joint(&mut self, _: JointId) -> bool {
        false
    }
    fn is_jointed(&self, _: BodyId) -> Option<bool> {
        None
    }
    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        if self.fail {
            return Err(PhysicsError::Native {
                engine: self.engine(),
                message: "step".into(),
            });
        }
        self.steps.push(dt);
        Ok(())
    }
    fn reset(&mut self) -> Result<(), PhysicsError> {
        if self.fail {
            return Err(PhysicsError::Native {
                engine: self.engine(),
                message: "reset".into(),
            });
        }
        self.resets += 1;
        Ok(())
    }
    fn initial_state(&self) -> InitialPhysicsState {
        InitialPhysicsState::new(&self.owner, self.resets)
    }
    fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool {
        initial.is_owned_by(&self.owner)
    }
    fn prepare_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError> {
        let resets = *initial.data::<u32>(&self.owner)?;
        Ok(PreparedPhysicsReset::new(&self.owner, resets))
    }
    fn apply_initial(&mut self, prepared: PreparedPhysicsReset) -> Result<(), PhysicsError> {
        self.resets = prepared.into_data(&self.owner)?;
        Ok(())
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

fn recorder(sim: &Simulation) -> &Recorder {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}

#[test]
fn installed_backend_steps_only_advancing_ticks_and_resets_unless_restored() {
    let mut sim = Simulation::new(Duration::from_millis(2)).unwrap();
    install(&mut sim, Recorder::default()).unwrap();
    assert_eq!(
        install(&mut sim, Recorder::default()),
        Err(PhysicsError::AlreadyInstalled)
    );
    sim.step_exact(2).unwrap();
    assert_eq!(recorder(&sim).steps, [Duration::from_millis(2); 2]);
    sim.reset().unwrap();
    assert_eq!(recorder(&sim).resets, 1);
    let epoch = sim.world().resource::<UpdateInfo>().epoch + 1;
    sim.world_mut().insert_resource(RestoredPhysics { epoch });
    sim.reset().unwrap();
    assert_eq!(
        recorder(&sim).resets,
        1,
        "a restored epoch skips rebuilding"
    );
    let backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    assert_eq!(backend.engine(), "recorder");
    assert!(backend.into_inner().downcast_mut::<Recorder>().is_some());
}

#[test]
fn removed_backend_resource_cannot_be_installed_twice() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim, Recorder::default()).unwrap();
    sim.world_mut().remove_resource::<PhysicsBackend>();
    assert_eq!(
        install(&mut sim, Recorder::default()),
        Err(PhysicsError::AlreadyInstalled)
    );
}

#[test]
fn step_and_reset_failures_fault_the_simulation() {
    for reset in [false, true] {
        let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
        install(&mut sim, Recorder::default()).unwrap();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<Recorder>()
            .unwrap()
            .fail = true;
        let result = if reset {
            sim.reset()
        } else {
            sim.step_exact(1)
        };
        assert!(result.is_err());
        let fault = sim.fault().unwrap().downcast_ref::<PhysicsError>().unwrap();
        let expected = if reset {
            "recorder failed: reset"
        } else {
            "recorder failed: step"
        };
        assert_eq!(fault.to_string(), expected);
    }
}

#[test]
fn initial_state_is_bound_to_its_owner_and_type() {
    let mut backend = Recorder {
        resets: 3,
        ..Default::default()
    };
    let other = Recorder::default();
    let initial = backend.initial_state();
    assert!(backend.owns_initial_state(&initial));
    assert!(!other.owns_initial_state(&initial));
    assert_eq!(
        other.prepare_initial(&initial).err(),
        Some(PhysicsError::ForeignInitialState)
    );
    assert_eq!(
        initial.data::<u64>(&backend.owner).err(),
        Some(PhysicsError::ForeignInitialState)
    );
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.resets = 0;
    let mut foreign = Recorder::default();
    let wrong = backend.prepare_initial(&initial).unwrap();
    assert_eq!(
        foreign.apply_initial(wrong),
        Err(PhysicsError::ForeignInitialState)
    );
    let mistyped = PreparedPhysicsReset::new(&backend.owner, "not a count");
    assert_eq!(
        backend.apply_initial(mistyped),
        Err(PhysicsError::ForeignInitialState)
    );
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.resets, 3);
}

#[test]
fn identifiers_round_trip_and_errors_name_the_engine() {
    assert_eq!(BodyId::from_raw(7).raw(), 7);
    assert_eq!(JointId::from_raw(u64::MAX).raw(), u64::MAX);
    let mut backend = Recorder::default();
    let spec = BodySpec {
        shape: Shape::Sphere { radius: 1.0 },
        position: [0.0; 3],
        velocity: [0.0; 3],
        mass: Some(1.0),
    };
    assert_eq!(
        backend
            .spawn_link(spec, RotationSpec::default(), None)
            .unwrap_err()
            .to_string(),
        "recorder does not support bodies"
    );
}
