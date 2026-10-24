use super::*;

#[test]
fn exhausted_ids_do_not_insert_a_body() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    backend.next_id = u64::MAX;
    assert_eq!(
        backend.spawn(BodySpec {
            shape: Shape::Sphere { radius: 1.0 },
            position: [0.0; 3],
            velocity: [0.0; 3],
            mass: Some(1.0)
        }),
        Err(PhysicsError::IdOverflow)
    );
    assert!(backend.world.bodies.is_empty());
}
