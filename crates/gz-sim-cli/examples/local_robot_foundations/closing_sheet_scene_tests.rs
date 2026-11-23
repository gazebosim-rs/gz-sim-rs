// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::WorldPose;

#[test]
fn calibrated_sheets_bind_every_material_vertex_at_the_correct_world_position() -> Result<()> {
    assert!(scene(4).is_err());
    for n in [5, 7] {
        let src = scene(n)?;
        assert_eq!(src.matches("<pad_sample ").count(), 16);
        let mut r = Recording::new(&src, "fold-closure.sdf", "default")?;
        for _ in 0..3 {
            let nodes = arm_sheet::cloth(&r.sim)?.particles();
            assert_eq!(nodes.len(), 4 * n);
            let mass: f64 = nodes.iter().map(|p| p.inverse_mass.recip()).sum();
            assert!((mass - 12. * NODE_MASS).abs() < 1e-15);
            for (i, p) in nodes.iter().enumerate() {
                let id = entity_id(&r.sim, &arm_sheet::name(i))?;
                let e = entity_by_id(r.sim.world(), id).unwrap();
                assert_eq!(
                    r.sim.world().get::<WorldPose>(e).unwrap().translation,
                    p.position
                );
            }
            r.advance()?;
        }
    }
    Ok(())
}
