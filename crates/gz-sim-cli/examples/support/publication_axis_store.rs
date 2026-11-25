use super::*;
use bevy_ecs::change_detection::ContiguousMut;
use bevy_tasks::ComputeTaskPool;
use gz_sim_scene::EntityKind;
pub fn measure(sim: &mut gz_sim_core::Simulation) {
    let before = pages::snapshot(sim.world());
    let x = sim.elapsed().as_secs_f64() * 0.25;
    let mut query = sim
        .world_mut()
        .query::<(&EntityKind, &mut WorldPose, &mut LocalPose)>();
    let mut cases = [0usize; 2];
    for (k, w, l) in query.iter(sim.world()) {
        if *k != EntityKind::World {
            assert_eq!(w.translation[0].to_bits(), x.to_bits());
            cases[0] += 1;
        }
        if *k == EntityKind::Model {
            assert_eq!(l.0.translation[0].to_bits(), x.to_bits());
            cases[1] += 1;
        }
    }
    for (k, _, _) in query
        .query_mut(sim.world_mut())
        .contiguous_iter_mut()
        .unwrap()
    {
        if let Some(head) = k.first() {
            assert!(k.iter().all(|i| i == head));
        }
    }
    let mut samples = Vec::new();
    let pool = ComputeTaskPool::get_or_init(bevy_tasks::TaskPool::new);
    let mut check = 0;
    for i in 0..400 {
        let value = if i % 2 == 0 { x + 0.0125 } else { x };
        let start = Instant::now();
        let mut live = query.query_mut(sim.world_mut());
        pool.scope(|tasks| {
            for (k, w, l) in live.contiguous_iter_mut().unwrap() {
                let Some(&kind) = k.first() else { continue };
                if kind == EntityKind::World {
                    continue;
                }
                let batch = k.len().div_ceil(pool.thread_num()).max(2048);
                let (w, _) = ContiguousMut::bypass_change_detection_split(w);
                for block in w.chunks_mut(batch) {
                    tasks.spawn(async move {
                        for p in block {
                            p.translation[0] = value;
                        }
                    });
                }
                if kind == EntityKind::Model {
                    let (l, _) = ContiguousMut::bypass_change_detection_split(l);
                    for block in l.chunks_mut(batch) {
                        tasks.spawn(async move {
                            for p in block {
                                p.0.translation[0] = value;
                            }
                        });
                    }
                }
            }
        });
        let t = start.elapsed().as_nanos() as u64;
        if i >= 40 {
            samples.push(t);
        }
        if i == 10 {
            check = query
                .iter(sim.world())
                .filter(|(k, w, _)| {
                    **k != EntityKind::World && w.translation[0].to_bits() == value.to_bits()
                })
                .count();
        }
    }
    assert_eq!(check, cases[0]);
    let restored = pages::compare(&before, &pages::snapshot(sim.world()));
    for row in restored["table_rows"].as_array().unwrap() {
        assert_eq!(row[2][1], 0);
        assert_eq!(row[2][3], 0);
    }
    eprintln!(
        "AXIS {}",
        json!({"mutable_world_poses":cases[0],"mutable_model_locals":cases[1],
        "workers":pool.thread_num(),"samples_ns":samples,"restored":true})
    );
}
