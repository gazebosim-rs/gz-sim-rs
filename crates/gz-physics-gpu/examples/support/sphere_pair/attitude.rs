// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    Result, population,
    primitive_replay_view::View,
    reference::{self, Q, V},
};
use gz_physics_gpu::{SpherePairBatch, SpherePairBlockBatch, SphereRotation};
use serde_json::json;
use std::{fs, path::Path, time::Instant};

fn error(a: [f64; 4], b: [f64; 4]) -> f64 {
    let (a, b) = (Q::from_array(a), Q::from_array(b));
    4. * ((a - b).length().min((a + b).length()) * 0.5)
        .min(1.)
        .asin()
}
pub(super) fn compare(adapter: &str, seed: u64, n: usize, ticks: u32) -> Result<()> {
    let initial = population(seed, n);
    let mut old = SpherePairBlockBatch::new(adapter, initial.clone())?;
    let mut accurate =
        SpherePairBlockBatch::with_rotation(adapter, initial.clone(), SphereRotation::FourthOrder)?;
    let mut peak = [0f64; 2];
    let mut sum = [0.; 2];
    let mut physical = 0u64;
    let mut eval = |a: &SpherePairBlockBatch, b: &SpherePairBlockBatch| -> Result<()> {
        for (i, ((s, x), y)) in initial.iter().zip(a.lanes()).zip(b.lanes()).enumerate() {
            let mut ref_state = *s;
            reference::exact_interval(&mut ref_state, ticks as f64 * 0.001);
            let mut attitude_only = x.state;
            attitude_only.a.orientation = y.state.a.orientation;
            attitude_only.b.orientation = y.state.b.orientation;
            if bytemuck::bytes_of(&attitude_only) != bytemuck::bytes_of(&y.state)
                || x.impulse_on_b != y.impulse_on_b
                || x.contact_times() != y.contact_times()
                || x.contact_ticks() != y.contact_ticks()
            {
                return Err(
                    format!("rotation changed physical motion for seed {seed}, lane {i}").into(),
                );
            }
            for (actual, other, expect) in [
                (x.state.a, y.state.a, ref_state.a),
                (x.state.b, y.state.b, ref_state.b),
            ] {
                let mid = error(actual.orientation, expect.orientation);
                let improved = error(other.orientation, expect.orientation);
                if improved > mid + 2e-8 || improved > 0.00005 {
                    return Err(format!("unexpected attitude errors, seed {seed} lane {i}, midpoint {mid}, fourth {improved}").into());
                }
                for (j, e) in [mid, improved].into_iter().enumerate() {
                    peak[j] = peak[j].max(e);
                    sum[j] += e;
                }
                physical += 1;
            }
        }
        Ok(())
    };
    // Both arms warm up with an actual resident dispatch. Alternate timed order
    // and compare all observable quantities at the same physical horizon.
    old.step_block(1)?;
    accurate.step_block(1)?;
    old.reset();
    accurate.reset();
    let (a, b) = if seed.is_multiple_of(2) {
        (old.step_block(ticks)?, accurate.step_block(ticks)?)
    } else {
        let b = accurate.step_block(ticks)?;
        (old.step_block(ticks)?, b)
    };
    eval(&old, &accurate)?;
    let first = accurate.lanes().to_vec();
    accurate.reset();
    accurate.step_block(ticks)?;
    assert_eq!(accurate.lanes(), first);
    println!(
        "{}",
        json!({"seed":seed,"lanes":n,"ticks":ticks,"bodies_compared":physical,"max_midpoint_rad":peak[0],"max_fourth_rad":peak[1],"mean_midpoint_rad":sum[0]/physical as f64,"mean_fourth_rad":sum[1]/physical as f64,"midpoint_gpu_us":a.gpu_dispatch.map(|t|t.as_micros()),"fourth_gpu_us":b.gpu_dispatch.map(|t|t.as_micros()),"midpoint_wall_us":a.latency.as_micros(),"fourth_wall_us":b.latency.as_micros()})
    );
    Ok(())
}
pub(super) fn rotor(adapter: &str, path: &Path) -> Result<()> {
    let mut lane = reference::fixture(1)[0];
    lane.a.position = [0.; 3];
    lane.a.velocity = [0.; 3];
    lane.a.angular_velocity = [160., 0., 0.];
    lane.b.position = [5., 0., 0.];
    lane.b.velocity = [0.; 3];
    lane.b.angular_velocity = [0.; 3];
    lane.gravity = [0.; 3];
    let mut mid = SpherePairBlockBatch::new(adapter, vec![lane])?;
    let mut fine =
        SpherePairBlockBatch::with_rotation(adapter, vec![lane], SphereRotation::FourthOrder)?;
    let mut dense =
        SpherePairBatch::with_rotation(adapter, vec![lane], SphereRotation::FourthOrder)?;
    let mut rows = Vec::new();
    let mut frames = Vec::new();
    let start = Instant::now();
    for i in 0..=100u64 {
        if i > 0 {
            mid.step_block(40)?;
            fine.step_block(40)?;
            for _ in 0..40 {
                dense.step()?;
            }
        }
        let tick = mid.tick();
        assert_eq!(
            bytemuck::bytes_of(&fine.lanes()[0].state),
            bytemuck::bytes_of(&dense.lanes()[0])
        );
        let a = mid.lanes()[0].state.a;
        let b = fine.lanes()[0].state.a;
        let mut exact = lane;
        reference::exact_interval(&mut exact, tick as f64 * 0.001);
        let err = [
            error(a.orientation, exact.a.orientation),
            error(b.orientation, exact.a.orientation),
        ];
        let mut v = View::default();
        for (k, (tag, q, c)) in [
            ("midpoint", a.orientation, [0.90, 0.41, 0.16, 1.]),
            ("fourth_order", b.orientation, [0.1, 0.64, 0.80, 1.]),
            ("analytic", exact.a.orientation, [0.48, 0.77, 0.37, 1.]),
        ]
        .into_iter()
        .enumerate()
        {
            let center = V::new(0., (1. - k as f64) * 0.62, 0.);
            let base = 10 + k as u64 * 20;
            v.add(
                base,
                tag,
                center.to_array(),
                q,
                json!({"kind":"sphere","radius":a.radius}),
                c,
            );
            let rot = Q::from_array(q);
            let direction = rot * V::Y;
            v.add(
                base + 2,
                &format!("{tag}_body_axis"),
                (center + 0.16 * direction).to_array(),
                q,
                json!({"kind":"box","size":[0.026,0.32,0.026]}),
                c,
            );
            let angle = if k < 2 { err[k] } else { 0. };
            v.cuboid(
                base + 4,
                &format!("{tag}_scale"),
                [0., center.y, -0.39],
                [0.018, 0.46, 0.012],
                [0.42, 0.48, 0.55, 1.],
            );
            v.add(
                base + 6,
                &format!("{tag}_error"),
                [0., center.y - 0.23 + 0.46 * (angle / 1.5).min(1.), -0.39],
                [0., 0., 0., 1.],
                json!({"kind":"sphere","radius":0.027}),
                c,
            );
        }
        frames.push(v.finish_at(i + 1, tick * 1_000_000));
        rows.push(json!({"tick":tick,"midpoint_q":a.orientation,"fourth_q":b.orientation,"reference_q":exact.a.orientation,"angular_errors_rad":err}));
    }
    let final_errors = rows.last().unwrap()["angular_errors_rad"].clone();
    assert!(final_errors[0].as_f64().unwrap() > 1. && final_errors[1].as_f64().unwrap() < 0.001);
    fs::create_dir(path)?;
    gz_sim_cli::export_archive(&path.join("replay"), Default::default(), frames)?;
    fs::write(
        path.join("index.json"),
        serde_json::to_vec(
            &json!({"elapsed_s":start.elapsed().as_secs_f64(),"spin_radps":160.,"scale_rad":1.5,"frames":rows,"final_errors_rad":final_errors}),
        )?,
    )?;
    Ok(())
}
