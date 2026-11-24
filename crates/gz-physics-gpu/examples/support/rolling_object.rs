// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Default)]
struct Metrics {
    first: [Option<usize>; 2],
    total: [f64; 2],
    max_pos: f64,
    max_tip: f64,
    max_spin: f64,
    linear_balance: f64,
    angular_balance: f64,
    max_cone_excess: f64,
    max_friction_work: f64,
    work: [f64; 3],
    max_work: f64,
    final_ref_v: f64,
    final_ref_spin: f64,
    final_v: f64,
    final_spin: f64,
    final_x: f64,
    rolling_ms: Option<usize>,
}
pub(super) fn run(adapter: &str, dir: &Path) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(dir)?;
    // Disturbed and undisturbed twins share their initial placement and tool drive.
    const GRAZE: usize = 6;
    const PRESSED: usize = 7;
    const PRESSED_PULSE: usize = 9;
    const RAISED_PULSE: usize = 10;
    const RAISED: usize = 11;
    let mut inputs = [
        (0.503, 0., 0.3),
        (1.1, 0.4, 0.3),
        (1.1, -0.4, 0.3),
        (1.1, 0., 0.3),
        (0.516, 0., 0.3),
        (0.525, 0., 0.3),
        (0.533, 0., 0.3),
        (0.48, 0., 0.3),
        (0.503, 0., 0.1),
        (0.48, 0., 0.3),
        (0.48, 0., 0.3),
        (0.48, 0., 0.3),
    ]
    .map(|(x, v, mu)| {
        let mut l = fixture(0.08, x);
        if x == 0.48 {
            l.tool.arm.angle[0] = reference::turn(0.51);
        }
        l.tool.friction = mu;
        l.object.surface.tool_mu = mu;
        l.object.surface.floor_mu = mu;
        l.object.velocity[0] = v;
        l
    });
    for i in [RAISED_PULSE, RAISED] {
        inputs[i].tool.arm.target = [0.82, -0.15].map(reference::turn);
    }
    let mut gpu = PlanarObjectBatch::new(adapter, inputs.to_vec())?;
    let mut oracles = inputs.each_ref().map(Oracle::new);
    for o in &mut oracles {
        o.arm
            .world
            .integration_parameters
            .num_internal_pgs_iterations = 4;
    }
    let mut previous = inputs.map(|l| l.object);
    let mut metrics = inputs.map(|_| Metrics::default());
    let mut history = Vec::new();
    let mut frames = Vec::new();
    let visual = |tick: usize, l: &PlanarObjectLane| {
        let mut sample = visual_sample(tick, l);
        let b = l.object;
        let (s, c) = (-b.surface.angle * 0.5).sin_cos();
        let rotation = json!([0., s, 0., c]);
        sample["poses"]["ball"]["rotation"] = rotation.clone();
        sample["poses"]["marker"] = json!({"translation":[b.position[0]+0.014*b.surface.angle.cos(),0.015,0.6+b.position[1]+0.014*b.surface.angle.sin()],"rotation":rotation});
        sample
    };
    let mut visuals = vec![visual(0, &inputs[GRAZE])];
    let mut held_visuals = vec![visual(0, &inputs[PRESSED_PULSE])];
    let mut hold_rows = Vec::new();
    let mut max_held_shift = 0_f64;
    let mut open_shift = 0.;
    let mut released_shift = 0.;
    let mut min_press_impulse = f64::INFINITY;
    let commands = |gpu: &mut PlanarObjectBatch, tick: usize| -> Result<(), Box<dyn Error>> {
        if tick == 301 {
            for i in 0..inputs.len() {
                gpu.set_target_angles(i, [0.82, -0.15])?;
            }
        }
        if matches!(tick, 201 | 351) {
            for i in [PRESSED_PULSE, RAISED_PULSE] {
                gpu.set_center_impulse(i, [0.024, 0.])?;
            }
        }
        Ok(())
    };
    for tick in 1..=400 {
        commands(&mut gpu, tick)?;
        if matches!(tick, 201 | 351) {
            for i in [PRESSED_PULSE, RAISED_PULSE] {
                let o = &mut oracles[i];
                o.arm.world.bodies[o.ball].apply_impulse(Vector::X * 0.024, true);
            }
        }
        gpu.step()?;
        history.push(gpu.lanes().to_vec());
        let lanes = gpu.lanes();
        let shift =
            |a: usize, b: usize| (lanes[a].object.position[0] - lanes[b].object.position[0]).abs();
        if (201..=300).contains(&tick) {
            let ball = lanes[PRESSED_PULSE].object;
            max_held_shift = max_held_shift.max(shift(PRESSED_PULSE, PRESSED));
            min_press_impulse = min_press_impulse.min(ball.normal_impulse.min(ball.floor_impulse));
        }
        if tick == 300 {
            open_shift = shift(RAISED_PULSE, RAISED);
        }
        if tick == 400 {
            released_shift = shift(PRESSED_PULSE, PRESSED);
        }

        visuals.push(visual(tick, &gpu.lanes()[GRAZE]));
        held_visuals.push(visual(tick, &gpu.lanes()[PRESSED_PULSE]));
        let phase: Vec<_> = [PRESSED, PRESSED_PULSE, RAISED_PULSE, RAISED]
            .into_iter()
            .map(|i| {
                let b = gpu.lanes()[i].object;
                json!({
                    "x":b.position[0],"vx":b.velocity[0],"spin":b.surface.spin,
                    "tool":b.normal_impulse,"floor":b.floor_impulse,
                    "tool_t":b.surface.tool_tangent,"floor_t":b.surface.floor_tangent
                })
            })
            .collect();
        hold_rows.push(json!({"tick":tick,"states":phase}));
        for (i, l) in gpu.lanes().iter().enumerate() {
            let b = l.object;
            let surf = b.surface;
            let s = &mut metrics[i];
            let oracle = &mut oracles[i];
            let native_n = oracle.step(l);
            for (j, n) in [b.normal_impulse, native_n].into_iter().enumerate() {
                if n > 1e-12 {
                    s.first[j].get_or_insert(tick);
                }
                s.total[j] += n;
            }
            s.max_pos = s.max_pos.max(distance(b.position, oracle.position()));
            s.max_tip = s.max_tip.max(distance(
                l.tool.arm.points()[1],
                oracle.arm.tip(&l.tool.arm),
            ));
            s.max_spin = s
                .max_spin
                .max((surf.spin + oracle.arm.world.bodies[oracle.ball].angvel().y).abs());
            let old = previous[i];
            let impulse = [
                b.tool_impulse[0]
                    + surf.floor_tangent
                    + if matches!(i, PRESSED_PULSE | RAISED_PULSE) && matches!(tick, 201 | 351) {
                        0.024
                    } else {
                        0.
                    },
                b.tool_impulse[1] + b.floor_impulse,
            ];
            s.linear_balance = s.linear_balance.max(distance(
                impulse,
                [
                    b.mass * (b.velocity[0] - old.velocity[0]),
                    b.mass * (b.velocity[1] - old.velocity[1] + l.tool.arm.gravity * 0.001),
                ],
            ));
            let inertia = 0.4 * b.mass * b.radius * b.radius;
            let angular_impulse = b.radius * (surf.floor_tangent + surf.tool_tangent);
            s.angular_balance = s
                .angular_balance
                .max((inertia * (surf.spin - old.surface.spin) - angular_impulse).abs());
            s.max_cone_excess = s.max_cone_excess.max(
                (surf.floor_tangent.abs() - surf.floor_mu * b.floor_impulse)
                    .max(surf.tool_tangent.abs() - surf.tool_mu * b.normal_impulse)
                    .max(l.tool.tangent_impulse.abs() - l.tool.friction * l.tool.normal_impulse),
            );
            for j in 0..3 {
                s.work[j] += surf.friction_work[j];
            }
            s.max_work = s.max_work.max(surf.contact_work);
            s.final_ref_v = oracle.arm.world.bodies[oracle.ball].linvel().x;
            s.final_ref_spin = -oracle.arm.world.bodies[oracle.ball].angvel().y;
            s.max_friction_work = s.max_friction_work.max(surf.friction_work.iter().sum());
            s.final_v = b.velocity[0];
            s.final_x = b.position[0];
            s.final_spin = surf.spin;
            if matches!(i, 1 | 2) && (s.final_v + b.radius * s.final_spin).abs() < 1e-9 {
                s.rolling_ms.get_or_insert(tick);
            }
            if i == 0 {
                frames.push(json!({"step":tick,"tip":l.tool.arm.points()[1],"ref_tip":oracle.arm.tip(&l.tool.arm),"ball":b.position,"ref_ball":oracle.position(),"n":b.normal_impulse,"ref_n":native_n,"tool_floor":l.tool.normal_impulse,"ball_floor":b.floor_impulse,"ball_floor_t":surf.floor_tangent,"tool_t":surf.tool_tangent,"spin":surf.spin,"ref_spin":s.final_ref_spin}));
            }
            previous[i] = b;
        }
    }
    gpu.reset();
    for (k, expected) in history.iter().enumerate() {
        commands(&mut gpu, k + 1)?;
        gpu.step()?;
        assert_eq!(
            bytemuck::cast_slice::<_, u8>(expected),
            bytemuck::cast_slice::<_, u8>(gpu.lanes())
        );
    }
    let names = [
        "oblique strike",
        "positive slide",
        "negative slide",
        "stationary miss",
        "off-centre A",
        "off-centre B",
        "grazing strike",
        "centered press",
        "low friction",
        "pressed pulse",
        "raised pulse",
        "raised control",
    ];
    let rows: Vec<_> = metrics.iter().enumerate().map(|(i, m)| json!({
        "case":names[i], "initial_x":inputs[i].object.position[0], "mu":inputs[i].object.surface.tool_mu,
        "first_ms":m.first, "impulse_gpu_native_ns":m.total, "max_ball_error_m":m.max_pos,
        "max_tip_error_m":m.max_tip,"max_spin_error_rad_s":m.max_spin,
        "linear_balance_ns":m.linear_balance,"angular_balance_nms":m.angular_balance,
        "max_friction_cone_excess_ns":m.max_cone_excess,"friction_work_j":m.work,
        "max_positive_friction_work_j":m.max_friction_work,"max_positive_contact_work_j":m.max_work,
        "native_final_speed_m_s":m.final_ref_v,"native_final_spin_rad_s":m.final_ref_spin,
        "final_x_m":m.final_x,"final_speed_m_s":m.final_v,"final_spin_rad_s":m.final_spin,"rolling_ms":m.rolling_ms
    })).collect();
    let hold = json!({"max_pressed_pulse_displacement_m":max_held_shift,"raised_pulse_displacement_m_at_300_ms":open_shift,"released_displacement_m_at_400_ms":released_shift,"minimum_opposing_impulse_ns":min_press_impulse,"pulse_ns":0.024});
    let d = gpu.device_info();
    let report = json!({"adapter":d.name,"driver":d.driver_info,"precision":"f64","gpu_dt_us":250,"rapier_dt_us":31.25,"ticks":400,"rows":rows,"retention":hold});
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    fs::write(
        dir.join("held-poses.json"),
        serde_json::to_vec(&held_visuals)?,
    )?;
    fs::write(dir.join("hold.json"), serde_json::to_vec(&hold_rows)?)?;
    fs::write(dir.join("frames.json"), serde_json::to_vec(&frames)?)?;
    fs::write(dir.join("source-poses.json"), serde_json::to_vec(&visuals)?)?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    for m in &metrics {
        assert!(m.linear_balance < 1e-10 && m.angular_balance < 1e-12 && m.max_cone_excess < 1e-10);
    }
    for i in [1, 2] {
        let m = &metrics[i];
        let b = inputs[i].object;
        let final_speed = 5. * b.velocity[0] / 7.;
        assert!((m.final_v - final_speed).abs() < 1e-10);
        assert!((m.final_spin + final_speed / b.radius).abs() < 1e-9);
        assert_eq!(m.rolling_ms, Some(39));
        let kinetic_loss = b.mass * b.velocity[0].powi(2) / 7.;
        assert!((m.work[1] + kinetic_loss).abs() < 1e-10);
    }
    assert!(
        max_held_shift < 0.0004
            && open_shift > 0.022
            && released_shift > 0.012
            && min_press_impulse > 0.015
    );
    for i in [PRESSED, PRESSED_PULSE] {
        let m = &metrics[i];
        assert_eq!(m.first, [Some(30); 2]);
        assert!((m.total[0] / m.total[1] - 1.).abs() < 0.005 && m.max_pos < 0.001);
    }
    assert!(
        metrics[GRAZE].max_pos < 0.0025
            && (metrics[GRAZE].total[0] / metrics[GRAZE].total[1] - 1.).abs() < 0.002
    );
    assert!(metrics.iter().all(|m| m.max_work < 1e-12));
    assert_eq!(metrics[RAISED_PULSE].total, [0.; 2]);
    assert_eq!(metrics[3].final_x, inputs[3].object.position[0]);
    assert_eq!(metrics[3].final_spin, 0.);
    Ok(())
}
