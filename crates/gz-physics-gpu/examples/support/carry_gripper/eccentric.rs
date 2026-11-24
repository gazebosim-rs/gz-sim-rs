// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[derive(Default)]
struct Metrics {
    first: Option<usize>,
    highest: f64,
    settled_z: f64,
    force: [f64; 2],
    grip_angle: f64,
    peak_spin: f64,
    settled_angle: f64,
    jaw_z: f64,
    slip: [f64; 2],
}
impl Metrics {
    fn sample(
        &mut self,
        t: usize,
        z: f64,
        jaw: f64,
        rotation: [f64; 2],
        normal: [f64; 2],
        up: [f64; 2],
    ) {
        if self.first.is_none() && normal[0].min(normal[1]) > 1e-8 {
            self.first = Some(t);
            self.grip_angle = rotation[0];
        }
        self.highest = self.highest.max(z);
        self.peak_spin = self.peak_spin.max(rotation[1].abs());
        if t >= 801 {
            self.settled_z += z / 200.;
            self.jaw_z += jaw / 200.;
            self.settled_angle += rotation[0] / 200.;
            for (v, imp) in self.force.iter_mut().zip(up) {
                *v += imp / 0.2;
            }
        }
    }
    // Integrate the tangential surface speed while a pad actually carries an impulse.
    // The radius witness follows the nearest finite pad face/corner in each solver's own state.
    fn measure_slip(
        &mut self,
        l: &CarryGripperLane,
        (p, v, spin): ([f64; 2], [f64; 2], f64),
        pads: [[f64; 2]; 2],
        pad_vel: [[f64; 2]; 2],
        impulses: [f64; 2],
    ) {
        for i in 0..2 {
            if impulses[i] <= 1e-8 {
                continue;
            }
            let local = [p[0] - pads[i][0], p[1] - pads[i][1]];
            let separation: [f64; 2] = std::array::from_fn(|j| {
                let h = l.hand.half_size[i][j];
                local[j] - local[j].clamp(-h, h)
            });
            let length = separation[0].hypot(separation[1]);
            let tangent = [separation[1] / length, -separation[0] / length];
            self.slip[i] += 0.001
                * ((v[0] - pad_vel[i][0]) * tangent[0]
                    + (v[1] - pad_vel[i][1]) * tangent[1]
                    + spin * l.hand.radius)
                    .abs();
        }
    }
    fn json(&self) -> serde_json::Value {
        json!({"first_grip_ms":self.first,"settled_z_m":self.settled_z,
        "settled_pad_loads_n":self.force,"sampled_pad_sliding_m":self.slip,"peak_spin_rad_s":self.peak_spin,
        "first_grip_angle_rad":self.grip_angle,"settled_angle_rad":self.settled_angle,
        "maximum_z_m":self.highest,"settled_center_minus_jaw_m":self.settled_z-self.jaw_z})
    }
}
/// With every collider clear, only the mass center follows the ballistic parabola.
fn free_flight(adapter: &str) -> Result<[f64; 2], Box<dyn Error>> {
    let mut lane = fixture::fixture(0.08, 0.6, 0.18);
    lane.com_offset = [0.006, -0.003];
    lane.hand.angle = 0.3;
    lane.hand.spin = 11.;
    lane.hand.ball = [0.5, 1.5];
    lane.hand.velocity = [0.2, 4.8];
    let offset = |angle: f64| {
        let (s, c) = angle.sin_cos();
        [
            c * lane.com_offset[0] - s * lane.com_offset[1],
            s * lane.com_offset[0] + c * lane.com_offset[1],
        ]
    };
    let [dx, dz] = offset(lane.hand.angle);
    let initial = [lane.hand.ball[0] + dx, lane.hand.ball[1] + dz];
    let speed = [
        lane.hand.velocity[0] - lane.hand.spin * dz,
        lane.hand.velocity[1] + lane.hand.spin * dx,
    ];
    let mut b = CarryGripperBatch::new(adapter, vec![lane])?;
    let mut max = [0.0_f64; 2];
    for t in 1..=1000 {
        b.step()?;
        let h = b.lanes()[0].hand;
        let s = t as f64 * 0.001;
        assert_eq!(h.normal_impulse, [0.; 3]);
        assert_eq!(b.lanes()[0].shelf_up, 0.);
        let [dx, dz] = offset(h.angle);
        let expected = [
            initial[0] + s * speed[0],
            initial[1] + s * speed[1] - 0.5 * s * s * h.gravity,
        ];
        max[0] = max[0].max((h.ball[0] + dx - expected[0]).hypot(h.ball[1] + dz - expected[1]));
        max[1] = max[1].max(
            (h.velocity[0] - h.spin * dz - speed[0])
                .hypot(h.velocity[1] + h.spin * dx - speed[1] + s * h.gravity),
        );
        assert!(
            (h.angle - lane.hand.angle - s * lane.hand.spin).abs() < 1e-10
                && h.spin == lane.hand.spin
        );
    }
    assert!(max[0] < 1e-10 && max[1] < 1e-10);
    Ok(max)
}
pub(super) fn run(adapter: &str, dir: &Path) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(dir)?;
    let variants = [
        ("centered", 0., 0.6, 0.),
        ("heavy_right", 0.006, 0.6, 0.),
        ("heavy_left", -0.006, 0.6, 0.),
        ("compact_core", 0.006, 0.6, 0.),
        ("zero_friction", 0.006, 0., 0.),
        ("weak_grip", 0.006, 0.05, 0.),
        ("off_center_pickup", 0.006, 0.6, 0.006),
        ("marginal_grip", 0.006, 0.071, 0.),
        ("mu08", 0.006, 0.08, 0.),
        ("mu09", 0.006, 0.09, 0.),
        ("mu10", 0.006, 0.10, 0.),
        ("mu11", 0.006, 0.11, 0.),
        ("mu12", 0.006, 0.12, 0.),
    ];
    let initial = variants.map(|(name, offset, mu, x)| {
        let mut l = fixture::fixture(0.08, mu, 0.18);
        l.com_offset = [offset, 0.];
        l.hand.ball[0] = x;
        if name == "compact_core" {
            l.angular_inertia *= 0.55;
        }
        l
    });
    let mut batch = CarryGripperBatch::new(adapter, initial.to_vec())?;
    let mut references = initial.each_ref().map(|l| native::Native::new(l, 16));
    let mut data = initial.map(|_| [Metrics::default(), Metrics::default()]);
    let mut errors = initial.map(|_| [0.0_f64; 2]);
    let mut frames = Vec::new();
    let mut visual = vec![poses(0, &initial[1])];
    let mut slide = vec![poses(0, initial.last().unwrap())];
    let mut trace = Vec::new();
    let mut gpu_time = Vec::new();
    let mut cpu_time = Vec::new();
    for t in 1..=1000 {
        let command = fixture::targets(t.min(570), false, false);
        if t == 1 || t == 201 {
            for i in 0..initial.len() {
                batch.set_targets(i, command)?;
            }
        }
        let begin = Instant::now();
        batch.step()?;
        gpu_time.push(begin.elapsed().as_secs_f64() * 1e6);
        frames.push(batch.lanes().to_vec());
        let mut native_micros = 0.;
        for (i, l) in batch.lanes().iter().enumerate() {
            let h = l.hand;
            let start = Instant::now();
            let (n, _) = references[i].step(l, command);
            native_micros += start.elapsed().as_secs_f64() * 1e6;
            let (q, p, native_velocity) = references[i].state();
            let rot = references[i].payload_rotation();
            let angle_error = (h.angle - rot[0])
                .sin()
                .atan2((h.angle - rot[0]).cos())
                .abs();
            errors[i][0] = errors[i][0].max((h.ball[0] - p[0]).hypot(h.ball[1] - p[1]));
            errors[i][1] = errors[i][1].max(angle_error);
            data[i][0].sample(
                t,
                h.ball[1],
                h.position[0],
                [h.angle, h.spin],
                [h.normal_impulse[0], h.normal_impulse[1]],
                h.support_impulse,
            );
            data[i][1].sample(t, p[1], q[1], rot, [n[0], n[1]], references[i].pad_up);
            if t >= 201 {
                let pad_positions = |q: [f64; 3]| [[q[1], q[0]], [q[2], q[0]]];
                data[i][0].measure_slip(
                    l,
                    (h.ball, h.velocity, h.spin),
                    pad_positions(h.position),
                    [[h.speed[1], h.speed[0]], [h.speed[2], h.speed[0]]],
                    [h.normal_impulse[0], h.normal_impulse[1]],
                );
                data[i][1].measure_slip(
                    l,
                    (p, native_velocity, rot[1]),
                    pad_positions([q[1], q[2], q[3]]),
                    references[i].pad_velocity(),
                    [n[0], n[1]],
                );
            }
            if t % 5 == 0 {
                trace.push(json!([
                    t,
                    variants[i].0,
                    h.ball,
                    p,
                    h.angle,
                    rot[0],
                    h.support_impulse,
                    references[i].pad_up
                ]));
            }
        }
        cpu_time.push(native_micros);
        visual.push(poses(t, &batch.lanes()[1]));
        slide.push(poses(t, batch.lanes().last().unwrap()));
    }
    gpu_time.sort_by(f64::total_cmp);
    cpu_time.sort_by(f64::total_cmp);
    let rows:Vec<_>=data.iter().enumerate().map(|(i,m)|json!({"case":variants[i].0,
       "center_offset_x_m":variants[i].1,"central_inertia_kg_m2":initial[i].angular_inertia,"friction":variants[i].2,"initial_geometric_center_x_m":variants[i].3,
       "max_center_error_m":errors[i][0],"max_angle_error_rad":errors[i][1],"gpu":m[0].json(),"rapier":m[1].json()})).collect();
    for (i, case) in data.iter().enumerate() {
        let held = variants[i].2 == 0.6;
        for m in case {
            assert!((28..=30).contains(&m.first.unwrap()));
            if held {
                assert!(m.settled_z > 0.117 && m.slip.iter().all(|&v| v < 0.00005));
                assert!((m.force.iter().sum::<f64>() - 0.08 * 9.81).abs() < 0.0001);
                let predicted = (variants[i].1 * m.settled_angle.cos()) / 0.022 * 0.08 * 9.81;
                assert!((m.force[1] - m.force[0] - predicted).abs() < 0.012);
            } else {
                assert!(m.settled_z < 0.0221 && m.force == [0.; 2] && m.slip[1] > 0.008);
            }
        }
        if held {
            let max_angle = if variants[i].0 == "compact_core" {
                1.0_f64.to_radians()
            } else {
                0.01
            };
            assert!(
                errors[i][0] < 0.0012 && errors[i][1] < max_angle,
                "{}: {:?}",
                variants[i].0,
                errors[i]
            );
        }
        if variants[i].0 == "compact_core" {
            for (compact, nominal) in case.iter().zip(&data[1]) {
                assert!(compact.peak_spin > 1.10 * nominal.peak_spin);
            }
        }
        if variants[i].0 == "mu12" {
            assert!(case.iter().all(|m| m.highest > 0.029 && m.slip[1] > 0.015));
        }
    }
    let ballistic = free_flight(adapter)?;
    let report = json!({"adapter":adapter,"free_flight_max_errors_m_and_mps":ballistic,"device_batch_p95_us":gpu_time[949],"native_batch_p95_us":cpu_time[949],"rows":rows});
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    fs::write(dir.join("source-poses.json"), serde_json::to_vec(&visual)?)?;
    fs::write(dir.join("slip-poses.json"), serde_json::to_vec(&slide)?)?;
    fs::write(dir.join("trajectory.json"), serde_json::to_vec(&trace)?)?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    batch.reset();
    for (i, frame) in frames.iter().enumerate() {
        let tick = i + 1;
        if tick == 1 || tick == 201 {
            for j in 0..initial.len() {
                batch.set_targets(j, fixture::targets(tick.min(570), false, false))?;
            }
        }
        batch.step()?;
        assert_eq!(
            bytemuck::cast_slice::<_, u8>(batch.lanes()),
            bytemuck::cast_slice::<_, u8>(frame)
        );
    }
    Ok(())
}
