// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[derive(Default)]
struct Outcome {
    first_grip: Option<usize>,
    last_angle: f64,
    first_air: Option<usize>,
    highest: f64,
    mean_z: f64,
    load: [f64; 2],
    max_tilt: f64,
    land: Option<usize>,
}
impl Outcome {
    fn sample(&mut self, t: usize, (z, angle): (f64, f64), extent: f64, n: [f64; 3], up: [f64; 2]) {
        if n[0].min(n[1]) > 1e-8 {
            self.first_grip.get_or_insert(t);
        }
        let clearance = z - extent;
        if clearance > 0.003 {
            self.first_air.get_or_insert(t);
        }
        if self.first_air.is_some() && n[2] > 0.0001 && clearance < 0.0001 {
            self.land.get_or_insert(t);
        }
        self.highest = self.highest.max(clearance);
        self.max_tilt = self.max_tilt.max(angle.abs());
        self.last_angle = angle;
        if t >= 801 {
            self.mean_z += clearance / 200.;
            for (i, force) in self.load.iter_mut().enumerate() {
                *force += up[i] / 0.2;
            }
        }
    }
    fn json(&self) -> serde_json::Value {
        json!({
            "bilateral_contact_ms":self.first_grip,"cleared_3mm_ms":self.first_air,"first_return_ms":self.land,
            "peak_clearance_m":self.highest,"settled_clearance_m":self.mean_z,"final_heading_rad":self.last_angle,
            "max_abs_heading_rad":self.max_tilt,"settled_finger_loads_n":self.load
        })
    }
}
pub(super) fn run(adapter: &str, dir: &Path) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(dir)?;
    let cases = [
        ("flat", 0., 0.6, 0.),
        ("tipped", 0.24, 0.6, 0.),
        ("reverse_tip", -0.24, 0.6, 0.),
        ("slippery", 0., 0.035, 0.),
        ("miss", 0., 0.6, 0.105),
        ("released", 0., 0.6, 0.),
        ("weighted", 0., 0.6, 0.),
    ];
    let initial = cases.map(|(name, heading, mu, x)| {
        let mut l = fixture::fixture(0.08, mu, 0.18);
        l.box_half = [0.024, 0.020];
        l.box_heading = heading;
        l.hand.radius = 0.;
        l.angular_inertia = l.hand.ball_mass * (l.box_half[0].powi(2) + l.box_half[1].powi(2)) / 3.;
        l.hand.ball = [x, l.payload_reach()[1]];
        if name == "weighted" {
            l.com_offset = [0.005, 0.];
        }
        l
    });
    let sphere = fixture::fixture(0.08, 0.6, 0.18);
    assert!(matches!(
        CarryGripperBatch::new(adapter, vec![sphere, initial[0]]),
        Err(gz_physics_gpu::GpuError::InvalidLane(1))
    ));
    for modify in [0, 1, 2, 3] {
        let mut invalid = initial[0];
        match modify {
            0 => invalid.box_half[1] = 0.,
            1 => invalid.hand.radius = 0.02,
            2 => invalid.com_offset[0] = invalid.box_half[0] * 1.01,
            _ => invalid.point_impulse[2][0] = 0.01,
        }
        assert!(!invalid.valid());
    }
    let mut batch = CarryGripperBatch::new(adapter, initial.to_vec())?;
    let mut refs = initial.each_ref().map(|l| native::Native::new(l, 16));
    let mut planar = initial
        .each_ref()
        .map(|l| native::Native::create(l, 16, true));
    let mut planar_metrics = initial.map(|_| Outcome::default());
    let mut trans = initial.map(|_| [0.0_f64; 3]);
    let mut full_path = initial.map(|_| 0.0_f64);
    let mut planar_errors = initial.map(|_| [0.0_f64; 2]);
    let mut outcomes = initial.map(|_| [Outcome::default(), Outcome::default()]);
    let mut errors = initial.map(|_| [0.0_f64; 2]);
    let mut counts = initial.map(|_| [0_usize; 4]);
    let mut frames = Vec::new();
    let mut flat = vec![poses(0, &initial[0])];
    let mut tipped = vec![poses(0, &initial[1])];
    let mut slip = vec![poses(0, &initial[3])];
    let mut trace = Vec::new();
    let mut gpu_cost = Vec::new();
    let mut cpu_cost = Vec::new();
    let target = |tick: usize, i| fixture::targets(tick.min(570), i == 5 && tick >= 701, false);
    for tick in 1..=1000 {
        if [1, 201, 701].contains(&tick) {
            for i in 0..initial.len() {
                batch.set_targets(i, target(tick, i))?;
            }
        }
        let now = Instant::now();
        batch.step()?;
        gpu_cost.push(now.elapsed().as_secs_f64() * 1e6);
        frames.push(batch.lanes().to_vec());
        let mut cpu = 0.;
        for (i, l) in batch.lanes().iter().enumerate() {
            let now = Instant::now();
            let (n, _) = refs[i].step(l, target(tick, i));
            cpu += now.elapsed().as_secs_f64() * 1e6;
            let (_, p, _) = refs[i].state();
            let angle = refs[i].payload_rotation()[0];
            let h = l.hand;
            let heading = l.box_heading + h.angle;
            let [rz, y, tilt, full] = refs[i].spatial_box(l, heading);
            full_path[i] = full_path[i].max((h.ball[0] - p[0]).hypot(h.ball[1] - p[1]).hypot(y));
            trans[i][0] = trans[i][0].max(y.abs());
            trans[i][1] = trans[i][1].max(tilt);
            trans[i][2] = trans[i][2].max(full);
            let (pn, _) = planar[i].step(l, target(tick, i));
            let (_, pp, _) = planar[i].state();
            let pa = planar[i].payload_rotation()[0];
            let [pr, _, _, err] = planar[i].spatial_box(l, heading);
            planar_metrics[i].sample(
                tick,
                (pp[1], pa),
                pr,
                [pn[0], pn[1], pn[2]],
                planar[i].pad_up,
            );
            planar_errors[i][0] =
                planar_errors[i][0].max((pp[0] - h.ball[0]).hypot(pp[1] - h.ball[1]));
            planar_errors[i][1] = planar_errors[i][1].max(err);
            outcomes[i][0].sample(
                tick,
                (h.ball[1], heading),
                l.payload_reach()[1],
                h.normal_impulse,
                h.support_impulse,
            );
            outcomes[i][1].sample(tick, (p[1], angle), rz, [n[0], n[1], n[2]], refs[i].pad_up);
            errors[i][0] = errors[i][0].max((p[0] - h.ball[0]).hypot(p[1] - h.ball[1]));
            errors[i][1] =
                errors[i][1].max((angle - heading).sin().atan2((angle - heading).cos()).abs());
            for (count, ends) in counts[i].iter_mut().zip(l.point_impulse) {
                if ends[0].min(ends[1]) > 1e-9 {
                    *count += 1;
                }
            }
            if tick % 5 == 0 {
                trace.push(json!([
                    tick,
                    cases[i].0,
                    h.ball,
                    p,
                    heading,
                    angle,
                    l.point_impulse,
                    h.normal_impulse,
                    n
                ]));
            }
        }
        cpu_cost.push(cpu);
        flat.push(poses(tick, &batch.lanes()[0]));
        tipped.push(poses(tick, &batch.lanes()[1]));
        slip.push(poses(tick, &batch.lanes()[3]));
    }
    gpu_cost.sort_by(f64::total_cmp);
    cpu_cost.sort_by(f64::total_cmp);
    let rows:Vec<_>=outcomes.iter().enumerate().map(|(i,m)|json!({"case":cases[i].0,"gpu":m[0].json(),"rapier":m[1].json(),
        "max_center_error_m":errors[i][0],"max_spatial_center_error_m":full_path[i],"max_orientation_error_rad":errors[i][1],"two_loaded_points_ms":counts[i],"native_transverse_m_tilt_rad_full_rotation_error_rad":trans[i],
        "planar_native":planar_metrics[i].json(),"planar_native_path_orientation_errors":planar_errors[i]})).collect();
    let report = json!({"named_adapter":adapter,"device_p95_us":gpu_cost[949],"native_fine_p95_us":cpu_cost[949],"rows":rows});
    for (n, v) in [
        ("summary", report),
        ("trajectory", json!(trace)),
        ("flat-poses", json!(flat)),
        ("tipped-poses", json!(tipped)),
        ("slip-poses", json!(slip)),
    ] {
        fs::write(dir.join(format!("{n}.json")), serde_json::to_vec(&v)?)?;
    }
    for (i, pair) in outcomes.iter().enumerate() {
        match cases[i].0 {
            "flat" | "tipped" | "reverse_tip" | "weighted" => {
                assert!(pair.iter().all(|o| o.mean_z > 0.09
                    && (o.load.iter().sum::<f64>() - 0.08 * 9.81).abs() < 0.0001));
                assert!(
                    pair[0]
                        .first_grip
                        .unwrap()
                        .abs_diff(pair[1].first_grip.unwrap())
                        <= 1
                );
                assert!(
                    counts[i][0].min(counts[i][1]) > 800
                        && full_path[i] < 0.005
                        && trans[i][2] < 0.12
                );
            }
            "released" => {
                assert!(
                    pair[0].land.unwrap().abs_diff(pair[1].land.unwrap()) <= 2
                        && pair.iter().all(|o| o.highest > 0.09)
                );
            }
            _ => assert!(
                pair.iter()
                    .all(|o| o.first_air.is_none() && o.mean_z.abs() < 0.001 && o.load == [0.; 2])
            ),
        }
    }
    batch.reset();
    for (i, frame) in frames.iter().enumerate() {
        let t = i + 1;
        if [1, 201, 701].contains(&t) {
            for j in 0..initial.len() {
                batch.set_targets(j, target(t, j))?;
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
