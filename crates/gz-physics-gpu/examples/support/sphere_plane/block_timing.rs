// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    reference::{analytical, fixture},
    summary::Summary,
};
use gz_physics_gpu::{
    SpherePlaneBatch, SpherePlaneBlockBatch, SpherePlaneLane as Lane, StepMetrics,
};
use plotters::prelude::*;
use std::{
    error::Error,
    fs::File,
    hint::black_box,
    io::{BufWriter, Write},
    path::Path,
    time::{Duration, Instant},
};

const WARM: u32 = 256;
const MEASURE: u32 = 8192;
pub const SIZES: [usize; 4] = [64, 512, 2048, 8192];
pub const BLOCKS: [u32; 4] = [1, 4, 16, 64];
#[derive(Clone, Copy, Debug, PartialEq)]
enum Method {
    Cpu,
    Dense,
    Resident,
}
impl Method {
    fn name(self) -> &'static str {
        match self {
            Self::Cpu => "CPU serial + block summary",
            Self::Dense => "GPU dense 1 ms",
            Self::Resident => "GPU resident block",
        }
    }
}
#[derive(Debug)]
pub struct Row {
    method: Method,
    lanes: usize,
    block: u32,
    samples: usize,
    rate: f64,
    mean_us: f64,
    p99_us: f64,
    upload: u64,
    read: u64,
    gpu_compute_us: f64,
    gpu_copy_us: f64,
    pack_us: f64,
    upload_us: f64,
    submit_us: f64,
    wait_us: f64,
    decode_us: f64,
    observe_us: f64,
}
#[derive(Default)]
struct Timings {
    samples: Vec<Duration>,
    upload: u64,
    read: u64,
    device: Duration,
    copy: Duration,
    pack: Duration,
    upload_time: Duration,
    submit: Duration,
    wait: Duration,
    decode: Duration,
    observe: Duration,
}
impl Timings {
    fn add_metrics(&mut self, m: StepMetrics) {
        self.upload += m.uploaded_bytes;
        self.read += m.readback_bytes;
        self.device += m.gpu_dispatch.unwrap_or_default();
        self.copy += m.gpu_copy.unwrap_or_default();
        self.pack += m.pack;
        self.upload_time += m.upload;
        self.submit += m.encode_submit;
        self.wait += m.wait_map;
        self.decode += m.decode;
    }
    fn row(mut self, method: Method, lanes: usize, block: u32) -> Row {
        self.samples.sort_unstable();
        let samples = self.samples.len();
        let duration = self.samples.iter().sum::<Duration>().as_secs_f64();
        let mean = |sum: Duration| sum.as_secs_f64() * 1e6 / samples as f64;
        Row {
            method,
            lanes,
            block,
            samples,
            rate: lanes as f64 * f64::from(MEASURE) / duration,
            mean_us: duration * 1e6 / samples as f64,
            p99_us: self.samples[(99 * samples).div_ceil(100) - 1].as_secs_f64() * 1e6,
            upload: self.upload,
            read: self.read,
            gpu_compute_us: mean(self.device),
            gpu_copy_us: mean(self.copy),
            pack_us: mean(self.pack),
            upload_us: mean(self.upload_time),
            submit_us: mean(self.submit),
            wait_us: mean(self.wait),
            decode_us: mean(self.decode),
            observe_us: mean(self.observe),
        }
    }
}
fn hash<'a>(values: impl IntoIterator<Item = &'a f64>, h: &mut u64) {
    for x in values {
        *h = h.rotate_left(7) ^ x.to_bits();
    }
}
fn observe_state(s: &Lane, h: &mut u64) {
    hash(
        s.position
            .iter()
            .chain(&s.velocity)
            .chain(&s.angular_velocity)
            .chain(&s.orientation)
            .chain(std::iter::once(&s.normal_impulse))
            .chain(&s.tangential_impulse),
        h,
    );
}
fn observe_summary(v: &Summary, h: &mut u64) {
    hash(
        [&v.normal, &v.peak, &v.clearance]
            .into_iter()
            .chain(&v.tangent),
        h,
    );
    *h ^= u64::from(v.contacts).rotate_left(5)
        ^ u64::from(v.first).rotate_left(17)
        ^ u64::from(v.last);
}
fn cpu(initial: &[Lane], block: u32) -> Row {
    let mut state = initial.to_vec();
    let mut acc: Vec<_> = state.iter().map(Summary::initial).collect();
    let mut time = Timings::default();
    for end in (block..=WARM + MEASURE).step_by(block as usize) {
        let start = Instant::now();
        for (s, v) in state.iter_mut().zip(&mut acc) {
            *v = Summary::initial(s);
            for tick in 1..=block {
                analytical(s);
                v.add(s, tick);
            }
        }
        let observation = Instant::now();
        let mut h = 0;
        for (s, v) in state.iter().zip(&acc) {
            observe_state(s, &mut h);
            observe_summary(v, &mut h);
        }
        black_box(h);
        if end > WARM {
            time.observe += observation.elapsed();
            time.samples.push(start.elapsed());
        }
    }
    time.row(Method::Cpu, initial.len(), block)
}
fn dense(adapter: &str, initial: &[Lane]) -> Result<Row, Box<dyn Error>> {
    let mut batch = SpherePlaneBatch::new(adapter, initial.to_vec())?;
    let mut time = Timings::default();
    // Full readback and observation after each native prepared one-tick stage.
    for end in 1..=WARM + MEASURE {
        let start = Instant::now();
        let m = batch.step()?;
        let observation = Instant::now();
        let mut h = 0;
        for s in batch.lanes() {
            observe_state(s, &mut h);
        }
        black_box(h);
        if end > WARM {
            time.observe += observation.elapsed();
            time.samples.push(start.elapsed());
            time.add_metrics(m);
        }
    }
    Ok(time.row(Method::Dense, initial.len(), 1))
}
fn resident(batch: &mut SpherePlaneBlockBatch, block: u32) -> Result<Row, Box<dyn Error>> {
    batch.reset();
    let mut time = Timings::default();
    for end in (block..=WARM + MEASURE).step_by(block as usize) {
        let start = Instant::now();
        let m = batch.step_block(block)?;
        let observation = Instant::now();
        let mut h = 0;
        for v in batch.lanes() {
            observe_state(&v.state, &mut h);
            let (a, b) = v.contact_range().unwrap_or_default();
            let s = Summary {
                normal: v.normal_impulse_sum,
                tangent: v.tangential_impulse_sum,
                clearance: v.minimum_clearance,
                peak: v.maximum_normal_impulse,
                contacts: v.contact_ticks(),
                first: a,
                last: b,
            };
            observe_summary(&s, &mut h);
        }
        black_box(h);
        if end > WARM {
            time.observe += observation.elapsed();
            time.samples.push(start.elapsed());
            time.add_metrics(m);
        }
    }
    assert_eq!(batch.tick(), u64::from(WARM + MEASURE));
    Ok(time.row(Method::Resident, batch.lanes().len(), block))
}
pub fn run(adapter: &str) -> Result<Vec<Row>, Box<dyn Error>> {
    let mut rows = Vec::new();
    for n in SIZES {
        let initial = fixture(n);
        rows.push(dense(adapter, &initial)?);
        let mut gpu = SpherePlaneBlockBatch::new(adapter, initial.clone())?;
        for k in BLOCKS {
            rows.push(cpu(&initial, k));
            rows.push(resident(&mut gpu, k)?);
        }
        println!("finished observed timing for {n} environments");
    }
    Ok(rows)
}
pub fn write(out: &Path, rows: &[Row]) -> Result<(), Box<dyn Error>> {
    let mut file = BufWriter::new(File::create(out.join("timing.csv"))?);
    writeln!(
        file,
        "method,lanes,block_ticks,measured_blocks,lane_steps_s,mean_us,p99_us,upload_bytes,readback_bytes,gpu_dispatch_us,gpu_copy_us,pack_us,upload_us,submit_us,wait_us,decode_us,observe_us"
    )?;
    for r in rows {
        writeln!(
            file,
            "{},{},{},{},{:.1},{:.3},{:.3},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}",
            r.method.name(),
            r.lanes,
            r.block,
            r.samples,
            r.rate,
            r.mean_us,
            r.p99_us,
            r.upload,
            r.read,
            r.gpu_compute_us,
            r.gpu_copy_us,
            r.pack_us,
            r.upload_us,
            r.submit_us,
            r.wait_us,
            r.decode_us,
            r.observe_us
        )?;
        if r.lanes == 2048 {
            println!(
                "{} | {:2} ticks | {:.2} M lane-steps/s | mean {:.1} us | p99 {:.1} us | {} B read",
                r.method.name(),
                r.block,
                r.rate / 1e6,
                r.mean_us,
                r.p99_us,
                r.read
            );
        }
    }
    file.flush()?;
    let path = out.join("throughput.svg");
    let root = SVGBackend::new(&path, (1160, 800)).into_drawing_area();
    root.fill(&WHITE)?;
    let root = root.titled(
        "Measured sphere/plane throughput by observation interval",
        ("sans-serif", 30),
    )?;
    let panels = root.split_evenly((2, 2));
    let colors = [
        RGBColor(13, 110, 119),
        RGBColor(200, 91, 40),
        RGBColor(90, 100, 112),
    ];
    for (panel, n) in panels.iter().zip(SIZES) {
        let relevant: Vec<_> = rows.iter().filter(|r| r.lanes == n).collect();
        let peak = relevant.iter().map(|r| r.rate / 1e6).fold(0f64, f64::max);
        let mut chart = ChartBuilder::on(panel)
            .caption(format!("{n} independent spheres"), ("sans-serif", 20))
            .margin(16)
            .x_label_area_size(44)
            .y_label_area_size(60)
            .build_cartesian_2d(-0.08f64..3.08, 0f64..peak * 1.28)?;
        chart
            .configure_mesh()
            .x_desc("Device ticks between host observations")
            .y_desc("Million lane-steps/s")
            .x_labels(4)
            .x_label_formatter(&|v| {
                if (*v - v.round()).abs() < 0.001 && (0.0..=3.0).contains(v) {
                    BLOCKS[v.round() as usize].to_string()
                } else {
                    String::new()
                }
            })
            .draw()?;
        for (method, color) in [Method::Resident, Method::Cpu, Method::Dense]
            .into_iter()
            .zip(colors)
        {
            let series: Vec<_> = relevant
                .iter()
                .filter(|r| r.method == method)
                .map(|r| {
                    (
                        BLOCKS.iter().position(|x| *x == r.block).unwrap() as f64,
                        r.rate / 1e6,
                    )
                })
                .collect();
            chart
                .draw_series(LineSeries::new(series.clone(), color.stroke_width(3)))?
                .label(method.name())
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 16, y)], color.stroke_width(3))
                });
            chart.draw_series(
                series
                    .into_iter()
                    .map(|p| Circle::new(p, 4, color.filled())),
            )?;
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.88))
            .border_style(RGBColor(221, 224, 227))
            .label_font(("sans-serif", 12))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
