// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read wheel encoders and link poses from the unaltered DART drive world.
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use std::fmt::Write;
use std::path::Path;
use std::time::{Duration, Instant};

use drive_native_reference::{HEADER, LAST, STRIDE};

const STATES: &str = "/world/diff_drive/model/vehicle/joint_state";

#[pyclass]
#[derive(Default)]
struct Observer {
    links: Vec<PyObject>,
    joints: Vec<PyObject>,
    rows: Vec<(u64, [f64; 13])>,
}

fn value(object: &Bound<'_, PyAny>, method: &str) -> PyResult<f64> {
    object.call_method0(method)?.extract()
}

#[pymethods]
impl Observer {
    fn pre(
        &mut self,
        py: Python<'_>,
        _info: &Bound<'_, PyAny>,
        ecm: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        if !self.links.is_empty() {
            return Ok(());
        }
        let sim = py.import("gz.sim")?;
        let world = sim
            .getattr("World")?
            .call1((sim.getattr("world_entity")?.call1((ecm,))?,))?;
        let model = sim
            .getattr("Model")?
            .call1((world.call_method1("model_by_name", (ecm, "vehicle"))?,))?;
        for name in ["chassis", "left_wheel", "right_wheel"] {
            let id = model.call_method1("link_by_name", (ecm, name))?;
            self.links.push(sim.getattr("Link")?.call1((id,))?.unbind());
        }
        for name in ["left_wheel_joint", "right_wheel_joint"] {
            let id = model.call_method1("joint_by_name", (ecm, name))?;
            let joint = sim.getattr("Joint")?.call1((id,))?;
            joint.call_method1("enable_position_check", (ecm,))?;
            joint.call_method1("enable_velocity_check", (ecm,))?;
            self.joints.push(joint.unbind());
        }
        self.sample(py, ecm, 0)
    }

    fn post(
        &mut self,
        py: Python<'_>,
        info: &Bound<'_, PyAny>,
        ecm: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let tick = info.getattr("iterations")?.extract::<u64>()?;
        if tick.is_multiple_of(STRIDE) {
            let seconds = value(&info.getattr("sim_time")?, "total_seconds")?;
            if (seconds - tick as f64 * 0.001).abs() > 1e-12 {
                return Err(PyRuntimeError::new_err(
                    "unexpected native simulation clock",
                ));
            }
            self.sample(py, ecm, tick)?;
        }
        Ok(())
    }
}

impl Observer {
    fn sample(&mut self, py: Python<'_>, ecm: &Bound<'_, PyAny>, tick: u64) -> PyResult<()> {
        let mut row = [0.0; 13];
        let body = self.links[0].bind(py).call_method1("world_pose", (ecm,))?;
        for (i, method) in ["x", "y", "z", "roll", "pitch", "yaw"].iter().enumerate() {
            row[i] = value(&body, method)?;
        }
        // Encoder components are initially empty. The source specifies zero initial motion.
        if tick > 0 {
            for (i, joint) in self.joints.iter().enumerate() {
                let joint = joint.bind(py);
                row[6 + i] = joint
                    .call_method1("position", (ecm,))?
                    .get_item(0)?
                    .extract()?;
                row[8 + i] = joint
                    .call_method1("velocity", (ecm,))?
                    .get_item(0)?
                    .extract()?;
            }
        }
        for wheel in &self.links[1..] {
            let pose = wheel.bind(py).call_method1("world_pose", (ecm,))?;
            for (i, method) in ["x", "y", "z"].iter().enumerate() {
                row[10 + i] += 0.5 * value(&pose, method)?;
            }
        }
        if !row.iter().all(|v| v.is_finite()) {
            return Err(PyRuntimeError::new_err("nonfinite native observation"));
        }
        self.rows.push((tick, row));
        Ok(())
    }
}

#[pyclass]
#[derive(Default)]
struct Packet {
    text: Option<String>,
}
#[pymethods]
impl Packet {
    fn receive(&mut self, msg: &Bound<'_, PyAny>) -> PyResult<()> {
        let stamp = msg.getattr("header")?.getattr("stamp")?;
        if stamp.getattr("sec")?.extract::<u64>()? == 4
            && stamp.getattr("nsec")?.extract::<u64>()? == 500_000_000
        {
            self.text = Some(msg.str()?.to_string());
        }
        Ok(())
    }
}

fn run(server: &Bound<'_, PyAny>, count: u64) -> PyResult<()> {
    if server
        .call_method1("run", (true, count, false))?
        .extract::<bool>()?
    {
        Ok(())
    } else {
        Err(PyRuntimeError::new_err("native server run failed"))
    }
}

fn capture(py: Python<'_>, world: &Path, out: &Path) -> PyResult<()> {
    py.import("gz.common")?
        .getattr("set_verbosity")?
        .call1((4,))?;
    py.import("gz.math")?;
    py.import("sdformat")?;
    let sim = py.import("gz.sim")?;
    let fixture = sim.getattr("TestFixture")?.call1((world,))?;
    let observer = Py::new(py, Observer::default())?;
    fixture.call_method1("on_pre_update", (observer.bind(py).getattr("pre")?,))?;
    fixture.call_method1("on_post_update", (observer.bind(py).getattr("post")?,))?;
    fixture.call_method0("finalize")?;
    let server = fixture.call_method0("server")?;
    let node = py.import("gz.transport")?.getattr("Node")?.call0()?;
    let twist = py.import("gz.msgs.twist_pb2")?.getattr("Twist")?;
    let publisher = node.call_method1("advertise", ("/model/vehicle/cmd_vel", &twist))?;
    let command = |linear, angular| -> PyResult<()> {
        if !publisher
            .call_method0("has_connections")?
            .extract::<bool>()?
        {
            return Err(PyRuntimeError::new_err(
                "stock DiffDrive subscriber missing",
            ));
        }
        let msg = twist.call0()?;
        msg.getattr("linear")?.setattr("x", linear)?;
        msg.getattr("angular")?.setattr("z", angular)?;
        if !publisher
            .call_method1("publish", (msg,))?
            .extract::<bool>()?
        {
            return Err(PyRuntimeError::new_err("Twist delivery failed"));
        }
        Ok(())
    };
    run(&server, 500)?;
    command(0.5, 0.0)?;
    run(&server, 2000)?;
    command(0.35, 0.8)?;
    run(&server, 1999)?;
    let packet = Py::new(py, Packet::default())?;
    let model_msg = py.import("gz.msgs.model_pb2")?.getattr("Model")?;
    if !node
        .call_method1(
            "subscribe",
            (model_msg, STATES, packet.bind(py).getattr("receive")?),
        )?
        .extract::<bool>()?
    {
        return Err(PyRuntimeError::new_err("joint-state subscription failed"));
    }
    run(&server, 1)?;
    let timeout = Instant::now() + Duration::from_secs(3);
    while packet.borrow(py).text.is_none() && Instant::now() < timeout {
        py.allow_threads(|| std::thread::sleep(Duration::from_millis(5)));
    }
    node.call_method1("unsubscribe", (STATES,))?;
    let packet_text = packet
        .borrow(py)
        .text
        .clone()
        .ok_or_else(|| PyRuntimeError::new_err("missing joint-state packet at 4.500 s"))?;
    command(0.0, 0.0)?;
    run(&server, 1000)?;
    let maps = std::fs::read_to_string("/proc/self/maps")?;
    for stem in [
        "libgz-physics-dartsim-plugin.so",
        "libgz-sim-diff-drive-system.so",
        "libgz-sim-joint-state-publisher-system.so",
    ] {
        let path = maps
            .lines()
            .filter_map(|s| s.split_whitespace().last())
            .find(|p| p.contains(stem))
            .ok_or_else(|| PyRuntimeError::new_err(format!("missing native plugin {stem}")))?;
        println!("loaded {path}");
    }
    let result = observer.borrow(py);
    if result.rows.len() != (LAST / STRIDE + 1) as usize {
        return Err(PyRuntimeError::new_err(format!(
            "incomplete native samples: {}",
            result.rows.len()
        )));
    }
    let mut csv = format!("{HEADER}\n");
    for (i, (tick, row)) in result.rows.iter().enumerate() {
        if *tick != i as u64 * STRIDE {
            return Err(PyRuntimeError::new_err("out-of-order native samples"));
        }
        write!(csv, "{tick}").unwrap();
        for v in row {
            write!(csv, ",{v:.16e}").unwrap();
        }
        csv.push('\n');
    }
    std::fs::create_dir_all(out)?;
    std::fs::write(out.join("native.csv"), csv)?;
    std::fs::write(out.join("joint-state-4500.textproto"), packet_text)?;
    println!("{} ordered samples through tick {LAST}", result.rows.len());
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err("usage: drive-native-reference WORLD.sdf OUTPUT_DIRECTORY".into());
    }
    Python::with_gil(|py| {
        capture(py, Path::new(&args[0]), Path::new(&args[1])).inspect_err(|e| e.print(py))
    })?;
    Ok(())
}
