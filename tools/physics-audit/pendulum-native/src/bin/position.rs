// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Capture the original position controller and DART hinge via Gazebo's TestFixture bindings.
use pyo3::{exceptions::PyRuntimeError, prelude::*};
use std::{fmt::Write, path::Path};

const MODEL: &str = "joint_position_controller_test";
const TOPIC: &str = "/model/joint_position_controller_test/joint/j1/0/cmd_pos";

#[pyclass]
#[derive(Default)]
struct Encoder {
    joint: Option<PyObject>,
    samples: Vec<(u64, f64, f64)>,
}

#[pymethods]
impl Encoder {
    fn pre(
        &mut self,
        py: Python<'_>,
        _info: &Bound<'_, PyAny>,
        ecm: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        if self.joint.is_none() {
            let sim = py.import("gz.sim")?;
            let world = sim
                .getattr("World")?
                .call1((sim.getattr("world_entity")?.call1((ecm,))?,))?;
            let model = sim
                .getattr("Model")?
                .call1((world.call_method1("model_by_name", (ecm, MODEL))?,))?;
            let joint = sim
                .getattr("Joint")?
                .call1((model.call_method1("joint_by_name", (ecm, "j1"))?,))?;
            joint.call_method1("enable_position_check", (ecm,))?;
            joint.call_method1("enable_velocity_check", (ecm,))?;
            self.joint = Some(joint.unbind());
        }
        Ok(())
    }

    fn post(
        &mut self,
        py: Python<'_>,
        info: &Bound<'_, PyAny>,
        ecm: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let tick = info.getattr("iterations")?.extract::<u64>()?;
        let seconds = info
            .getattr("sim_time")?
            .call_method0("total_seconds")?
            .extract::<f64>()?;
        if (seconds - tick as f64 * 0.001).abs() > 1e-12 || tick as usize != self.samples.len() + 1
        {
            return Err(PyRuntimeError::new_err("unexpected simulation clock"));
        }
        let joint = self.joint.as_ref().unwrap().bind(py);
        let read = |key: &str| {
            joint
                .call_method1(key, (ecm,))?
                .get_item(0)?
                .extract::<f64>()
        };
        self.samples
            .push((tick, read("position")?, read("velocity")?));
        Ok(())
    }
}

fn capture(py: Python<'_>, world: &Path, output: &Path) -> PyResult<()> {
    py.import("gz.math")?;
    py.import("sdformat")?;
    py.import("gz.common")?
        .getattr("set_verbosity")?
        .call1((4,))?;
    let fixture = py
        .import("gz.sim")?
        .getattr("TestFixture")?
        .call1((world,))?;
    let observer = Py::new(py, Encoder::default())?;
    fixture.call_method1("on_pre_update", (observer.bind(py).getattr("pre")?,))?;
    fixture.call_method1("on_post_update", (observer.bind(py).getattr("post")?,))?;
    fixture.call_method0("finalize")?;
    let server = fixture.call_method0("server")?;
    let node = py.import("gz.transport")?.getattr("Node")?.call0()?;
    let double = py.import("gz.msgs.double_pb2")?.getattr("Double")?;
    let publisher = node.call_method1("advertise", (TOPIC, &double))?;
    let run = |ticks| {
        if server
            .call_method1("run", (true, ticks, false))?
            .extract::<bool>()?
        {
            Ok(())
        } else {
            Err(PyRuntimeError::new_err("Gazebo run failed"))
        }
    };
    run(50_u64)?;
    for target in [1., -0.5, 0.25] {
        let msg = double.call0()?;
        msg.setattr("data", target)?;
        if !publisher
            .call_method0("has_connections")?
            .extract::<bool>()?
            || !publisher
                .call_method1("publish", (msg,))?
                .extract::<bool>()?
        {
            return Err(PyRuntimeError::new_err(
                "position command was not published to a connected controller",
            ));
        }
        run(400)?;
    }
    let map = std::fs::read_to_string("/proc/self/maps")?;
    for plugin in [
        "libgz-physics-dartsim-plugin.so",
        "libgz-sim-joint-position-controller-system.so",
        "libgz-sim-scene-broadcaster-system.so",
    ] {
        let line = map
            .lines()
            .find(|s| s.contains(plugin))
            .ok_or_else(|| PyRuntimeError::new_err(plugin.to_string()))?;
        println!("loaded {}", line.split_whitespace().last().unwrap());
    }
    let mut csv = String::from("tick,position_rad,velocity_rad_s\n");
    for (tick, position, velocity) in &observer.borrow(py).samples {
        writeln!(csv, "{tick},{position:.16e},{velocity:.16e}").unwrap();
    }
    std::fs::write(output, csv)?;
    Ok(())
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err("usage: position WORLD.sdf OUTPUT.csv".into());
    }
    Python::with_gil(|py| {
        capture(py, Path::new(&args[0]), Path::new(&args[1])).inspect_err(|e| e.print(py))
    })?;
    Ok(())
}
