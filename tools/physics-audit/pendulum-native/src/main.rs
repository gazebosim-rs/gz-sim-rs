// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read an unmodified Gazebo world using the installed native TestFixture binding.
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use std::fmt::Write;
use std::path::Path;

use pendulum_native_reference::{HEADER, LAST_TICK, STRIDE};

struct Pendulum {
    joints: Vec<PyObject>,
    links: Vec<PyObject>,
}

#[pyclass]
struct Observer {
    pendulums: Vec<Pendulum>,
    gravity: [f64; 3],
    rows: Vec<(u64, usize, [f64; 15])>,
}

fn xyz(vector: &Bound<'_, PyAny>) -> PyResult<[f64; 3]> {
    let coord = |axis| vector.call_method0(axis)?.extract::<f64>();
    Ok([coord("x")?, coord("y")?, coord("z")?])
}

#[pymethods]
impl Observer {
    fn pre(
        &mut self,
        py: Python<'_>,
        _info: &Bound<'_, PyAny>,
        ecm: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        if !self.pendulums.is_empty() {
            return Ok(());
        }
        let sim = py.import("gz.sim")?;
        let world_id = sim.getattr("world_entity")?.call1((ecm,))?;
        let world = sim.getattr("World")?.call1((world_id,))?;
        self.gravity = xyz(&world.call_method1("gravity", (ecm,))?)?;
        for index in 0..2 {
            let id = world.call_method1(
                "model_by_name",
                (ecm, format!("double_pendulum_with_base{index}")),
            )?;
            let model = sim.getattr("Model")?.call1((id,))?;
            let mut joints = Vec::new();
            for name in ["upper_joint", "lower_joint"] {
                let id = model.call_method1("joint_by_name", (ecm, name))?;
                let joint = sim.getattr("Joint")?.call1((id,))?;
                joint.call_method1("enable_position_check", (ecm,))?;
                joint.call_method1("enable_velocity_check", (ecm,))?;
                joints.push(joint.unbind());
            }
            let mut links = Vec::new();
            for name in ["base", "upper_link", "lower_link"] {
                let id = model.call_method1("link_by_name", (ecm, name))?;
                let link = sim.getattr("Link")?.call1((id,))?;
                link.call_method1("enable_velocity_checks", (ecm,))?;
                links.push(link.unbind());
            }
            self.pendulums.push(Pendulum { joints, links });
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
            let seconds = info
                .getattr("sim_time")?
                .call_method0("total_seconds")?
                .extract::<f64>()?;
            if (seconds - tick as f64 * 0.001).abs() > 1e-12 {
                return Err(PyValueError::new_err(format!(
                    "unexpected native time {seconds} at tick {tick}"
                )));
            }
            self.sample(py, ecm, tick)?;
        }
        Ok(())
    }
}

impl Observer {
    fn sample(&mut self, py: Python<'_>, ecm: &Bound<'_, PyAny>, tick: u64) -> PyResult<()> {
        for (index, pendulum) in self.pendulums.iter().enumerate() {
            let mut row = [0.0; 15];
            // SDF initial joint coordinates and velocities are zero. Native encoder
            // components contain empty vectors until the first physics update.
            if tick > 0 {
                for (i, joint) in pendulum.joints.iter().enumerate() {
                    let joint = joint.bind(py);
                    row[i] = joint
                        .call_method1("position", (ecm,))?
                        .get_item(0)?
                        .extract()?;
                    row[2 + i] = joint
                        .call_method1("velocity", (ecm,))?
                        .get_item(0)?
                        .extract()?;
                }
            }
            for (i, link) in pendulum.links.iter().enumerate() {
                let link = link.bind(py);
                let inertial = link.call_method1("world_inertial", (ecm,))?;
                let mass = inertial
                    .call_method0("mass_matrix")?
                    .call_method0("mass")?
                    .extract::<f64>()?;
                let pose = inertial.call_method0("pose")?;
                let com = xyz(&pose)?;
                row[12] -= mass
                    * com
                        .iter()
                        .zip(self.gravity)
                        .map(|(x, g)| x * g)
                        .sum::<f64>();
                row[13] += link
                    .call_method1("world_kinetic_energy", (ecm,))?
                    .extract::<f64>()?;
                if i > 0 {
                    let body = link.call_method1("world_pose", (ecm,))?;
                    row[3 + i] = body.call_method0("roll")?.extract()?;
                    let col = 6 + 3 * (i - 1);
                    row[col..col + 3].copy_from_slice(&com);
                }
            }
            row[14] = row[12] + row[13];
            if !row.iter().all(|x| x.is_finite()) {
                return Err(PyValueError::new_err("nonfinite native observation"));
            }
            self.rows.push((tick, index, row));
        }
        Ok(())
    }
}

fn capture(py: Python<'_>, world: &Path, output: &Path) -> PyResult<()> {
    py.import("gz.common")?
        .getattr("set_verbosity")?
        .call1((4,))?;
    py.import("gz.math")?;
    py.import("sdformat")?;
    let sim = py.import("gz.sim")?;
    let fixture = sim.getattr("TestFixture")?.call1((world,))?;
    let observer = Py::new(
        py,
        Observer {
            pendulums: Vec::new(),
            gravity: [0.0; 3],
            rows: Vec::new(),
        },
    )?;
    fixture.call_method1("on_pre_update", (observer.bind(py).getattr("pre")?,))?;
    fixture.call_method1("on_post_update", (observer.bind(py).getattr("post")?,))?;
    fixture.call_method0("finalize")?;
    let server = fixture.call_method0("server")?;
    if !server
        .call_method1("run", (true, LAST_TICK, false))?
        .extract::<bool>()?
    {
        return Err(PyRuntimeError::new_err("native server failed to run"));
    }
    let mappings = std::fs::read_to_string("/proc/self/maps")?;
    let dart = mappings
        .lines()
        .filter_map(|line| line.split_whitespace().last())
        .find(|p| p.contains("/libgz-physics-dartsim-plugin.so"))
        .ok_or_else(|| PyRuntimeError::new_err("DART plugin missing from native process"))?;
    let result = observer.borrow(py);
    if result.rows.len() != (LAST_TICK / STRIDE + 1) as usize * 2 {
        return Err(PyRuntimeError::new_err(format!(
            "incomplete native trajectory, {} rows",
            result.rows.len()
        )));
    }
    let mut csv = format!("{HEADER}\n");
    for (n, (tick, model, row)) in result.rows.iter().enumerate() {
        if (*tick, *model) != ((n / 2) as u64 * STRIDE, n % 2) {
            return Err(PyRuntimeError::new_err("out-of-order native samples"));
        }
        write!(&mut csv, "{tick},{model}").unwrap();
        for value in row {
            write!(&mut csv, ",{value:.16e}").unwrap();
        }
        csv.push('\n');
    }
    std::fs::write(output, csv)?;
    println!(
        "native DART {dart}, gravity {:?}, {} samples",
        result.gravity,
        result.rows.len()
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err("usage: pendulum-native-reference WORLD.sdf OUTPUT.csv".into());
    }
    Python::with_gil(|py| {
        capture(py, Path::new(&args[0]), Path::new(&args[1])).inspect_err(|e| e.print(py))
    })?;
    Ok(())
}
