// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Grip, Result, XML};
use gz_physics::JointVelocityMotor;
use serde::{Deserialize, Serialize};
use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
    time::{Instant, SystemTime, UNIX_EPOCH},
};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(super) struct Program {
    pub seed: u64,
    pub grip: Grip,
    pub mass: f64,
    pub position: [f64; 3],
    pub load: [f64; 3],
    pub load_at: usize,
    pub open_at: usize,
    pub end: usize,
    pub target: f64,
    pub cap: f64,
    pub weak_cap: f64,
    pub torque_period: usize,
    pub transfer_period: usize,
    pub death_at: usize,
    pub mixed_lock: bool,
    pulse: f64,
}
impl Program {
    pub fn baseline(grip: Grip) -> Self {
        Self {
            seed: 0,
            grip,
            mass: 0.72,
            position: [0.317, 0.004, 0.58],
            load: [0., 0., 1.2],
            load_at: 650,
            open_at: 1100,
            end: 1500,
            target: 6.,
            cap: 4.,
            weak_cap: 0.08,
            torque_period: 71,
            transfer_period: 51,
            death_at: 175,
            mixed_lock: false,
            pulse: 1.,
        }
    }
    fn random(seed: u64) -> Self {
        let mut r = fastrand::Rng::with_seed(seed ^ 0x5cfb_2a1d_781b_0e97);
        let grip = match r.u8(0..8) {
            0 => Grip::Open,
            1 => Grip::Weak,
            2 | 3 => Grip::PreloadedWeak,
            _ => Grip::Strong,
        };
        let mut s = Self::baseline(grip);
        s.seed = seed;
        s.mass = 0.24 + 1.2 * r.f64();
        s.position = [
            0.311 + 0.012 * r.f64(),
            -0.008 + 0.016 * r.f64(),
            0.578 + 0.004 * r.f64(),
        ];
        s.load = [
            (r.f64() - 0.5) * 0.85,
            (r.f64() - 0.5) * 0.55,
            (0.3 + 1.6 * r.f64()) * if r.bool() { 1. } else { -1. },
        ];
        s.load_at = r.usize(470..710);
        s.open_at = s.load_at + r.usize(270..620);
        s.end = s.open_at + r.usize(210..570);
        s.target = 3. + 5. * r.f64();
        s.cap = 1.5 + 3.5 * r.f64();
        s.weak_cap = 0.025 + 0.16 * r.f64();
        s.torque_period = r.usize(39..97);
        s.transfer_period = r.usize(36..83);
        s.death_at = r.usize(51..s.load_at);
        s.mixed_lock = r.u8(0..10) == 0;
        s.pulse = if r.bool() { 1. } else { -0.4 + 0.9 * r.f64() };
        s
    }
    pub fn worker(&self) -> PathBuf {
        std::env::var_os("GZ_GRASP_WORKER").map_or_else(
            || PathBuf::from(env!("CARGO_BIN_EXE_physics_worker")),
            PathBuf::from,
        )
    }
    pub fn xml(&self) -> String {
        let [x, y, z] = self.position;
        let mut s = XML
            .replace("<mass>0.72</mass>", &format!("<mass>{}</mass>", self.mass))
            .replace(">0.010<", &format!(">{}<", self.mass * (0.010 / 0.72)))
            .replace("0.317 0.004 0.58", &format!("{x} {y} {z}"));
        if self.mixed_lock {
            s=s.replace("name=\"gz::sim::systems::SwitchableArticulation\"/>","name=\"gz::sim::systems::SwitchableArticulation\"><exact_revolute_locks>true</exact_revolute_locks></plugin>")
                .replacen("<lower>-0.42</lower><upper>0.42</upper>","<lower>0</lower><upper>0</upper>",1);
        }
        s
    }
    pub fn motor(&self, tick: usize, sign: f64) -> Option<JointVelocityMotor> {
        let (open, cap) = match (self.grip, tick >= self.open_at) {
            (Grip::Open, _) => return None,
            (_, true) => (true, self.cap),
            (Grip::Weak, false) => (false, self.weak_cap),
            (Grip::PreloadedWeak, false) if tick >= self.load_at => (false, self.weak_cap),
            _ => (false, self.cap),
        };
        Some(JointVelocityMotor {
            target_velocity: if open {
                -sign * self.target
            } else {
                sign * self.target
            },
            max_force: cap,
        })
    }
    pub fn force(&self, t: usize) -> [f64; 3] {
        if t < self.load_at {
            return [0.; 3];
        }
        if t > self.load_at + (self.open_at - self.load_at) / 2 && t < self.open_at {
            return self.load.map(|f| f * self.pulse);
        }
        self.load
    }
}

pub(super) struct Journal {
    dir: PathBuf,
    output: Option<PathBuf>,
    tick: usize,
    pub success: bool,
}
impl Journal {
    pub fn new(dir: &Path, p: &Program, xml: &str) -> Result<Self> {
        let output = std::env::var_os("GZ_GRASP_CAMPAIGN_DIR").map(PathBuf::from);
        if output.is_some() {
            fs::write(dir.join("program.json"), serde_json::to_vec_pretty(p)?)?;
            fs::write(dir.join("scene.sdf"), xml)?;
        }
        Ok(Self {
            dir: dir.into(),
            output,
            success: false,
            tick: 0,
        })
    }
    pub fn pin(&mut self, tick: usize) -> Result<()> {
        self.tick = tick;
        if self.output.is_some() {
            let prior = self.dir.join("prior.gzgrasptree");
            if prior.exists() {
                fs::remove_file(&prior)?;
            }
            fs::hard_link(self.dir.join("barrier.gzgrasptree"), prior)?;
        }
        Ok(())
    }
    pub fn oracle(&self, bytes: &[u8]) -> Result<()> {
        if self.output.is_some() {
            fs::write(self.dir.join("oracle.native"), bytes)?;
        }
        Ok(())
    }
}
impl Drop for Journal {
    fn drop(&mut self) {
        if self.success {
            return;
        }
        let Some(out) = &self.output else {
            return;
        };
        let witness = out.join("first-counterexample");
        if fs::create_dir(&witness).is_err() {
            return;
        }
        for file in [
            "program.json",
            "scene.sdf",
            "barrier.gzgrasptree",
            "prior.gzgrasptree",
            "oracle.native",
        ] {
            if self.dir.join(file).is_file() {
                let _ = fs::copy(self.dir.join(file), witness.join(file));
            }
        }
        let _ = fs::write(witness.join("input-interval.txt"), self.tick.to_string());
    }
}
fn now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs()
}
#[test]
#[ignore = "duration-bounded randomized ordinary Bevy and real worker recovery campaign"]
fn seeded_payload_load_endurance() -> Result<()> {
    let out = PathBuf::from(std::env::var("GZ_GRASP_CAMPAIGN_DIR")?);
    fs::create_dir_all(&out)?;
    let deadline: u64 = std::env::var("GZ_GRASP_UNTIL_UNIX")?.parse()?;
    let start: u64 = std::env::var("GZ_GRASP_FIRST_SEED")
        .unwrap_or_else(|_| "1".to_owned())
        .parse()?;
    let limit: u64 = std::env::var("GZ_GRASP_CASE_LIMIT")
        .unwrap_or_else(|_| u64::MAX.to_string())
        .parse()?;
    let mut records = fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(out.join("cases.jsonl"))?;
    let mut frames = 0;
    let mut failures = 0;
    let mut completed = 0;
    let start_time = now();
    for seed in start..start.saturating_add(limit) {
        if now() >= deadline {
            break;
        }
        let s = Program::random(seed);
        let t = Instant::now();
        let result = std::panic::catch_unwind(|| super::trial(&s).map_err(|e| e.to_string()))
            .unwrap_or_else(|data| {
                Err(data
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| data.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_else(|| "test panic".into()))
            });
        let row = match result {
            Ok(m) => {
                completed += 1;
                frames += s.end;
                serde_json::json!({"seed":seed,"program":s,"metrics":m,"wall_seconds":t.elapsed().as_secs_f64()})
            }
            Err(message) => {
                failures += 1;
                serde_json::json!({"seed":seed,"program":s,"error":message,"wall_seconds":t.elapsed().as_secs_f64()})
            }
        };
        writeln!(records, "{}", serde_json::to_string(&row)?)?;
        records.flush()?;
        let progress = serde_json::json!({"started_at_unix":start_time,"as_of_unix":now(),"deadline_unix":deadline,"first_seed":start,"last_seed":seed,"successful_seeds":completed,"failed_seeds":failures,"exact_frames":frames});
        let tmp = out.join("progress.next");
        fs::write(&tmp, serde_json::to_vec_pretty(&progress)?)?;
        fs::rename(tmp, out.join("progress.json"))?;
        if failures >= 3 {
            return Err("three failing cases observed; preserving the first replay and stopping for diagnosis".into());
        }
    }
    assert_eq!(failures, 0, "campaign found a counterexample");
    Ok(())
}
