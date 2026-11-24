// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One loopback distributed physics partition. The parent supplies its seed via stdin.
use gz_physics_distributed::transport;
use sha2::{Digest, Sha256};
use std::io;

#[cfg(target_os = "linux")]
#[path = "physics_worker/ssh_supervisor.rs"]
mod ssh_supervisor;

fn verify_executable(expected: &str) -> io::Result<()> {
    if expected.len() != 64
        || !expected
            .bytes()
            .all(|c| matches!(c, b'0'..=b'9' | b'a'..=b'f'))
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "expected worker SHA-256 as 64 lowercase hexadecimal digits",
        ));
    }
    // The path an operator launched may have been atomically replaced.
    #[cfg(target_os = "linux")]
    let mut image = std::fs::File::open("/proc/self/exe")?;
    #[cfg(not(target_os = "linux"))]
    let mut image = std::fs::File::open(std::env::current_exe()?)?;
    let mut hash = Sha256::new();
    io::copy(&mut image, &mut hash)?;
    if format!("{:x}", hash.finalize()) != expected {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "running worker SHA-256 differs from the expected executable",
        ));
    }
    Ok(())
}

struct ForwardedSlot<'a> {
    slot: u8,
    lease: &'a std::path::Path,
    remote: std::num::NonZeroU16,
    local: std::num::NonZeroU16,
}

fn forwarded_slot(args: &[String]) -> Result<ForwardedSlot<'_>, Box<dyn std::error::Error>> {
    let [slot, lease, remote, local, expected] = args else {
        return Err(
            "usage: --managed-multisphere-ssh-v1 SLOT LEASE REMOTE_PORT LOCAL_PORT EXPECTED_SHA256"
                .into(),
        );
    };
    verify_executable(expected)?;
    let slot = ForwardedSlot {
        slot: slot.parse()?,
        lease: std::path::Path::new(lease),
        remote: remote.parse()?,
        local: local.parse()?,
    };
    gz_physics_distributed::multi_sphere::validate_forwarded_slot(slot.slot, slot.lease)?;
    Ok(slot)
}

fn serve_forwarded(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let slot = forwarded_slot(args)?;
    gz_physics_distributed::multi_sphere::serve_forwarded(
        slot.slot,
        slot.lease,
        slot.remote,
        slot.local,
    )?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if let Some((flag, rest)) = args.split_first() {
        #[cfg(target_os = "linux")]
        if flag == ssh_supervisor::SOLVER {
            let (parent, forwarded) = rest.split_last().ok_or("missing SSH supervisor PID")?;
            ssh_supervisor::arm(parent.parse()?)?;
            return serve_forwarded(forwarded);
        }
        if flag == "--managed-multisphere-ssh-v1" {
            #[cfg(target_os = "linux")]
            return ssh_supervisor::supervise(rest);
            #[cfg(not(target_os = "linux"))]
            return serve_forwarded(rest);
        }
    }
    if let [flag] = args.as_slice()
        && matches!(
            flag.as_str(),
            "--probe-floating-tree-v2"
                | "--probe-grasp-fleet-prior-v3"
                | "--probe-grounded-robot-v2"
                | "--probe-grounded-robot-v3"
        )
    {
        return Ok(());
    }
    if let [flag, slot, lease] = args.as_slice()
        && flag == "--managed-trajectory-material-v1"
    {
        return Ok(gz_physics_distributed::trajectory::serve_material(
            slot.parse()?,
            std::path::Path::new(lease),
        )?);
    }
    if let [flag, slot, lease] = args.as_slice()
        && flag == "--managed-trajectory-v1"
    {
        return Ok(gz_physics_distributed::trajectory::serve(
            slot.parse()?,
            std::path::Path::new(lease),
        )?);
    }
    if let [flag, slot, lock] = args.as_slice()
        && flag == "--managed-moving-v1"
    {
        return Ok(gz_physics_distributed::moving::serve(
            slot.parse()?,
            std::path::Path::new(lock),
        )?);
    }
    if let [flag, slot, lock] = args.as_slice()
        && flag == "--managed-multisphere-v2"
    {
        gz_physics_distributed::multi_sphere::serve(slot.parse()?, std::path::Path::new(lock))?;
        return Ok(());
    }
    if let [flag, slot, lock] = args.as_slice() {
        use gz_physics_distributed::floating_tree;
        let serve = match flag.as_str() {
            "--managed-floating-tree-v1" => {
                Some(floating_tree::serve as fn(u8, &std::path::Path) -> io::Result<()>)
            }
            "--managed-floating-tree-v2" => Some(floating_tree::serve_vector as _),
            "--managed-stopped-tree-v1" => Some(floating_tree::serve_stopped as _),
            "--managed-motor-tree-v1" => Some(floating_tree::serve_motor as _),
            "--managed-grounded-robot-v3" => {
                Some(gz_physics_distributed::grounded_robot::serve_controlled as _)
            }
            "--managed-grounded-robot-v2" => {
                Some(gz_physics_distributed::grounded_robot::serve_packed as _)
            }
            "--managed-grounded-robot-v1" => {
                Some(gz_physics_distributed::grounded_robot::serve as _)
            }
            "--managed-grounded-tree-v1" => Some(floating_tree::serve_grounded as _),
            "--managed-payload-tree-v2" => Some(floating_tree::serve_payload as _),
            "--managed-payload-tree-v3" => Some(floating_tree::serve_exact_payload as _),
            "--managed-rotor-material-v1" => Some(floating_tree::automatic::serve_rotor as _),
            "--managed-timed-material-v1" => Some(floating_tree::automatic::serve as _),
            "--managed-contact-tree-v1" => Some(floating_tree::serve_contact as _),
            "--managed-grasp-tree-v1" => Some(floating_tree::serve_grasp as _),
            "--managed-split-grasp-v1" => Some(floating_tree::placement::serve as _),
            "--managed-grasp-fleet-v3" => Some(floating_tree::placement::fleet::serve_prior as _),
            "--managed-grasp-fleet-v2" => Some(floating_tree::placement::fleet::serve_warm as _),
            "--managed-grasp-fleet-v1" => Some(floating_tree::placement::fleet::serve as _),
            "--managed-coupled-grasp-fleet-v2" => Some(floating_tree::cluster::serve_v2 as _),
            "--managed-coupled-grasp-fleet-v1" => Some(floating_tree::cluster::serve as _),
            _ => None,
        };
        if let Some(serve) = serve {
            serve(slot.parse()?, std::path::Path::new(lock))?;
            return Ok(());
        }
    }
    let managed = args
        .last()
        .and_then(|flag| transport::managed_worker_profile(flag));
    let hinge = args
        .last()
        .map(String::as_str)
        .filter(|flag| matches!(*flag, "--managed-hinge-v1" | "--managed-ground-hinge-v1"));
    let positional = &args[..args.len() - usize::from(hinge.is_some() || managed.is_some())];
    let [run, r, worker, w] = positional else {
        return Err("usage: physics_worker --run RUN_ID --worker WORKER_ID".into());
    };
    if run != "--run" || worker != "--worker" {
        return Err("usage: physics_worker --run RUN_ID --worker WORKER_ID".into());
    }
    if let Some(flag) = hinge {
        let serve = if flag == "--managed-ground-hinge-v1" {
            transport::serve_managed_ground_hinge
        } else {
            transport::serve_managed_hinge
        };
        serve(r.parse()?, w.parse()?)?;
        return Ok(());
    }
    if let Some((profile, image)) = managed {
        return Ok(transport::serve_managed_profile(
            r.parse()?,
            w.parse()?,
            image,
            profile,
        )?);
    }
    transport::serve(
        r.parse()?,
        w.parse()?,
        io::stdin().lock(),
        io::stdout().lock(),
    )?;
    Ok(())
}
