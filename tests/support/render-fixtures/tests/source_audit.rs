// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::{
    path::PathBuf,
    process::Command,
    sync::atomic::{AtomicU64, Ordering},
};

fn run(source: &str) -> bool {
    static SERIAL: AtomicU64 = AtomicU64::new(0);
    let path: PathBuf = std::env::temp_dir().join(format!(
        "gz-render-source-audit-{}-{}",
        std::process::id(),
        SERIAL.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::write(&path, source).unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_source_audit"))
        .arg(&path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    output.status.success() && output.stdout == b"renderer-facade-v1\n"
}
#[test]
fn accepts_only_the_declaration_only_gpu_facade() {
    let good = "//! facade\n#[cfg(feature = \"gpu\")] mod gpu;\n#[cfg(feature = \"gpu\")] pub use gpu::*;\n";
    assert!(run(good));
    for bad in [
        "",
        "not valid Rust",
        "#[cfg(feature=\"gpu\")] mod gpu;",
        "#[cfg(feature=\"gpu\")] mod gpu {} #[cfg(feature=\"gpu\")] pub use gpu::*;",
        "mod gpu; #[cfg(feature=\"gpu\")] pub use gpu::*;",
        "#[cfg(feature=\"gpu\")] mod gpu; #[cfg(feature=\"other\")] pub use gpu::*;",
        "#[cfg(feature=\"gpu\")] mod gpu; #[cfg(feature=\"gpu\")] pub use gpu::*; pub fn hidden() {}",
        "#[cfg(feature=\"gpu\")] mod gpu; #[cfg(feature=\"gpu\")] pub use gpu::*; invoke!();",
        "#[cfg_attr(feature=\"gpu\", inline)] mod gpu; #[cfg(feature=\"gpu\")] pub use gpu::*;",
    ] {
        assert!(!run(bad), "accepted non-facade source: {bad}");
    }
}
