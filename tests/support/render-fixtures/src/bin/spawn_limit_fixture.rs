// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_client::{Client, ClientError, TrustedWorker};
use std::path::PathBuf;
fn main() {
    match Client::start(
        TrustedWorker::from_operator_config(PathBuf::from("/missing/render-worker")).unwrap(),
    ) {
        Err(ClientError::Io(error)) if error.raw_os_error() == Some(11) => {}
        Err(error) => panic!("unexpected startup failure: {error:?}"),
        Ok(_) => panic!("thread creation unexpectedly succeeded"),
    }
}
