/*
 * Copyright (C) 2014, 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Application naming and resource bounds; contains no network state.
use std::{collections::HashMap, ffi::OsString};

/// Errors from application setup and bounded command admission.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SetupError {
    #[error("invalid application name")]
    InvalidName,
    #[error("invalid application configuration")]
    InvalidConfig,
    #[error("topic already remapped")]
    AlreadyRemapped,
    #[error("application queue full")]
    QueueFull,
    #[error("application deadline expired")]
    Timeout,
    #[error("partition identity lookup failed: {0}")]
    Identity(String),
}

/// Explicit simulator options. `Default` never reads the process environment.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ServerOptions {
    pub partition: String,
    pub namespace: String,
    /// Exact raw-topic substitutions, applied once before qualification.
    pub topic_remaps: HashMap<String, String>,
    /// Maximum queued commands and topic remappings; must fit a signed 32-bit count.
    pub capacity: usize,
    /// Maximum admitted protobuf payload, in bytes; must fit a signed 64-bit count.
    pub max_message_bytes: usize,
}

impl Default for ServerOptions {
    fn default() -> Self {
        Self {
            partition: String::new(),
            namespace: String::new(),
            topic_remaps: HashMap::new(),
            capacity: 1000,
            max_message_bytes: 16 * 1024 * 1024,
        }
    }
}

impl ServerOptions {
    /// Add a bounded, single-hop substitution. Errors leave the map unchanged.
    pub fn add_topic_remap(&mut self, from: &str, to: &str) -> Result<(), SetupError> {
        for topic in [from, to] {
            if topic.is_empty() || !valid_name(topic) {
                return Err(SetupError::InvalidName);
            }
        }
        if self.topic_remaps.contains_key(from) {
            return Err(SetupError::AlreadyRemapped);
        }
        if self.topic_remaps.len() >= self.capacity {
            return Err(SetupError::QueueFull);
        }
        self.topic_remaps.insert(from.into(), to.into());
        Ok(())
    }

    /// Look up the exact raw key without following another remapping.
    pub fn topic_remap(&self, from: &str) -> Option<&str> {
        self.topic_remaps.get(from).map(String::as_str)
    }

    /// Validate options without changing them. Endpoint lengths are also checked
    /// when their actual topics are qualified.
    pub fn validate(&self) -> Result<(), SetupError> {
        qualified_name(&self.partition, &self.namespace, "validation")?;
        if self.capacity == 0
            || self.capacity > i32::MAX as usize
            || self.max_message_bytes == 0
            || self.max_message_bytes > i64::MAX as usize
            || self.topic_remaps.len() > self.capacity
        {
            return Err(SetupError::InvalidConfig);
        }
        for (from, to) in &self.topic_remaps {
            if from.is_empty() || to.is_empty() || !valid_name(from) || !valid_name(to) {
                return Err(SetupError::InvalidName);
            }
        }
        Ok(())
    }

    /// Explicitly capture GZ_PARTITION, or the native hostname:USER identity.
    /// An absent USER uses the current UID's account name. Hostname and account
    /// lookup may block; call this during setup, outside stepping.
    /// Other legacy transport environment variables are not read.
    pub fn from_env() -> Result<Self, SetupError> {
        captured_options(std::env::var_os("GZ_PARTITION"), default_partition)
    }
}

fn captured_options(
    partition: Option<OsString>,
    identity: impl FnOnce() -> Result<String, SetupError>,
) -> Result<ServerOptions, SetupError> {
    let partition = match partition {
        Some(value) => value
            .into_string()
            .map_err(|_| SetupError::Identity("GZ_PARTITION is not UTF-8".into()))?,
        None => identity()?,
    };
    let options = ServerOptions {
        partition,
        ..Default::default()
    };
    options.validate()?;
    Ok(options)
}

#[cfg(unix)]
fn default_partition() -> Result<String, SetupError> {
    use nix::unistd::{User, gethostname, getuid};
    partition_identity(gethostname(), std::env::var_os("USER"), || {
        User::from_uid(getuid()).map(|user| user.map(|user| user.name))
    })
}

#[cfg(not(unix))]
fn default_partition() -> Result<String, SetupError> {
    Err(SetupError::Identity(
        "automatic partition identity is unavailable on this platform".into(),
    ))
}

#[cfg(unix)]
fn partition_identity(
    hostname: nix::Result<OsString>,
    user: Option<OsString>,
    system_user: impl FnOnce() -> nix::Result<Option<String>>,
) -> Result<String, SetupError> {
    let hostname = hostname
        .map_err(|error| SetupError::Identity(format!("hostname: {error}")))?
        .into_string()
        .map_err(|_| SetupError::Identity("hostname is not UTF-8".into()))?;
    let username = match user {
        Some(user) => user
            .into_string()
            .map_err(|_| SetupError::Identity("USER is not UTF-8".into()))?,
        None => {
            let name = system_user()
                .map_err(|error| SetupError::Identity(format!("UID account lookup: {error}")))?
                .ok_or_else(|| SetupError::Identity("current UID has no account name".into()))?;
            // nix decodes NSS bytes lossily. Do not silently change identity.
            if name.contains('\u{fffd}') {
                return Err(SetupError::Identity(
                    "ambiguous NSS username encoding".into(),
                ));
            }
            name
        }
    };
    Ok(format!("{hostname}:{username}"))
}

// TopicUtils.cc uses byte lengths and ASCII space, not Unicode whitespace.
fn valid_name(name: &str) -> bool {
    name.len() <= 65535
        && name != "/"
        && !["~", " ", "//", "@", ":="].iter().any(|s| name.contains(s))
}

pub(crate) fn as_valid_topic(topic: &str) -> String {
    let mut result = String::with_capacity(topic.len());
    let mut input = topic.chars().peekable();
    while let Some(c) = input.next() {
        match c {
            ' ' => result.push('_'),
            '@' | '~' => {}
            ':' if input.peek() == Some(&'=') => {
                input.next();
            }
            '/' if input.peek() == Some(&'/') => {
                input.next();
            }
            c => result.push(c),
        }
    }
    if result.is_empty() || !valid_name(&result) {
        result.clear();
    }
    result
}

pub(crate) fn qualified_name(
    partition: &str,
    namespace: &str,
    topic: &str,
) -> Result<String, SetupError> {
    if !valid_name(partition) || !valid_name(namespace) || !valid_name(topic) || topic.is_empty() {
        return Err(SetupError::InvalidName);
    }
    let partition = partition.trim_matches('/');
    let namespace = namespace.trim_matches('/');
    let topic = topic.trim_end_matches('/');
    let path = if topic.starts_with('/') || namespace.is_empty() {
        format!("/{}", topic.trim_start_matches('/'))
    } else {
        format!("/{namespace}/{topic}")
    };
    let partition = if partition.is_empty() {
        String::new()
    } else {
        format!("/{partition}")
    };
    let name = format!("@{partition}@{path}");
    if name.len() > 65535 {
        Err(SetupError::InvalidName)
    } else {
        Ok(name)
    }
}

#[cfg(test)]
#[path = "../tests/private/options.rs"]
mod tests;
