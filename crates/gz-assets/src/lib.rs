// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded local and cache-only Fuel SDF loading before simulation construction.
use gz_fuel_tools::{FuelClient, ResourceKind};
use sdformat::{ModelConfigWarning, Resource, ResourceResolver};
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{self, Read},
    path::{Path, PathBuf},
    sync::Mutex,
};

#[derive(Debug, thiserror::Error)]
pub enum AssetError {
    #[error("resource was not found: {0}")]
    NotFound(String),
    #[error("{path}: {source}")]
    Io { path: PathBuf, source: io::Error },
    #[error("{0}: resource exceeds the configured XML byte limit")]
    TooLarge(PathBuf),
    #[error("{0}: XML must be UTF-8")]
    Utf8(PathBuf),
    #[error(transparent)]
    Config(#[from] sdformat::Diagnostic),
    #[error("resource diagnostic budget exceeded")]
    WarningLimit,
    #[error("resource diagnostic log was poisoned")]
    Poisoned,
    #[error("resource base directory must be absolute")]
    RelativeBase,
    #[error(transparent)]
    Fuel(#[from] gz_fuel_tools::Error),
    #[error("no usable cached Fuel model")]
    CacheMiss,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Warning {
    Config(ModelConfigWarning),
    LegacyManifest(PathBuf),
}

/// Snapshot of an explicit working directory and ordered lookup roots.
/// URI maps precede cwd; other roots follow it. Include lookup does not implicitly
/// search the including file's directory. URI suffixes retain native literal `%`.
/// Canonical file URLs are source identities, not a URL-decoding lookup policy.
pub struct LocalResources {
    cwd: PathBuf,
    roots: Vec<PathBuf>,
    uri_paths: BTreeMap<String, Vec<PathBuf>>,
    max_bytes: u64,
    warnings: Mutex<Vec<Warning>>,
    max_warnings: usize,
    fuel: Option<FuelClient>,
}

impl LocalResources {
    pub fn new(cwd: PathBuf, roots: Vec<PathBuf>, max_bytes: u64) -> Result<Self, AssetError> {
        if !cwd.is_absolute() {
            return Err(AssetError::RelativeBase);
        }
        Ok(Self {
            cwd,
            roots,
            uri_paths: BTreeMap::new(),
            max_bytes,
            warnings: Mutex::new(Vec::new()),
            max_warnings: 1024,
            fuel: None,
        })
    }

    /// Enables cache-only Fuel model lookup after local paths miss.
    /// Construction, URL parsing and cache lookup make no network request.
    pub fn with_cached_fuel(mut self, client: FuelClient) -> Self {
        self.fuel = Some(client);
        self
    }

    pub fn add_uri_path(&mut self, prefix: String, root: PathBuf) {
        self.uri_paths.entry(prefix).or_default().push(root);
    }

    pub fn set_warning_limit(&mut self, limit: usize) {
        self.max_warnings = limit;
    }

    pub fn take_warnings(&self) -> Result<Vec<Warning>, AssetError> {
        Ok(std::mem::take(
            &mut *self.warnings.lock().map_err(|_| AssetError::Poisoned)?,
        ))
    }

    /// Reads an explicit path, selecting model.config (or legacy manifest.xml)
    /// once if it is a directory. Selection never downloads or retries another file.
    pub fn read_path(&self, path: &Path) -> Result<Resource, AssetError> {
        let path = self.cwd.join(path);
        let metadata = fs::metadata(&path).map_err(|source| AssetError::Io {
            path: path.clone(),
            source,
        })?;
        let path = if metadata.is_dir() {
            self.model_file(&path)?
        } else {
            path
        };
        let (canonical, canonical_uri) = canonical_source(&path)?;
        let xml = self.read_text(&canonical)?;
        Ok(Resource { canonical_uri, xml })
    }

    pub fn load_uri(&self, uri: &str) -> Result<Resource, AssetError> {
        // Native lookup treats every failed stat as a nonmatching candidate.
        for (prefix, roots) in &self.uri_paths {
            if let Some(suffix) = uri.strip_prefix(prefix) {
                for root in roots {
                    let path = append(&self.cwd.join(root), suffix);
                    if fs::metadata(&path).is_ok() {
                        return self.read_path(&path);
                    }
                }
            }
        }
        let suffix = uri.split_once("://").map_or(uri, |(_, suffix)| suffix);
        let direct = Path::new(suffix)
            .is_absolute()
            .then(|| PathBuf::from(suffix));
        let candidates = std::iter::once(append(&self.cwd, suffix))
            .chain(direct)
            .chain(
                self.roots
                    .iter()
                    .map(|root| append(&self.cwd.join(root), suffix)),
            );
        for path in candidates {
            if fs::metadata(&path).is_ok() {
                return self.read_path(&path);
            }
        }
        if let Some(client) = &self.fuel {
            let scheme = uri.split_once("://").map_or("", |(scheme, _)| scheme);
            if scheme.eq_ignore_ascii_case("http") || scheme.eq_ignore_ascii_case("https") {
                let id = client.parse(uri, ResourceKind::Model)?;
                let path = client
                    .cached(ResourceKind::Model, &id)
                    .ok_or(AssetError::CacheMiss)?;
                return self.read_path(&path);
            }
        }
        Err(AssetError::NotFound(uri.into()))
    }

    fn read_text(&self, path: &Path) -> Result<String, AssetError> {
        let read = || -> Result<Vec<u8>, io::Error> {
            let mut bytes = Vec::new();
            File::open(path)?
                .take(self.max_bytes.saturating_add(1))
                .read_to_end(&mut bytes)?;
            Ok(bytes)
        };
        let bytes = read().map_err(|source| AssetError::Io {
            path: path.into(),
            source,
        })?;
        if bytes.len() as u64 > self.max_bytes {
            return Err(AssetError::TooLarge(path.into()));
        }
        String::from_utf8(bytes).map_err(|_| AssetError::Utf8(path.into()))
    }

    fn model_file(&self, directory: &Path) -> Result<PathBuf, AssetError> {
        let mut path = directory.join("model.config");
        let legacy = fs::metadata(&path).is_err();
        if legacy {
            path = directory.join("manifest.xml");
        }
        let (canonical, source) = canonical_source(&path)?;
        if legacy {
            self.record_warnings(vec![Warning::LegacyManifest(path)])?;
        }
        let xml = self.read_text(&canonical)?;
        let selection = sdformat::select_model_config(&xml, &source)?;
        let warnings: Vec<_> = selection
            .warnings
            .into_iter()
            .map(Warning::Config)
            .collect();
        self.record_warnings(warnings)?;
        Ok(append(directory, &selection.file_name))
    }

    fn record_warnings(&self, warnings: Vec<Warning>) -> Result<(), AssetError> {
        let mut log = self.warnings.lock().map_err(|_| AssetError::Poisoned)?;
        if warnings.len() > self.max_warnings.saturating_sub(log.len()) {
            return Err(AssetError::WarningLimit);
        }
        log.extend(warnings);
        Ok(())
    }
}

fn canonical_source(path: &Path) -> Result<(PathBuf, String), AssetError> {
    let canonical = fs::canonicalize(path).map_err(|source| AssetError::Io {
        path: path.into(),
        source,
    })?;
    let uri = url::Url::from_file_path(&canonical)
        .map_err(|_| AssetError::RelativeBase)?
        .into();
    Ok((canonical, uri))
}

// Gazebo concatenates even absolute-looking suffixes; Path::join discards the root.
fn append(base: &Path, suffix: &str) -> PathBuf {
    let mut path = base.as_os_str().to_owned();
    path.push(std::path::MAIN_SEPARATOR_STR);
    path.push(suffix);
    path.into()
}

impl ResourceResolver for LocalResources {
    fn resolve(&self, uri: &str, _parent: &str) -> Result<Resource, String> {
        self.load_uri(uri).map_err(|error| error.to_string())
    }
}
