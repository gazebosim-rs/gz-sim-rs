// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Immutable local glTF resources, including bounded STL-to-GLB visuals.
use bytes::Bytes;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::VisualGeometry;
use http_body_util::Full;
use hyper::{Request, Response, StatusCode, body::Incoming, service::service_fn};
use hyper_util::rt::TokioIo;
use std::{
    collections::BTreeMap,
    convert::Infallible,
    fs::File,
    io::{self, Read},
    net::{Ipv4Addr, TcpListener},
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

const FILE_LIMIT: usize = 16 * 1024 * 1024;
const TOTAL_LIMIT: usize = 64 * 1024 * 1024;
const FILE_COUNT: usize = 128;

pub(super) type Files = BTreeMap<String, Bytes>;

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

/// Deliberately small admitted filename subset, shared with the viewer's asset source.
fn relative_path(value: &str) -> bool {
    !value.is_empty()
        && value.split('/').all(|segment| {
            !segment.is_empty()
                && segment != "."
                && segment != ".."
                && segment
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || b"._-".contains(&b))
        })
}

fn canonical(path: &Path) -> io::Result<PathBuf> {
    path.canonicalize()
        .map_err(|e| invalid(format!("{}: {e}", path.display())))
}

fn read_file(path: &Path) -> io::Result<Bytes> {
    if !std::fs::metadata(path)
        .map_err(|e| invalid(format!("{}: {e}", path.display())))?
        .is_file()
    {
        return Err(invalid(format!(
            "mesh resource must be a regular file: {}",
            path.display()
        )));
    }
    let mut bytes = Vec::new();
    File::open(path)
        .and_then(|file| file.take((FILE_LIMIT + 1) as u64).read_to_end(&mut bytes))
        .map_err(|e| invalid(format!("{}: {e}", path.display())))?;
    if bytes.len() > FILE_LIMIT {
        return Err(invalid(format!(
            "mesh resource exceeds 16 MiB: {}",
            path.display()
        )));
    }
    Ok(bytes.into())
}

fn mesh_path(mesh: &sdformat::Mesh) -> io::Result<PathBuf> {
    if !mesh.submesh.is_empty() || mesh.center_submesh {
        return Err(invalid(
            "M2 does not support submesh selection or recentering",
        ));
    }
    let base = url::Url::parse(&mesh.source.uri).map_err(io::Error::other)?;
    if base.scheme() != "file" {
        return Err(invalid("M2 mesh source must be a local file"));
    }
    let uri = base.join(&mesh.uri).map_err(io::Error::other)?;
    if uri.scheme() != "file" || uri.query().is_some() || uri.fragment().is_some() {
        return Err(invalid(
            "mesh assets accept only local glTF or STL file URIs",
        ));
    }
    let path = uri
        .to_file_path()
        .map_err(|()| invalid("invalid local mesh file URI"))?;
    if !matches!(
        path.extension().and_then(|s| s.to_str()),
        Some("gltf" | "glb" | "stl")
    ) {
        return Err(invalid(
            "mesh must be glTF 2.0 (.gltf or .glb) or local .stl",
        ));
    }
    if path.extension().and_then(|v| v.to_str()) == Some("stl")
        && (mesh.optimization != sdformat::MeshOptimization::None
            || mesh.convex_decomposition.is_some()
            || !mesh.scale.iter().all(|v| v.is_finite() && *v > 0.0))
    {
        return Err(invalid(
            "STL requires positive finite scale and no optimization/decomposition",
        ));
    }
    Ok(path)
}

fn insert_file(
    files: &mut Files,
    route: String,
    bytes: Bytes,
    total: &mut usize,
) -> io::Result<()> {
    if files.contains_key(&route) {
        return Ok(());
    }
    if files.len() == FILE_COUNT || bytes.len() > TOTAL_LIMIT.saturating_sub(*total) {
        return Err(invalid("M2 exceeds 128 declared resources or 64 MiB total"));
    }
    *total += bytes.len();
    files.insert(route, bytes);
    Ok(())
}

fn document_paths(mesh: &sdformat::Mesh) -> io::Result<(PathBuf, PathBuf, String)> {
    let path = mesh_path(mesh)?;
    let name = path
        .file_name()
        .and_then(|v| v.to_str())
        .filter(|name| relative_path(name))
        .ok_or_else(|| {
            invalid("M2 mesh filenames require ASCII letters, digits, dot, dash or underscore")
        })?;
    let root = canonical(
        path.parent()
            .ok_or_else(|| invalid("mesh needs a parent directory"))?,
    )?;
    let document_path = canonical(&path)?;
    if !document_path.starts_with(&root) {
        return Err(invalid("mesh symlink escapes its resource directory"));
    }
    Ok((root, document_path, name.to_owned()))
}

fn parse_document(bytes: &[u8], label: &str) -> io::Result<gltf::Gltf> {
    let gltf = gltf::Gltf::from_slice(bytes).map_err(|e| invalid(format!("{label}: {e}")))?;
    let asset = &gltf.as_json().asset;
    if asset.version != "2.0" || asset.min_version.as_deref().is_some_and(|v| v != "2.0") {
        return Err(invalid(
            "M2 requires glTF version 2.0 and absent or 2.0 minVersion",
        ));
    }
    if gltf.scenes().next().is_none() {
        return Err(invalid("M2 glTF must contain a scene"));
    }
    Ok(gltf)
}

fn resource_uris(gltf: &gltf::Gltf) -> impl Iterator<Item = &str> {
    gltf.buffers()
        .filter_map(|buffer| match buffer.source() {
            gltf::buffer::Source::Uri(uri) => Some(uri),
            gltf::buffer::Source::Bin => None,
        })
        .chain(gltf.images().filter_map(|image| match image.source() {
            gltf::image::Source::Uri { uri, .. } => Some(uri),
            gltf::image::Source::View { .. } => None,
        }))
}

fn admit(
    mesh: &sdformat::Mesh,
    scope: usize,
    files: &mut Files,
    total: &mut usize,
) -> io::Result<String> {
    let (root, document_path, name) = document_paths(mesh)?;
    let bytes = read_file(&document_path)?;
    if Path::new(&name).extension().and_then(|v| v.to_str()) == Some("stl") {
        let generated = gz_assets::mesh::stl_visual_glb(&bytes).map_err(io::Error::other)?;
        let entry = format!("/assets/{scope}/{name}.glb");
        insert_file(files, entry.clone(), generated.into(), total)?;
        return Ok(entry);
    }
    let gltf = parse_document(&bytes, &document_path.display().to_string())?;
    let prefix = format!("/assets/{scope}/");
    let entry = format!("{prefix}{name}");
    for resource in resource_uris(&gltf) {
        // Embedded resources remain the maintained glTF loader's responsibility.
        if resource.starts_with("data:") {
            continue;
        }
        if !relative_path(resource) {
            return Err(invalid(format!(
                "unsupported local glTF resource URI: {resource}"
            )));
        }
        let path = canonical(&root.join(resource))?;
        if !path.starts_with(&root) {
            return Err(invalid(
                "glTF dependency symlink escapes its resource directory",
            ));
        }
        let route = format!("{prefix}{resource}");
        if !files.contains_key(&route) {
            insert_file(files, route, read_file(&path)?, total)?;
        }
    }
    insert_file(files, entry.clone(), bytes, total)?;
    Ok(entry)
}

pub(super) struct Assets {
    files: Arc<Files>,
    entries: BTreeMap<u64, (sdformat::Mesh, String)>,
    aliases: Option<std::collections::BTreeSet<u64>>,
    port: u16,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

/// Startup-captured resources. Bytes and routes cannot change through this view;
/// the original prefix identifies recorded URLs, not a listener kept alive here.
#[derive(Clone)]
pub(super) struct CapturedAssets {
    prefix: String,
    files: Arc<Files>,
}
impl CapturedAssets {
    pub(super) fn prefix(&self) -> &str {
        &self.prefix
    }
    pub(super) fn files(&self) -> &Files {
        &self.files
    }
}

fn asset_route(route: &str) -> Option<(&str, &str)> {
    route
        .strip_prefix("/assets/")
        .and_then(|path| path.split_once('/'))
        .filter(|(scope, relative)| {
            !scope.is_empty()
                && scope.bytes().all(|b| b.is_ascii_digit())
                && relative_path(relative)
        })
}

fn validate_files(files: &Files) -> io::Result<()> {
    if files.len() > FILE_COUNT {
        return Err(invalid("playback exceeds 128 declared resources"));
    }
    let mut total = 0;
    for (route, bytes) in files {
        if asset_route(route).is_none()
            || bytes.len() > FILE_LIMIT
            || bytes.len() > TOTAL_LIMIT - total
        {
            return Err(invalid(
                "invalid playback resource route or 16 MiB/64 MiB byte bound",
            ));
        }
        total += bytes.len();
    }
    Ok(())
}

/// Validate a captured glTF entry and its declared URI closure before playback
/// publishes a scene. Never reads local paths, follows URLs or decodes pixels.
/// Internal buffer/accessor/pixel decoding remains the maintained loader's job,
/// matching the live asset admission boundary.
pub(super) fn validate_recorded_document(uri: &str, prefix: &str, files: &Files) -> io::Result<()> {
    validate_files(files)?;
    let port = prefix
        .strip_prefix("http://127.0.0.1:")
        .and_then(|port| port.parse::<u16>().ok())
        .filter(|port| *port != 0)
        .ok_or_else(|| invalid("recorded assets require a canonical loopback HTTP prefix"))?;
    if prefix != format!("http://127.0.0.1:{port}") || uri.len() > 2048 {
        return Err(invalid("invalid recorded mesh URI prefix or length"));
    }
    let route = uri
        .strip_prefix(prefix)
        .ok_or_else(|| invalid("recorded mesh URI is outside its captured origin"))?;
    let (scope, relative) =
        asset_route(route).ok_or_else(|| invalid("invalid recorded mesh route"))?;
    if !matches!(relative.rsplit('.').next(), Some("gltf" | "glb")) {
        return Err(invalid("recorded mesh entry must be .gltf or .glb"));
    }
    let bytes = files
        .get(route)
        .ok_or_else(|| invalid("recorded mesh document is missing"))?;
    let gltf = parse_document(bytes, uri)?;
    let directory = route.rsplit_once('/').expect("validated asset route").0;
    for resource in resource_uris(&gltf) {
        if resource.starts_with("data:") {
            continue;
        }
        if !relative_path(resource) {
            return Err(invalid(
                "recorded glTF dependency is outside its admitted path subset",
            ));
        }
        let dependency = format!("{directory}/{resource}");
        if asset_route(&dependency).is_none_or(|(actual_scope, _)| actual_scope != scope)
            || !files.contains_key(&dependency)
        {
            return Err(invalid(
                "recorded glTF declared resource is missing from its captured scope",
            ));
        }
    }
    Ok(())
}

/// Read-only HTTP ownership for an already admitted recording's immutable files.
/// File format/manifest/glTF validation belongs to recording admission; this
/// boundary independently validates route and byte budgets before binding.
pub(super) struct PlaybackAssets {
    port: u16,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}
impl PlaybackAssets {
    /// Empty recordings allocate no listener, runtime or worker. Nonempty input
    /// is moved into the existing server, without filesystem or native access.
    pub(super) fn new(files: Files) -> io::Result<Option<Self>> {
        validate_files(&files)?;
        if files.is_empty() {
            return Ok(None);
        }
        let (port, stop, worker) = start_server(Arc::new(files))?;
        Ok(Some(Self {
            port,
            stop,
            worker: Some(worker),
        }))
    }
    /// New loopback origin; admitted route suffixes remain byte-for-byte stable.
    pub(super) fn prefix(&self) -> String {
        format!("http://127.0.0.1:{}", self.port)
    }
    pub(super) fn healthy(&self) -> bool {
        self.worker
            .as_ref()
            .is_some_and(|worker| !worker.is_finished())
    }
}
impl Drop for PlaybackAssets {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn start_server(files: Arc<Files>) -> io::Result<(u16, Arc<AtomicBool>, JoinHandle<()>)> {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0))?;
    listener.set_nonblocking(true)?;
    let port = listener.local_addr()?.port();
    let stop = Arc::new(AtomicBool::new(false));
    let worker_stop = stop.clone();
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_io()
        .enable_time()
        .build()?;
    let worker = thread::Builder::new()
        .name("gzweb-assets".into())
        .spawn(move || {
            runtime.block_on(serve(listener, files, worker_stop));
        })?;
    Ok((port, stop, worker))
}

/// A checked reference to already captured immutable bytes, never a new path.
pub(super) struct Alias {
    id: u64,
    entry: (sdformat::Mesh, String),
}

impl Assets {
    pub(super) fn new(sim: &Simulation) -> io::Result<Option<Self>> {
        let mut entries: BTreeMap<u64, (sdformat::Mesh, String)> = BTreeMap::new();
        let mut files = Files::new();
        let mut documents: BTreeMap<(PathBuf, PathBuf), String> = BTreeMap::new();
        let mut total = 0;
        for entity in sim.world().iter_entities() {
            let Some(geometry) = entity.get::<VisualGeometry>() else {
                continue;
            };
            let sdformat::Geometry::Mesh(mesh) = geometry.0.as_ref() else {
                continue;
            };
            let id = entity
                .get::<EntityId>()
                .ok_or_else(|| invalid("mesh visual has no identity"))?
                .get();
            if entries.len() == 128 {
                return Err(invalid("M2 exceeds 128 mesh visuals"));
            }
            // Document/base identity is independent of each visual's scale/source position.
            let (base, document, _) = document_paths(mesh)?;
            let key = (base, document);
            let route = if let Some(route) = documents.get(&key) {
                route.clone()
            } else {
                let route = admit(mesh, entries.len(), &mut files, &mut total)?;
                documents.insert(key, route.clone());
                route
            };
            entries.insert(id, (mesh.clone(), route));
        }
        if entries.is_empty() {
            return Ok(None);
        }
        let files = Arc::new(files);
        let (port, stop, worker) = start_server(files.clone())?;
        eprintln!("gzweb-rs local glTF resources: http://127.0.0.1:{port}/assets/");
        Ok(Some(Self {
            files,
            entries,
            aliases: None,
            port,
            stop,
            worker: Some(worker),
        }))
    }

    /// The original origin and exact declared bytes outlive visual alias retirement.
    /// This shallow immutable view owns no new file handles or resource worker.
    pub(super) fn snapshot(&self) -> CapturedAssets {
        CapturedAssets {
            prefix: format!("http://127.0.0.1:{}", self.port),
            files: self.files.clone(),
        }
    }

    pub(super) fn uri(&self, id: u64, mesh: &sdformat::Mesh) -> io::Result<String> {
        let (_, route) = self
            .entries
            .get(&id)
            .filter(|(original, _)| original == mesh)
            .ok_or_else(|| invalid("mesh visual is outside the admitted M2 asset snapshot"))?;
        Ok(format!("http://127.0.0.1:{}{route}", self.port))
    }

    pub(super) fn prepare_alias(
        &self,
        source: u64,
        id: u64,
        mesh: &sdformat::Mesh,
    ) -> io::Result<Alias> {
        if self.entries.len() >= 256
            || self.entries.contains_key(&id)
            || id == 0
            || id > u32::MAX as u64
        {
            return Err(invalid("runtime mesh alias exceeds admitted identities"));
        }
        let entry = self
            .entries
            .get(&source)
            .filter(|(original, _)| original == mesh)
            .ok_or_else(|| invalid("runtime mesh is not a declared template"))?
            .clone();
        Ok(Alias { id, entry })
    }

    pub(super) fn commit_alias(&mut self, alias: Alias) {
        assert!(
            self.entries.insert(alias.id, alias.entry).is_none(),
            "exclusive preflighted asset alias"
        );
        self.aliases
            .get_or_insert_with(Default::default)
            .insert(alias.id);
    }

    pub(super) fn retain_aliases(&mut self, live: &std::collections::BTreeSet<u64>) {
        if let Some(aliases) = &mut self.aliases {
            aliases.retain(|id| {
                if live.contains(id) {
                    true
                } else {
                    self.entries.remove(id);
                    false
                }
            });
            if aliases.is_empty() {
                self.aliases = None;
            }
        }
    }

    pub(super) fn healthy(&self) -> bool {
        self.worker
            .as_ref()
            .is_some_and(|worker| !worker.is_finished())
    }
}

impl Drop for Assets {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn reply<B>(request: Request<B>, files: &Files) -> Response<Full<Bytes>> {
    let origin = request
        .headers()
        .get("origin")
        .and_then(|value| value.to_str().ok());
    let allowed = !request.headers().contains_key("origin")
        || matches!(
            origin,
            Some(
                "http://127.0.0.1:8080"
                    | "http://127.0.0.1:8081"
                    | "http://localhost:8080"
                    | "http://localhost:8081"
            )
        );
    let path = request.uri().path();
    let admissible = allowed
        && request.method() == hyper::Method::GET
        && request.uri().query().is_none()
        && request.uri().scheme().is_none()
        && !request.headers().contains_key("transfer-encoding")
        && request
            .headers()
            .get("content-length")
            .is_none_or(|v| v == "0");
    let data = admissible.then(|| files.get(path)).flatten();
    let (status, body) = match data {
        Some(bytes) => (StatusCode::OK, bytes.clone()),
        None if !admissible => (
            StatusCode::BAD_REQUEST,
            Bytes::from_static(b"Unsupported local asset request"),
        ),
        None => (
            StatusCode::NOT_FOUND,
            Bytes::from_static(b"Asset was not declared by the loaded glTF"),
        ),
    };
    let content_type = if path.ends_with(".gltf") {
        "model/gltf+json"
    } else if path.ends_with(".glb") {
        "model/gltf-binary"
    } else if path.ends_with(".png") {
        "image/png"
    } else if path.ends_with(".jpg") || path.ends_with(".jpeg") {
        "image/jpeg"
    } else {
        "application/octet-stream"
    };
    let mut response = Response::new(Full::new(body));
    *response.status_mut() = status;
    let headers = response.headers_mut();
    headers.insert("connection", "close".parse().expect("constant header"));
    headers.insert(
        "cache-control",
        "no-store".parse().expect("constant header"),
    );
    headers.insert(
        "content-type",
        content_type.parse().expect("constant header"),
    );
    headers.insert(
        "x-content-type-options",
        "nosniff".parse().expect("constant header"),
    );
    if let Some(origin) = origin.filter(|_| allowed) {
        headers.insert(
            "access-control-allow-origin",
            origin.parse().expect("validated request header"),
        );
        headers.insert("vary", "Origin".parse().expect("constant header"));
    }
    response
}

async fn serve(listener: TcpListener, files: Arc<Files>, stop: Arc<AtomicBool>) {
    let Ok(listener) = tokio::net::TcpListener::from_std(listener) else {
        return;
    };
    while !stop.load(Ordering::Acquire) {
        let accepted = tokio::time::timeout(Duration::from_millis(10), listener.accept()).await;
        let Ok(Ok((stream, _))) = accepted else {
            continue;
        };
        let files = files.clone();
        let service = service_fn(move |request: Request<Incoming>| {
            let response = reply(request, &files);
            std::future::ready(Ok::<_, Infallible>(response))
        });
        let mut http = hyper::server::conn::http1::Builder::new();
        http.keep_alive(false).max_headers(32).max_buf_size(8192);
        // At most one active peer; slow readers and incomplete headers cannot hold shutdown.
        let _ = tokio::time::timeout(
            Duration::from_secs(1),
            http.serve_connection(TokioIo::new(stream), service),
        )
        .await;
    }
}

#[cfg(test)]
#[path = "../../tests/private/assets.rs"]
mod tests;

#[cfg(test)]
#[path = "../../tests/private/playback_assets.rs"]
mod playback_tests;

#[cfg(test)]
#[path = "../../tests/private/stl_assets.rs"]
mod stl_tests;
