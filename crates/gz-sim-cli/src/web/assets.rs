// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Immutable, declared-only local glTF resources. Constructed only for a mesh viewer.
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

type Files = BTreeMap<String, Bytes>;

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
        return Err(invalid("M2 accepts only local glTF file URIs"));
    }
    let path = uri
        .to_file_path()
        .map_err(|()| invalid("invalid local mesh file URI"))?;
    if !matches!(
        path.extension().and_then(|s| s.to_str()),
        Some("gltf" | "glb")
    ) {
        return Err(invalid("M2 mesh must be glTF 2.0 (.gltf or .glb)"));
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

fn admit(
    mesh: &sdformat::Mesh,
    scope: usize,
    files: &mut Files,
    total: &mut usize,
) -> io::Result<String> {
    let (root, document_path, name) = document_paths(mesh)?;
    let bytes = read_file(&document_path)?;
    let gltf = gltf::Gltf::from_slice(&bytes)
        .map_err(|e| invalid(format!("{}: {e}", document_path.display())))?;
    let asset = &gltf.as_json().asset;
    if asset.version != "2.0" || asset.min_version.as_deref().is_some_and(|v| v != "2.0") {
        return Err(invalid(
            "M2 requires glTF version 2.0 and absent or 2.0 minVersion",
        ));
    }
    if gltf.scenes().next().is_none() {
        return Err(invalid("M2 glTF must contain a scene"));
    }
    let prefix = format!("/assets/{scope}/");
    let entry = format!("{prefix}{name}");
    let resources = gltf
        .buffers()
        .filter_map(|buffer| match buffer.source() {
            gltf::buffer::Source::Uri(uri) => Some(uri),
            gltf::buffer::Source::Bin => None,
        })
        .chain(gltf.images().filter_map(|image| match image.source() {
            gltf::image::Source::Uri { uri, .. } => Some(uri),
            gltf::image::Source::View { .. } => None,
        }));
    for resource in resources {
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
    entries: BTreeMap<u64, (sdformat::Mesh, String)>,
    port: u16,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
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
                runtime.block_on(serve(listener, Arc::new(files), worker_stop));
            })?;
        eprintln!("gzweb-rs local glTF resources: http://127.0.0.1:{port}/assets/");
        Ok(Some(Self {
            entries,
            port,
            stop,
            worker: Some(worker),
        }))
    }

    pub(super) fn uri(&self, id: u64, mesh: &sdformat::Mesh) -> io::Result<String> {
        let (_, route) = self
            .entries
            .get(&id)
            .filter(|(original, _)| original == mesh)
            .ok_or_else(|| invalid("mesh visual is outside the admitted M2 asset snapshot"))?;
        Ok(format!("http://127.0.0.1:{}{route}", self.port))
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
