// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded local glTF, STL and OBJ collision input, decoded only when needed.
use base64::Engine;
use glam::{DMat4, DVec3};
use std::{
    fs::File,
    io::Read,
    path::{Path, PathBuf},
};

const FILE_BYTES: usize = 16 * 1024 * 1024;
const TOTAL_BYTES: usize = 64 * 1024 * 1024;
const MAX_VERTICES: usize = 131072;
const MAX_TRIANGLES: usize = 262144;
const MAX_NODE_VISITS: usize = 1024;
const MAX_DEPTH: usize = 32;

mod obj;
mod stl;
pub use obj::{decode as decode_obj, visual_glb as obj_visual_glb};
pub use stl::{decode as decode_stl, visual_glb as stl_visual_glb};

#[derive(Debug, thiserror::Error)]
#[error("{0}")]
pub struct MeshError(pub String);

/// Owned Z-up geometry in collision-local axes, before the collision's rigid pose.
#[derive(Debug, Clone)]
pub struct TriangleMesh {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
}

fn invalid(message: impl Into<String>) -> MeshError {
    MeshError(message.into())
}
fn io_error(path: &Path, error: std::io::Error) -> MeshError {
    invalid(format!("{}: {error}", path.display()))
}
fn read(path: &Path) -> Result<Vec<u8>, MeshError> {
    if !path.metadata().map_err(|e| io_error(path, e))?.is_file() {
        return Err(invalid(format!(
            "{}: mesh resource must be a regular file",
            path.display()
        )));
    }
    let mut bytes = Vec::new();
    File::open(path)
        .and_then(|file| file.take((FILE_BYTES + 1) as u64).read_to_end(&mut bytes))
        .map_err(|e| io_error(path, e))?;
    if bytes.len() > FILE_BYTES {
        return Err(invalid("mesh resource exceeds 16 MiB"));
    }
    Ok(bytes)
}
fn relative(value: &str) -> bool {
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
fn local_path(mesh: &sdformat::Mesh) -> Result<PathBuf, MeshError> {
    if mesh.optimization != sdformat::MeshOptimization::None || mesh.convex_decomposition.is_some()
    {
        return Err(invalid(
            "mesh collision optimization/decomposition requests are unsupported",
        ));
    }
    if !mesh.submesh.is_empty() || mesh.center_submesh {
        return Err(invalid(
            "mesh collision submesh selection/recentering is unsupported",
        ));
    }
    if !mesh.scale.iter().all(|v| v.is_finite() && *v > 0.0) {
        return Err(invalid("mesh collision scale must be positive and finite"));
    }
    let base = url::Url::parse(&mesh.source.uri).map_err(|e| invalid(e.to_string()))?;
    if base.scheme() != "file" {
        return Err(invalid("mesh collision source must be a local SDF file"));
    }
    let uri = base.join(&mesh.uri).map_err(|e| invalid(e.to_string()))?;
    if uri.scheme() != "file" || uri.query().is_some() || uri.fragment().is_some() {
        return Err(invalid(
            "mesh collisions require local glTF, STL or OBJ file URIs",
        ));
    }
    let path = uri
        .to_file_path()
        .map_err(|()| invalid("invalid local mesh file URI"))?;
    if !matches!(
        path.extension().and_then(|v| v.to_str()),
        Some("gltf" | "glb" | "stl" | "obj")
    ) {
        return Err(invalid(
            "mesh collisions require glTF 2.0 (.gltf or .glb) or local .stl/.obj",
        ));
    }
    if !path
        .file_name()
        .and_then(|v| v.to_str())
        .is_some_and(relative)
    {
        return Err(invalid(
            "mesh filename is outside the admitted ASCII path subset",
        ));
    }
    Ok(path)
}

/// Decode a default (otherwise first) glTF scene or an admitted STL/OBJ surface.
/// glTF preserves every node instance and converts Y-up to Z-up; STL/OBJ are Z-up.
/// All formats apply the positive finite SDF scale once before the rigid pose.
/// No texture/image loading, network request, Bevy resource or worker is created.
pub fn load(mesh: &sdformat::Mesh) -> Result<TriangleMesh, MeshError> {
    let path = local_path(mesh)?;
    let root = path
        .parent()
        .ok_or_else(|| invalid("mesh needs a parent directory"))?
        .canonicalize()
        .map_err(|e| io_error(&path, e))?;
    let canonical = path.canonicalize().map_err(|e| io_error(&path, e))?;
    if !canonical.starts_with(&root) {
        return Err(invalid(
            "mesh document symlink escapes its resource directory",
        ));
    }
    let bytes = read(&canonical)?;
    if path.extension().and_then(|v| v.to_str()) == Some("stl") {
        return stl::scaled(&bytes, mesh.scale);
    }
    if path.extension().and_then(|v| v.to_str()) == Some("obj") {
        return obj::scaled(&bytes, mesh.scale);
    }
    let gltf =
        gltf::Gltf::from_slice(&bytes).map_err(|e| invalid(format!("{}: {e}", path.display())))?;
    let asset = &gltf.as_json().asset;
    if asset.version != "2.0" || asset.min_version.as_deref().is_some_and(|v| v != "2.0") {
        return Err(invalid(
            "collision input requires glTF version2.0 and absent or2.0 minVersion",
        ));
    }
    if gltf.extensions_required().next().is_some() || gltf.animations().next().is_some() {
        return Err(invalid(
            "required glTF extensions and animated collision input are unsupported",
        ));
    }
    if gltf.buffers().len() > 128 {
        return Err(invalid("mesh exceeds128 binary buffers"));
    }
    let mut buffers = Vec::new();
    let mut total = bytes.len();
    for buffer in gltf.buffers() {
        if buffer.length() > FILE_BYTES {
            return Err(invalid("claimed mesh buffer exceeds16 MiB"));
        }
        let data = match buffer.source() {
            gltf::buffer::Source::Bin => gltf
                .blob
                .as_ref()
                .ok_or_else(|| invalid("GLB has no declared BIN chunk"))?
                .clone(),
            gltf::buffer::Source::Uri(uri) => {
                if let Some(data) = uri
                    .strip_prefix("data:application/octet-stream;base64,")
                    .or_else(|| uri.strip_prefix("data:application/gltf-buffer;base64,"))
                {
                    base64::engine::general_purpose::STANDARD
                        .decode(data)
                        .map_err(|e| invalid(format!("invalid embedded mesh buffer: {e}")))?
                } else {
                    if !relative(uri) {
                        return Err(invalid(
                            "mesh buffer URI is outside the admitted local path subset",
                        ));
                    }
                    let file = root
                        .join(uri)
                        .canonicalize()
                        .map_err(|e| io_error(&root.join(uri), e))?;
                    if !file.starts_with(&root) {
                        return Err(invalid(
                            "mesh buffer symlink escapes its resource directory",
                        ));
                    }
                    read(&file)?
                }
            }
        };
        if data.len() > FILE_BYTES || data.len() < buffer.length() {
            return Err(invalid(
                "mesh buffer has invalid declared or actual byte length",
            ));
        }
        total = total
            .checked_add(data.len())
            .filter(|v| *v <= TOTAL_BYTES)
            .ok_or_else(|| invalid("mesh exceeds64 MiB of document/buffer data"))?;
        buffers.push(data);
    }
    let scene = gltf
        .default_scene()
        .or_else(|| gltf.scenes().next())
        .ok_or_else(|| invalid("mesh document has no scene"))?;
    let basis = DMat4::from_scale(DVec3::from(mesh.scale))
        * DMat4::from_rotation_x(std::f64::consts::FRAC_PI_2);
    let mut decoder = Decoder {
        buffers: &buffers,
        basis,
        mesh: TriangleMesh {
            vertices: Vec::new(),
            triangles: Vec::new(),
        },
        visits: 0,
    };
    for node in scene.nodes() {
        decoder.node(node, DMat4::IDENTITY, 0)?;
    }
    if decoder.mesh.triangles.is_empty() {
        return Err(invalid("collision scene has no triangles"));
    }
    Ok(decoder.mesh)
}

struct Decoder<'a> {
    buffers: &'a [Vec<u8>],
    basis: DMat4,
    mesh: TriangleMesh,
    visits: usize,
}

/// Preflight arithmetic before maintained glTF readers, whose iterators assume
/// validated count/stride ranges. Sparse collision accessors are outside M3.
fn accessor_range(
    accessor: &gltf::Accessor<'_>,
    buffers: &[Vec<u8>],
    limit: usize,
) -> Result<(), MeshError> {
    if accessor.count() == 0 || accessor.count() > limit || accessor.sparse().is_some() {
        return Err(invalid(
            "invalid, excessive or sparse collision accessor count",
        ));
    }
    let view = accessor
        .view()
        .ok_or_else(|| invalid("collision accessor needs a buffer view"))?;
    let buffer = buffers
        .get(view.buffer().index())
        .ok_or_else(|| invalid("collision accessor names an absent buffer"))?;
    let size = accessor.size();
    let stride = view.stride().unwrap_or(size);
    if stride < size || !stride.is_multiple_of(accessor.data_type().size()) {
        return Err(invalid("collision accessor has an invalid byte stride"));
    }
    let end = (accessor.count() - 1)
        .checked_mul(stride)
        .and_then(|v| v.checked_add(size))
        .and_then(|v| v.checked_add(accessor.offset()))
        .ok_or_else(|| invalid("collision accessor byte range overflows"))?;
    if end > view.length()
        || view
            .offset()
            .checked_add(view.length())
            .is_none_or(|v| v > buffer.len() || v > view.buffer().length())
    {
        return Err(invalid(
            "collision accessor exceeds its declared buffer view",
        ));
    }
    Ok(())
}

impl Decoder<'_> {
    fn node(&mut self, node: gltf::Node<'_>, parent: DMat4, depth: usize) -> Result<(), MeshError> {
        self.visits += 1;
        if depth > MAX_DEPTH || self.visits > MAX_NODE_VISITS {
            return Err(invalid("collision node depth/instance budget exceeded"));
        }
        if node.skin().is_some() || node.weights().is_some() {
            return Err(invalid("skinned or morph collision nodes are unsupported"));
        }
        let raw = node.transform().matrix().map(|col| col.map(f64::from));
        let local = DMat4::from_cols_array_2d(&raw);
        if !local.is_finite() || local.row(3) != glam::DVec4::W {
            return Err(invalid(
                "collision node transform must be finite and affine",
            ));
        }
        let matrix = parent * local;
        let baked = self.basis * matrix;
        let determinant = baked.determinant();
        if !baked.is_finite() || !determinant.is_finite() || determinant == 0.0 {
            return Err(invalid(
                "collision transform must be finite and nonsingular",
            ));
        }
        if let Some(mesh) = node.mesh() {
            for primitive in mesh.primitives() {
                self.primitive(primitive, baked, determinant < 0.0)?;
            }
        }
        for child in node.children() {
            self.node(child, matrix, depth + 1)?;
        }
        Ok(())
    }
    fn primitive(
        &mut self,
        primitive: gltf::Primitive<'_>,
        matrix: DMat4,
        reflected: bool,
    ) -> Result<(), MeshError> {
        if primitive.mode() != gltf::mesh::Mode::Triangles
            || primitive.morph_targets().next().is_some()
        {
            return Err(invalid(
                "collision primitives require TRIANGLES without morph targets",
            ));
        }
        let positions = primitive
            .get(&gltf::Semantic::Positions)
            .ok_or_else(|| invalid("collision primitive has no POSITION accessor"))?;
        if positions.data_type() != gltf::accessor::DataType::F32
            || positions.dimensions() != gltf::accessor::Dimensions::Vec3
            || positions.normalized()
        {
            return Err(invalid("collision POSITION must be unnormalized F32 VEC3"));
        }
        accessor_range(&positions, self.buffers, MAX_VERTICES)?;
        if self
            .mesh
            .vertices
            .len()
            .checked_add(positions.count())
            .is_none_or(|v| v > MAX_VERTICES)
        {
            return Err(invalid("collision decoded vertex budget exceeded"));
        }
        if let Some(indices) = primitive.indices() {
            if indices.dimensions() != gltf::accessor::Dimensions::Scalar
                || indices.normalized()
                || !matches!(
                    indices.data_type(),
                    gltf::accessor::DataType::U8
                        | gltf::accessor::DataType::U16
                        | gltf::accessor::DataType::U32
                )
            {
                return Err(invalid(
                    "collision indices require unsigned scalar accessors",
                ));
            }
            accessor_range(&indices, self.buffers, MAX_TRIANGLES * 3)?;
        }
        let reader = primitive.reader(|buffer| self.buffers.get(buffer.index()).map(Vec::as_slice));
        let vertices = reader
            .read_positions()
            .ok_or_else(|| invalid("collision POSITION decoding failed"))?;
        let count = positions.count();
        let base = u32::try_from(self.mesh.vertices.len())
            .map_err(|_| invalid("collision vertex index overflow"))?;
        for point in vertices {
            let point = matrix.transform_point3(DVec3::from(point.map(f64::from)));
            if !point.is_finite() {
                return Err(invalid(
                    "collision vertex must remain finite after all transforms",
                ));
            }
            self.mesh.vertices.push(point.to_array());
        }
        let indices: Vec<u32> = match reader.read_indices() {
            Some(indices) => indices.into_u32().collect(),
            None if primitive.indices().is_none() => (0..count as u32).collect(),
            None => return Err(invalid("collision index decoding failed")),
        };
        if indices.is_empty()
            || !indices.len().is_multiple_of(3)
            || self
                .mesh
                .triangles
                .len()
                .checked_add(indices.len() / 3)
                .is_none_or(|v| v > MAX_TRIANGLES)
        {
            return Err(invalid(
                "collision triangle/index budget or topology is invalid",
            ));
        }
        for triangle in indices.chunks_exact(3) {
            if triangle.iter().any(|v| *v as usize >= count) {
                return Err(invalid("collision triangle index is out of range"));
            }
            let mut triangle = [triangle[0] + base, triangle[1] + base, triangle[2] + base];
            if reflected {
                triangle.swap(1, 2);
            }
            let [a, b, c] = triangle.map(|v| DVec3::from(self.mesh.vertices[v as usize]));
            let normal = (b - a).cross(c - a);
            if !normal.is_finite()
                || !normal.length_squared().is_finite()
                || normal.length_squared() == 0.0
            {
                return Err(invalid(
                    "collision triangle must be finite and nondegenerate",
                ));
            }
            self.mesh.triangles.push(triangle);
        }
        Ok(())
    }
}
