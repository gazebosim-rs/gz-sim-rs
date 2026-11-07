# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Copy source-backed distribution notices for Root-selected M12 package graphs.

No Cargo, subprocess, network, SPDX policy decision or dependency discovery occurs
here. The caller must first bind native/helper compiler artifacts and both accepted
viewer feature graphs to the supplied Cargo metadata package sets. Build-only
packages may be included conservatively; this inventory does not call them linked.
Rust standard-library notices, OSRF/project texts, embedded Geist OFL, vendor/proto
texts and other non-Cargo shipped material belong in explicit ``extra_files``.

Every SPDX branch is retained, never silently choose an OR alternative. Deprecated
flat Cargo ID/ID declarations are admitted as alternatives without rewriting their
raw declared strings. Mixed slash/SPDX expressions remain refused. Common
LICENSE-APACHE/LICENSE-MIT and exact identifier filenames resolve separate texts;
a package-root generic LICENSE/COPYING resolves only a single identifier, not
WITH exceptions. Unrelated generic ancestor texts cannot satisfy a child license.
An author-declared Cargo license_file or explicitly supplied
``licenses/<complete SPDX expression>.txt`` can supply the complete text without
this helper interpreting its legal meaning. Shared ``licenses/Apache-2.0.txt``
fallback is limited to exact Apache-2.0 local packages. Other missing texts fail.
For a missing unit, explicit extra_files may bind source text to its exact package
and unit using licenses/packages/<SHA256(CargoID)>/<SHA256(unit)>.txt. Hash UTF-8
strings, including the complete existing parser unit for WITH exceptions; this
never binds a bare license to its exception or a different package identity.
Named package texts keep priority and used bindings are recorded without host paths.
The caller may explicitly approve a missing-author-body case using an immutable
license-publisher/SPDX full definition plus separately shipped unmodified author
metadata/notices and per-ID/unit provenance. That definition is not an author
grant or attribution; this helper cannot decide that case or a legal verdict.
Unknown/custom and incomplete WITH texts remain unresolved, never synthesized.
Workspace ancestor discovery and declared license_file stay inside workspace_root;
registry packages stay inside their own source root. No guessed/downloaded text.
An exact zero-byte registry-package-root generic file is preserved separately only
for a multi-unit declaration without WITH or license_file. It cannot satisfy a
unit or enter the strict text cache; every unit still needs nonempty source text.

The destination must be new with an existing real parent. Inputs are fully read
and validated before destination creation; OS write failure may leave a partial
owned destination, which the package assembler must never publish as complete.
"""
from __future__ import annotations

import hashlib
from pathlib import Path, PurePosixPath
import re
import stat

_GENERIC_NAMES = {"license", "license.txt", "license.md", "licence", "copying", "copying.txt"}

MAX_PACKAGES = 2048
MAX_PACKAGE_ENTRIES = 2048
MAX_EXTRA_FILES = 256
MAX_TEXT_FILES = 8192
MAX_CANDIDATE_FILES = 8192
MAX_OUTPUT_FILES = 8192
MAX_DIRECTORY_ENTRIES = 131072
MAX_READ_BYTES = 64 * 1024 * 1024
MAX_TEXT_BYTES = 1024 * 1024
MAX_TOTAL_BYTES = 64 * 1024 * 1024
_NAMES = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.+\-]{0,127}$")
_TEXT_NAME = re.compile(r"^(?:licen[cs]e|copying|copyright|notice)(?:[._\- ].*)?$", re.I)
_TOKEN = re.compile(r"\(|\)|[A-Za-z0-9][A-Za-z0-9.\-+:]*")
_ALIASES = {
    "Apache-2.0": {"apache", "apache2", "apache20"},
    "MIT": {"mit"}, "MIT-0": {"mit0"},
    "BSD-2-Clause": {"bsd2", "bsd2clause"},
    "BSD-3-Clause": {"bsd3", "bsd3clause"},
    "ISC": {"isc"}, "Zlib": {"zlib"}, "0BSD": {"0bsd"},
    "Unlicense": {"unlicense"}, "Unicode-3.0": {"unicode", "unicode30"},
    "CC0-1.0": {"cc0", "cc010"}, "BSL-1.0": {"bsl", "bsl10"},
    "CDLA-Permissive-2.0": {"cdlapermissive20"},
}


class LicenseCollectionError(ValueError):
    """Missing, inconsistent or unbounded distribution notice input."""


def _fail(message: str) -> None:
    raise LicenseCollectionError(message)


def _path(value: object, description: str) -> Path:
    if not isinstance(value, (str, Path)):
        _fail(f"{description} must be a path")
    return Path(value)


def _relative(name: str, *, package: bool = False) -> str:
    if not isinstance(name, str) or not name or "\\" in name or ":" in name:
        _fail("notice output name must be a relative POSIX path")
    if any(ord(char) < 32 or ord(char) == 127 for char in name):
        _fail("control character in notice output name")
    if len(name.encode("utf-8")) > 4095 or any(len(part.encode("utf-8")) > 255 for part in name.split("/")):
        _fail("notice output name exceeds filesystem path bounds")
    path = PurePosixPath(name)
    if path.is_absolute() or any(p in {"", ".", ".."} for p in name.split("/")):
        _fail("notice output path escapes or is not canonical")
    if path.parts[0] == "packages" and not package:
        _fail("extra notice may not occupy the package notice namespace")
    return str(path)


def _file(path: Path, bound: Path | None = None) -> Path:
    if not path.is_absolute():
        _fail("notice source must be absolute")
    try:
        if any(p.is_symlink() for p in [path, *path.parents]):
            _fail(f"symlink in notice source: {path}")
        resolved = path.resolve(strict=True)
        if bound is not None and not resolved.is_relative_to(bound.resolve(strict=True)):
            _fail(f"license_file escapes its admitted source root: {path}")
        info = resolved.stat()
        if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_TEXT_BYTES:
            _fail(f"notice is not a bounded regular file: {path}")
    except OSError as error:
        _fail(f"cannot read notice source {path}: {error}")
    return resolved


def _text(path: Path, bound: Path | None = None, read_limit: int | None = None) -> tuple[Path, bytes]:
    source = _file(path, bound)
    # Bound the actual read too, even if the source changed after stat.
    maximum = min(MAX_TEXT_BYTES, read_limit) if read_limit is not None else MAX_TEXT_BYTES
    with source.open("rb") as stream:
        data = stream.read(maximum + 1)
    if len(data) > maximum:
        _fail(f"notice read exceeds per-file or aggregate bound: {source}")
    if not data or b"\0" in data:
        _fail(f"empty, binary or oversized notice: {source}")
    try:
        if not data.decode("utf-8-sig").strip():
            _fail(f"empty notice: {source}")
    except UnicodeError:
        _fail(f"notice text is not UTF-8: {source}")
    return source, data


def _units(expression: str) -> list[str]:
    """Validate a bounded token sequence, collect all IDs; do not choose OR branches."""
    if len(expression) > 512:
        _fail("license expression is too long")
    if "/" in expression:
        # Cargo's deprecated spelling is a flat list, not an additional SPDX operator.
        parts = [part.strip() for part in expression.split("/")]
        if (len(parts) < 2 or any(not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.\-+:]*", part)
                                  or part in {"AND", "OR", "WITH"} for part in parts)):
            _fail(f"malformed legacy license expression: {expression}")
        return list(dict.fromkeys(parts))
    tokens = _TOKEN.findall(expression)
    if "".join(tokens) != re.sub(r"\s+", "", expression):
        _fail(f"unresolved license expression: {expression}")
    depth, expect_id, units = 0, True, []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if expect_id:
            if token == "(":
                depth += 1
            elif token in {"AND", "OR", "WITH", ")"}:
                _fail(f"malformed license expression: {expression}")
            else:
                unit = token
                if index + 1 < len(tokens) and tokens[index + 1] == "WITH":
                    index += 2
                    if index >= len(tokens) or tokens[index] in {"(", ")", "AND", "OR", "WITH"}:
                        _fail(f"missing license exception: {expression}")
                    unit += " WITH " + tokens[index]
                units.append(unit)
                expect_id = False
        elif token == ")" and depth:
            depth -= 1
        elif token in {"AND", "OR"}:
            expect_id = True
        else:
            _fail(f"malformed license expression: {expression}")
        index += 1
    if not units or depth or expect_id:
        _fail(f"incomplete license expression: {expression}")
    return list(dict.fromkeys(units))


def _normal(name: str) -> str:
    return re.sub(r"[^a-z0-9]", "", name.lower())


def _named_for(path: Path, unit: str) -> bool:
    stem = path.name
    if stem.lower().endswith((".txt", ".md")):
        stem = stem.rsplit(".", 1)[0]
    stem = re.sub(r"^(?:licen[cs]e|copying)[._\- ]*", "", stem, flags=re.I)
    return _normal(stem) in (_ALIASES.get(unit, set()) | {_normal(unit)})


def collect_notices(metadata_documents: list[dict], extra_files: dict[str, Path], destination: Path) -> dict:
    """Write new source-backed notices; return JSON-safe package/text inventory.

    ``metadata_documents`` retain Cargo ``workspace_root`` and selected ``packages``;
    an optional ``label`` identifies a caller-validated native/helper/WG/GL graph.
    IDs/source/declared SPDX strings are preserved as metadata, not legal verdicts.
    ``extra_files`` keys are distribution-relative paths; source paths are explicit.
    """
    if not isinstance(metadata_documents, list) or not 1 <= len(metadata_documents) <= 16:
        _fail("supply bounded selected Cargo metadata documents")
    if not isinstance(extra_files, dict) or len(extra_files) > MAX_EXTRA_FILES:
        _fail("explicit extra notices must be a mapping")
    destination = _path(destination, "destination")
    if not destination.is_absolute() or destination.exists() or destination.is_symlink():
        _fail("notice destination must be a new absolute directory")
    if not destination.parent.is_dir() or any(p.is_symlink() for p in destination.parents):
        _fail("notice destination parent is missing or crosses a symlink")
    outputs: dict[str, bytes] = {}
    origins: dict[str, str] = {}
    directories: set[str] = set()
    text_cache: dict[Path, bytes] = {}
    placeholder_cache: dict[Path, Path] = {}
    output_bytes = read_bytes = candidate_count = directory_entries = 0
    extra_inventory = []

    def add(relative: str, source: Path, data: bytes) -> None:
        nonlocal output_bytes
        _relative(relative, package=True)
        prefixes = {str(p) for p in PurePosixPath(relative).parents if str(p) != "."}
        if relative in directories or prefixes.intersection(outputs):
            _fail(f"ancestor/descendant notice output collision: {relative}")
        if relative in outputs:
            if outputs[relative] != data or origins[relative] != str(source):
                _fail(f"conflicting notice output: {relative}")
            return
        if len(outputs) >= MAX_OUTPUT_FILES:
            _fail("notice output count exceeds bound")
        if output_bytes + len(data) > MAX_TOTAL_BYTES:
            _fail("aggregate notice bytes exceed the package bound")
        output_bytes += len(data)
        directories.update(prefixes)
        outputs[relative], origins[relative] = data, str(source)

    def load_text(path: Path, bound: Path | None = None) -> tuple[Path, bytes]:
        nonlocal read_bytes
        source = _file(path, bound)
        if source in text_cache:
            return source, text_cache[source]
        if source not in placeholder_cache and len(text_cache.keys() | placeholder_cache.keys()) >= MAX_TEXT_FILES:
            _fail("source text file count exceeds bound")
        source, data = _text(source, bound, MAX_READ_BYTES - read_bytes)
        read_bytes += len(data)
        text_cache[source] = data
        return source, data

    def children(parent: Path) -> list[Path]:
        nonlocal directory_entries
        result = []
        for path in parent.iterdir():
            directory_entries += 1
            if directory_entries > MAX_DIRECTORY_ENTRIES:
                _fail("notice directory entry count exceeds bound")
            result.append(path)
        return sorted(result)

    def empty_placeholder(path: Path, bound: Path) -> Path:
        source = _file(path, bound)
        if source in text_cache:
            _fail("nonempty notice source became an empty placeholder")
        if source not in placeholder_cache and len(text_cache.keys() | placeholder_cache.keys()) >= MAX_TEXT_FILES:
            _fail("source text file count exceeds bound")
        # A zero stat alone is not evidence: check the actual bytes, including reuse.
        with source.open("rb") as stream:
            if stream.read(1) != b"":
                _fail("empty placeholder source changed or is not actually empty")
        placeholder_cache[source] = bound
        return source

    def candidate(path: Path, bound: Path, *, allow_placeholder: bool = False) -> tuple[Path, bytes] | None:
        nonlocal candidate_count
        candidate_count += 1
        if candidate_count > MAX_CANDIDATE_FILES:
            _fail("license candidate count exceeds bound")
        if allow_placeholder:
            source = _file(path, bound)
            if source.stat().st_size == 0:
                empty_placeholder(source, bound)
                return None
        return load_text(path, bound)

    for name, path in sorted(extra_files.items()):
        name = _relative(name)
        source, data = load_text(_path(path, "extra notice"))
        add(name, source, data)
        extra_inventory.append({"path": name, "sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data)})

    records = {}
    graphs = []
    for number, document in enumerate(metadata_documents):
        if not isinstance(document, dict) or not isinstance(document.get("packages"), list):
            _fail("Cargo metadata packages are missing")
        if len(document["packages"]) > MAX_PACKAGE_ENTRIES:
            _fail("per-document package entry count exceeds bound")
        label = document.get("label", f"graph-{number}")
        if not isinstance(label, str) or not label or len(label) > 128 or label in graphs:
            _fail("graph labels must be unique bounded strings")
        graphs.append(label)
        workspace = _path(document.get("workspace_root"), "workspace_root")
        if not workspace.is_absolute() or not workspace.is_dir() or any(p.is_symlink() for p in [workspace, *workspace.parents]):
            _fail("Cargo metadata workspace_root must be a real absolute directory")
        for package in document["packages"]:
            if not isinstance(package, dict):
                _fail("invalid Cargo package record")
            name, version, identity = (package.get(k) for k in ("name", "version", "id"))
            if not all(isinstance(v, str) and _NAMES.fullmatch(v) for v in (name, version)):
                _fail("invalid package name/version")
            if not isinstance(identity, str) or not identity or len(identity) > 4096:
                _fail("missing bounded Cargo package ID")
            manifest = _file(_path(package.get("manifest_path"), "manifest_path"))
            if manifest.name != "Cargo.toml":
                _fail("package manifest_path must name Cargo.toml")
            package_root = manifest.parent
            source_id = package.get("source")
            if source_id is not None and not isinstance(source_id, str):
                _fail("invalid Cargo source identity")
            expression, declared = package.get("license"), package.get("license_file")
            if expression is not None and (not isinstance(expression, str) or not expression.strip()):
                _fail("empty/invalid declared license expression")
            if declared is not None and (not isinstance(declared, str) or not declared):
                _fail("invalid Cargo license_file")
            if expression is None and declared is None:
                _fail(f"package has no declared license text: {identity}")
            signature = (name, version, str(manifest), source_id, expression, declared)
            if identity in records:
                if records[identity]["signature"] != signature:
                    _fail("one Cargo package ID has inconsistent source/license metadata")
                if label not in records[identity]["entry"]["graphs"]:
                    records[identity]["entry"]["graphs"].append(label)
                continue
            if len(records) >= MAX_PACKAGES:
                _fail("selected package count exceeds bound")
            folder = f"packages/{name}-{version}-{hashlib.sha256(identity.encode()).hexdigest()[:16]}"
            search = [package_root]
            if source_id is None and package_root.is_relative_to(workspace):
                while search[-1] != workspace:
                    search.append(search[-1].parent)
            bound = workspace if len(search) > 1 else package_root
            units = _units(expression) if expression is not None and declared is None else []
            allow_placeholder = (isinstance(source_id, str) and source_id.startswith("registry+")
                                 and declared is None and len(units) >= 2
                                 and all(" WITH " not in unit for unit in units))
            candidates = []
            package_placeholders = []
            for parent in search:
                for path in children(parent):
                    if _TEXT_NAME.fullmatch(path.name) and not path.is_dir():
                        item = candidate(path, bound, allow_placeholder=(allow_placeholder
                                         and path.parent == package_root and path.name.lower() in _GENERIC_NAMES))
                        if item is None:
                            package_placeholders.append(path)
                        else:
                            candidates.append(item)
                    elif path.name.lower() == "licenses" and path.is_dir():
                        if path.is_symlink():
                            _fail("symlink license directory")
                        for nested in children(path):
                            if not nested.is_dir():
                                candidates.append(candidate(nested, bound))
            selected = []
            used_unit_bindings = []
            if declared is not None:
                path = Path(declared)
                selected.append(load_text(path if path.is_absolute() else package_root / path, bound))
            elif expression is not None:
                supplied = f"licenses/{expression}.txt"
                found, missing = [], []
                for unit in units:
                    matched = [(path, data) for path, data in candidates if _named_for(path, unit)]
                    if not matched and len(units) == 1 and " WITH " not in unit:
                        matched = [(path, data) for path, data in candidates if path.parent == package_root and path.name.lower() in _GENERIC_NAMES]
                    if not matched:
                        identity_hash = hashlib.sha256(identity.encode("utf-8")).hexdigest()
                        unit_hash = hashlib.sha256(unit.encode("utf-8")).hexdigest()
                        binding = f"licenses/packages/{identity_hash}/{unit_hash}.txt"
                        if binding in outputs:
                            matched = [(Path(origins[binding]), outputs[binding])]
                            used_unit_bindings.append({"unit": unit, "path": binding,
                                                       "sha256": hashlib.sha256(outputs[binding]).hexdigest()})
                    if matched:
                        found.extend(matched)
                    else:
                        missing.append(unit)
                selected.extend(found)
                if missing:
                    if supplied in outputs and not (expression == "Apache-2.0" and source_id is not None):
                        selected.append((Path(origins[supplied]), outputs[supplied]))
                    else:
                        _fail(f"missing actual text for {', '.join(missing)} in {identity}")
            # Include discovered attribution/copyright notices as well as license texts.
            selected.extend((path, data) for path, data in candidates if path.name.lower().startswith(("notice", "copyright")))
            paths, unique = [], {}
            for path, data in selected:
                if str(path) in unique and unique[str(path)][1] != data:
                    _fail("notice source changed during collection")
                unique[str(path)] = (path, data)
            for index, (path, data) in enumerate(unique.values()):
                # Source basenames are descriptive only, never used as an unchecked path.
                relative = f"{folder}/{index:02d}-{hashlib.sha256(str(path).encode()).hexdigest()[:12]}.txt"
                add(relative, path, data)
                paths.append(relative)
            if not paths:
                _fail(f"unresolved package notice: {identity}")
            raw_placeholders = []
            for source in package_placeholders:
                relative = f"{folder}/source-placeholders/{source.name}.raw"
                # Raw source evidence is not license text and is never a unit binding.
                add(relative, source, b"")
                raw_placeholders.append({"path": relative, "source_basename": source.name,
                                         "bytes": 0, "sha256": hashlib.sha256(b"").hexdigest(),
                                         "kind": "empty_source_placeholder", "used_as_license_text": False})
            entry = {"id": identity, "name": name, "version": version, "source": source_id,
                     "license": expression, "license_file": declared, "texts": paths, "graphs": [label],
                     "used_unit_bindings": used_unit_bindings, "source_placeholders": raw_placeholders}
            records[identity] = {"signature": signature, "entry": entry}

    if not records:
        _fail("selected graphs contain no packages")
    # Recheck the original bytes before publishing even the new owned destination.
    for source, bound in list(placeholder_cache.items()):
        empty_placeholder(source, bound)
    destination.mkdir()
    inventory = []
    for relative, data in sorted(outputs.items()):
        path = destination / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(data)
        inventory.append({"path": relative, "sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data), "source_path": origins[relative]})
    return {"version": 1, "graphs": graphs, "packages": [records[k]["entry"] for k in sorted(records)],
            "extra_files": extra_inventory, "files": inventory,
            "limits": "Caller-selected compiled/feature graphs, not every lockfile package or a legal policy verdict; all SPDX alternatives retained. Non-Cargo/Rust-stdlib/embedded notices require explicit supplied source texts."}
