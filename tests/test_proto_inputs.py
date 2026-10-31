# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Negative controls for source-bound generated accounting, not compiler stand-ins."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import proto_inputs

spec = importlib.util.spec_from_file_location("proto_verify", Path(__file__).resolve().parents[1] / "scripts/verify.py")
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


def summary(path, covered=100, total=100):
    return {"filename": str(path), "summary": {
        metric: {"covered": covered, "count": total} for metric in ("lines", "regions")}}


class ProtoAccounting(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="proto-accounting-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.target = self.root / "target"
        self.output = self.target / "build/proto/out"
        self.output.mkdir(parents=True)
        self.package = self.root / "crates/gz-sim-proto"
        self.facade = self.root / proto_inputs.FACADE
        self.build = self.root / proto_inputs.BUILD
        self.facade.parent.mkdir(parents=True)
        self.facade.write_text("// AST is checked by the real Syn child; this fixture checks accounting only")
        self.build.write_text("// owned executable build")
        self.crates = {proto_inputs.PACKAGE: [self.facade, self.build]}

    def proof(self):
        return {"proof": proto_inputs.PROOF, "source": str(self.facade),
                "source_sha256": proto_inputs.digest(self.facade)}

    def test_only_absent_facade_with_real_build_denominator_can_be_classified(self):
        report = {"data": [{"files": [summary(self.build)]}]}
        proof = proto_inputs.zero_facade(self.root, report, "", self.crates, self.proof())
        self.assertEqual(proof, {self.facade})
        with patch.object(verify, "ROOT", self.root):
            self.assertEqual(verify.check_coverage(report, self.crates, proof)[proto_inputs.PACKAGE]["regions"], [100, 100])
            with self.assertRaisesRegex(ValueError, "uninstrumented"):
                verify.check_coverage(report, self.crates)
        for evidence in [None, {}, dict(self.proof(), source_sha256="stale"), dict(self.proof(), proof="other")]:
            with self.subTest(evidence=evidence), self.assertRaisesRegex(ValueError, "syntax proof"):
                proto_inputs.zero_facade(self.root, report, "", self.crates, evidence)
        for metric in ("lines", "regions"):
            bad = {"data": [{"files": [summary(self.build)]}]}
            bad["data"][0]["files"][0]["summary"][metric]["count"] = 0
            with self.assertRaisesRegex(ValueError, "inactive or uninstrumented"):
                proto_inputs.zero_facade(self.root, bad, "", self.crates, self.proof())
        with self.assertRaisesRegex(ValueError, "sources are missing"):
            proto_inputs.zero_facade(self.root, report, "", {proto_inputs.PACKAGE: [self.facade]}, self.proof())

    def test_real_report_da_zero_and_regions_have_precedence(self):
        report = {"data": [{"files": [summary(self.build), summary(self.facade, 0, 1)]}]}
        lcov = f"SF:{self.build}\nDA:1,1\nend_of_record\nSF:{self.facade}\nDA:5,0\nend_of_record\n"
        self.assertEqual(proto_inputs.zero_facade(self.root, report, lcov, self.crates, self.proof()), set())
        with patch.object(verify, "ROOT", self.root):
            measured = verify.check_coverage(report, self.crates)
            self.assertEqual(measured[proto_inputs.PACKAGE]["lines"], [100, 101])
            with self.assertRaisesRegex(ValueError, "changed executable"):
                verify.check_changed_coverage(lcov, {str(proto_inputs.FACADE): [5]}, self.crates,
                                              {self.build: 100, self.facade: 1})
            with self.assertRaisesRegex(ValueError, "has executable regions"):
                verify.check_coverage(report, self.crates, {self.facade})
        report["data"][0]["files"].pop()
        with self.assertRaisesRegex(ValueError, "LCOV instrumentation"):
            proto_inputs.zero_facade(self.root, report, lcov, self.crates, self.proof())

    def test_missing_build_and_exactly_95_are_never_empty_passes(self):
        for report in [{"data": [{"files": []}]}, {"data": [{"files": [summary(self.build, 0, 0)]}]}]:
            with self.assertRaisesRegex(ValueError, "build script"):
                proto_inputs.zero_facade(self.root, report, "", self.crates, self.proof())
        report = {"data": [{"files": [summary(self.build, 95)]}]}
        proof = proto_inputs.zero_facade(self.root, report, "", self.crates, self.proof())
        with patch.object(verify, "ROOT", self.root):
            with self.assertRaisesRegex(ValueError, "must exceed 95"):
                verify.check_coverage(report, self.crates, proof)
            with self.assertRaisesRegex(ValueError, "unrecognized zero-executable"):
                verify.check_coverage(report, self.crates, {self.build})

    def inputs(self):
        # These are data-integrity fixtures. Real standard compilation and Syn
        # parsing are exercised by the Rust tests, not asserted by this fixture.
        schemas = []
        for index in range(216):
            name = f"schemas/gz/msgs/type{index}.proto"
            path = self.package / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"syntax=\"proto3\"; message M{index} {{}}")
            schemas.append({"path": name, "sha256": proto_inputs.digest(path)})
        manifest = self.package / "schemas/manifest.json"
        manifest.write_text(json.dumps({"schemas": schemas}))
        self.compiler = self.root / "compiler-identity-fixture"
        self.compiler.write_bytes(b"compiler identity")
        self.any = self.root / "any.proto"
        self.any.write_bytes(b"standard import identity")
        descriptor = self.output / "schema-descriptor.pb"
        descriptor.write_bytes(b"descriptor identity")
        (self.output / "gz.msgs.rs").write_bytes(b"generated identity")
        policy = {"schema_only_candidate": "1" * 40,
                  "compiler_version": "libprotoc 3.21.12\n",
                  "external_any_sha256": proto_inputs.digest(self.any),
                  "descriptor_sha256": proto_inputs.digest(descriptor),
                  "manifest_sha256": proto_inputs.digest(manifest)}
        policy_path = self.package / "schema-source.json"
        policy_path.write_text(json.dumps(policy))
        evidence = {"schema_source_sha256": proto_inputs.digest(policy_path),
                    "manifest_sha256": policy["manifest_sha256"],
                    "schema_revision": policy["schema_only_candidate"], "schema_count": 216,
                    "generator": "prost-build 0.14.4", "compiler_version": policy["compiler_version"],
                    "compiler": str(self.compiler), "compiler_sha256": proto_inputs.digest(self.compiler),
                    "external_any_sha256": policy["external_any_sha256"], "external_any_path": str(self.any),
                    "descriptor_sha256": policy["descriptor_sha256"],
                    "generated": {"path": str(self.output / "gz.msgs.rs"),
                                  "sha256": proto_inputs.digest(self.output / "gz.msgs.rs")}}
        (self.output / "schema-validation.json").write_text(json.dumps(evidence))
        return evidence

    def test_generated_source_compiler_import_descriptor_and_inventory_drift_fail(self):
        evidence = self.inputs()
        self.assertEqual(proto_inputs.generated_inputs(self.root, self.target, self.output), evidence)
        for path in [self.compiler, self.any, self.output / "schema-descriptor.pb",
                     self.output / "gz.msgs.rs", self.package / "schema-source.json",
                     self.package / "schemas/manifest.json", self.package / "schemas/gz/msgs/type0.proto"]:
            data = path.read_bytes()
            path.write_bytes(data + b"\n")
            with self.subTest(path=path), self.assertRaises((ValueError, OSError)):
                proto_inputs.generated_inputs(self.root, self.target, self.output)
            path.write_bytes(data)
        extra = self.package / "schemas/extra.proto"
        extra.write_text("extra")
        with self.assertRaisesRegex(ValueError, "schema inputs"):
            proto_inputs.generated_inputs(self.root, self.target, self.output)
        extra.unlink()
        evidence_path = self.output / "schema-validation.json"
        for key in ["schema_count", "generator", "schema_revision", "compiler_version", "generated"]:
            wrong = dict(evidence, **{key: None})
            evidence_path.write_text(json.dumps(wrong))
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "identity differs"):
                proto_inputs.generated_inputs(self.root, self.target, self.output)
        evidence_path.write_text(json.dumps(evidence))
        self.assertEqual(proto_inputs.generated_inputs(self.root, self.target, self.output), evidence)

    def test_generated_outputs_cannot_point_outside_target_or_through_links(self):
        self.inputs()
        with self.assertRaisesRegex(ValueError, "outside the owned target"):
            proto_inputs.generated_inputs(self.root, self.root / "other-target", self.output)
        evidence = self.output / "schema-validation.json"
        saved = evidence.read_bytes()
        evidence.unlink()
        foreign = self.root / "foreign-evidence.json"
        foreign.write_bytes(saved)
        evidence.symlink_to(foreign)
        with self.assertRaisesRegex(ValueError, "unsafe proto build evidence"):
            proto_inputs.generated_inputs(self.root, self.target, self.output)

    def test_generated_accounting_is_exact_and_cannot_hide_a_second_output(self):
        evidence = self.inputs()
        binding = self.output / "gz.msgs.rs"
        report = {"data": [{"files": [summary(binding)]}]}
        proto_inputs.generated_report(report, evidence)
        for bad in [{"data": [{"files": []}]},
                    {"data": [{"files": [summary(binding, 0, 0)]}]},
                    {"data": [{"files": [summary(binding), summary(self.output / "extra.rs")]}]}]:
            with self.assertRaisesRegex(ValueError, "absent or unexpected"):
                proto_inputs.generated_report(bad, evidence)

    def test_external_symlinked_schema_root_is_not_owned_build_input(self):
        self.inputs()
        with tempfile.TemporaryDirectory(prefix="external-proto-schemas-") as directory:
            schemas = self.package / "schemas"
            external = Path(directory) / "schemas"
            schemas.rename(external)
            schemas.symlink_to(external, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, "schema root is not a real directory"):
                proto_inputs.generated_inputs(self.root, self.target, self.output)

    def test_real_llvm_runtime_da_zero_cannot_be_forged_into_a_facade_proof(self):
        self.build.write_text("pub fn run() { std::hint::black_box(1); }\n")
        self.facade.write_text('''#[path = "../build.rs"] mod build;
fn main() {
    build::run();
    if std::hint::black_box(true) { std::hint::black_box(2); }
    else { std::hint::black_box(99); }
}
''')
        binary = self.root / "llvm-probe"
        subprocess.run(["rustc", "--edition=2024", "-C", "instrument-coverage",
                        "--crate-name", "proto_policy_llvm", str(self.facade), "-o", str(binary)],
                       check=True, capture_output=True)
        raw = self.root / "probe.profraw"
        subprocess.run([str(binary)], env=dict(os.environ, LLVM_PROFILE_FILE=str(raw)),
                       check=True, capture_output=True)
        sysroot = Path(subprocess.check_output(["rustc", "--print", "sysroot"], text=True).strip())
        version = subprocess.check_output(["rustc", "-vV"], text=True)
        host = next(line.removeprefix("host: ") for line in version.splitlines() if line.startswith("host: "))
        llvm = sysroot / "lib/rustlib" / host / "bin"
        merged = self.root / "merged.profdata"
        subprocess.run([str(llvm / "llvm-profdata"), "merge", "-sparse", str(raw), "-o", str(merged)],
                       check=True, capture_output=True)
        command = [str(llvm / "llvm-cov"), "export", "--fatal-warnings", "-instr-profile=" + str(merged),
                   "-object", str(binary)]
        report = json.loads(subprocess.check_output(command))
        lcov = subprocess.check_output([*command, "-format=lcov"], text=True)
        counts, source = {}, None
        for line in lcov.splitlines():
            if line.startswith("SF:"):
                source = Path(line[3:]).resolve()
            elif line.startswith("DA:") and source == self.facade:
                number, count, *_ = line[3:].split(",")
                counts[int(number)] = int(count)
        self.assertEqual(counts[5], 0)  # A genuinely instrumented unexecuted body.
        self.assertGreater(counts[4], 0)
        forged = self.proof()
        self.assertEqual(proto_inputs.zero_facade(self.root, report, lcov, self.crates, forged), set())
        with patch.object(verify, "ROOT", self.root):
            with self.assertRaisesRegex(ValueError, "changed executable"):
                verify.check_changed_coverage(lcov, {str(proto_inputs.FACADE): [5]}, self.crates,
                                              {self.facade: len(counts), self.build: 1})
            with self.assertRaisesRegex(ValueError, "has executable regions"):
                verify.check_coverage(report, self.crates, {self.facade})


if __name__ == "__main__":
    unittest.main()
