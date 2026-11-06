# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Source-backed notice ownership and refusal controls; no Cargo/network/build."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("preview_licenses", Path(__file__).resolve().parents[1] / "scripts/preview_licenses.py")
licenses = importlib.util.module_from_spec(spec)
spec.loader.exec_module(licenses)


class PreviewLicenses(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.workspace = self.root / "source"
        self.workspace.mkdir()

    def package(self, name, expression="MIT", source="registry+https://github.com/rust-lang/crates.io-index"):
        directory = self.workspace / name
        directory.mkdir()
        manifest = directory / "Cargo.toml"
        manifest.write_text(f'[package]\nname = "{name}"\nversion = "1.0.0"\n')
        return {"id": f"{source or 'path+' + str(directory)}#{name}@1.0.0", "name": name,
                "version": "1.0.0", "source": source, "manifest_path": str(manifest),
                "license": expression, "license_file": None}

    def document(self, packages, label="native"):
        return {"workspace_root": str(self.workspace), "packages": packages, "label": label}

    def text(self, package, name, content):
        path = Path(package["manifest_path"]).parent / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        return path

    def test_copied_bytes_provenance_and_graph_union_survive_source_removal(self):
        package = self.package("demo")
        mit = self.text(package, "LICENSE-MIT", "MIT fixture text\nCopyright upstream\n")
        notice = self.text(package, "NOTICE", "Upstream attribution must remain.\n")
        font = self.root / "OFL.txt"
        font.write_text("Geist fixture font copyright and OFL notice\n")
        output = self.root / "notices"
        inventory = licenses.collect_notices(
            [self.document([package], label) for label in ["native", "helper", "webgpu", "webgl2"]],
            {"fonts/Geist-OFL.txt": font}, output)
        self.assertEqual(inventory["graphs"], ["native", "helper", "webgpu", "webgl2"])
        self.assertEqual(len(inventory["packages"]), 1)
        self.assertEqual(inventory["packages"][0]["graphs"], inventory["graphs"])
        self.assertEqual(inventory["packages"][0]["license"], "MIT")
        expected = {mit.read_bytes(), notice.read_bytes(), font.read_bytes()}
        actual = {(output / row["path"]).read_bytes() for row in inventory["files"]}
        self.assertEqual(actual, expected)
        for row in inventory["files"]:
            self.assertFalse((output / row["path"]).is_symlink())
            self.assertEqual(hashlib.sha256((output / row["path"]).read_bytes()).hexdigest(), row["sha256"])
        # Notices own bytes rather than host symlinks; installed use needs no source tree.
        mit.unlink(); notice.unlink(); font.unlink()
        self.assertEqual({(output / row["path"]).read_bytes() for row in inventory["files"]}, expected)
        json.dumps(inventory)
        with self.assertRaises(licenses.LicenseCollectionError):
            licenses.collect_notices([self.document([package])], {}, output)

    def test_or_expression_cannot_silently_drop_an_alternative(self):
        package = self.package("dual", "MIT OR Apache-2.0")
        self.text(package, "LICENSE-MIT", "MIT text supplied by package\n")
        output = self.root / "dual-notices"
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "Apache-2.0"):
            licenses.collect_notices([self.document([package])], {}, output)
        self.assertFalse(output.exists())
        self.text(package, "LICENSE-APACHE", "Apache text supplied by package\n")
        inventory = licenses.collect_notices([self.document([package])], {}, output)
        self.assertEqual(len(inventory["packages"][0]["texts"]), 2)
        self.assertEqual(inventory["packages"][0]["license"], "MIT OR Apache-2.0")

    def test_workspace_ancestor_and_explicit_shared_apache_are_bounded(self):
        local = self.package("local", "Apache-2.0", source=None)
        shared = self.workspace / "LICENSE-APACHE"
        shared.write_text("Project Apache text\n")
        local["license_file"] = "../LICENSE-APACHE"
        inventory = licenses.collect_notices([self.document([local])], {}, self.root / "ancestor")
        self.assertEqual(len(inventory["packages"][0]["texts"]), 1)
        shared.unlink(); local["license_file"] = None
        supplied = self.root / "Apache-2.0.txt"
        supplied.write_text("Explicit source-provided Apache text\n")
        extras = {"licenses/Apache-2.0.txt": supplied}
        licenses.collect_notices([self.document([local])], extras, self.root / "shared")
        registry = self.package("registry", "Apache-2.0")
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "missing actual text"):
            licenses.collect_notices([self.document([registry])], extras, self.root / "registry-output")
        self.assertFalse((self.root / "registry-output").exists())

    def test_declared_custom_license_and_complete_expression_text_are_explicit(self):
        package = self.package("custom", None)
        source = self.text(package, "legal/TERMS.txt", "Author-supplied custom distribution terms\n")
        package["license_file"] = "legal/TERMS.txt"
        inventory = licenses.collect_notices([self.document([package])], {}, self.root / "custom-notices")
        self.assertIsNone(inventory["packages"][0]["license"])
        self.assertEqual((self.root / "custom-notices" / inventory["packages"][0]["texts"][0]).read_bytes(), source.read_bytes())
        compound = self.package("compound", "(MIT OR Apache-2.0) AND Unicode-3.0")
        complete = self.root / "complete.txt"
        complete.write_text("Explicit complete source-provided compound texts\n")
        key = f'licenses/{compound["license"]}.txt'
        inventory = licenses.collect_notices([self.document([compound])], {key: complete}, self.root / "complete-notices")
        self.assertEqual(inventory["packages"][0]["license"], compound["license"])

    def test_license_file_cannot_escape_or_follow_symlinks(self):
        package = self.package("bounded")
        outside = self.root / "outside.txt"; outside.write_text("outside text\n")
        for declared in [str(outside), "../../outside.txt"]:
            with self.subTest(declared=declared):
                package["license_file"] = declared
                with self.assertRaisesRegex(licenses.LicenseCollectionError, "escapes"):
                    licenses.collect_notices([self.document([package])], {}, self.root / "escaped")
                self.assertFalse((self.root / "escaped").exists())
        link = Path(package["manifest_path"]).parent / "LICENSE"
        link.symlink_to(outside); package["license_file"] = "LICENSE"
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "symlink"):
            licenses.collect_notices([self.document([package])], {}, self.root / "linked")

    def test_extra_output_names_and_destination_parent_cannot_escape(self):
        package = self.package("paths"); self.text(package, "LICENSE", "MIT text\n")
        extra = self.root / "extra.txt"; extra.write_text("Additional notice\n")
        for name in ["../escape", "/absolute", "C:\\notice", "a//b", "a/./b", "a/../b", "packages/collision", "bad\0name", "bad\nname", "x" * 256]:
            with self.subTest(name=name):
                with self.assertRaises(licenses.LicenseCollectionError):
                    licenses.collect_notices([self.document([package])], {name: extra}, self.root / "invalid-output")
                self.assertFalse((self.root / "invalid-output").exists())
        alias = self.root / "alias"; alias.symlink_to(self.workspace, target_is_directory=True)
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "symlink"):
            licenses.collect_notices([self.document([package])], {}, alias / "notices")
        extra_link = self.root / "extra-link.txt"; extra_link.symlink_to(extra)
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "symlink"):
            licenses.collect_notices([self.document([package])], {"NOTICE": extra_link}, self.root / "extra-output")

    def test_empty_binary_oversize_and_missing_actual_text_fail_before_output(self):
        package = self.package("text")
        path = Path(package["manifest_path"]).parent / "LICENSE"
        for content in [b"", b" \n\t", b"binary\0notice", b"\xff", b"x" * (licenses.MAX_TEXT_BYTES + 1)]:
            with self.subTest(size=len(content)):
                path.write_bytes(content)
                with self.assertRaises(licenses.LicenseCollectionError):
                    licenses.collect_notices([self.document([package])], {}, self.root / "bad-text")
                self.assertFalse((self.root / "bad-text").exists())
        path.unlink()
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "missing actual text"):
            licenses.collect_notices([self.document([package])], {}, self.root / "missing")

    def test_no_ambiguous_identity_missing_declaration_or_unresolved_expression(self):
        package = self.package("identity"); self.text(package, "LICENSE", "MIT text\n")
        changed = dict(package, license="Apache-2.0")
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "inconsistent"):
            licenses.collect_notices([self.document([package, changed])], {}, self.root / "identity-output")
        for expression in [None, "", "MIT OR", "(MIT", "MIT Apache-2.0", "MIT/Apache-2.0", "MIT WITH", "()", "MIT AND OR Apache-2.0"]:
            with self.subTest(expression=expression):
                with self.assertRaises(licenses.LicenseCollectionError):
                    licenses.collect_notices([self.document([dict(package, license=expression)])], {}, self.root / "expression-output")
                self.assertFalse((self.root / "expression-output").exists())
        exception = dict(package, license="Apache-2.0 WITH LLVM-exception")
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "missing actual text"):
            licenses.collect_notices([self.document([exception])], {}, self.root / "exception-output")

    def test_unknown_license_text_namespace_and_aggregate_limits_remain_explicit(self):
        package = self.package("unknown", "LicenseRef-Upstream-Custom")
        with self.assertRaises(licenses.LicenseCollectionError):
            licenses.collect_notices([self.document([package])], {}, self.root / "unknown-output")
        self.text(package, "LICENSE-LicenseRef-Upstream-Custom.txt", "Exact identified upstream terms\n")
        licenses.collect_notices([self.document([package])], {}, self.root / "known-output")
        other = self.package("other"); self.text(other, "LICENSE", "MIT text\n")
        with patch.object(licenses, "MAX_PACKAGES", 1):
            with self.assertRaisesRegex(licenses.LicenseCollectionError, "package count"):
                licenses.collect_notices([self.document([package, other])], {}, self.root / "bounded-output")
        with patch.object(licenses, "MAX_TOTAL_BYTES", 4):
            with self.assertRaisesRegex(licenses.LicenseCollectionError, "aggregate"):
                licenses.collect_notices([self.document([other])], {}, self.root / "total-output")
        self.assertFalse((self.root / "total-output").exists())

    def test_ancestor_descendant_extra_collision_never_creates_destination(self):
        package = self.package("collision"); self.text(package, "LICENSE", "MIT text\n")
        extra = self.root / "notice.txt"; extra.write_text("Source notice\n")
        for entries in [
            {"NOTICE": extra, "NOTICE/child.txt": extra},
            {"nested/child.txt": extra, "nested": extra},
        ]:
            with self.subTest(entries=list(entries)):
                output = self.root / "collision-output"
                with self.assertRaisesRegex(licenses.LicenseCollectionError, "collision"):
                    licenses.collect_notices([self.document([package])], entries, output)
                self.assertFalse(output.exists())

    def test_generic_workspace_license_cannot_satisfy_unrelated_child_declaration(self):
        child = self.package("child", "MIT", source=None)
        (self.workspace / "LICENSE").write_text("Unrelated workspace Apache license text\n")
        output = self.root / "unrelated-output"
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "missing actual text for MIT"):
            licenses.collect_notices([self.document([child])], {}, output)
        self.assertFalse(output.exists())
        # An actual specifically named ancestor text is admitted, not a guessed generic.
        (self.workspace / "LICENSE-MIT").write_text("Explicit workspace MIT text\n")
        inventory = licenses.collect_notices([self.document([child])], {}, output)
        texts = [(output / path).read_text() for path in inventory["packages"][0]["texts"]]
        self.assertEqual(texts, ["Explicit workspace MIT text\n"])

    def test_actual_reads_candidates_outputs_and_duplicate_entries_are_bounded(self):
        package = self.package("bounded-reads"); self.text(package, "LICENSE-MIT", "MIT text\n")
        self.text(package, "LICENSE-APACHE", "Unselected Apache text still consumes read budget\n")
        extra = self.root / "extra.txt"; extra.write_text("Extra source text\n")
        cases = [
            ("MAX_READ_BYTES", 12, [package], {}),
            ("MAX_TEXT_FILES", 1, [package], {}),
            ("MAX_CANDIDATE_FILES", 1, [package], {}),
            ("MAX_DIRECTORY_ENTRIES", 1, [package], {}),
            ("MAX_OUTPUT_FILES", 1, [package], {"NOTICE": extra}),
            ("MAX_EXTRA_FILES", 1, [package], {"NOTICE": extra, "COPYRIGHT": extra}),
            ("MAX_PACKAGE_ENTRIES", 1, [package, package], {}),
        ]
        for constant, limit, packages, extras in cases:
            with self.subTest(constant=constant):
                output = self.root / "bounded-read-output"
                with patch.object(licenses, constant, limit):
                    with self.assertRaises(licenses.LicenseCollectionError):
                        licenses.collect_notices([self.document(packages)], extras, output)
                self.assertFalse(output.exists())
        # Cross-graph reuse does not cause another copy/read of the same package text.
        with patch.object(licenses, "MAX_TEXT_FILES", 2):
            result = licenses.collect_notices([self.document([package], label) for label in ["native", "helper"]], {}, self.root / "dedup")
        self.assertEqual(len(result["packages"]), 1)
        self.assertEqual(result["packages"][0]["graphs"], ["native", "helper"])


if __name__ == "__main__":
    unittest.main()
