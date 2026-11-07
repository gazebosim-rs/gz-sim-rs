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


    def test_legacy_slash_alternatives_keep_raw_strings_and_every_branch(self):
        for index, expression in enumerate(["MIT/Apache-2.0", "Apache-2.0/MIT", "  Apache-2.0 / MIT  "]):
            with self.subTest(expression=expression):
                package = self.package(f"legacy-{index}", expression)
                mit = self.text(package, "LICENSE-MIT", "Author MIT terms\n")
                output = self.root / f"legacy-output-{index}"
                with self.assertRaisesRegex(licenses.LicenseCollectionError, "Apache-2.0"):
                    licenses.collect_notices([self.document([package])], {}, output)
                self.assertFalse(output.exists())
                apache = self.text(package, "LICENSE-APACHE", "Author Apache terms\n")
                inventory = licenses.collect_notices([self.document([package])], {}, output)
                entry = inventory["packages"][0]
                self.assertEqual(entry["license"], expression)
                self.assertEqual(entry["used_unit_bindings"], [])
                self.assertEqual({(output / name).read_bytes() for name in entry["texts"]},
                                 {mit.read_bytes(), apache.read_bytes()})

    def test_malformed_or_mixed_slash_syntax_refuses_even_with_all_named_texts(self):
        package = self.package("malformed-legacy")
        self.text(package, "LICENSE-MIT", "MIT\n")
        self.text(package, "LICENSE-APACHE", "Apache\n")
        self.text(package, "LICENSE-ISC", "ISC\n")
        for expression in ["/MIT", "MIT/", "MIT//Apache-2.0", "MIT / OR", "MIT / AND", "MIT / WITH",
                           "MIT OR Apache-2.0/MIT", "(MIT/Apache-2.0)", "MIT/Apache-2.0 AND ISC",
                           "MIT WITH LLVM-exception/Apache-2.0", "MIT / Apache 2.0"]:
            with self.subTest(expression=expression):
                output = self.root / "malformed-legacy-output"
                with self.assertRaisesRegex(licenses.LicenseCollectionError, "malformed legacy"):
                    licenses.collect_notices([self.document([dict(package, license=expression)])], {}, output)
                self.assertFalse(output.exists())

    def test_unit_bindings_require_exact_package_and_unit_identity(self):
        package = self.package("bound-identity", "MIT OR Apache-2.0")
        other = self.package("other-identity", "MIT OR Apache-2.0")
        mit = self.root / "author-mit.txt"; mit.write_text("Exact author MIT terms\n")
        apache = self.root / "author-apache.txt"; apache.write_text("Exact author Apache terms\n")
        key = lambda owner, unit: ("licenses/packages/" + hashlib.sha256(owner["id"].encode()).hexdigest()
                                   + "/" + hashlib.sha256(unit.encode()).hexdigest() + ".txt")
        for wrong_key in [key(other, "MIT"), key(package, "mit"), key(package, "MIT-0")]:
            with self.subTest(wrong_key=wrong_key):
                output = self.root / "wrong-binding"
                with self.assertRaisesRegex(licenses.LicenseCollectionError, "missing actual text for MIT"):
                    licenses.collect_notices([self.document([package])],
                                            {wrong_key: mit, key(package, "Apache-2.0"): apache}, output)
                self.assertFalse(output.exists())
        extras = {key(package, "MIT"): mit, key(package, "Apache-2.0"): apache}
        output = self.root / "exact-bindings"
        inventory = licenses.collect_notices([self.document([package])], extras, output)
        entry = inventory["packages"][0]
        self.assertEqual(entry["license"], "MIT OR Apache-2.0")
        self.assertEqual({row["unit"] for row in entry["used_unit_bindings"]}, {"MIT", "Apache-2.0"})
        for row in entry["used_unit_bindings"]:
            self.assertEqual(row["path"], key(package, row["unit"]))
            self.assertEqual(hashlib.sha256((output / row["path"]).read_bytes()).hexdigest(), row["sha256"])
            self.assertNotIn("source_path", row)
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "missing actual text"):
            licenses.collect_notices([self.document([other])], extras, self.root / "no-cross-package")
        self.assertFalse((self.root / "no-cross-package").exists())

    def test_exact_with_exception_binding_cannot_be_satisfied_by_bare_license(self):
        expression = "Apache-2.0 WITH LLVM-exception OR MIT"
        package = self.package("exception-binding", expression)
        mit = self.text(package, "LICENSE-MIT", "MIT terms\n")
        source = self.root / "llvm-source.txt"; source.write_text("Full Apache license WITH author LLVM exception\n")
        key = lambda unit: ("licenses/packages/" + hashlib.sha256(package["id"].encode()).hexdigest()
                            + "/" + hashlib.sha256(unit.encode()).hexdigest() + ".txt")
        output = self.root / "llvm-output"
        for wrong in ["Apache-2.0", "LLVM-exception", "Apache-2.0 WITH Other-exception"]:
            with self.subTest(wrong=wrong), self.assertRaisesRegex(licenses.LicenseCollectionError, "LLVM-exception"):
                licenses.collect_notices([self.document([package])], {key(wrong): source}, output)
            self.assertFalse(output.exists())
        inventory = licenses.collect_notices([self.document([package])],
                                            {key("Apache-2.0 WITH LLVM-exception"): source}, output)
        entry = inventory["packages"][0]
        self.assertEqual(entry["license"], expression)
        self.assertEqual(entry["used_unit_bindings"][0]["unit"], "Apache-2.0 WITH LLVM-exception")
        self.assertEqual({(output / name).read_bytes() for name in entry["texts"]},
                         {mit.read_bytes(), source.read_bytes()})

    def test_combined_generic_source_requires_explicit_bindings_and_keeps_attribution(self):
        package = self.package("combined-source", "MIT/Apache-2.0")
        combined = self.text(package, "LICENSE", "Author complete MIT and Apache source texts\n")
        notice = self.text(package, "NOTICE", "Author attribution\n")
        key = lambda unit: ("licenses/packages/" + hashlib.sha256(package["id"].encode()).hexdigest()
                            + "/" + hashlib.sha256(unit.encode()).hexdigest() + ".txt")
        output = self.root / "combined-output"
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "missing actual text"):
            licenses.collect_notices([self.document([package])], {}, output)
        self.assertFalse(output.exists())
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "Apache-2.0"):
            licenses.collect_notices([self.document([package])], {key("MIT"): combined}, output)
        self.assertFalse(output.exists())
        inventory = licenses.collect_notices([self.document([package])],
                                            {key(unit): combined for unit in ["MIT", "Apache-2.0"]}, output)
        entry = inventory["packages"][0]
        self.assertEqual(len(entry["used_unit_bindings"]), 2)
        self.assertEqual({(output / name).read_bytes() for name in entry["texts"]},
                         {combined.read_bytes(), notice.read_bytes()})

    def test_compound_binding_is_exact_and_never_replaces_named_source(self):
        expression = "(MIT OR Apache-2.0) AND BSD-3-Clause"
        package = self.package("compound-binding", expression)
        mit = self.text(package, "LICENSE-MIT", "Actual named MIT source\n")
        apache = self.text(package, "LICENSE-APACHE", "Actual named Apache source\n")
        bsd = self.root / "bsd-author.txt"; bsd.write_text("Author full BSD3 terms\n")
        unused = self.root / "unused-mit.txt"; unused.write_text("Explicit input never overrides named source\n")
        key = lambda unit: ("licenses/packages/" + hashlib.sha256(package["id"].encode()).hexdigest()
                            + "/" + hashlib.sha256(unit.encode()).hexdigest() + ".txt")
        output = self.root / "compound-binding-output"
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "BSD-3-Clause"):
            licenses.collect_notices([self.document([package])], {key("BSD-2-Clause"): bsd}, output)
        self.assertFalse(output.exists())
        inventory = licenses.collect_notices([self.document([package])],
                                            {key("MIT"): unused, key("BSD-3-Clause"): bsd}, output)
        entry = inventory["packages"][0]
        self.assertEqual(entry["license"], expression)
        self.assertEqual([row["unit"] for row in entry["used_unit_bindings"]], ["BSD-3-Clause"])
        self.assertEqual({(output / name).read_bytes() for name in entry["texts"]},
                         {mit.read_bytes(), apache.read_bytes(), bsd.read_bytes()})

    def test_complete_expression_fallback_keeps_named_source_and_scoped_text_bounds(self):
        package = self.package("fallback-with-named", "MIT OR Apache-2.0")
        mit = self.text(package, "LICENSE-MIT", "Named MIT source must remain\n")
        complete = self.root / "complete-author.txt"; complete.write_text("Complete author expression texts\n")
        output = self.root / "fallback-with-named-output"
        inventory = licenses.collect_notices([self.document([package])],
                                            {"licenses/MIT OR Apache-2.0.txt": complete}, output)
        self.assertEqual({(output / name).read_bytes() for name in inventory["packages"][0]["texts"]},
                         {mit.read_bytes(), complete.read_bytes()})
        scoped = self.package("bounded-scoped")
        key = ("licenses/packages/" + hashlib.sha256(scoped["id"].encode()).hexdigest()
               + "/" + hashlib.sha256(b"MIT").hexdigest() + ".txt")
        link = self.root / "scoped-link.txt"; link.symlink_to(complete)
        for path in [link, complete]:
            with self.subTest(path=path):
                with patch.object(licenses, "MAX_READ_BYTES", 4):
                    with self.assertRaises(licenses.LicenseCollectionError):
                        licenses.collect_notices([self.document([scoped])], {key: path}, self.root / "bounded-scoped-output")
                self.assertFalse((self.root / "bounded-scoped-output").exists())


    def test_empty_registry_generic_is_raw_evidence_never_license_text(self):
        package = self.package("empty-raw", "MIT OR Apache-2.0")
        placeholder = self.text(package, "LICENSE.md", "")
        mit = self.text(package, "LICENSE-MIT", "Author MIT terms\n")
        apache = self.text(package, "LICENSE-APACHE", "Author Apache terms\n")
        notice = self.text(package, "NOTICE", "Author attribution\n")
        output = self.root / "empty-raw-output"
        inventory = licenses.collect_notices([self.document([package])], {}, output)
        entry = inventory["packages"][0]
        self.assertEqual(placeholder.read_bytes(), b"")
        self.assertEqual(entry["license"], "MIT OR Apache-2.0")
        self.assertEqual(entry["used_unit_bindings"], [])
        raw = entry["source_placeholders"]
        self.assertEqual(len(raw), 1)
        self.assertEqual(raw[0]["source_basename"], "LICENSE.md")
        self.assertEqual(raw[0]["bytes"], 0)
        self.assertEqual(raw[0]["sha256"], hashlib.sha256(b"").hexdigest())
        self.assertIs(raw[0]["used_as_license_text"], False)
        self.assertEqual(raw[0]["kind"], "empty_source_placeholder")
        self.assertNotIn(raw[0]["path"], entry["texts"])
        self.assertEqual((output / raw[0]["path"]).read_bytes(), b"")
        self.assertEqual({(output / path).read_bytes() for path in entry["texts"]},
                         {mit.read_bytes(), apache.read_bytes(), notice.read_bytes()})
        self.assertFalse(any("source_path" in row for row in raw))
        placeholder.unlink(); mit.unlink(); apache.unlink(); notice.unlink()
        self.assertEqual((output / raw[0]["path"]).read_bytes(), b"")
        self.assertEqual({(output / path).read_bytes() for path in entry["texts"]},
                         {b"Author MIT terms\n", b"Author Apache terms\n", b"Author attribution\n"})

    def test_empty_placeholder_requires_all_nonempty_named_or_scoped_units(self):
        package = self.package("empty-scoped", "Apache-2.0/MIT")
        self.text(package, "COPYING.txt", "")
        mit = self.root / "empty-scoped-mit.txt"; mit.write_text("Actual MIT terms\n")
        apache = self.root / "empty-scoped-apache.txt"; apache.write_text("Actual Apache terms\n")
        key = lambda unit: ("licenses/packages/" + hashlib.sha256(package["id"].encode()).hexdigest()
                            + "/" + hashlib.sha256(unit.encode()).hexdigest() + ".txt")
        output = self.root / "empty-scoped-output"
        for extras in [{}, {key("MIT"): mit}, {key("Apache-2.0"): apache}]:
            with self.subTest(extras=list(extras)), self.assertRaisesRegex(licenses.LicenseCollectionError, "missing actual text"):
                licenses.collect_notices([self.document([package])], extras, output)
            self.assertFalse(output.exists())
        inventory = licenses.collect_notices([self.document([package])],
                                            {key("MIT"): mit, key("Apache-2.0"): apache}, output)
        entry = inventory["packages"][0]
        self.assertEqual({row["unit"] for row in entry["used_unit_bindings"]}, {"MIT", "Apache-2.0"})
        self.assertEqual({(output / path).read_bytes() for path in entry["texts"]},
                         {mit.read_bytes(), apache.read_bytes()})
        self.assertTrue(set(row["path"] for row in entry["source_placeholders"]).isdisjoint(entry["texts"]))
        # Existing nonempty complete-expression admission also resolves every unit.
        combined = self.root / "empty-scoped-complete.txt"; combined.write_text("Author full Apache and MIT terms\n")
        other = self.root / "empty-complete-output"
        inventory = licenses.collect_notices([self.document([package])],
                                            {"licenses/Apache-2.0/MIT.txt": combined}, other)
        self.assertEqual({(other / path).read_bytes() for path in inventory["packages"][0]["texts"]},
                         {combined.read_bytes()})

    def test_zero_placeholder_domain_refuses_single_local_git_declared_with_and_malformed(self):
        for number, (expression, source, declared) in enumerate([
            ("MIT", "registry+https://github.com/rust-lang/crates.io-index", None),
            ("MIT/MIT", "registry+https://github.com/rust-lang/crates.io-index", None),
            ("MIT OR Apache-2.0", None, None),
            ("MIT OR Apache-2.0", "git+https://example.invalid/source#revision", None),
            ("MIT OR Apache-2.0", "registry+https://github.com/rust-lang/crates.io-index", "LICENSE"),
            (None, "registry+https://github.com/rust-lang/crates.io-index", "LICENSE"),
            ("Apache-2.0 WITH LLVM-exception OR MIT", "registry+https://github.com/rust-lang/crates.io-index", None),
            ("MIT//Apache-2.0", "registry+https://github.com/rust-lang/crates.io-index", None),
        ]):
            with self.subTest(expression=expression, source=source, declared=declared):
                package = self.package(f"empty-domain-{number}", expression, source)
                package["license_file"] = declared
                self.text(package, "LICENSE", "")
                self.text(package, "LICENSE-MIT", "MIT\n")
                self.text(package, "LICENSE-APACHE", "Apache\n")
                self.text(package, "LICENSE-Apache-2.0-WITH-LLVM-exception", "Full exception\n")
                output = self.root / f"empty-domain-output-{number}"
                with self.assertRaises(licenses.LicenseCollectionError):
                    licenses.collect_notices([self.document([package])], {}, output)
                self.assertFalse(output.exists())

    def test_zero_and_nonzero_invalid_notices_bindings_links_and_nested_files_stay_strict(self):
        for number, (name, content) in enumerate([
            ("LICENSE-MIT", b""), ("NOTICE", b""), ("COPYRIGHT", b""),
            ("licenses/LICENSE", b""), ("LICENSE", b" \n\t"), ("LICENSE", b"x\0y"),
            ("LICENSE", b"x" * (licenses.MAX_TEXT_BYTES + 1)),
        ]):
            with self.subTest(name=name, content_length=len(content)):
                package = self.package(f"empty-invalid-{number}", "MIT OR Apache-2.0")
                self.text(package, "LICENSE-MIT", "MIT\n")
                self.text(package, "LICENSE-APACHE", "Apache\n")
                path = Path(package["manifest_path"]).parent / name
                path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(content)
                output = self.root / f"empty-invalid-output-{number}"
                with self.assertRaises(licenses.LicenseCollectionError):
                    licenses.collect_notices([self.document([package])], {}, output)
                self.assertFalse(output.exists())
        package = self.package("empty-linked", "MIT OR Apache-2.0")
        self.text(package, "LICENSE-MIT", "MIT\n"); self.text(package, "LICENSE-APACHE", "Apache\n")
        outside = self.root / "outside-empty"; outside.write_bytes(b"")
        (Path(package["manifest_path"]).parent / "LICENSE").symlink_to(outside)
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "symlink"):
            licenses.collect_notices([self.document([package])], {}, self.root / "empty-link-output")
        (Path(package["manifest_path"]).parent / "LICENSE").unlink()
        package["license_file"] = str(outside)
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "escapes"):
            licenses.collect_notices([self.document([package])], {}, self.root / "empty-outbound-output")
        package["license_file"] = None
        key = ("licenses/packages/" + hashlib.sha256(package["id"].encode()).hexdigest()
               + "/" + hashlib.sha256(b"MIT").hexdigest() + ".txt")
        for extras in [{"unused-empty.txt": outside}, {key: outside}]:
            with self.subTest(extras=list(extras)), self.assertRaisesRegex(licenses.LicenseCollectionError, "empty"):
                licenses.collect_notices([self.document([package])], extras, self.root / "empty-extra-output")
            self.assertFalse((self.root / "empty-extra-output").exists())

    def test_empty_placeholder_never_poisons_later_strict_license_file_reads(self):
        package = self.package("empty-cache", "MIT OR Apache-2.0")
        self.text(package, "LICENSE", "")
        self.text(package, "LICENSE-MIT", "MIT\n"); self.text(package, "LICENSE-APACHE", "Apache\n")
        declared = dict(package, id=package["id"] + "-declared", license="MIT", license_file="LICENSE")
        output = self.root / "empty-cache-output"
        with self.assertRaisesRegex(licenses.LicenseCollectionError, "empty"):
            licenses.collect_notices([self.document([package, declared])], {}, output)
        self.assertFalse(output.exists())

    def test_empty_placeholder_uses_actual_read_instead_of_only_zero_stat(self):
        package = self.package("empty-race", "MIT OR Apache-2.0")
        placeholder = self.text(package, "LICENSE", "")
        self.text(package, "LICENSE-MIT", "MIT\n"); self.text(package, "LICENSE-APACHE", "Apache\n")
        original_open = Path.open
        changed = []
        def change_before_read(path, mode="r", *args, **kwargs):
            if path == placeholder and mode == "rb" and not changed:
                changed.append(True)
                with original_open(path, "wb") as writer:
                    writer.write(b"x")
            return original_open(path, mode, *args, **kwargs)
        output = self.root / "empty-race-output"
        with patch.object(Path, "open", change_before_read):
            with self.assertRaisesRegex(licenses.LicenseCollectionError, "not actually empty"):
                licenses.collect_notices([self.document([package])], {}, output)
        self.assertEqual(changed, [True])
        self.assertFalse(output.exists())

    def test_empty_placeholder_counts_file_candidate_output_caps_and_cross_graph_dedup(self):
        package = self.package("empty-caps", "MIT OR Apache-2.0")
        self.text(package, "LICENSE", "")
        mit = self.text(package, "LICENSE-MIT", "MIT\n")
        apache = self.text(package, "LICENSE-APACHE", "Apache\n")
        for constant in ["MAX_TEXT_FILES", "MAX_CANDIDATE_FILES", "MAX_OUTPUT_FILES"]:
            with self.subTest(constant=constant), patch.object(licenses, constant, 2):
                output = self.root / "empty-cap-output"
                with self.assertRaises(licenses.LicenseCollectionError):
                    licenses.collect_notices([self.document([package])], {}, output)
                self.assertFalse(output.exists())
        size = len(mit.read_bytes()) + len(apache.read_bytes())
        with patch.object(licenses, "MAX_TOTAL_BYTES", size - 1):
            with self.assertRaisesRegex(licenses.LicenseCollectionError, "aggregate"):
                licenses.collect_notices([self.document([package])], {}, self.root / "empty-byte-cap")
        with patch.object(licenses, "MAX_TEXT_FILES", 3), patch.object(licenses, "MAX_CANDIDATE_FILES", 3), patch.object(licenses, "MAX_OUTPUT_FILES", 3), patch.object(licenses, "MAX_TOTAL_BYTES", size), patch.object(licenses, "MAX_READ_BYTES", size):
            output = self.root / "empty-cap-success"
            inventory = licenses.collect_notices([self.document([package], label) for label in ["native", "wg", "gl"]], {}, output)
        self.assertEqual(len(inventory["packages"]), 1)
        self.assertEqual(inventory["packages"][0]["graphs"], ["native", "wg", "gl"])
        self.assertEqual(len(inventory["files"]), 3)
        self.assertEqual(len(inventory["packages"][0]["texts"]), 2)
        self.assertEqual(len(inventory["packages"][0]["source_placeholders"]), 1)


if __name__ == "__main__":
    unittest.main()
