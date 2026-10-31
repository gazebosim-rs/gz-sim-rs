// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use sha2::{Digest, Sha256};
use syn::{
    Expr, Item, Lit, MacroDelimiter, Meta, Token, UseTree, Visibility, punctuated::Punctuated,
};

fn literal(expression: &Expr, expected: &str) -> bool {
    matches!(expression, Expr::Lit(value) if value.attrs.is_empty()
        && matches!(&value.lit, Lit::Str(text) if text.value() == expected))
}

fn generated_include(source: &str) -> bool {
    let Ok(file) = syn::parse_file(source) else {
        return false;
    };
    if file.shebang.is_some() || file.attrs.iter().any(|attribute| {
        !matches!(&attribute.meta, Meta::NameValue(value) if value.path.is_ident("doc")
            && matches!(&value.value, Expr::Lit(expression) if matches!(&expression.lit, Lit::Str(_))))
    }) { return false; }
    let [Item::Mod(module), Item::Use(reexport)] = file.items.as_slice() else {
        return false;
    };
    if module.ident != "msgs"
        || !module.attrs.is_empty()
        || module.unsafety.is_some()
        || !matches!(module.vis, Visibility::Public(_))
        || !reexport.attrs.is_empty()
        || !matches!(reexport.vis, Visibility::Public(_))
        || reexport.leading_colon.is_some()
        || !matches!(&reexport.tree, UseTree::Name(name) if name.ident == "prost")
    {
        return false;
    }
    let Some((_, items)) = &module.content else {
        return false;
    };
    let [Item::Macro(include)] = items.as_slice() else {
        return false;
    };
    if !include.attrs.is_empty()
        || include.ident.is_some()
        || include.semi_token.is_none()
        || !include.mac.path.is_ident("include")
        || !matches!(include.mac.delimiter, MacroDelimiter::Paren(_))
    {
        return false;
    }
    let Ok(Expr::Macro(concat)) = include.mac.parse_body::<Expr>() else {
        return false;
    };
    if !concat.attrs.is_empty()
        || !concat.mac.path.is_ident("concat")
        || !matches!(concat.mac.delimiter, MacroDelimiter::Paren(_))
    {
        return false;
    }
    let Ok(arguments) = concat
        .mac
        .parse_body_with(Punctuated::<Expr, Token![,]>::parse_terminated)
    else {
        return false;
    };
    let arguments: Vec<_> = arguments.iter().collect();
    let [Expr::Macro(environment), suffix] = arguments.as_slice() else {
        return false;
    };
    if !environment.attrs.is_empty()
        || !environment.mac.path.is_ident("env")
        || !matches!(environment.mac.delimiter, MacroDelimiter::Paren(_))
        || !literal(suffix, "/gz.msgs.rs")
    {
        return false;
    }
    environment
        .mac
        .parse_body::<Expr>()
        .is_ok_and(|value| literal(&value, "OUT_DIR"))
}

#[test]
fn only_builtin_generated_include_and_exact_reexport_are_declarations() {
    let source = include_str!("../src/lib.rs");
    assert!(generated_include(source));
    assert!(generated_include(
        "//! Docs\npub mod msgs { include!(concat!(env!(\"OUT_DIR\"), \"/gz.msgs.rs\")); } pub use prost;"
    ));
    for source in [
        "pub mod msgs; pub use prost;",
        "pub mod msgs {} pub use prost;",
        "pub mod msgs { fn run() {} } pub use prost;",
        "pub mod msgs { include!(\"generated.rs\"); } pub use prost;",
        "pub mod msgs { include!(concat!(env!(\"OUT_DIR\"), \"/other.rs\")); } pub use prost;",
        "pub mod msgs { include!(concat!(env!(\"OTHER\"), \"/gz.msgs.rs\")); } pub use prost;",
        "pub mod msgs { include!(concat!(runtime(), \"/gz.msgs.rs\")); } pub use prost;",
        "pub mod msgs { include!(custom!(env!(\"OUT_DIR\"), \"/gz.msgs.rs\")); } pub use prost;",
        "pub mod msgs { include!(concat!(custom!(\"OUT_DIR\"), \"/gz.msgs.rs\")); } pub use prost;",
        "pub mod msgs { include! { concat!(env!(\"OUT_DIR\"), \"/gz.msgs.rs\") } } pub use prost;",
        "pub mod msgs { include!(concat!{env!(\"OUT_DIR\"), \"/gz.msgs.rs\"}); } pub use prost;",
        "pub mod msgs { include!(concat!(env!{\"OUT_DIR\"}, \"/gz.msgs.rs\")); } pub use prost;",
        "pub mod msgs { include!(concat!(env!(\"OUT_DIR\"), \"/gz.msgs.rs\", runtime())); } pub use prost;",
        "pub mod msgs { include!(concat!(env!(\"OUT_DIR\", \"error\"), \"/gz.msgs.rs\")); } pub use prost;",
        "pub mod msgs { #[cfg(any())] include!(concat!(env!(\"OUT_DIR\"), \"/gz.msgs.rs\")); } pub use prost;",
        "#[cfg(any())] pub mod msgs { include!(concat!(env!(\"OUT_DIR\"), \"/gz.msgs.rs\")); } pub use prost;",
    ] {
        assert!(!generated_include(source), "{source}");
    }
    for source in [
        format!("#![allow(dead_code)]\n{source}"),
        format!("#![doc = custom!()]\n{source}"),
        format!("#!/bin/sh\n{source}"),
        format!("macro_rules! include {{ ($($t:tt)*) => {{}} }} {source}"),
        format!("use other::include; {source}"),
        format!("{source}\nfn runtime() {{}}"),
        source.replace("pub mod msgs", "mod msgs"),
        source.replace("pub use prost;", "pub use prost as renamed;"),
        source.replace("pub use prost;", "pub use prost::*;"),
        source.replace("pub use prost;", "pub use ::prost;"),
        source.replace("pub use prost;", "#[cfg(any())] pub use prost;"),
    ] {
        assert!(!generated_include(&source), "{source}");
    }
}

#[test]
fn source_audit_child_checks_actual_file_and_rejects_runtime_or_missing_source() {
    let temporary = tempfile::tempdir().unwrap();
    let source = temporary.path().join("lib.rs");
    let proof = temporary.path().join("proof.json");
    let child = || {
        let mut command = std::process::Command::new(std::env::current_exe().unwrap());
        command
            .args(["--exact", "source_audit_child", "--ignored", "--nocapture"])
            .env("GZ_PROTO_FACADE", &source)
            .env("GZ_PROTO_FACADE_PROOF", &proof);
        command
    };
    fs_write(&source, include_str!("../src/lib.rs"));
    let output = child().output().unwrap();
    assert!(output.status.success(), "{output:?}");
    let evidence: serde_json::Value =
        serde_json::from_slice(&std::fs::read(&proof).unwrap()).unwrap();
    assert_eq!(evidence["proof"], "syn2-proto-generated-include-v1");
    assert_eq!(
        evidence["source_sha256"],
        format!("{:x}", Sha256::digest(std::fs::read(&source).unwrap()))
    );
    std::fs::remove_file(&proof).unwrap();
    fs_write(&source, "pub fn runtime() {}");
    assert!(!child().output().unwrap().status.success());
    assert!(!proof.exists());
    std::fs::remove_file(&source).unwrap();
    assert!(!child().output().unwrap().status.success());
    assert!(!proof.exists());
}

fn fs_write(path: &std::path::Path, source: &str) {
    std::fs::write(path, source).unwrap();
}

#[test]
#[ignore = "executed on actual source by parent tests and the coverage verifier"]
fn source_audit_child() {
    let path = std::path::PathBuf::from(std::env::var_os("GZ_PROTO_FACADE").unwrap());
    let source = std::fs::read_to_string(&path).unwrap();
    assert!(
        generated_include(&source),
        "not the generated-include-only proto facade"
    );
    let proof = serde_json::json!({"proof": "syn2-proto-generated-include-v1", "source": path,
        "source_sha256": format!("{:x}", Sha256::digest(source.as_bytes()))});
    std::fs::write(
        std::env::var_os("GZ_PROTO_FACADE_PROOF").unwrap(),
        serde_json::to_vec(&proof).unwrap(),
    )
    .unwrap();
}
