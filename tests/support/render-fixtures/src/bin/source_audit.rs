// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use syn::{Attribute, Expr, Item, Lit, Meta, UseTree, Visibility};

fn gpu_cfg_only(attributes: &[Attribute]) -> bool {
    if attributes.len() != 1 || !attributes[0].path().is_ident("cfg") {
        return false;
    }
    let Meta::List(cfg) = &attributes[0].meta else {
        return false;
    };
    matches!(cfg.parse_args::<Meta>(), Ok(Meta::NameValue(value))
        if value.path.is_ident("feature")
            && matches!(&value.value, Expr::Lit(expr)
                if matches!(&expr.lit, Lit::Str(name) if name.value() == "gpu")))
}
fn zero_executable_facade(source: &str) -> bool {
    let Ok(file) = syn::parse_file(source) else {
        return false;
    };
    if file.shebang.is_some()
        || file
            .attrs
            .iter()
            .any(|attribute| !attribute.path().is_ident("doc"))
    {
        return false;
    }
    let [Item::Mod(module), Item::Use(reexport)] = file.items.as_slice() else {
        return false;
    };
    if module.ident != "gpu"
        || module.content.is_some()
        || !matches!(module.vis, Visibility::Inherited)
        || !gpu_cfg_only(&module.attrs)
        || !gpu_cfg_only(&reexport.attrs)
        || !matches!(reexport.vis, Visibility::Public(_))
        || reexport.leading_colon.is_some()
    {
        return false;
    }
    matches!(&reexport.tree, UseTree::Path(path) if path.ident == "gpu" && matches!(&*path.tree, UseTree::Glob(_)))
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args_os().skip(1);
    let path = args.next().ok_or("source path required")?;
    if args.next().is_some() {
        return Err("one source path required".into());
    }
    let source = std::fs::read_to_string(path)?;
    if !zero_executable_facade(&source) {
        return Err("not the declaration-only GPU facade".into());
    }
    println!("renderer-facade-v1");
    Ok(())
}
