#![allow(dead_code)]

extern crate nalgebra as na;

use inflector::Inflector;

use rapier_testbed2d::{Example, TestbedApp};

mod basic2;
mod custom_forces2;
mod elasticity2;
mod layers2;
mod surface_tension2;

fn demo_name_from_command_line() -> Option<String> {
    let mut args = std::env::args();

    while let Some(arg) = args.next() {
        if &arg[..] == "--example" {
            return args.next();
        }
    }

    None
}

#[cfg(target_arch = "wasm32")]
fn demo_name_from_url() -> Option<String> {
    let window = stdweb::web::window();
    let hash = window.location()?.search().ok()?;
    if !hash.is_empty() {
        Some(hash[1..].to_string())
    } else {
        None
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn demo_name_from_url() -> Option<String> {
    None
}

fn main() {
    let demo = demo_name_from_command_line()
        .or_else(|| demo_name_from_url())
        .unwrap_or(String::new())
        .to_camel_case();

    let mut builders = vec![
        Example::demo("Basic", basic2::init_world),
        Example::demo("Layers", layers2::init_world),
        Example::demo("Custom forces", custom_forces2::init_world),
        Example::demo("Elasticity", elasticity2::init_world),
        Example::demo("Surface tension", surface_tension2::init_world),
    ];
    builders.sort_by_key(|builder| builder.name);

    let i = builders
        .iter()
        .position(|builder| builder.name.to_camel_case().as_str() == demo.as_str())
        .unwrap_or(0);
    builders.rotate_left(i);
    let testbed = TestbedApp::from_builders(builders);

    pollster::block_on(testbed.run());
}
