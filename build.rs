fn main () {
    println!("cargo::rerun-if-changed=build.rs");
    println!("cargo::rustc-env=MACOSX_DEPLOYMENT_TARGET=11.0");
}
