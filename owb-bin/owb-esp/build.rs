use std::path::PathBuf;

fn main() {
    // WiFi credentials — set these in your environment before building.
    // Never commit real values. Copy from .env.example and source it:
    //   cp .env.example .env && source .env
    //
    // In firmware source, read with:
    //   const WIFI_SSID: &str = env!("OWB_WIFI_SSID");
    //   const WIFI_PASSWORD: &str = env!("OWB_WIFI_PASSWORD");
    //
    // The legacy bare SSID/PASSWORD env vars still work via option_env!() in
    // owb-esp.rs; this build script provides the canonical OWB_WIFI_* names
    // for documentation and future migrations.
    println!(
        "cargo:rustc-env=OWB_WIFI_SSID={}",
        std::env::var("OWB_WIFI_SSID")
            .or_else(|_| std::env::var("SSID"))
            .unwrap_or_default()
    );
    println!(
        "cargo:rustc-env=OWB_WIFI_PASSWORD={}",
        std::env::var("OWB_WIFI_PASSWORD")
            .or_else(|_| std::env::var("PASSWORD"))
            .unwrap_or_default()
    );
    println!("cargo:rerun-if-env-changed=OWB_WIFI_SSID");
    println!("cargo:rerun-if-env-changed=OWB_WIFI_PASSWORD");
    println!("cargo:rerun-if-env-changed=SSID");
    println!("cargo:rerun-if-env-changed=PASSWORD");
    println!("cargo:rerun-if-env-changed=AP_SSID");
    println!("cargo:rerun-if-env-changed=AP_PASSWORD");
    println!("cargo:rerun-if-env-changed=CONTROL_KEY");
    println!("cargo:rerun-if-env-changed=OWB_ALLOW_OPEN_AP");

    // ---- PWA asset availability check ----
    // `ui_assets.rs` embeds the Trunk build output from owb-ui/dist/ via
    // include_bytes!.  When dist/ is missing the compiler error is a cryptic
    // "file not found" from include_bytes!.  Emit an explicit warning here so
    // the root cause is immediately obvious in the build output.
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let dist = manifest_dir.join("../../owb-ui/dist");

    // Re-run whenever dist/ changes so incremental builds pick up new assets.
    println!("cargo:rerun-if-changed=../../owb-ui/dist/");

    if !dist.exists() {
        println!(
            "cargo:warning=owb-ui/dist/ not found — \
             run `cd owb-ui && trunk build --release` before building the firmware"
        );
        return;
    }

    for name in &[
        "index.html",
        "app.wasm",
        "app.js",
        "style.css",
        "manifest.json",
        "icons/icon-192.svg",
        "icons/icon-512.svg",
    ] {
        if !dist.join(name).exists() {
            println!(
                "cargo:warning=owb-ui/dist/{name} missing — \
                 re-run `cd owb-ui && trunk build --release` \
                 (the Trunk.toml post-build hook renames hashed outputs to \
                 app.wasm / app.js)"
            );
        }
    }
}
