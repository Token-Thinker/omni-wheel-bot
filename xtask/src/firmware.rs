//! ESP32-S3 firmware build planning: features, container env, docker args,
//! target dir, artifact lookup.
//!
//! Everything here is pure (inputs in, plan out) so it can be unit-tested
//! against the behaviour of `scripts/makers/owb-esp.sh` and `scripts/lib.sh`,
//! which this module replaces. Executing the plan lives in `main.rs`.

use std::{
    path::{Path, PathBuf},
    time::SystemTime,
};

use crate::env::Env;

pub const MAKER: &str = "owb-esp";
pub const BOARD: &str = "esp32s3";
pub const BIN: &str = "owb-esp";
pub const TRIPLE: &str = "xtensa-esp32s3-none-elf";

/// Where the repo is mounted inside the toolchain container.
pub const CONTAINER_WORKSPACE: &str = "/workspace";
/// The idf-rust image runs as `esp` (uid/gid 1000, also in `users`=100).
/// `/home/esp` is mode 0700, so the image cannot be run as another uid; the
/// host's gid is added instead (`--group-add`) so the container can write
/// into group-writable dirs the host user owns.
pub const CONTAINER_UID: u32 = 1000;
pub const CONTAINER_GIDS: [u32; 2] = [1000, 100];
/// Present only inside the idf-rust image; sourcing it puts the Xtensa GCC
/// and libclang on the path.
pub const EXPORT_ESP: &str = "/home/esp/export-esp.sh";

/// `select_docker_target_dir` candidates (relative to the repo root).
pub const PREFERRED_TARGET_DIR: &str = "target/docker/owb-esp/esp32s3";
pub const FALLBACK_TARGET_DIR: &str = ".owb_target/docker/owb-esp/esp32s3";
/// A native (non-Docker) `cargo esp32s3` build from `owb-bin/owb-esp` lands
/// in the workspace target dir; the bash flash-only path looked here.
pub const NATIVE_TARGET_DIR: &str = "target";

/// Build-time `option_env!` vars for the SoftAP / control-auth gates, passed
/// through only when set (`owb-esp.sh`). `OWB_ALLOW_NO_KEY` and
/// `OWB_CAMERA_FORMAT` are read by the firmware too but the bash path never
/// forwarded them; they are forwarded here so `.env`/CI settings take effect.
pub const AP_PASSTHROUGH: [&str; 6] = [
    "OWB_ALLOW_OPEN_AP",
    "AP_SSID",
    "AP_PASSWORD",
    "CONTROL_KEY",
    "OWB_ALLOW_NO_KEY",
    "OWB_CAMERA_FORMAT",
];

/// DevKit pin overrides read by `env_pin!` in `owb-esp/src/profile/mod.rs`.
pub const DEVKIT_PASSTHROUGH: [&str; 14] = [
    "OWB_DEVKIT_MTR_SLEEP",
    "OWB_DEVKIT_MTR1_PHASE",
    "OWB_DEVKIT_MTR1_DRIVE",
    "OWB_DEVKIT_MTR2_PHASE",
    "OWB_DEVKIT_MTR2_DRIVE",
    "OWB_DEVKIT_MTR3_PHASE",
    "OWB_DEVKIT_MTR3_DRIVE",
    "OWB_DEVKIT_ENC1_A",
    "OWB_DEVKIT_ENC1_B",
    "OWB_DEVKIT_ENC2_A",
    "OWB_DEVKIT_ENC2_B",
    "OWB_DEVKIT_ENC3_A",
    "OWB_DEVKIT_ENC3_B",
    "OWB_DEVKIT_LED_PIN",
];

/// `profile_supports_real_executors` (lib.sh), board fixed to esp32s3.
pub fn supports_real_executors(profile: &str) -> bool {
    matches!(
        profile,
        "esp32-s3-devkitc-1-n32r16" | "owb-esp32s3-v1" | "owb-esp32s3-v1-alpha" | "owb-esp32s3-v2"
    )
}

/// `profile_has_camera` (lib.sh): only the explicitly camera-less boards
/// return false; anything else, including an unknown name, has a camera.
pub fn has_camera(profile: &str) -> bool { !matches!(profile, "esp32-s3-devkitc-1-n32r16") }

/// `rust-version = "x.y.z"` from a Cargo.toml (owb-core's is the source of
/// truth for the image tag).
pub fn rust_version(cargo_toml: &str) -> Option<String> {
    cargo_toml.lines().find_map(|line| {
        let rest = line.trim_start().strip_prefix("rust-version")?;
        let rest = rest.trim_start().strip_prefix('=')?.trim();
        let inner = rest.strip_prefix('"')?;
        Some(inner[..inner.find('"')?].to_string())
    })
}

/// `1.95.0` → `espressif/idf-rust:all_1.95.0.0`.
pub fn image_tag(rust_version: &str) -> String {
    format!("espressif/idf-rust:all_{rust_version}.0")
}

/// Profile names declared in `owb-esp/src/profile/mod.rs` (`name: "..."`),
/// in file order — the same scrape `select_profile.sh` did.
pub fn profile_names(profile_mod_rs: &str) -> Vec<String> {
    profile_mod_rs
        .lines()
        .filter_map(|line| {
            let rest = line.trim_start().strip_prefix("name: \"")?;
            Some(rest[..rest.find('"')?].to_string())
        })
        .collect()
}

/// Everything the user chose for one firmware build.
#[derive(Debug, Clone)]
pub struct BuildRequest {
    pub profile: String,
    pub real_executors: bool,
    pub hil_low_mode: bool,
    pub debug_i2c: bool,
    pub ssid: String,
    pub password: String,
    /// Repo-relative cargo target dir (see [`choose_target_dir`]).
    pub target_dir: String,
}

/// The resolved build: cargo features, the env the toolchain sees, and
/// human-readable notes explaining non-obvious choices.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BuildPlan {
    pub features: Vec<&'static str>,
    /// Ordered `(name, value)` env for the build, with paths expressed
    /// relative to `workspace` (the repo root as the toolchain sees it).
    pub env: Vec<(String, String)>,
    pub real_executors: bool,
    pub notes: Vec<String>,
}

/// Port of `maker_build`'s env/feature assembly.
///
/// `workspace` is where the repo lives for the process running cargo:
/// [`CONTAINER_WORKSPACE`] for `docker run`, the real path when xtask already
/// runs inside the image.
pub fn plan(
    req: &BuildRequest,
    env: &Env,
    workspace: &str,
) -> BuildPlan {
    let mut vars: Vec<(String, String)> = Vec::new();
    let mut push = |k: &str, v: &str| vars.push((k.to_string(), v.to_string()));
    let mut notes = Vec::new();
    let mut features = Vec::new();

    // Always passed, possibly empty (empty = SoftAP fallback on boot).
    push("SSID", &req.ssid);
    push("PASSWORD", &req.password);
    push(
        "CARGO_TARGET_DIR",
        &format!("{workspace}/{}", req.target_dir),
    );
    // CARGO_HOME is deliberately left at the image default (/home/esp/.cargo,
    // thrown away with the container). Registry source paths end up in the
    // firmware's panic-location strings, so a different CARGO_HOME would
    // change the flashed bytes relative to the bash build.
    if req.debug_i2c {
        push("OWB_DEBUG_I2C", "1");
    }
    if let Some(level) = env.get("OWB_LOG_LEVEL") {
        push("OWB_LOG_LEVEL", level);
    }
    if env.get("OWB_DUMP_PACKETS").is_some() {
        push("ESP_RADIO_CONFIG_DUMP_PACKETS", "true");
    }
    for name in AP_PASSTHROUGH {
        if let Some(v) = env.get(name) {
            push(name, v);
        }
    }
    push("OWB_IO_PROFILE", &req.profile);
    for name in DEVKIT_PASSTHROUGH {
        if let Some(v) = env.get(name) {
            push(name, v);
        }
    }

    if env.is_one("OWB_FORCE_NO_CAMERA") {
        features.push("no-camera");
        notes.push("No-camera compile path forced (OWB_FORCE_NO_CAMERA=1)".to_string());
    }
    else if !has_camera(&req.profile) {
        features.push("no-camera");
        notes.push(format!(
            "No-camera compile path selected for profile {}",
            req.profile
        ));
    }
    if req.hil_low_mode {
        push("OWB_HIL_LOW_MODE", "1");
    }
    if env.is_one("OWB_DEV_TELEMETRY") {
        features.push("dev-telemetry");
        notes.push(
            "Dev telemetry: compiling debug/trace tracing events into the release build"
                .to_string(),
        );
    }
    let mut real = false;
    if req.real_executors {
        if supports_real_executors(&req.profile) {
            push("OWB_REAL_EXECUTORS", "1");
            features.push("real-executors");
            real = true;
        }
        else {
            notes.push(format!(
                "Profile {} does not support hardware-backed executors; using the shadow runtime.",
                req.profile
            ));
        }
    }

    BuildPlan {
        features,
        env: vars,
        real_executors: real,
        notes,
    }
}

/// The `sh -c` script run inside the toolchain image (owb-esp.sh, verbatim
/// apart from the workspace path and color choice). `cargo esp32s3` is the
/// alias in `owb-bin/owb-esp/.cargo/config.toml`:
/// `build --release --features=esp32s3 --target=xtensa-esp32s3-none-elf`.
pub fn build_script(
    workspace: &str,
    features: &[&str],
    color: bool,
) -> String {
    let mut cargo = format!(
        "rustup run esp cargo {BOARD} --bin {BIN} --ignore-rust-version -q --color={}",
        if color { "always" } else { "never" }
    );
    if !features.is_empty() {
        cargo.push_str(&format!(" --features {}", features.join(",")));
    }
    format!(
        ". {EXPORT_ESP} && rustc --version --verbose && rustup show active-toolchain && \
         cd {workspace}/owb-bin/{MAKER} && {cargo}"
    )
}

/// `docker run` argv (without the leading `docker`).
///
/// Env vars are passed by *name only* (`-e NAME`); the values are set on the
/// `docker` process itself. Same result as `-e NAME=value`, but Wi-Fi
/// passwords and control keys stay out of `ps` and out of `--dry-run` output.
pub fn docker_args(
    repo: &Path,
    host_gid: u32,
    env: &[(String, String)],
    image: &str,
    script: &str,
) -> Vec<String> {
    let mut args = vec![
        "run".to_string(),
        "--rm".to_string(),
        "-v".to_string(),
        format!("{}:{CONTAINER_WORKSPACE}", repo.display()),
        "--group-add".to_string(),
        host_gid.to_string(),
    ];
    for (name, _) in env {
        args.push("-e".to_string());
        args.push(name.clone());
    }
    args.extend([
        "--entrypoint".to_string(),
        "sh".to_string(),
        image.to_string(),
        "-c".to_string(),
        script.to_string(),
    ]);
    args
}

/// Who will write the target dir: the uid and every gid of that process.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Writer {
    pub uid: u32,
    pub gids: Vec<u32>,
}

impl Writer {
    /// The toolchain container's `esp` user plus the `--group-add` host gid.
    pub fn container(host_gid: u32) -> Self {
        let mut gids = CONTAINER_GIDS.to_vec();
        gids.push(host_gid);
        Writer {
            uid: CONTAINER_UID,
            gids,
        }
    }
}

/// Ownership and mode bits of a path, as far as write access goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Owner {
    pub uid: u32,
    pub gid: u32,
    pub mode: u32,
}

/// Can `who` create entries in a directory owned as `dir`? Needs w+x on the
/// class (owner/group/other) that applies.
pub fn can_write(
    dir: Owner,
    who: &Writer,
) -> bool {
    let bits = if who.uid == 0 {
        return true;
    }
    else if dir.uid == who.uid {
        dir.mode >> 6
    }
    else if who.gids.contains(&dir.gid) {
        dir.mode >> 3
    }
    else {
        dir.mode
    };
    bits & 0o3 == 0o3
}

/// Pick the cargo target dir for the containerised build.
///
/// Same candidates as `select_docker_target_dir`, but the check is whether
/// the *build process* (container uid 1000 + host gid) can write the dir — or
/// its nearest existing ancestor if it does not exist yet. The bash version
/// tested whether the *host user* could write the parent, which is the wrong
/// identity: once a container build created `target/docker/owb-esp` (owned by
/// uid 1000), a host user with a different uid failed the test and the next
/// build silently switched to the fallback dir, so builds alternated between
/// two caches and a flash-only run could pick up a stale ELF.
///
/// `stat` returns the owner of a repo-relative path, or `None` if it does not
/// exist ("" is the repo root).
pub fn choose_target_dir(
    env: &Env,
    who: &Writer,
    stat: impl Fn(&Path) -> Option<Owner>,
) -> Result<String, String> {
    if let Some(dir) = env.get("OWB_CARGO_TARGET_DIR") {
        return Ok(dir.trim_end_matches('/').to_string());
    }
    for candidate in [PREFERRED_TARGET_DIR, FALLBACK_TARGET_DIR] {
        let mut path = Some(Path::new(candidate));
        while let Some(p) = path {
            if let Some(owner) = stat(p) {
                if can_write(owner, who) {
                    return Ok(candidate.to_string());
                }
                break;
            }
            path = p.parent();
        }
    }
    Err(format!(
        "neither {PREFERRED_TARGET_DIR} nor {FALLBACK_TARGET_DIR} is writable by the build \
         (uid {}, gids {:?}). Make one of them (or its parent) group-writable for your group, \
         or set OWB_CARGO_TARGET_DIR to a repo-relative dir that is.",
        who.uid, who.gids
    ))
}

/// Firmware ELF inside a cargo target dir.
pub fn artifact_path(target_dir: &str) -> PathBuf {
    Path::new(target_dir).join(TRIPLE).join("release").join(BIN)
}

/// Repo-relative target dirs a flash-only run should look in, in priority
/// order. With `OWB_CARGO_TARGET_DIR` set, only that one.
pub fn flash_search_dirs(env: &Env) -> Vec<String> {
    match env.get("OWB_CARGO_TARGET_DIR") {
        Some(dir) => vec![dir.trim_end_matches('/').to_string()],
        None => vec![
            PREFERRED_TARGET_DIR.to_string(),
            FALLBACK_TARGET_DIR.to_string(),
            NATIVE_TARGET_DIR.to_string(),
        ],
    }
}

/// The newest of the ELFs that exist. Ties go to the earlier (higher
/// priority) candidate.
pub fn newest_artifact(found: &[(PathBuf, SystemTime)]) -> Option<&(PathBuf, SystemTime)> {
    found.iter().reduce(|best, cur| {
        if cur.1 > best.1 {
            cur
        }
        else {
            best
        }
    })
}

#[cfg(test)]
mod tests {
    use std::{collections::HashMap, time::Duration};

    use super::*;

    fn req(profile: &str) -> BuildRequest {
        BuildRequest {
            profile: profile.to_string(),
            real_executors: false,
            hil_low_mode: false,
            debug_i2c: false,
            ssid: String::new(),
            password: String::new(),
            target_dir: PREFERRED_TARGET_DIR.to_string(),
        }
    }

    #[test]
    fn image_tag_comes_from_owb_core_rust_version() {
        let toml = "[package]\nname = \"owb-core\"\nrust-version = \"1.95.0\"\n";
        let v = rust_version(toml).unwrap();
        assert_eq!(v, "1.95.0");
        assert_eq!(image_tag(&v), "espressif/idf-rust:all_1.95.0.0");
        assert_eq!(rust_version("[package]\nname = \"x\"\n"), None);
    }

    #[test]
    fn image_tag_matches_the_real_owb_core_manifest() {
        let toml = include_str!("../../owb-core/Cargo.toml");
        assert!(rust_version(toml).is_some());
    }

    #[test]
    fn profile_names_scrape_the_real_profile_table() {
        let names = profile_names(include_str!("../../owb-bin/owb-esp/src/profile/mod.rs"));
        for expected in [
            "xiao-seedstudio-sense-cam",
            "esp32-s3-devkitc-1-n32r16",
            "owb-esp32s3-v1",
            "owb-esp32s3-v1-alpha",
            "owb-esp32s3-v2",
        ] {
            assert!(names.iter().any(|n| n == expected), "{expected} missing");
        }
    }

    #[test]
    fn features_v1_alpha_real_executors() {
        let mut r = req("owb-esp32s3-v1-alpha");
        r.real_executors = true;
        let p = plan(&r, &Env::default(), CONTAINER_WORKSPACE);
        assert_eq!(p.features, vec!["real-executors"]);
        assert!(p.real_executors);
        assert!(p.env.contains(&("OWB_REAL_EXECUTORS".into(), "1".into())));
    }

    #[test]
    fn features_camera_less_profile_gets_no_camera_first() {
        let mut r = req("esp32-s3-devkitc-1-n32r16");
        r.real_executors = true;
        let env = Env::from_pairs([("OWB_DEV_TELEMETRY", "1")]);
        let p = plan(&r, &env, CONTAINER_WORKSPACE);
        assert_eq!(
            p.features,
            vec!["no-camera", "dev-telemetry", "real-executors"]
        );
    }

    #[test]
    fn features_unsupported_profile_drops_real_executors() {
        let mut r = req("xiao-seedstudio-sense-cam");
        r.real_executors = true;
        let p = plan(&r, &Env::default(), CONTAINER_WORKSPACE);
        assert!(p.features.is_empty());
        assert!(!p.real_executors);
        assert!(!p.env.iter().any(|(k, _)| k == "OWB_REAL_EXECUTORS"));
        assert!(p.notes.iter().any(|n| n.contains("shadow runtime")));
    }

    #[test]
    fn features_force_no_camera_only_on_exactly_one() {
        let r = req("owb-esp32s3-v2");
        let on = plan(&r, &Env::from_pairs([("OWB_FORCE_NO_CAMERA", "1")]), "/w");
        assert_eq!(on.features, vec!["no-camera"]);
        let off = plan(
            &r,
            &Env::from_pairs([("OWB_FORCE_NO_CAMERA", "true")]),
            "/w",
        );
        assert!(off.features.is_empty());
    }

    #[test]
    fn env_order_and_passthrough_match_the_bash_path() {
        let mut r = req("esp32-s3-devkitc-1-n32r16");
        r.ssid = "lab".into();
        r.password = "pw".into();
        r.hil_low_mode = true;
        r.debug_i2c = true;
        r.real_executors = true;
        let env = Env::from_pairs([
            ("OWB_LOG_LEVEL", "debug"),
            ("OWB_DUMP_PACKETS", "1"),
            ("CONTROL_KEY", "k"),
            ("AP_SSID", ""), // empty = not passed
            ("OWB_DEVKIT_LED_PIN", "38"),
            ("UNRELATED", "x"),
        ]);
        let p = plan(&r, &env, CONTAINER_WORKSPACE);
        let names: Vec<&str> = p.env.iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(
            names,
            vec![
                "SSID",
                "PASSWORD",
                "CARGO_TARGET_DIR",
                "OWB_DEBUG_I2C",
                "OWB_LOG_LEVEL",
                "ESP_RADIO_CONFIG_DUMP_PACKETS",
                "CONTROL_KEY",
                "OWB_IO_PROFILE",
                "OWB_DEVKIT_LED_PIN",
                "OWB_HIL_LOW_MODE",
                "OWB_REAL_EXECUTORS",
            ]
        );
        let get = |k: &str| p.env.iter().find(|(n, _)| n == k).unwrap().1.clone();
        assert_eq!(
            get("CARGO_TARGET_DIR"),
            "/workspace/target/docker/owb-esp/esp32s3"
        );
        assert_eq!(get("ESP_RADIO_CONFIG_DUMP_PACKETS"), "true");
        assert_eq!(get("OWB_IO_PROFILE"), "esp32-s3-devkitc-1-n32r16");
        assert_eq!(p.features, vec!["no-camera", "real-executors"]);
    }

    #[test]
    fn script_matches_owb_esp_sh() {
        assert_eq!(
            build_script("/workspace", &["no-camera", "real-executors"], true),
            ". /home/esp/export-esp.sh && rustc --version --verbose && rustup show \
             active-toolchain && cd /workspace/owb-bin/owb-esp && rustup run esp cargo esp32s3 \
             --bin owb-esp --ignore-rust-version -q --color=always --features \
             no-camera,real-executors"
        );
        assert!(!build_script("/w", &[], false).contains("--features"));
    }

    #[test]
    fn docker_args_pass_env_by_name_only() {
        let env = vec![
            ("SSID".to_string(), "lab".to_string()),
            ("PASSWORD".to_string(), "secret".to_string()),
        ];
        let args = docker_args(
            Path::new("/repo"),
            1001,
            &env,
            "espressif/idf-rust:all_1.95.0.0",
            "true",
        );
        assert_eq!(
            args,
            vec![
                "run",
                "--rm",
                "-v",
                "/repo:/workspace",
                "--group-add",
                "1001",
                "-e",
                "SSID",
                "-e",
                "PASSWORD",
                "--entrypoint",
                "sh",
                "espressif/idf-rust:all_1.95.0.0",
                "-c",
                "true",
            ]
        );
        assert!(!args.iter().any(|a| a.contains("secret")));
    }

    #[test]
    fn write_permission_classes() {
        let esp = Writer::container(1001);
        // Created by an earlier container build: owner esp, 0755.
        assert!(can_write(
            Owner {
                uid: 1000,
                gid: 1000,
                mode: 0o755
            },
            &esp
        ));
        // Host user dir, group-writable for the host gid.
        assert!(can_write(
            Owner {
                uid: 1001,
                gid: 1001,
                mode: 0o770
            },
            &esp
        ));
        // Host user dir under umask 022: the container cannot write it.
        assert!(!can_write(
            Owner {
                uid: 1001,
                gid: 1001,
                mode: 0o755
            },
            &esp
        ));
        // World-writable.
        assert!(can_write(
            Owner {
                uid: 0,
                gid: 0,
                mode: 0o1777
            },
            &esp
        ));
    }

    #[test]
    fn target_dir_prefers_existing_container_owned_dir() {
        let esp = Writer::container(1001);
        let fs: HashMap<&str, Owner> = HashMap::from([
            (
                "",
                Owner {
                    uid: 1001,
                    gid: 1001,
                    mode: 0o770,
                },
            ),
            (
                "target",
                Owner {
                    uid: 1001,
                    gid: 1001,
                    mode: 0o770,
                },
            ),
            (
                "target/docker",
                Owner {
                    uid: 1000,
                    gid: 1000,
                    mode: 0o755,
                },
            ),
            (
                "target/docker/owb-esp",
                Owner {
                    uid: 1000,
                    gid: 1000,
                    mode: 0o755,
                },
            ),
            (
                "target/docker/owb-esp/esp32s3",
                Owner {
                    uid: 1000,
                    gid: 1000,
                    mode: 0o755,
                },
            ),
        ]);
        let stat = |p: &Path| fs.get(p.to_str().unwrap()).copied();
        assert_eq!(
            choose_target_dir(&Env::default(), &esp, stat).unwrap(),
            PREFERRED_TARGET_DIR
        );
    }

    #[test]
    fn target_dir_falls_back_when_preferred_is_unwritable() {
        let esp = Writer::container(1001);
        let fs: HashMap<&str, Owner> = HashMap::from([
            (
                "",
                Owner {
                    uid: 1001,
                    gid: 1001,
                    mode: 0o770,
                },
            ),
            (
                "target",
                Owner {
                    uid: 1001,
                    gid: 1001,
                    mode: 0o755,
                },
            ),
        ]);
        let stat = |p: &Path| fs.get(p.to_str().unwrap()).copied();
        assert_eq!(
            choose_target_dir(&Env::default(), &esp, stat).unwrap(),
            FALLBACK_TARGET_DIR
        );
        // Nothing writable at all: a clear error, not a guess.
        let locked: HashMap<&str, Owner> = HashMap::from([(
            "",
            Owner {
                uid: 1001,
                gid: 1001,
                mode: 0o755,
            },
        )]);
        let err = choose_target_dir(&Env::default(), &esp, |p: &Path| {
            locked.get(p.to_str().unwrap()).copied()
        })
        .unwrap_err();
        assert!(err.contains("OWB_CARGO_TARGET_DIR"));
    }

    #[test]
    fn target_dir_override_wins() {
        let env = Env::from_pairs([("OWB_CARGO_TARGET_DIR", "target/docker/")]);
        let got = choose_target_dir(&env, &Writer::container(1), |_| None).unwrap();
        assert_eq!(got, "target/docker");
        assert_eq!(flash_search_dirs(&env), vec!["target/docker"]);
        assert_eq!(
            artifact_path("target/docker"),
            PathBuf::from("target/docker/xtensa-esp32s3-none-elf/release/owb-esp")
        );
    }

    #[test]
    fn newest_artifact_wins_ties_to_priority() {
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(100);
        let t1 = t0 + Duration::from_secs(1);
        let found = vec![
            (PathBuf::from("a"), t0),
            (PathBuf::from("b"), t1),
            (PathBuf::from("c"), t1),
        ];
        assert_eq!(newest_artifact(&found).unwrap().0, PathBuf::from("b"));
        assert!(newest_artifact(&[]).is_none());
    }
}
