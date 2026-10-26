//! `cargo xtask` — build, flash, monitor, probe, CI and e2e for the
//! omni-wheel bot. Replaces the bash deployer under `scripts/`.
//!
//! Runs in two places:
//! * on a host with Rust: firmware builds go through `docker run` with the
//!   `espressif/idf-rust` image, exactly like `scripts/makers/owb-esp.sh`;
//! * inside that image (via the `./owb` launcher on Docker-only machines):
//!   detected through `/home/esp/export-esp.sh`, and the firmware cargo build
//!   runs directly instead of Docker-in-Docker.
//!
//! Non-interactive first: every choice has a flag. Prompts appear only when
//! stdin is a terminal; otherwise a missing choice is an error naming the flag.

mod e2e;
mod env;
mod firmware;
mod monitor;
mod probe;
mod prompt;
mod resolve;
mod run;
mod state;

use std::{
    os::unix::fs::MetadataExt,
    path::{Path, PathBuf},
    process::ExitCode,
};

use clap::{Args, Parser, Subcommand, ValueEnum};

use crate::{
    env::Env,
    firmware::{BuildRequest, Owner, Writer},
    prompt::{Ask, Terminal},
    run::{note, ok, section, warn, Cmd, Result},
    state::LastTarget,
};

#[derive(Parser)]
#[command(
    name = "cargo xtask",
    about = "Build / flash / monitor tooling for the omni-wheel bot",
    after_help = "On a machine with Docker but no Rust, run the same commands via ./owb"
)]
struct Cli {
    #[command(subcommand)]
    cmd: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Cross-compile the ESP32-S3 firmware in the idf-rust image (no flash)
    Build(BuildArgs),
    /// Flash the firmware with espflash, then attach its monitor
    Flash(FlashArgs),
    /// Build, then flash (same as `flash --build`)
    BuildFlash(FlashArgs),
    /// Attach a serial monitor to a running board (no reset unless --reset)
    Monitor(MonitorArgs),
    /// Identify connected boards from USB/sysfs data (never opens a port)
    Probe,
    /// Toolchain, Docker image, serial ports, profiles and binaries
    Info,
    /// Run the GitHub Actions host checks locally (port of ci_local.sh)
    Ci {
        #[arg(value_enum, default_value_t = CiTarget::All)]
        target: CiTarget,
        /// Profile for the `esp32s3` target
        #[arg(long, default_value = "owb-esp32s3-v1-alpha")]
        profile: String,
    },
    /// Host-sim end-to-end smoke test (port of e2e_host_sim.sh)
    E2e {
        /// host:port to bind the sim to [env: OWB_E2E_HOST / OWB_E2E_PORT]
        #[arg(long)]
        addr: Option<String>,
    },
    /// Run the host simulator (http://127.0.0.1:3000/)
    Sim,
}

#[derive(Args, Clone, Default)]
struct BuildArgs {
    /// IO profile (pin map) to compile in [env: IO_PROFILE]
    #[arg(long)]
    profile: Option<String>,
    /// Reuse the profile/port saved in .owb_last_target
    #[arg(long)]
    last: bool,
    /// Enable the hardware-backed board executors (supported profiles only)
    #[arg(long)]
    real_executors: bool,
    /// Reduced motor authority for safer HIL bench tests
    #[arg(long, conflicts_with = "normal_drive_mode")]
    hil_low_mode: bool,
    /// Full motor authority (the default; suppresses the guided-mode prompt)
    #[arg(long)]
    normal_drive_mode: bool,
    /// No STA credentials: boot straight into the SoftAP [env: OWB_FORCE_SOFTAP=1]
    #[arg(long)]
    force_softap: bool,
    /// Live I2C/PWM telemetry (OWB_DEBUG_I2C=1)
    #[arg(long)]
    debug_i2c: bool,
    /// Wi-Fi SSID [env: SSID]; the password only comes from $PASSWORD or a prompt
    #[arg(long)]
    ssid: Option<String>,
    /// Print the exact commands instead of running them
    #[arg(long)]
    dry_run: bool,
}

impl BuildArgs {
    fn has_build_options(&self) -> bool {
        self.profile.is_some()
            || self.real_executors
            || self.hil_low_mode
            || self.normal_drive_mode
            || self.force_softap
            || self.debug_i2c
            || self.ssid.is_some()
    }
}

#[derive(Args, Clone, Default)]
struct FlashArgs {
    /// Serial port, e.g. /dev/ttyACM0 (auto-detected when exactly one exists)
    #[arg(long)]
    port: Option<String>,
    /// Do not attach the serial monitor after flashing
    #[arg(long, alias = "no-attach-monitor")]
    no_monitor: bool,
    /// Flash this ELF instead of the newest build output
    #[arg(long)]
    elf: Option<PathBuf>,
    /// Build first
    #[arg(long)]
    build: bool,
    #[command(flatten)]
    build_args: BuildArgs,
}

#[derive(Args)]
struct MonitorArgs {
    /// Serial port (auto-detected when exactly one exists)
    #[arg(long)]
    port: Option<String>,
    /// Reuse the port saved in .owb_last_target
    #[arg(long)]
    last: bool,
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
    /// Pulse the board into a normal boot first (DTR low, RTS 150 ms) to
    /// catch the boot log. Never uses `espflash reset`.
    #[arg(long)]
    reset: bool,
    /// Print what would happen without opening the port
    #[arg(long)]
    dry_run: bool,
}

#[derive(Clone, Copy, ValueEnum)]
enum CiTarget {
    /// cargo fmt --check, clippy -D warnings, test -p owb-core
    Check,
    /// cargo build -p owb-host-sim
    Build,
    /// cargo publish --dry-run -p owb-core (informational)
    PublishDry,
    /// Firmware build in the idf-rust image
    Esp32s3,
    /// check + build + publish-dry (host-side only)
    All,
}

/// Shared state for one invocation.
struct Ctx {
    repo: PathBuf,
    env: Env,
    in_container: bool,
    ask: Terminal,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match real_main(cli) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            run::error(&e);
            ExitCode::FAILURE
        }
    }
}

fn real_main(cli: Cli) -> Result<()> {
    // xtask/ sits directly under the workspace root. Prefer the runtime
    // value cargo sets for `cargo run`; fall back to the compile-time one.
    let manifest_dir = std::env::var_os("CARGO_MANIFEST_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")));
    let repo = manifest_dir
        .parent()
        .ok_or("cannot locate the workspace root")?
        .to_path_buf();
    let env = Env::load(&repo)?;
    let in_container = Path::new(firmware::EXPORT_ESP).is_file() || env.is_one("OWB_IN_CONTAINER");
    let mut ctx = Ctx {
        repo,
        env,
        in_container,
        ask: Terminal::detect(),
    };
    if let Some(f) = &ctx.env.wiring_file {
        note(&format!(
            "Loaded devkit wiring overrides from {}",
            f.display()
        ));
    }

    match cli.cmd {
        Command::Build(args) => build(&mut ctx, &args).map(|_| ()),
        Command::Flash(args) => flash(&mut ctx, &args),
        Command::BuildFlash(mut args) => {
            args.build = true;
            flash(&mut ctx, &args)
        }
        Command::Monitor(args) => monitor_cmd(&mut ctx, &args),
        Command::Probe => probe_cmd(&ctx),
        Command::Info => info(&ctx),
        Command::Ci { target, profile } => ci(&mut ctx, target, &profile),
        Command::E2e { addr } => {
            let addr = addr.unwrap_or_else(|| {
                format!(
                    "{}:{}",
                    ctx.env.get("OWB_E2E_HOST").unwrap_or("127.0.0.1"),
                    ctx.env.get("OWB_E2E_PORT").unwrap_or("3300")
                )
            });
            e2e::run(&ctx.repo, &addr)
        }
        Command::Sim => {
            note("Phone UI: http://127.0.0.1:3000/  Dev console: http://127.0.0.1:3000/dev");
            Cmd::new("cargo", ["run", "-p", "owb-host-sim"])
                .cwd(&ctx.repo)
                .run(false)
        }
    }
}

fn read_repo_file(
    repo: &Path,
    rel: &str,
) -> Result<String> {
    std::fs::read_to_string(repo.join(rel)).map_err(|e| format!("reading {rel}: {e}"))
}

fn image(repo: &Path) -> Result<String> {
    let toml = read_repo_file(repo, "owb-core/Cargo.toml")?;
    let v = firmware::rust_version(&toml).ok_or("no rust-version in owb-core/Cargo.toml")?;
    Ok(firmware::image_tag(&v))
}

fn profiles(repo: &Path) -> Result<Vec<String>> {
    let names =
        firmware::profile_names(&read_repo_file(repo, "owb-bin/owb-esp/src/profile/mod.rs")?);
    if names.is_empty() {
        return Err("no profiles found in owb-bin/owb-esp/src/profile/mod.rs".to_string());
    }
    Ok(names)
}

fn first_gid() -> Result<u32> {
    run::id("-g")?
        .first()
        .copied()
        .ok_or_else(|| "`id -g` printed nothing".to_string())
}

/// `OWB_CARGO_TARGET_DIR` may be absolute; the container mounts the repo at
/// /workspace, so it must live inside the repo.
fn repo_relative(
    repo: &Path,
    dir: &str,
) -> Result<String> {
    let p = Path::new(dir);
    if p.is_relative() {
        return Ok(dir.to_string());
    }
    p.strip_prefix(repo)
        .map(|r| r.to_string_lossy().to_string())
        .map_err(|_| {
            format!(
                "OWB_CARGO_TARGET_DIR={dir} is outside the repo ({}); the toolchain container only \
                 sees the repo",
                repo.display()
            )
        })
}

/// A probed profile suggestion for the interactive menu (first port that
/// maps to a concrete profile).
fn detected_profile(repo: &Path) -> Option<(String, String)> {
    probe::list_ports().into_iter().find_map(|port| {
        let m = probe::identify(repo, &probe::port_info(&port))?;
        (!m.profile.is_empty()).then_some((port, m.profile))
    })
}

/// Resolve choices, run the firmware build, return the ELF path (repo-relative
/// when the target dir is). The saved state's port is preserved.
fn build(
    ctx: &mut Ctx,
    args: &BuildArgs,
) -> Result<PathBuf> {
    let last = load_last(ctx, args.last);
    let known = profiles(&ctx.repo)?;
    let guided = args.profile.is_none() && !args.last;
    let detected = if guided && ctx.ask.interactive() {
        detected_profile(&ctx.repo)
    }
    else {
        None
    };
    if let Some((port, p)) = &detected {
        note(&format!("Probe: {port} looks like profile {p}"));
    }
    let profile = resolve::profile(
        args.profile.as_deref(),
        last.as_ref(),
        &ctx.env,
        &known,
        detected.as_ref().map(|(_, p)| p.as_str()),
        &mut ctx.ask,
    )?;
    let wifi = resolve::wifi(
        args.ssid.as_deref(),
        args.force_softap,
        &ctx.env,
        &mut ctx.ask,
    )?;
    let hil = resolve::hil_low_mode(
        args.hil_low_mode,
        args.normal_drive_mode,
        guided,
        &mut ctx.ask,
    )?;

    let host_gid = first_gid()?;
    let writer = if ctx.in_container {
        Writer {
            uid: run::id("-u")?
                .first()
                .copied()
                .unwrap_or(firmware::CONTAINER_UID),
            gids: run::id("-G")?,
        }
    }
    else {
        Writer::container(host_gid)
    };
    let repo = ctx.repo.clone();
    let target_dir = firmware::choose_target_dir(&ctx.env, &writer, |rel| {
        let meta = std::fs::metadata(repo.join(rel)).ok()?;
        Some(Owner {
            uid: meta.uid(),
            gid: meta.gid(),
            mode: meta.mode(),
        })
    })?;
    let target_dir = repo_relative(&ctx.repo, &target_dir)?;

    let req = BuildRequest {
        profile: profile.clone(),
        real_executors: args.real_executors,
        hil_low_mode: hil,
        debug_i2c: args.debug_i2c,
        ssid: wifi.ssid.clone(),
        password: wifi.password.clone(),
        target_dir: target_dir.clone(),
    };
    let workspace = if ctx.in_container {
        ctx.repo.to_string_lossy().to_string()
    }
    else {
        firmware::CONTAINER_WORKSPACE.to_string()
    };
    let plan = firmware::plan(&req, &ctx.env, &workspace);
    let image = image(&ctx.repo)?;

    section(&format!(
        "Build ({}/{}/{})",
        firmware::MAKER,
        firmware::BOARD,
        firmware::BIN
    ));
    note(&format!("IO profile: {profile}"));
    if wifi.softap {
        note("Wi-Fi: no STA credentials; the firmware boots into its SoftAP fallback");
    }
    else {
        note(&format!(
            "Wi-Fi: SSID `{}` (password {})",
            wifi.ssid,
            if wifi.password.is_empty() {
                "empty"
            }
            else {
                "set"
            }
        ));
    }
    note(&format!(
        "Drive mode: {}",
        if hil { "hil-low-mode" } else { "normal" }
    ));
    if args.real_executors && !plan.real_executors {
        warn(&format!(
            "Profile {profile} does not support hardware-backed executors; building the shadow runtime."
        ));
    }
    for n in &plan.notes {
        note(n);
    }
    note(&format!(
        "Features: esp32s3{}",
        plan.features
            .iter()
            .map(|f| format!(",{f}"))
            .collect::<String>()
    ));
    note(&format!("Cargo target dir: {target_dir}"));

    let script = firmware::build_script(&workspace, &plan.features, run::stdout_is_tty());
    let cmd = if ctx.in_container {
        note("Running inside the toolchain image: building directly (no Docker)");
        // Start from a clean env like `docker run` does, so stray OWB_* vars
        // in the caller's shell cannot leak into the build. This also drops
        // the launcher's CARGO_HOME/CARGO_TARGET_DIR (meant for compiling
        // xtask itself) so the firmware sees the same paths as `docker run`.
        let mut cmd = Cmd::new("sh", ["-c", script.as_str()]);
        cmd.clear_env = true;
        cmd.keep_env = ["PATH", "HOME", "USER", "TERM", "LANG", "RUSTUP_HOME"]
            .map(String::from)
            .to_vec();
        cmd.env = plan.env.clone();
        cmd
    }
    else {
        note(&format!("Docker image: {image}"));
        if !args.dry_run && run::which("docker").is_none() {
            return Err("Docker not found. Please install & start Docker.".to_string());
        }
        let mut cmd = Cmd::new(
            "docker",
            firmware::docker_args(&ctx.repo, host_gid, &plan.env, &image, &script),
        );
        cmd.env = plan.env.clone();
        cmd
    };
    cmd.run(args.dry_run)
        .map_err(|e| format!("Build failed: {e}"))?;

    let artifact = firmware::artifact_path(&target_dir);
    if args.dry_run {
        note(&format!(
            "[dry-run] artifact would be {}",
            artifact.display()
        ));
    }
    else {
        if !ctx.repo.join(&artifact).is_file() {
            return Err(format!(
                "Build failed (missing artifact: {})",
                artifact.display()
            ));
        }
        ok(&format!("Build complete: {}", artifact.display()));
        save_last(ctx, Some(profile), None);
    }
    Ok(artifact)
}

fn load_last(
    ctx: &Ctx,
    wanted: bool,
) -> Option<LastTarget> {
    if !wanted {
        return None;
    }
    let last = LastTarget::load(&ctx.repo);
    if last.is_none() {
        warn("No last selection saved yet (.owb_last_target).");
    }
    last
}

/// Update `.owb_last_target`, keeping fields this run did not decide.
fn save_last(
    ctx: &Ctx,
    profile: Option<String>,
    port: Option<String>,
) {
    if ctx.env.is_one("OWB_NO_SAVE_LAST_TARGET") {
        return;
    }
    let mut t = LastTarget::load(&ctx.repo).unwrap_or_default();
    t.maker = Some(firmware::MAKER.into());
    t.board = Some(firmware::BOARD.into());
    t.bin = Some(firmware::BIN.into());
    if profile.is_some() {
        t.profile = profile;
    }
    if port.is_some() {
        t.port = port;
    }
    if let Err(e) = t.save(&ctx.repo) {
        warn(&e);
    }
}

/// Ports this process can open: in the container, only `--device` nodes.
fn visible_ports() -> Vec<String> {
    probe::list_ports()
        .into_iter()
        .filter(|p| Path::new(p).exists())
        .collect()
}

fn resolve_port(
    ctx: &mut Ctx,
    flag: Option<&str>,
    last: Option<&LastTarget>,
    dry_run: bool,
) -> Result<String> {
    let port = resolve::port(flag, last, &visible_ports(), &mut ctx.ask)?;
    if !dry_run && !Path::new(&port).exists() {
        let hint = if ctx.in_container {
            " (inside the toolchain container: pass --port so ./owb forwards the device)"
        }
        else {
            ""
        };
        return Err(format!("serial port {port} does not exist{hint}"));
    }
    Ok(port)
}

/// The ELF to flash: explicit, just built, or the newest candidate.
fn find_elf(
    ctx: &Ctx,
    explicit: Option<&Path>,
) -> Result<PathBuf> {
    if let Some(p) = explicit {
        return Ok(p.to_path_buf());
    }
    let mut found = Vec::new();
    for dir in firmware::flash_search_dirs(&ctx.env) {
        let rel = firmware::artifact_path(&repo_relative(&ctx.repo, &dir)?);
        if let Ok(mtime) = std::fs::metadata(ctx.repo.join(&rel)).and_then(|m| m.modified()) {
            found.push((rel, mtime));
        }
    }
    let (path, mtime) = firmware::newest_artifact(&found).ok_or_else(|| {
        format!(
            "no firmware ELF found (looked in {}). Run `cargo xtask build` first or pass --elf.",
            firmware::flash_search_dirs(&ctx.env).join(", ")
        )
    })?;
    if found.len() > 1 {
        note(&format!(
            "Several builds exist; using the newest ({})",
            path.display()
        ));
    }
    if let Ok(age) = mtime.elapsed() {
        let mins = age.as_secs() / 60;
        let msg = format!(
            "Firmware: {} (built {})",
            path.display(),
            match mins {
                0..120 => format!("{mins} min ago"),
                120..2880 => format!("{} h ago", mins / 60),
                _ => format!("{} days ago", mins / 1440),
            }
        );
        if mins >= 1440 {
            warn(&format!(
                "{msg}; rebuild with --build if that is not what you want"
            ));
        }
        else {
            note(&msg);
        }
    }
    Ok(path.clone())
}

fn flash(
    ctx: &mut Ctx,
    args: &FlashArgs,
) -> Result<()> {
    let dry_run = args.build_args.dry_run;
    if !args.build && args.build_args.has_build_options() {
        return Err(
            "build options (--profile, --real-executors, ...) need --build \
                    (or use `cargo xtask build-flash`)"
                .to_string(),
        );
    }
    let built = if args.build {
        Some(build(ctx, &args.build_args)?)
    }
    else {
        None
    };
    let last = load_last(ctx, args.build_args.last);
    let port = resolve_port(ctx, args.port.as_deref(), last.as_ref(), dry_run)?;
    let elf = match (&args.elf, built) {
        (Some(e), _) => e.clone(),
        (None, Some(b)) => b,
        (None, None) if dry_run => find_elf(ctx, None).unwrap_or_else(|e| {
            warn(&e);
            firmware::artifact_path(firmware::PREFERRED_TARGET_DIR)
        }),
        (None, None) => find_elf(ctx, None)?,
    };
    if !dry_run && !ctx.repo.join(&elf).is_file() {
        return Err(format!("Missing {}", elf.display()));
    }

    section(&format!(
        "Flash ({}/{}/{})",
        firmware::MAKER,
        firmware::BOARD,
        firmware::BIN
    ));
    if !dry_run && run::which("espflash").is_none() {
        return Err(
            "espflash not found. Install it with `cargo install espflash`, or run \
                    ./owb flash ... to use the copy in the idf-rust image."
                .to_string(),
        );
    }
    let mut espflash_args = vec!["flash".to_string()];
    if !args.no_monitor {
        espflash_args.push("--monitor".to_string());
    }
    if !ctx.ask.interactive() {
        // Never let espflash wait for a confirmation nobody can give.
        espflash_args.push("--non-interactive".to_string());
    }
    espflash_args.extend([
        "--port".to_string(),
        port.clone(),
        elf.to_string_lossy().to_string(),
    ]);
    let cmd = Cmd::new("espflash", espflash_args).cwd(&ctx.repo);

    // Up to 3 attempts, and only re-prompted on a TTY (backlog: the bash
    // retry prompt once looped 4889 times on EOF).
    let mut attempt = 1;
    loop {
        match cmd.run(dry_run) {
            Ok(()) => break,
            Err(e) if attempt < 3 && ctx.ask.interactive() => {
                warn(&format!("Flash failed: {e}"));
                let again = ctx.ask.line("Retry flash? [Y/n]")?;
                if again.trim().to_lowercase().starts_with('n') {
                    return Err("Flash aborted.".to_string());
                }
                attempt += 1;
            }
            Err(e) => return Err(format!("Flash failed after {attempt} attempt(s): {e}")),
        }
    }
    if !dry_run {
        ok("Flash complete");
        save_last(ctx, None, Some(port));
    }
    Ok(())
}

fn monitor_cmd(
    ctx: &mut Ctx,
    args: &MonitorArgs,
) -> Result<()> {
    let last = load_last(ctx, args.last);
    let port = resolve_port(ctx, args.port.as_deref(), last.as_ref(), args.dry_run)?;
    section("Monitor Serial");
    if args.dry_run {
        if args.reset {
            println!("[dry-run] open {port}; DTR=0; RTS=1; sleep 150ms; RTS=0; close; sleep 600ms");
        }
        println!(
            "[dry-run] open {port} at {} baud (DTR=1, RTS=0); stream output, forward stdin lines",
            args.baud
        );
        return Ok(());
    }
    if args.reset {
        note("Resetting into the app (DTR low, RTS pulse 150 ms)");
        monitor::reset_into_app(&port)?;
    }
    else {
        note("Attaching without a reset. Use --reset to catch the boot log.");
    }
    save_last(ctx, None, Some(port.clone()));
    monitor::monitor(&port, args.baud)
}

fn probe_cmd(ctx: &Ctx) -> Result<()> {
    let ports = probe::list_ports();
    if ports.is_empty() {
        warn("No serial ports detected.");
        return Ok(());
    }
    for port in ports {
        let info = probe::port_info(&port);
        let found = probe::identify(&ctx.repo, &info);
        section("Board Probe");
        for line in probe::report(&info, found.as_ref(), Path::new(&port).exists()) {
            note(&line);
        }
        println!();
    }
    Ok(())
}

fn info(ctx: &Ctx) -> Result<()> {
    section("Environment");
    let image = image(&ctx.repo)?;
    note(&format!(
        "Rust (workspace rust-version): {}",
        image
            .trim_start_matches("espressif/idf-rust:all_")
            .trim_end_matches(".0")
    ));
    note(&format!("Toolchain image: {image}"));
    note(&format!(
        "Running {}",
        if ctx.in_container {
            "inside the toolchain image"
        }
        else {
            "on the host"
        }
    ));
    if !ctx.in_container && run::which("docker").is_some() {
        let present = std::process::Command::new("docker")
            .args(["image", "inspect", "--format", "{{.Id}}", &image])
            .output()
            .is_ok_and(|o| o.status.success());
        if present {
            ok(&format!("{image} is pulled"));
        }
        else {
            warn(&format!(
                "{image} not pulled yet (first build downloads ~6 GB)"
            ));
        }
    }

    section("Serial ports");
    let ports = probe::list_ports();
    if ports.is_empty() {
        warn("none detected");
    }
    for p in ports {
        note(&p);
    }

    section("Installed tools");
    for tool in ["docker", "espflash", "cargo", "trunk", "just"] {
        match run::which(tool) {
            Some(p) => ok(&format!("{tool} ({})", p.display())),
            None => warn(&format!("{tool} not found")),
        }
    }

    section("Targets");
    note(&format!(
        "{} / {} / bin {} ({})",
        firmware::MAKER,
        firmware::BOARD,
        firmware::BIN,
        firmware::TRIPLE
    ));
    note("owb-host-sim / host (`cargo xtask sim`)");

    section("IO profiles");
    for p in profiles(&ctx.repo)? {
        note(&format!(
            "{p:<28} camera={:<5} real-executors={}",
            firmware::has_camera(&p),
            firmware::supports_real_executors(&p)
        ));
    }
    if let Some(t) = LastTarget::load(&ctx.repo) {
        section("Last target (.owb_last_target)");
        note(&format!(
            "profile={} port={}",
            t.profile.as_deref().unwrap_or("-"),
            t.port.as_deref().unwrap_or("-")
        ));
    }
    Ok(())
}

/// Pinned nightly for rustfmt; keep in step with the `fmt` job in
/// .github/workflows/ci.yml.
const RUSTFMT_TOOLCHAIN: &str = "nightly-2026-06-20";

fn ci(
    ctx: &mut Ctx,
    target: CiTarget,
    profile: &str,
) -> Result<()> {
    let cargo = |args: &[&str]| Cmd::new("cargo", args.iter().copied()).cwd(&ctx.repo);
    let check = || -> Result<()> {
        // rustfmt.toml uses nightly-only options, and stable rustfmt both
        // ignores them and reformats against them, so this must match the
        // pinned toolchain in .github/workflows/ci.yml.
        section("cargo fmt --check (nightly rustfmt)");
        cargo(
            &[
                "+".to_owned() + RUSTFMT_TOOLCHAIN,
                "fmt".into(),
                "--all".into(),
                "--".into(),
                "--check".into(),
            ]
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>(),
        )
        .run(false)?;
        ok("fmt clean");
        section("cargo clippy (owb-core, owb-host-sim)");
        cargo(&[
            "clippy",
            "-p",
            "owb-core",
            "-p",
            "owb-host-sim",
            "--no-deps",
            "--",
            "-D",
            "warnings",
        ])
        .run(false)?;
        ok("clippy clean");
        section("cargo test -p owb-core");
        cargo(&["test", "-p", "owb-core"]).run(false)?;
        ok("tests passed");
        Ok(())
    };
    let build_sim = || -> Result<()> {
        section("cargo build -p owb-host-sim");
        cargo(&["build", "-p", "owb-host-sim"]).run(false)?;
        ok("host-sim built");
        Ok(())
    };
    let publish_dry = || {
        section("cargo publish --dry-run -p owb-core --allow-dirty (informational)");
        match cargo(&["publish", "--dry-run", "-p", "owb-core", "--allow-dirty"]).run(false) {
            Ok(()) => ok("publish dry-run succeeded"),
            Err(_) => warn(
                "publish dry-run failed (informational only; expected when the version already \
                 exists on crates.io)",
            ),
        }
    };
    match target {
        CiTarget::Check => check(),
        CiTarget::Build => build_sim(),
        CiTarget::PublishDry => {
            publish_dry();
            Ok(())
        }
        CiTarget::Esp32s3 => {
            // ci_local.sh ran the guided build here, which needs a TTY. Pin
            // the choices instead: a CI-style SoftAP image of `profile`.
            let args = BuildArgs {
                profile: Some(profile.to_string()),
                real_executors: true,
                force_softap: true,
                normal_drive_mode: true,
                ..Default::default()
            };
            build(ctx, &args)?;
            ok("esp32s3 firmware build complete");
            Ok(())
        }
        CiTarget::All => {
            check()?;
            build_sim()?;
            publish_dry();
            Ok(())
        }
    }
}
