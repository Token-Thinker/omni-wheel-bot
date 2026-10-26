//! Process spawning, `--dry-run` printing, and small output helpers.

use std::{
    io::IsTerminal,
    path::{Path, PathBuf},
    process::Command,
};

pub type Result<T> = std::result::Result<T, String>;

/// Env vars whose values never get printed (dry-run or otherwise).
const SECRET_VARS: &[&str] = &[
    "PASSWORD",
    "AP_PASSWORD",
    "CONTROL_KEY",
    "OWB_WIFI_PASSWORD",
];

fn paint(
    style: &str,
    msg: &str,
) {
    if std::io::stderr().is_terminal() {
        eprintln!("\x1b[{style}m{msg}\x1b[0m");
    }
    else {
        eprintln!("{msg}");
    }
}

pub fn section(msg: &str) { paint("36;1", &format!("▶ {msg}")); }
pub fn note(msg: &str) { paint("34", &format!("• {msg}")); }
pub fn warn(msg: &str) { paint("33", &format!("⚠ {msg}")); }
pub fn ok(msg: &str) { paint("32", &format!("✔ {msg}")); }
pub fn error(msg: &str) { paint("31", &format!("✖ {msg}")); }

pub fn stdout_is_tty() -> bool { std::io::stdout().is_terminal() }

/// A command to run, or to print under `--dry-run`.
#[derive(Debug, Clone)]
pub struct Cmd {
    pub program: String,
    pub args: Vec<String>,
    /// Extra env for the child (added on top of the inherited env, unless
    /// `clear_env` is set).
    pub env: Vec<(String, String)>,
    /// Start from an empty environment, keeping only `keep_env` from ours.
    pub clear_env: bool,
    pub keep_env: Vec<String>,
    pub cwd: Option<PathBuf>,
}

impl Cmd {
    pub fn new(
        program: impl Into<String>,
        args: impl IntoIterator<Item = impl Into<String>>,
    ) -> Self {
        Cmd {
            program: program.into(),
            args: args.into_iter().map(Into::into).collect(),
            env: Vec::new(),
            clear_env: false,
            keep_env: Vec::new(),
            cwd: None,
        }
    }

    pub fn cwd(
        mut self,
        dir: &Path,
    ) -> Self {
        self.cwd = Some(dir.to_path_buf());
        self
    }

    /// The exact command line, shell-quoted, with env assignments in front.
    /// Secret values are replaced with `<redacted>`.
    pub fn display(&self) -> String {
        let mut parts = Vec::new();
        if let Some(cwd) = &self.cwd {
            parts.push(format!("cd {} &&", shell_quote(&cwd.to_string_lossy())));
        }
        if self.clear_env {
            parts.push("env -i".to_string());
            for key in &self.keep_env {
                if std::env::var_os(key).is_some() {
                    parts.push(format!("{key}=\"${key}\""));
                }
            }
        }
        for (k, v) in &self.env {
            let v = if SECRET_VARS.contains(&k.as_str()) && !v.is_empty() {
                "<redacted>".to_string()
            }
            else {
                shell_quote(v)
            };
            parts.push(format!("{k}={v}"));
        }
        parts.push(shell_quote(&self.program));
        parts.extend(self.args.iter().map(|a| shell_quote(a)));
        parts.join(" ")
    }

    fn command(&self) -> Command {
        let mut cmd = Command::new(&self.program);
        cmd.args(&self.args);
        if self.clear_env {
            cmd.env_clear();
            for key in &self.keep_env {
                if let Some(v) = std::env::var_os(key) {
                    cmd.env(key, v);
                }
            }
        }
        cmd.envs(self.env.iter().map(|(k, v)| (k, v)));
        if let Some(cwd) = &self.cwd {
            cmd.current_dir(cwd);
        }
        cmd
    }

    /// Run to completion (or just print it under `dry_run`).
    pub fn run(
        &self,
        dry_run: bool,
    ) -> Result<()> {
        if dry_run {
            println!("[dry-run] {}", self.display());
            return Ok(());
        }
        note(&format!("$ {}", self.display()));
        let status = self
            .command()
            .status()
            .map_err(|e| format!("could not start `{}`: {e}", self.program))?;
        if status.success() {
            Ok(())
        }
        else {
            Err(format!("`{}` failed ({status})", self.program))
        }
    }

    pub fn spawn_with_output(
        &self,
        log: std::fs::File,
    ) -> Result<std::process::Child> {
        let err = log.try_clone().map_err(|e| e.to_string())?;
        self.command()
            .stdout(log)
            .stderr(err)
            .spawn()
            .map_err(|e| format!("could not start `{}`: {e}", self.program))
    }
}

/// Quote `s` for a POSIX shell, only when needed.
pub fn shell_quote(s: &str) -> String {
    let safe = !s.is_empty()
        && s.chars()
            .all(|c| c.is_ascii_alphanumeric() || "-_./:=,@%+".contains(c));
    if safe {
        s.to_string()
    }
    else {
        format!("'{}'", s.replace('\'', r"'\''"))
    }
}

/// Look `program` up on `$PATH` (what `command -v` did).
pub fn which(program: &str) -> Option<PathBuf> {
    use std::os::unix::fs::PermissionsExt;
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .map(|dir| dir.join(program))
        .find(|p| {
            p.metadata()
                .is_ok_and(|m| m.is_file() && m.permissions().mode() & 0o111 != 0)
        })
}

/// Output of `id <flag>`, e.g. `id -g`. Used instead of libc so the crate has
/// no unsafe code and no extra dependency; the bash path shelled out the same.
pub fn id(flag: &str) -> Result<Vec<u32>> {
    let out = Command::new("id")
        .arg(flag)
        .output()
        .map_err(|e| format!("running `id {flag}`: {e}"))?;
    if !out.status.success() {
        return Err(format!("`id {flag}` failed"));
    }
    String::from_utf8_lossy(&out.stdout)
        .split_whitespace()
        .map(|n| {
            n.parse()
                .map_err(|_| format!("unexpected `id {flag}` output: {n}"))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quoting() {
        assert_eq!(shell_quote("/dev/ttyACM0"), "/dev/ttyACM0");
        assert_eq!(shell_quote("a b"), "'a b'");
        assert_eq!(shell_quote("it's"), r"'it'\''s'");
        assert_eq!(shell_quote(""), "''");
    }

    #[test]
    fn display_redacts_secrets() {
        let mut cmd = Cmd::new("docker", ["run", "-e", "PASSWORD"]);
        cmd.env = vec![
            ("SSID".into(), "lab net".into()),
            ("PASSWORD".into(), "hunter2".into()),
        ];
        assert_eq!(
            cmd.display(),
            "SSID='lab net' PASSWORD=<redacted> docker run -e PASSWORD"
        );
    }
}
