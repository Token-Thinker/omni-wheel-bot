//! Environment snapshot used by every subcommand.
//!
//! The bash deployer read a scattering of env vars and also `source`d
//! `config/devkit_wiring.local.env` (with `set -a`) before doing anything, so
//! values from that file override the caller's shell. [`Env`] reproduces that:
//! it snapshots the process environment once, overlays the wiring file, and
//! every lookup afterwards goes through it. Keeping the lookups off
//! `std::env` also makes the planning code testable with a hand-built map.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

#[derive(Debug, Clone, Default)]
pub struct Env {
    vars: BTreeMap<String, String>,
    /// The wiring file that was overlaid, if any (for reporting).
    pub wiring_file: Option<PathBuf>,
}

impl Env {
    /// Snapshot the process environment and overlay the devkit wiring file.
    ///
    /// The file is `$OWB_DEVKIT_WIRING_ENV` if set, else
    /// `config/devkit_wiring.local.env` under the repo (same as `lib.sh`).
    pub fn load(repo: &Path) -> Result<Self, String> {
        let mut env = Env::from_pairs(
            std::env::vars_os()
                .filter_map(|(k, v)| Some((k.into_string().ok()?, v.into_string().ok()?))),
        );
        let file = env
            .get("OWB_DEVKIT_WIRING_ENV")
            .map(PathBuf::from)
            .unwrap_or_else(|| repo.join("config/devkit_wiring.local.env"));
        if file.is_file() {
            let text = std::fs::read_to_string(&file)
                .map_err(|e| format!("reading {}: {e}", file.display()))?;
            for (k, v) in parse_env_file(&text) {
                env.vars.insert(k, v);
            }
            env.wiring_file = Some(file);
        }
        Ok(env)
    }

    pub fn from_pairs<K, V>(pairs: impl IntoIterator<Item = (K, V)>) -> Self
    where
        K: Into<String>,
        V: Into<String>,
    {
        Env {
            vars: pairs
                .into_iter()
                .map(|(k, v)| (k.into(), v.into()))
                .collect(),
            wiring_file: None,
        }
    }

    /// Value of `key` when it is set *and non-empty* — the `[[ -n "$X" ]]`
    /// test the bash scripts used for nearly every variable.
    pub fn get(
        &self,
        key: &str,
    ) -> Option<&str> {
        self.vars
            .get(key)
            .map(String::as_str)
            .filter(|v| !v.is_empty())
    }

    /// `[[ "${X:-}" == "1" ]]`.
    pub fn is_one(
        &self,
        key: &str,
    ) -> bool {
        self.get(key) == Some("1")
    }
}

/// Parse a shell-style env file: `KEY=value` or `export KEY=value` lines,
/// `#` comments, optional single/double quotes around the value.
///
/// Deliberately not a shell: no expansion, no multi-line values. The wiring
/// files are plain GPIO assignments, which this covers.
pub fn parse_env_file(text: &str) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for raw in text.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let line = line.strip_prefix("export ").unwrap_or(line).trim_start();
        let Some((key, value)) = line.split_once('=')
        else {
            continue;
        };
        let key = key.trim();
        let valid_key = key
            .chars()
            .next()
            .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
            && key.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
        if !valid_key {
            continue;
        }
        let value = value.trim();
        let value = if let Some(inner) = value
            .strip_prefix('"')
            .and_then(|v| v.strip_suffix('"'))
            .or_else(|| value.strip_prefix('\'').and_then(|v| v.strip_suffix('\'')))
        {
            inner.to_string()
        }
        else {
            // Unquoted: a ` #` starts a trailing comment.
            value
                .split_once(" #")
                .map_or(value, |(v, _)| v)
                .trim_end()
                .to_string()
        };
        out.push((key.to_string(), value));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn env_file_parses_export_quotes_and_comments() {
        let text = "# header\n\
                    export OWB_DEVKIT_MTR_SLEEP=45\n\
                    OWB_DEVKIT_LED_PIN=\"38\"\n\
                    AP_SSID='my ap'\n\
                    OWB_DEVKIT_ENC1_A=39 # trailing\n\
                    not a var\n\
                    1BAD=x\n";
        assert_eq!(
            parse_env_file(text),
            vec![
                ("OWB_DEVKIT_MTR_SLEEP".into(), "45".into()),
                ("OWB_DEVKIT_LED_PIN".into(), "38".into()),
                ("AP_SSID".into(), "my ap".into()),
                ("OWB_DEVKIT_ENC1_A".into(), "39".into()),
            ]
        );
    }

    #[test]
    fn get_treats_empty_as_unset() {
        let env = Env::from_pairs([("A", ""), ("B", "1"), ("C", "yes")]);
        assert_eq!(env.get("A"), None);
        assert_eq!(env.get("B"), Some("1"));
        assert!(env.is_one("B"));
        assert!(!env.is_one("C"));
        assert!(!env.is_one("missing"));
    }
}
