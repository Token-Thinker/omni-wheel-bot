//! `.owb_last_target`: the last selected target, for `--last`.
//!
//! Same file and format as `scripts/state.sh` (bash `KEY=value` lines written
//! with `printf %q`), so the bash scripts and xtask can read each other's file
//! while both exist. Wi-Fi credentials are never stored.

use std::path::Path;

pub const STATE_FILE: &str = ".owb_last_target";

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LastTarget {
    pub maker: Option<String>,
    pub board: Option<String>,
    pub bin: Option<String>,
    pub port: Option<String>,
    pub profile: Option<String>,
}

impl LastTarget {
    pub fn parse(text: &str) -> Self {
        let mut t = LastTarget::default();
        for line in text.lines() {
            let line = line.trim();
            let Some((key, raw)) = line.split_once('=')
            else {
                continue;
            };
            let value = unquote(raw);
            if value.is_empty() {
                continue;
            }
            let slot = match key {
                "MAKER" => &mut t.maker,
                "BOARD" => &mut t.board,
                "BIN" => &mut t.bin,
                "PORT" => &mut t.port,
                "IO_PROFILE" => &mut t.profile,
                _ => continue,
            };
            *slot = Some(value);
        }
        t
    }

    pub fn render(&self) -> String {
        let mut out = String::new();
        for (key, value) in [
            ("MAKER", &self.maker),
            ("BOARD", &self.board),
            ("BIN", &self.bin),
            ("PORT", &self.port),
            ("IO_PROFILE", &self.profile),
        ] {
            if let Some(v) = value {
                out.push_str(&format!("{key}={}\n", quote(v)));
            }
        }
        out
    }

    pub fn load(repo: &Path) -> Option<Self> {
        std::fs::read_to_string(repo.join(STATE_FILE))
            .ok()
            .map(|t| Self::parse(&t))
    }

    pub fn save(
        &self,
        repo: &Path,
    ) -> Result<(), String> {
        let path = repo.join(STATE_FILE);
        std::fs::write(&path, self.render()).map_err(|e| format!("writing {}: {e}", path.display()))
    }
}

/// Undo bash quoting as produced by `printf %q`: `'...'`, `"..."`, `$'...'`
/// (common escapes only) and backslash escapes in bare words.
fn unquote(raw: &str) -> String {
    let raw = raw.trim();
    if let Some(inner) = raw.strip_prefix("$'").and_then(|r| r.strip_suffix('\'')) {
        return inner
            .replace("\\n", "\n")
            .replace("\\t", "\t")
            .replace("\\'", "'")
            .replace("\\\\", "\\");
    }
    let mut out = String::new();
    let mut chars = raw.chars();
    while let Some(c) = chars.next() {
        match c {
            '\'' => out.extend(chars.by_ref().take_while(|&c| c != '\'')),
            '"' => {
                while let Some(c) = chars.next() {
                    match c {
                        '"' => break,
                        '\\' => out.extend(chars.next()),
                        c => out.push(c),
                    }
                }
            }
            '\\' => out.extend(chars.next()),
            c => out.push(c),
        }
    }
    out
}

/// Quote for `source`-ing by bash: bare when safe, else single quotes.
fn quote(v: &str) -> String {
    if !v.is_empty()
        && v.chars()
            .all(|c| c.is_ascii_alphanumeric() || "-_./:@%+,".contains(c))
    {
        v.to_string()
    }
    else {
        format!("'{}'", v.replace('\'', r"'\''"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_bash_written_file() {
        let t = LastTarget::parse("MAKER=owb-esp\nBOARD=esp32s3\nBIN=owb-esp\n");
        assert_eq!(t.maker.as_deref(), Some("owb-esp"));
        assert_eq!(t.board.as_deref(), Some("esp32s3"));
        assert_eq!(t.bin.as_deref(), Some("owb-esp"));
        assert_eq!(t.port, None);
        assert_eq!(t.profile, None);
    }

    #[test]
    fn parses_printf_q_quoting() {
        let t = LastTarget::parse(
            "PORT=/dev/serial/by-id/usb-Espressif\\ USB\\ JTAG\nIO_PROFILE='odd name'\nJUNK\n",
        );
        assert_eq!(
            t.port.as_deref(),
            Some("/dev/serial/by-id/usb-Espressif USB JTAG")
        );
        assert_eq!(t.profile.as_deref(), Some("odd name"));
        assert_eq!(
            LastTarget::parse("PORT=$'/dev/a\\'b'\n").port.as_deref(),
            Some("/dev/a'b")
        );
    }

    #[test]
    fn render_round_trips() {
        let t = LastTarget {
            maker: Some("owb-esp".into()),
            board: Some("esp32s3".into()),
            bin: Some("owb-esp".into()),
            port: Some("/dev/it's here".into()),
            profile: Some("owb-esp32s3-v1-alpha".into()),
        };
        let text = t.render();
        assert!(text.starts_with("MAKER=owb-esp\nBOARD=esp32s3\nBIN=owb-esp\n"));
        assert_eq!(LastTarget::parse(&text), t);
    }
}
