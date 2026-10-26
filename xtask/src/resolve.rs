//! Turning flags + env + `.owb_last_target` (+ prompts, on a TTY) into the
//! concrete profile, Wi-Fi credentials and serial port for a run.
//!
//! Precedence everywhere: explicit flag, then `--last`, then environment,
//! then a prompt if interactive, else an error that names the flag to pass.

use crate::{env::Env, prompt::Ask, run::Result, state::LastTarget};

/// Firmware silently falls back to the v2 profile for an unknown
/// `OWB_IO_PROFILE` (`profile::select_profile`), so a typo would flash the
/// wrong pin map. Reject unknown names up front instead of the bash
/// "warn and use as-is".
fn validate_profile(
    name: &str,
    known: &[String],
) -> Result<String> {
    if known.iter().any(|k| k == name) {
        Ok(name.to_string())
    }
    else {
        Err(format!(
            "unknown IO profile `{name}` (the firmware would silently fall back to owb-esp32s3-v2). \
             Known profiles: {}",
            known.join(", ")
        ))
    }
}

/// `detected` is a profile suggested by probing a connected board; it becomes
/// the default in the interactive menu.
pub fn profile(
    flag: Option<&str>,
    last: Option<&LastTarget>,
    env: &Env,
    known: &[String],
    detected: Option<&str>,
    ask: &mut dyn Ask,
) -> Result<String> {
    let chosen = flag
        .map(str::to_string)
        .or_else(|| last.and_then(|l| l.profile.clone()))
        .or_else(|| env.get("IO_PROFILE").map(str::to_string))
        .or_else(|| env.get("OWB_IO_PROFILE").map(str::to_string));
    if let Some(name) = chosen {
        return validate_profile(&name, known);
    }
    if !ask.interactive() {
        return Err(format!(
            "no IO profile selected and stdin is not a terminal. Pass --profile <name> \
             (or set IO_PROFILE). Known profiles: {}",
            known.join(", ")
        ));
    }
    let default = detected.and_then(|d| known.iter().position(|k| k == d));
    let idx = ask.choose("IO profile", known, default)?;
    Ok(known[idx].clone())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Wifi {
    pub ssid: String,
    pub password: String,
    pub softap: bool,
}

/// Port of `prompt_wifi`. `--force-softap` (or `OWB_FORCE_SOFTAP=1`) blanks
/// the STA credentials so the firmware boots straight into its SoftAP
/// fallback. Otherwise credentials come from `--ssid`/`SSID`/`PASSWORD`
/// (or the `OWB_WIFI_*` names `.env.example` documents); with neither set we
/// prompt on a TTY and error otherwise.
pub fn wifi(
    ssid_flag: Option<&str>,
    force_softap: bool,
    env: &Env,
    ask: &mut dyn Ask,
) -> Result<Wifi> {
    if force_softap || env.is_one("OWB_FORCE_SOFTAP") {
        return Ok(Wifi {
            ssid: String::new(),
            password: String::new(),
            softap: true,
        });
    }
    let ssid = ssid_flag
        .filter(|s| !s.is_empty())
        .or_else(|| env.get("SSID"))
        .or_else(|| env.get("OWB_WIFI_SSID"))
        .unwrap_or_default()
        .to_string();
    let password = env
        .get("PASSWORD")
        .or_else(|| env.get("OWB_WIFI_PASSWORD"))
        .unwrap_or_default()
        .to_string();
    if !ssid.is_empty() || !password.is_empty() {
        return Ok(Wifi {
            ssid,
            password,
            softap: false,
        });
    }
    if !ask.interactive() {
        return Err(
            "no Wi-Fi credentials and stdin is not a terminal. Set SSID and PASSWORD \
                    in the environment (see .env.example) or pass --force-softap."
                .to_string(),
        );
    }
    let ssid = ask.line("Wi-Fi SSID (empty = SoftAP only)")?;
    if ssid.trim().is_empty() {
        return Ok(Wifi {
            ssid: String::new(),
            password: String::new(),
            softap: true,
        });
    }
    let password = ask.secret("Wi-Fi password")?;
    Ok(Wifi {
        ssid,
        password,
        softap: false,
    })
}

/// `--hil-low-mode` / `--normal-drive-mode`. The bash path prompted only in
/// its fully guided flow (no target flags); the equivalent here is "no
/// --profile and no --last". Everyone else gets normal drive by default.
pub fn hil_low_mode(
    hil_flag: bool,
    normal_flag: bool,
    guided: bool,
    ask: &mut dyn Ask,
) -> Result<bool> {
    if hil_flag || normal_flag {
        return Ok(hil_flag);
    }
    if guided && ask.interactive() {
        let opts = ["normal-drive-mode".to_string(), "hil-low-mode".to_string()];
        return Ok(ask.choose("drive mode", &opts, Some(0))? == 1);
    }
    Ok(false)
}

/// Serial port for flash/monitor. `visible` are the ports this process can
/// actually open (inside the container: only those passed with `--device`).
pub fn port(
    flag: Option<&str>,
    last: Option<&LastTarget>,
    visible: &[String],
    ask: &mut dyn Ask,
) -> Result<String> {
    if let Some(p) = flag.or_else(|| last.and_then(|l| l.port.as_deref())) {
        return Ok(p.to_string());
    }
    match visible {
        [] => Err(
            "no serial port found (looked for /dev/ttyACM* and /dev/ttyUSB*). Plug the \
                   board in or pass --port <path>."
                .to_string(),
        ),
        [only] => Ok(only.clone()),
        many if ask.interactive() => Ok(many[ask.choose("serial port", many, None)?].clone()),
        many => Err(format!(
            "several serial ports found ({}) and stdin is not a terminal; pass --port <path>",
            many.join(", ")
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prompt::testing::Scripted;

    fn known() -> Vec<String> {
        ["owb-esp32s3-v1", "owb-esp32s3-v1-alpha", "owb-esp32s3-v2"]
            .map(String::from)
            .to_vec()
    }

    #[test]
    fn profile_precedence() {
        let env = Env::from_pairs([("IO_PROFILE", "owb-esp32s3-v2")]);
        let last = LastTarget {
            profile: Some("owb-esp32s3-v1".into()),
            ..Default::default()
        };
        let mut no = Scripted::closed();
        assert_eq!(
            profile(
                Some("owb-esp32s3-v1-alpha"),
                Some(&last),
                &env,
                &known(),
                None,
                &mut no
            )
            .unwrap(),
            "owb-esp32s3-v1-alpha"
        );
        assert_eq!(
            profile(None, Some(&last), &env, &known(), None, &mut no).unwrap(),
            "owb-esp32s3-v1"
        );
        assert_eq!(
            profile(None, None, &env, &known(), None, &mut no).unwrap(),
            "owb-esp32s3-v2"
        );
    }

    #[test]
    fn profile_without_tty_errors_instead_of_prompting() {
        let err = profile(
            None,
            None,
            &Env::default(),
            &known(),
            None,
            &mut Scripted::closed(),
        )
        .unwrap_err();
        assert!(err.contains("--profile"), "{err}");
        assert!(err.contains("not a terminal"), "{err}");
    }

    #[test]
    fn profile_unknown_name_is_rejected() {
        let err = profile(
            Some("v3"),
            None,
            &Env::default(),
            &known(),
            None,
            &mut Scripted::closed(),
        )
        .unwrap_err();
        assert!(err.contains("unknown IO profile `v3`"), "{err}");
    }

    #[test]
    fn profile_prompt_defaults_to_detected() {
        let mut ask = Scripted::with(&[""]);
        let got = profile(
            None,
            None,
            &Env::default(),
            &known(),
            Some("owb-esp32s3-v1-alpha"),
            &mut ask,
        )
        .unwrap();
        assert_eq!(got, "owb-esp32s3-v1-alpha");
    }

    #[test]
    fn wifi_softap_and_env() {
        let env = Env::from_pairs([("SSID", "lab"), ("PASSWORD", "pw")]);
        let mut no = Scripted::closed();
        let w = wifi(None, true, &env, &mut no).unwrap();
        assert_eq!(
            (w.ssid.as_str(), w.password.as_str(), w.softap),
            ("", "", true)
        );
        let w = wifi(None, false, &env, &mut no).unwrap();
        assert_eq!((w.ssid.as_str(), w.password.as_str()), ("lab", "pw"));
        let w = wifi(
            None,
            false,
            &Env::from_pairs([("OWB_FORCE_SOFTAP", "1"), ("SSID", "x")]),
            &mut no,
        )
        .unwrap();
        assert!(w.softap);
        let w = wifi(
            None,
            false,
            &Env::from_pairs([("OWB_WIFI_SSID", "a"), ("OWB_WIFI_PASSWORD", "b")]),
            &mut no,
        )
        .unwrap();
        assert_eq!((w.ssid.as_str(), w.password.as_str()), ("a", "b"));
    }

    #[test]
    fn wifi_without_tty_errors() {
        let err = wifi(None, false, &Env::default(), &mut Scripted::closed()).unwrap_err();
        assert!(err.contains("--force-softap"), "{err}");
    }

    #[test]
    fn wifi_prompt_and_closed_stdin() {
        let w = wifi(
            None,
            false,
            &Env::default(),
            &mut Scripted::with(&["lab", "pw"]),
        )
        .unwrap();
        assert_eq!((w.ssid.as_str(), w.password.as_str()), ("lab", "pw"));
        // EOF mid-prompt is an error, not an endless loop.
        assert!(wifi(None, false, &Env::default(), &mut Scripted::with(&[])).is_err());
    }

    #[test]
    fn hil_mode_only_prompts_in_guided_tty_flow() {
        assert!(hil_low_mode(true, false, true, &mut Scripted::closed()).unwrap());
        assert!(!hil_low_mode(false, false, true, &mut Scripted::closed()).unwrap());
        assert!(!hil_low_mode(false, false, false, &mut Scripted::with(&[])).unwrap());
        assert!(hil_low_mode(false, false, true, &mut Scripted::with(&["hil-low-mode"])).unwrap());
    }

    #[test]
    fn port_resolution() {
        let ports = vec!["/dev/ttyACM0".to_string(), "/dev/ttyUSB0".to_string()];
        let mut no = Scripted::closed();
        assert_eq!(
            port(Some("/dev/x"), None, &ports, &mut no).unwrap(),
            "/dev/x"
        );
        let last = LastTarget {
            port: Some("/dev/ttyUSB0".into()),
            ..Default::default()
        };
        assert_eq!(
            port(None, Some(&last), &[], &mut no).unwrap(),
            "/dev/ttyUSB0"
        );
        assert_eq!(
            port(None, None, &ports[..1], &mut no).unwrap(),
            "/dev/ttyACM0"
        );
        let err = port(None, None, &ports, &mut no).unwrap_err();
        assert!(err.contains("--port"), "{err}");
        assert!(port(None, None, &[], &mut no)
            .unwrap_err()
            .contains("--port"));
        assert_eq!(
            port(None, None, &ports, &mut Scripted::with(&["/dev/ttyUSB0"])).unwrap(),
            "/dev/ttyUSB0"
        );
    }
}
