//! Interactive prompts — only ever used when stdin is a terminal.
//!
//! The bash deployer prompted unconditionally; with stdin at EOF its menu
//! loop re-read an empty answer forever (100% of a core, never exiting). Here
//! every prompt is behind [`Ask::interactive`], callers turn "not
//! interactive" into a clear error naming the flag to pass, and a closed stdin
//! is an error rather than an empty answer.

use std::{
    io::{BufRead, IsTerminal, Write},
    process::{Command, Stdio},
};

use crate::run::Result;

pub trait Ask {
    /// May we prompt at all?
    fn interactive(&self) -> bool;
    /// Pick one of `options`; returns its index.
    fn choose(
        &mut self,
        title: &str,
        options: &[String],
        default: Option<usize>,
    ) -> Result<usize>;
    fn line(
        &mut self,
        prompt: &str,
    ) -> Result<String>;
    fn secret(
        &mut self,
        prompt: &str,
    ) -> Result<String>;
}

/// The real terminal. `interactive` is decided once, up front: stdin and
/// stderr (where prompts go) must both be TTYs.
pub struct Terminal {
    interactive: bool,
}

impl Terminal {
    pub fn detect() -> Self {
        Terminal {
            interactive: std::io::stdin().is_terminal() && std::io::stderr().is_terminal(),
        }
    }
}

fn read_answer() -> Result<String> {
    let mut buf = String::new();
    let n = std::io::stdin()
        .lock()
        .read_line(&mut buf)
        .map_err(|e| format!("reading stdin: {e}"))?;
    if n == 0 {
        return Err("stdin closed while waiting for an answer".to_string());
    }
    Ok(buf.trim_end_matches(['\n', '\r']).to_string())
}

impl Ask for Terminal {
    fn interactive(&self) -> bool { self.interactive }

    fn choose(
        &mut self,
        title: &str,
        options: &[String],
        default: Option<usize>,
    ) -> Result<usize> {
        if !self.interactive {
            return Err(format!(
                "cannot prompt for {title}: stdin is not a terminal"
            ));
        }
        if options.is_empty() {
            return Err(format!("no options to choose {title} from"));
        }
        eprintln!("\x1b[36;1m▶ Select {title}\x1b[0m");
        for (i, opt) in options.iter().enumerate() {
            let mark = if Some(i) == default { " (default)" } else { "" };
            eprintln!("  {}) {opt}{mark}", i + 1);
        }
        for _ in 0..3 {
            eprint!("\x1b[33mChoice [1-{}]: \x1b[0m", options.len());
            let _ = std::io::stderr().flush();
            let answer = read_answer()?;
            let answer = answer.trim();
            if answer.is_empty() {
                if let Some(d) = default {
                    return Ok(d);
                }
            }
            if let Ok(n) = answer.parse::<usize>() {
                if (1..=options.len()).contains(&n) {
                    return Ok(n - 1);
                }
            }
            if let Some(i) = options.iter().position(|o| o == answer) {
                return Ok(i);
            }
            eprintln!("\x1b[31m✖ Invalid selection.\x1b[0m");
        }
        Err(format!("no valid {title} selected"))
    }

    fn line(
        &mut self,
        prompt: &str,
    ) -> Result<String> {
        if !self.interactive {
            return Err(format!(
                "cannot prompt for {prompt}: stdin is not a terminal"
            ));
        }
        eprint!("\x1b[33m{prompt}: \x1b[0m");
        let _ = std::io::stderr().flush();
        read_answer()
    }

    fn secret(
        &mut self,
        prompt: &str,
    ) -> Result<String> {
        if !self.interactive {
            return Err(format!(
                "cannot prompt for {prompt}: stdin is not a terminal"
            ));
        }
        eprint!("\x1b[33m{prompt}: \x1b[0m");
        let _ = std::io::stderr().flush();
        // Best effort: if `stty` is missing the answer simply echoes.
        let stty = |arg: &str| {
            let _ = Command::new("stty")
                .arg(arg)
                .stdin(Stdio::inherit())
                .status();
        };
        stty("-echo");
        let answer = read_answer();
        stty("echo");
        eprintln!();
        answer
    }
}

#[cfg(test)]
pub mod testing {
    use std::collections::VecDeque;

    use super::*;

    /// Scripted answers for tests. `NoTty` is `Scripted::closed()`.
    pub struct Scripted {
        pub interactive: bool,
        pub answers: VecDeque<String>,
    }

    impl Scripted {
        pub fn closed() -> Self {
            Scripted {
                interactive: false,
                answers: VecDeque::new(),
            }
        }

        pub fn with(answers: &[&str]) -> Self {
            Scripted {
                interactive: true,
                answers: answers.iter().map(|s| s.to_string()).collect(),
            }
        }

        fn next(&mut self) -> Result<String> {
            if !self.interactive {
                panic!("prompted while not interactive");
            }
            self.answers
                .pop_front()
                .ok_or_else(|| "stdin closed".to_string())
        }
    }

    impl Ask for Scripted {
        fn interactive(&self) -> bool { self.interactive }

        fn choose(
            &mut self,
            _title: &str,
            options: &[String],
            default: Option<usize>,
        ) -> Result<usize> {
            let a = self.next()?;
            if a.is_empty() {
                return default.ok_or_else(|| "no default".to_string());
            }
            options
                .iter()
                .position(|o| *o == a)
                .ok_or_else(|| format!("{a} not an option"))
        }

        fn line(
            &mut self,
            _prompt: &str,
        ) -> Result<String> {
            self.next()
        }

        fn secret(
            &mut self,
            _prompt: &str,
        ) -> Result<String> {
            self.next()
        }
    }
}
