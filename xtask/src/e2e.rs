//! Host-sim end-to-end smoke test (port of `scripts/e2e_host_sim.sh`).
//!
//! Starts `owb-host-sim` on a side port, waits for `/healthz`, then checks
//! every route the UI depends on plus one WebSocket command round-trip. Uses
//! a tiny std-only HTTP/WebSocket client so there is no dependency on `curl`
//! or `websocat` (the bash version silently skipped the WebSocket check when
//! `websocat` was missing).

use std::{
    io::{Read, Write},
    net::TcpStream,
    path::Path,
    process::Child,
    time::{Duration, Instant},
};

use crate::run::{note, ok, Cmd, Result};

/// What a route's body must look like.
#[derive(Debug, Clone, Copy)]
pub enum Expect {
    /// Any 2xx.
    Ok,
    /// Body, trimmed, equals this.
    Exact(&'static str),
    /// Body contains this.
    Contains(&'static str),
}

/// The route list from the bash script.
pub const CHECKS: &[(&str, Expect)] = &[
    ("/healthz", Expect::Exact("ok")),
    ("/", Expect::Ok),
    ("/app-config.js", Expect::Contains("hasCamera")),
    ("/embedded", Expect::Ok),
    ("/embedded/setup", Expect::Ok),
    ("/embedded-host", Expect::Ok),
    ("/embedded-host/setup", Expect::Ok),
    // Static assets come from owb-ui/dist/ via the ServeDir fallback. The
    // Leptos build emits app.js + app.wasm; there is no script.js.
    ("/style.css", Expect::Ok),
    ("/app.js", Expect::Ok),
    ("/app.wasm", Expect::Ok),
    ("/api/schema", Expect::Contains("\"system_command\"")),
    ("/api/schema", Expect::Contains("\"/ws\"")),
];

pub struct Response {
    pub status: u16,
    pub body: Vec<u8>,
}

/// Split a raw HTTP/1.1 response into status + (de-chunked) body.
pub fn parse_response(raw: &[u8]) -> Result<Response> {
    let split = raw
        .windows(4)
        .position(|w| w == b"\r\n\r\n")
        .ok_or("malformed HTTP response (no header terminator)")?;
    let head = String::from_utf8_lossy(&raw[..split]);
    let mut body = raw[split + 4..].to_vec();
    let status = head
        .lines()
        .next()
        .and_then(|l| l.split_whitespace().nth(1))
        .and_then(|s| s.parse().ok())
        .ok_or("malformed HTTP status line")?;
    let chunked = head.lines().any(|l| {
        let l = l.to_ascii_lowercase();
        l.starts_with("transfer-encoding:") && l.contains("chunked")
    });
    if chunked {
        let mut out = Vec::new();
        let mut rest = body.as_slice();
        loop {
            let eol = rest
                .windows(2)
                .position(|w| w == b"\r\n")
                .ok_or("truncated chunk header")?;
            let size_str = String::from_utf8_lossy(&rest[..eol]);
            let size = usize::from_str_radix(size_str.split(';').next().unwrap_or("").trim(), 16)
                .map_err(|_| format!("bad chunk size `{size_str}`"))?;
            rest = &rest[eol + 2..];
            if size == 0 {
                break;
            }
            if rest.len() < size {
                return Err("truncated chunk".to_string());
            }
            out.extend_from_slice(&rest[..size]);
            rest = rest.get(size + 2..).unwrap_or(&[]);
        }
        body = out;
    }
    Ok(Response { status, body })
}

pub fn get(
    addr: &str,
    path: &str,
) -> Result<Response> {
    let mut stream = TcpStream::connect(addr).map_err(|e| format!("connect {addr}: {e}"))?;
    stream
        .set_read_timeout(Some(Duration::from_secs(10)))
        .map_err(|e| e.to_string())?;
    write!(
        stream,
        "GET {path} HTTP/1.1\r\nHost: {addr}\r\nConnection: close\r\nUser-Agent: xtask-e2e\r\n\r\n"
    )
    .map_err(|e| format!("send: {e}"))?;
    let mut raw = Vec::new();
    stream
        .read_to_end(&mut raw)
        .map_err(|e| format!("read: {e}"))?;
    parse_response(&raw)
}

pub fn check(
    addr: &str,
    path: &str,
    expect: Expect,
) -> Result<()> {
    let resp = get(addr, path)?;
    if !(200..300).contains(&resp.status) {
        return Err(format!("HTTP {}", resp.status));
    }
    let body = String::from_utf8_lossy(&resp.body);
    match expect {
        Expect::Ok => Ok(()),
        Expect::Exact(want) if body.trim() == want => Ok(()),
        Expect::Contains(want) if body.contains(want) => Ok(()),
        Expect::Exact(want) | Expect::Contains(want) => Err(format!("body did not match `{want}`")),
    }
}

/// Minimal client-side text frame (masked, as RFC 6455 requires).
pub fn ws_text_frame(payload: &[u8]) -> Vec<u8> {
    let mask = [0x37, 0xfa, 0x21, 0x3d];
    let mut frame = vec![0x81];
    match payload.len() {
        n if n < 126 => frame.push(0x80 | n as u8),
        n if n <= u16::MAX as usize => {
            frame.push(0x80 | 126);
            frame.extend_from_slice(&(n as u16).to_be_bytes());
        }
        n => {
            frame.push(0x80 | 127);
            frame.extend_from_slice(&(n as u64).to_be_bytes());
        }
    }
    frame.extend_from_slice(&mask);
    frame.extend(payload.iter().enumerate().map(|(i, b)| b ^ mask[i % 4]));
    frame
}

fn read_exact(
    s: &mut TcpStream,
    n: usize,
) -> Result<Vec<u8>> {
    let mut buf = vec![0; n];
    s.read_exact(&mut buf)
        .map_err(|e| format!("ws read: {e}"))?;
    Ok(buf)
}

/// Send `{"ct":"i","ic":"enable"}` on `/ws` and wait for a `"status":"ok"`
/// reply (skipping the greeting and any telemetry broadcasts).
pub fn ws_roundtrip(addr: &str) -> Result<()> {
    let mut s = TcpStream::connect(addr).map_err(|e| format!("connect {addr}: {e}"))?;
    s.set_read_timeout(Some(Duration::from_secs(5)))
        .map_err(|e| e.to_string())?;
    write!(
        s,
        "GET /ws HTTP/1.1\r\nHost: {addr}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n\
         Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n"
    )
    .map_err(|e| format!("ws handshake: {e}"))?;
    let mut head = Vec::new();
    while !head.ends_with(b"\r\n\r\n") {
        head.extend(read_exact(&mut s, 1)?);
        if head.len() > 8192 {
            return Err("ws handshake response too large".to_string());
        }
    }
    let head = String::from_utf8_lossy(&head);
    if !head.starts_with("HTTP/1.1 101") {
        return Err(format!(
            "ws upgrade refused: {}",
            head.lines().next().unwrap_or("")
        ));
    }
    s.write_all(&ws_text_frame(br#"{"ct":"i","ic":"enable"}"#))
        .map_err(|e| format!("ws send: {e}"))?;

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut seen = Vec::new();
    while Instant::now() < deadline {
        let hdr = read_exact(&mut s, 2)?;
        let opcode = hdr[0] & 0x0f;
        let len = match hdr[1] & 0x7f {
            126 => u16::from_be_bytes(read_exact(&mut s, 2)?.try_into().unwrap()) as usize,
            127 => u64::from_be_bytes(read_exact(&mut s, 8)?.try_into().unwrap()) as usize,
            n => n as usize,
        };
        let payload = read_exact(&mut s, len)?;
        match opcode {
            0x1 => {
                let text = String::from_utf8_lossy(&payload).to_string();
                if text.contains("\"status\":\"ok\"") {
                    return Ok(());
                }
                seen.push(text);
            }
            0x8 => return Err(format!("ws closed by server; got {seen:?}")),
            _ => {}
        }
    }
    Err(format!("no \"status\":\"ok\" reply; got {seen:?}"))
}

/// Kills the server however the test exits.
struct Server(Child);

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

pub fn run(
    repo: &Path,
    addr: &str,
) -> Result<()> {
    // Build first so the 30 s readiness window measures startup, not a cold
    // compile (the bash version's `cargo run` spent it compiling).
    Cmd::new(
        "cargo",
        ["build", "-p", "owb-host-sim", "--bin", "owb-host-sim"],
    )
    .cwd(repo)
    .run(false)?;

    let log_path = std::env::temp_dir().join("owb-host-sim-e2e.log");
    let log = std::fs::File::create(&log_path)
        .map_err(|e| format!("creating {}: {e}", log_path.display()))?;
    let mut cmd = Cmd::new(
        "cargo",
        ["run", "-q", "-p", "owb-host-sim", "--bin", "owb-host-sim"],
    )
    .cwd(repo);
    cmd.env.push(("OWB_BIND_ADDR".into(), addr.to_string()));
    note(&format!(
        "starting host-sim on {addr} (log: {})",
        log_path.display()
    ));
    let mut server = Server(cmd.spawn_with_output(log)?);
    let dump_log = || std::fs::read_to_string(&log_path).unwrap_or_default();

    let mut ready = false;
    for _ in 0..60 {
        if let Ok(Some(status)) = server.0.try_wait() {
            return Err(format!("host-sim exited early ({status}):\n{}", dump_log()));
        }
        if check(addr, "/healthz", Expect::Exact("ok")).is_ok() {
            ready = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(500));
    }
    if !ready {
        return Err(format!(
            "host-sim never became healthy on {addr}:\n{}",
            dump_log()
        ));
    }

    let mut failures = Vec::new();
    for (path, expect) in CHECKS {
        match check(addr, path, *expect) {
            Ok(()) => println!("ok   {path}"),
            Err(e) => {
                println!("FAIL {path}: {e}");
                failures.push(format!("{path}: {e}"));
            }
        }
    }
    match ws_roundtrip(addr) {
        Ok(()) => println!("ok   /ws (enable -> status ok)"),
        Err(e) => {
            println!("FAIL /ws: {e}");
            failures.push(format!("/ws: {e}"));
        }
    }
    if !failures.is_empty() {
        return Err(format!("host-sim e2e failed:\n  {}", failures.join("\n  ")));
    }
    ok(&format!("host-sim e2e passed (http://{addr})"));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_plain_and_chunked_responses() {
        let r = parse_response(b"HTTP/1.1 200 OK\r\ncontent-length: 2\r\n\r\nok").unwrap();
        assert_eq!((r.status, r.body.as_slice()), (200, &b"ok"[..]));
        let r = parse_response(
            b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n4\r\nhasC\r\n5\r\namera\r\n0\r\n\r\n",
        )
        .unwrap();
        assert_eq!(r.body, b"hasCamera");
        assert_eq!(
            parse_response(b"HTTP/1.1 404 Not Found\r\n\r\n")
                .unwrap()
                .status,
            404
        );
        assert!(parse_response(b"garbage").is_err());
    }

    #[test]
    fn ws_frame_is_masked_and_sized() {
        let f = ws_text_frame(b"hi");
        assert_eq!(f[0], 0x81);
        assert_eq!(f[1], 0x80 | 2);
        let mask = &f[2..6];
        assert_eq!(f[6] ^ mask[0], b'h');
        assert_eq!(f[7] ^ mask[1], b'i');
        let big = ws_text_frame(&[b'a'; 300]);
        assert_eq!(big[1], 0x80 | 126);
        assert_eq!(u16::from_be_bytes([big[2], big[3]]), 300);
    }
}
