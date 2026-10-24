//! Native serial monitor (replaces `screen $PORT 115200`) and the
//! reset-into-app pulse.
//!
//! Never uses `espflash reset`: on this board it can leave the chip in the
//! ROM download mode. The reset here is the sequence validated on the bench:
//! DTR low, RTS high for 150 ms, RTS low, then reopen the port once the
//! USB Serial/JTAG peripheral has re-enumerated.

use std::{
    io::{Read, Write},
    sync::mpsc,
    thread,
    time::Duration,
};

use serialport::SerialPort;

use crate::run::{note, warn, Result};

fn open(
    path: &str,
    baud: u32,
) -> std::result::Result<Box<dyn SerialPort>, serialport::Error> {
    let mut port = serialport::new(path, baud)
        .timeout(Duration::from_millis(50))
        .open()?;
    // DTR high tells the USB Serial/JTAG peripheral a terminal is attached
    // (otherwise its TX stalls); RTS low keeps the chip out of reset. Same as
    // owb-host-sim's hil-check.
    let _ = port.write_data_terminal_ready(true);
    let _ = port.write_request_to_send(false);
    Ok(port)
}

/// Pulse the chip into a normal boot.
pub fn reset_into_app(path: &str) -> Result<()> {
    let mut port = serialport::new(path, 115_200)
        .timeout(Duration::from_millis(50))
        .open()
        .map_err(|e| format!("opening {path}: {e}"))?;
    port.write_data_terminal_ready(false)
        .map_err(|e| format!("DTR on {path}: {e}"))?;
    port.write_request_to_send(true)
        .map_err(|e| format!("RTS on {path}: {e}"))?;
    thread::sleep(Duration::from_millis(150));
    port.write_request_to_send(false)
        .map_err(|e| format!("RTS on {path}: {e}"))?;
    drop(port);
    // The reset re-enumerates the USB device; give it time to come back.
    thread::sleep(Duration::from_millis(600));
    Ok(())
}

/// Print everything the board sends; forward stdin lines (e.g. the USB JSON
/// control frames) to it. Survives the port disappearing across a reset by
/// reopening it. Runs until Ctrl-C.
pub fn monitor(
    path: &str,
    baud: u32,
) -> Result<()> {
    let mut port = open(path, baud).map_err(|e| format!("opening {path}: {e}"))?;
    note(&format!(
        "Monitoring {path} at {baud} baud. Ctrl-C to exit; typed lines are sent to the board."
    ));

    // stdin on its own thread so a blocking read never stalls the output.
    // At EOF (stdin = /dev/null) the thread just ends; monitoring continues.
    let (tx, rx) = mpsc::channel::<String>();
    thread::spawn(move || {
        let stdin = std::io::stdin();
        let mut line = String::new();
        while matches!(stdin.read_line(&mut line), Ok(n) if n > 0) {
            if tx.send(std::mem::take(&mut line)).is_err() {
                break;
            }
        }
    });

    let mut buf = [0u8; 1024];
    let stdout = std::io::stdout();
    loop {
        while let Ok(line) = rx.try_recv() {
            if let Err(e) = port.write_all(line.as_bytes()) {
                warn(&format!("write to {path} failed: {e}"));
            }
        }
        match port.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                let mut out = stdout.lock();
                let _ = out.write_all(&buf[..n]);
                let _ = out.flush();
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                warn(&format!("{path}: {e}; waiting for the port to come back"));
                port = loop {
                    thread::sleep(Duration::from_millis(250));
                    if let Ok(p) = open(path, baud) {
                        note(&format!("{path} reopened"));
                        break p;
                    }
                };
            }
        }
    }
}
