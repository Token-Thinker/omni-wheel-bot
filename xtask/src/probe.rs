//! Serial port discovery and board identification (port of `probe.sh`).
//!
//! Everything is read from sysfs; no port is ever opened here. Enumerating via
//! `/sys/class/tty` (rather than globbing `/dev`) also works inside the
//! toolchain container, which sees the host's sysfs but only the `/dev` nodes
//! passed with `--device`.

use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PortInfo {
    pub port: String,
    pub vendor_id: String,
    pub product_id: String,
    pub manufacturer: String,
    pub product: String,
    pub serial: String,
    pub driver: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoardMatch {
    pub maker: String,
    pub board: String,
    pub bin: String,
    pub profile: String,
    pub label: String,
    pub source: String,
    pub confidence: &'static str,
}

/// `/dev/ttyACM*` and `/dev/ttyUSB*` (Linux), `/dev/cu.*` (macOS), sorted.
pub fn list_ports() -> Vec<String> {
    let mut ports = Vec::new();
    if let Ok(entries) = std::fs::read_dir("/sys/class/tty") {
        for e in entries.flatten() {
            let name = e.file_name().to_string_lossy().to_string();
            if (name.starts_with("ttyACM") || name.starts_with("ttyUSB"))
                && e.path().join("device").exists()
            {
                ports.push(format!("/dev/{name}"));
            }
        }
    }
    else if let Ok(entries) = std::fs::read_dir("/dev") {
        for e in entries.flatten() {
            let name = e.file_name().to_string_lossy().to_string();
            if name.starts_with("cu.") {
                ports.push(format!("/dev/{name}"));
            }
        }
    }
    ports.sort();
    ports
}

fn read_attr(path: &Path) -> String {
    std::fs::read_to_string(path)
        .map(|s| s.trim().to_string())
        .unwrap_or_default()
}

/// USB identity of `port` from sysfs (empty fields when unavailable).
pub fn port_info(port: &str) -> PortInfo {
    let mut info = PortInfo {
        port: port.to_string(),
        ..Default::default()
    };
    let tty = Path::new(port)
        .file_name()
        .map(|n| n.to_string_lossy().to_string())
        .unwrap_or_default();
    let class = PathBuf::from("/sys/class/tty").join(&tty);
    if let Ok(dev) = std::fs::canonicalize(class.join("device")) {
        let mut dir = Some(dev.as_path());
        while let Some(d) = dir {
            if d.join("idVendor").is_file() && d.join("idProduct").is_file() {
                info.vendor_id = read_attr(&d.join("idVendor"));
                info.product_id = read_attr(&d.join("idProduct"));
                info.manufacturer = read_attr(&d.join("manufacturer"));
                info.product = read_attr(&d.join("product"));
                info.serial = read_attr(&d.join("serial"));
                break;
            }
            dir = d.parent();
        }
    }
    if let Ok(driver) = std::fs::canonicalize(class.join("device/driver")) {
        info.driver = driver
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_default();
    }
    info
}

/// First matching row of a `board_registry*.tsv` file.
pub fn registry_match(
    info: &PortInfo,
    registry: &str,
) -> Option<BoardMatch> {
    for line in registry.lines() {
        let f: Vec<&str> = line.split('\t').collect();
        let field = |i: usize| f.get(i).copied().unwrap_or("").to_string();
        let (kind, value) = (field(0), field(1));
        if kind.is_empty() || kind.starts_with('#') {
            continue;
        }
        let lc = |s: &str| s.to_lowercase();
        let hit = match kind.as_str() {
            "serial" => !info.serial.is_empty() && lc(&info.serial) == lc(&value),
            "vidpid" => {
                !info.vendor_id.is_empty()
                    && !info.product_id.is_empty()
                    && lc(&format!("{}:{}", info.vendor_id, info.product_id)) == lc(&value)
            }
            "port" => info.port == value,
            "product_contains" => {
                !info.product.is_empty() && lc(&info.product).contains(&lc(&value))
            }
            "manufacturer_contains" => {
                !info.manufacturer.is_empty() && lc(&info.manufacturer).contains(&lc(&value))
            }
            _ => false,
        };
        if hit {
            return Some(BoardMatch {
                maker: field(2),
                board: field(3),
                bin: field(4),
                profile: field(5),
                label: field(6),
                source: format!("registry:{kind}"),
                confidence: "high",
            });
        }
    }
    None
}

/// Built-in guesses when no registry row matches.
pub fn heuristic_match(info: &PortInfo) -> Option<BoardMatch> {
    let product = info.product.to_lowercase();
    let manufacturer = info.manufacturer.to_lowercase();
    let m = |board: &str, profile: &str, label: &str, confidence| {
        Some(BoardMatch {
            maker: "owb-esp".into(),
            board: board.into(),
            bin: "owb-esp".into(),
            profile: profile.into(),
            label: label.into(),
            source: "heuristic".into(),
            confidence,
        })
    };
    if manufacturer.contains("seeed") || product.contains("xiao") {
        return m(
            "esp32s3",
            "xiao-seedstudio-sense-cam",
            "Seeed XIAO ESP32S3 Sense",
            "high",
        );
    }
    if product.contains("devkitc-1") {
        return m(
            "esp32s3",
            "esp32-s3-devkitc-1-n32r16",
            "Espressif ESP32-S3-DevKitC-1",
            "high",
        );
    }
    if product.contains("usb jtag/serial debug unit")
        || manufacturer.contains("espressif")
        || product.contains("esp32-s3")
    {
        return m("esp32s3", "", "ESP32-S3 class device", "medium");
    }
    if info.vendor_id == "10c4" || info.vendor_id == "1a86" {
        return m("", "", "Generic USB serial bridge", "low");
    }
    None
}

/// Local (gitignored) registry wins over the tracked one, then heuristics.
pub fn identify(
    repo: &Path,
    info: &PortInfo,
) -> Option<BoardMatch> {
    [
        "config/board_registry.local.tsv",
        "config/board_registry.tsv",
    ]
    .iter()
    .find_map(|f| {
        let text = std::fs::read_to_string(repo.join(f)).ok()?;
        registry_match(info, &text)
    })
    .or_else(|| heuristic_match(info))
}

/// The `print_probe_report` block for one port.
pub fn report(
    info: &PortInfo,
    found: Option<&BoardMatch>,
    visible: bool,
) -> Vec<String> {
    let mut out = vec![format!("Port: {}", info.port)];
    if !visible {
        out.push("Not visible to this process (inside the container, pass --port so the launcher forwards it)".into());
    }
    let mut push = |label: &str, v: &str| {
        if !v.is_empty() {
            out.push(format!("{label}: {v}"));
        }
    };
    push(
        "Detected",
        found.map_or("Unknown serial target", |m| m.label.as_str()),
    );
    if let Some(m) = found {
        push("Match source", &m.source);
        push("Confidence", m.confidence);
    }
    if !info.vendor_id.is_empty() || !info.product_id.is_empty() {
        push(
            "USB VID:PID",
            &format!("{}:{}", info.vendor_id, info.product_id),
        );
    }
    push("Manufacturer", &info.manufacturer);
    push("Product", &info.product);
    push("Serial", &info.serial);
    push("Driver", &info.driver);
    if let Some(m) = found {
        push("Recommended board", &m.board);
        push("Recommended IO profile", &m.profile);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn jtag() -> PortInfo {
        PortInfo {
            port: "/dev/ttyACM0".into(),
            vendor_id: "303a".into(),
            product_id: "1001".into(),
            manufacturer: "Espressif".into(),
            product: "USB JTAG/serial debug unit".into(),
            serial: "90:E5:B1:D0:CE:A4".into(),
            driver: "cdc_acm".into(),
        }
    }

    #[test]
    fn registry_serial_beats_product_rows() {
        let reg = "# kind\tvalue\n\
                   product_contains\tUSB JTAG/serial debug unit\towb-esp\tesp32s3\towb-esp\t\tESP32-S3 class device\n";
        let local =
            "serial\t90:e5:b1:d0:ce:a4\towb-esp\tesp32s3\towb-esp\towb-esp32s3-v1\tBench unit\n";
        let m = registry_match(&jtag(), local).unwrap();
        assert_eq!(m.profile, "owb-esp32s3-v1");
        assert_eq!(m.source, "registry:serial");
        let m = registry_match(&jtag(), reg).unwrap();
        assert_eq!(m.profile, "");
        assert_eq!(m.label, "ESP32-S3 class device");
    }

    #[test]
    fn real_registry_parses() {
        let reg = include_str!("../../config/board_registry.tsv");
        let m = registry_match(&jtag(), reg).unwrap();
        assert_eq!(m.board, "esp32s3");
    }

    #[test]
    fn heuristics() {
        assert_eq!(heuristic_match(&jtag()).unwrap().confidence, "medium");
        let xiao = PortInfo {
            manufacturer: "Seeed Studio".into(),
            ..Default::default()
        };
        assert_eq!(
            heuristic_match(&xiao).unwrap().profile,
            "xiao-seedstudio-sense-cam"
        );
        let ch340 = PortInfo {
            vendor_id: "1a86".into(),
            ..Default::default()
        };
        assert_eq!(heuristic_match(&ch340).unwrap().confidence, "low");
        assert!(heuristic_match(&PortInfo::default()).is_none());
    }
}
