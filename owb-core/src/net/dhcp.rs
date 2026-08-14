#![allow(dead_code)]

use embassy_net::{
    udp::{PacketMetadata, UdpSocket},
    Ipv4Address,
    Stack,
};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

const SERVER_PORT: u16 = 67;
const CLIENT_PORT: u16 = 68;
const HIL_LOW_MODE_BUILD: bool = option_env!("OWB_HIL_LOW_MODE").is_some();
const DHCP_SOCKET_META_SLOTS: usize = if HIL_LOW_MODE_BUILD { 2 } else { 4 };
const DHCP_SOCKET_BUFFER_BYTES: usize = if HIL_LOW_MODE_BUILD { 512 } else { 1024 };

const DHCP_MAGIC_COOKIE: u32 = 0x6382_5363;

// DHCP message types
const DHCPDISCOVER: u8 = 1;
const DHCPOFFER: u8 = 2;
const DHCPREQUEST: u8 = 3;
const DHCPACK: u8 = 5;

// DHCP options
const OPT_SUBNET_MASK: u8 = 1;
const OPT_ROUTER: u8 = 3;
const OPT_DNS: u8 = 6;
const OPT_REQ_IP: u8 = 50;
const OPT_LEASE_TIME: u8 = 51;
const OPT_MSG_TYPE: u8 = 53;
const OPT_SERVER_ID: u8 = 54;
const OPT_END: u8 = 255;

pub async fn serve_dhcp_ap(
    stack: Stack<'static>,
    server_ip: Ipv4Address,
) -> ! {
    // Single static lease in AP mode
    let yiaddr = Ipv4Address::new(192, 168, 4, 2);
    let subnet = Ipv4Address::new(255, 255, 255, 0);
    let router = server_ip;
    let dns = server_ip;
    let lease_secs: u32 = 3600; // 1 hour

    // UDP socket buffers
    static RX_META: StaticCell<[PacketMetadata; DHCP_SOCKET_META_SLOTS]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; DHCP_SOCKET_META_SLOTS]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; DHCP_SOCKET_BUFFER_BYTES]> = StaticCell::new();
    static TX_BUF: StaticCell<[u8; DHCP_SOCKET_BUFFER_BYTES]> = StaticCell::new();

    let mut sock = UdpSocket::new(
        stack,
        RX_META.init([PacketMetadata::EMPTY; DHCP_SOCKET_META_SLOTS]),
        RX_BUF.init([0; DHCP_SOCKET_BUFFER_BYTES]),
        TX_META.init([PacketMetadata::EMPTY; DHCP_SOCKET_META_SLOTS]),
        TX_BUF.init([0; DHCP_SOCKET_BUFFER_BYTES]),
    );
    match sock.bind(SERVER_PORT) {
        Ok(()) => tracing::info!("dhcp: serving from {} on UDP/{}", server_ip, SERVER_PORT),
        Err(e) => tracing::warn!(?e, "dhcp: bind to port {} failed", SERVER_PORT),
    }

    loop {
        let mut buf = [0u8; 576]; // typical DHCP size
        let (n, _src) = match sock.recv_from(&mut buf).await {
            Ok(x) => x,
            Err(e) => {
                tracing::warn!(?e, "dhcp: recv_from error; retrying");
                continue;
            }
        };
        tracing::info!("dhcp: rx {} bytes", n);
        if n < 244 {
            // BOOTP 236 + cookie 4 + some options
            continue;
        }

        // BOOTP request
        if buf[0] != 1 {
            continue;
        }

        let xid = u32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]);
        let chaddr = &buf[28..44];

        // Validate magic cookie
        let cookie = u32::from_be_bytes([buf[236], buf[237], buf[238], buf[239]]);
        if cookie != DHCP_MAGIC_COOKIE {
            continue;
        }

        // Scan options to find message type and requested IP
        let mut msg_type: u8 = 0;
        let mut _req_ip = Ipv4Address::new(0, 0, 0, 0);
        let mut i = 240usize;
        while i < n {
            let code = buf[i];
            if code == OPT_END {
                break;
            }
            if code == 0 {
                i += 1;
                continue;
            }
            if i + 1 >= n {
                break;
            }
            let len = buf[i + 1] as usize;
            let val_start = i + 2;
            if val_start + len > n {
                break;
            }
            match code {
                OPT_MSG_TYPE if len == 1 => {
                    msg_type = buf[val_start];
                }
                OPT_REQ_IP if len == 4 => {
                    _req_ip = Ipv4Address::new(
                        buf[val_start],
                        buf[val_start + 1],
                        buf[val_start + 2],
                        buf[val_start + 3],
                    );
                }
                _ => {}
            }
            i = val_start + len;
        }

        let reply_type = match msg_type {
            DHCPDISCOVER => {
                tracing::info!("dhcp: DISCOVER -> OFFER {}", yiaddr);
                DHCPOFFER
            }
            DHCPREQUEST => {
                tracing::info!("dhcp: REQUEST -> ACK {}", yiaddr);
                DHCPACK
            }
            other => {
                tracing::debug!(msg_type = other, "dhcp: ignoring non-DISCOVER/REQUEST");
                continue;
            }
        };

        // Build reply packet
        let mut out = [0u8; 576];
        // BOOTP header
        out[0] = 2; // reply
        out[1] = 1; // htype ethernet
        out[2] = 6; // hlen
        out[3] = 0; // hops
        out[4..8].copy_from_slice(&xid.to_be_bytes());
        out[8..10].copy_from_slice(&[0, 0]); // secs
        out[10..12].copy_from_slice(&[0, 0]); // flags
        out[12..16].copy_from_slice(&[0, 0, 0, 0]); // ciaddr
        out[16..20].copy_from_slice(&yiaddr.octets()); // yiaddr
        out[20..24].copy_from_slice(&server_ip.octets()); // siaddr
        out[24..28].copy_from_slice(&[0, 0, 0, 0]); // giaddr
                                                    // chaddr
        out[28..44].copy_from_slice(chaddr);
        // magic cookie
        out[236..240].copy_from_slice(&DHCP_MAGIC_COOKIE.to_be_bytes());

        // Options
        let mut p = 240;
        p = push_opt_u8(&mut out, p, OPT_MSG_TYPE, reply_type);
        p = push_opt_ip(&mut out, p, OPT_SERVER_ID, server_ip);
        p = push_opt_u32(&mut out, p, OPT_LEASE_TIME, lease_secs);
        p = push_opt_ip(&mut out, p, OPT_SUBNET_MASK, subnet);
        p = push_opt_ip(&mut out, p, OPT_ROUTER, router);
        p = push_opt_ip(&mut out, p, OPT_DNS, dns);
        out[p] = OPT_END;
        p += 1;

        // Broadcast to clients
        let dest = (Ipv4Address::new(255, 255, 255, 255), CLIENT_PORT);
        match sock.send_to(&out[..p], dest).await {
            Ok(()) => tracing::info!("dhcp: tx {} bytes to 255.255.255.255:{}", p, CLIENT_PORT),
            Err(e) => tracing::warn!(?e, "dhcp: send_to error"),
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}

fn push_opt_u8(
    buf: &mut [u8],
    p: usize,
    code: u8,
    v: u8,
) -> usize {
    buf[p] = code;
    buf[p + 1] = 1;
    buf[p + 2] = v;
    p + 3
}

fn push_opt_u32(
    buf: &mut [u8],
    p: usize,
    code: u8,
    v: u32,
) -> usize {
    buf[p] = code;
    buf[p + 1] = 4;
    buf[p + 2..p + 6].copy_from_slice(&v.to_be_bytes());
    p + 6
}

fn push_opt_ip(
    buf: &mut [u8],
    p: usize,
    code: u8,
    ip: Ipv4Address,
) -> usize {
    buf[p] = code;
    buf[p + 1] = 4;
    buf[p + 2..p + 6].copy_from_slice(&ip.octets());
    p + 6
}
