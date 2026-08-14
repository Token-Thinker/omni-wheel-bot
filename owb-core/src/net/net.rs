#[cfg(feature = "net")]
use crate::ext::{net, static_cell::StaticCell};

#[cfg(feature = "net")]
/// Create a network stack and runner with the given driver and resources.
pub fn build_stack<D: net::driver::Driver, const SOCK: usize>(
    driver: D,
    config: net::Config,
    resources: &'static StaticCell<net::StackResources<SOCK>>,
    seed: u64,
) -> (net::Stack<'static>, net::Runner<'static, D>) {
    net::new(
        driver,
        config,
        resources.init(net::StackResources::new()),
        seed,
    )
}

#[cfg(feature = "net")]
/// Convenience DHCPv4 configuration.
pub fn dhcpv4_config() -> net::Config { net::Config::dhcpv4(Default::default()) }

#[cfg(feature = "net")]
/// Build a static IPv4 configuration for SoftAP mode.
pub fn ap_static_config(
    ip: net::Ipv4Address,
    prefix_len: u8,
) -> net::Config {
    let cfg = net::StaticConfigV4 {
        address: net::Ipv4Cidr::new(ip, prefix_len),
        gateway: None,
        dns_servers: Default::default(),
    };
    net::Config::ipv4_static(cfg)
}

#[cfg(feature = "net")]
/// Default SoftAP static configuration: 192.168.4.1/24 with no gateway/DNS.
pub fn ap_default_config() -> net::Config {
    ap_static_config(net::Ipv4Address::new(192, 168, 4, 1), 24)
}
