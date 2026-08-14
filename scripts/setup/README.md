# owb setup scripts

## Non-sudo serial access for ESP32 targets

Copy the udev rule, reload, and add yourself to the `dialout` group so you can
flash and monitor the owb-esp firmware without `sudo`:

```sh
sudo cp scripts/setup/99-owb-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER
# Log out and back in (or reboot) for the group change to take effect.
```

Covers: ESP32-S3 native USB-JTAG (303a:1001), CP210x UART bridge (10c4:ea60),
and CH340 UART bridge (1a86:7523).
