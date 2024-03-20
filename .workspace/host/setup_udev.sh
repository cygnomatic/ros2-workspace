echo 'SUBSYSTEMS=="usb", ATTR{idVendor}=="2bdf", MODE="0777"' | sudo tee -a /usr/lib/udev/rules.d/65-hik-camera.rules > /dev/null
echo 'KERNEL=="ttyTHS0", SUBSYSTEM=="tty", DRIVERS=="serial-tegra", MODE="0777"' | sudo tee -a /usr/lib/udev/rules.d/65-tegra-serial.rules > /dev/null
