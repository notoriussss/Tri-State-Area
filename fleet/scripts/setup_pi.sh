#!/usr/bin/env bash
# Sets up a collector Raspberry Pi 5 (Ubuntu 26.04, ROS 2 Lyrical, lgpio, I2C)
# and builds interfaces + fleet packages in Tri-State-Area.
#
# Run from the repository root on the Pi:
#   bash fleet/scripts/setup_pi.sh
# Then log out/in and reboot so I2C and group permissions apply.
set -euo pipefail

repo_root=$(cd "$(dirname "$0")/../.." && pwd)
config_txt=/boot/firmware/config.txt

sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-gpiozero \
  python3-lgpio \
  python3-smbus2 \
  i2c-tools \
  tmux

if [ -f "$config_txt" ] && ! grep -qx "dtparam=i2c_arm=on" "$config_txt"; then
  echo "dtparam=i2c_arm=on" | sudo tee -a "$config_txt" >/dev/null
fi

for group in gpio i2c video dialout; do
  if getent group "$group" >/dev/null; then
    sudo usermod -aG "$group" "$USER"
  fi
done

cd "$repo_root"
set +u
source /opt/ros/lyrical/setup.bash
set -u

colcon build --symlink-install --packages-up-to fleet_ros

echo "Done. Reboot the Pi (sudo reboot), then test hardware with python3 fleet/scripts/test_hat.py."
