#!/usr/bin/env bash
# Idempotent Raspberry Pi setup for Ubuntu 26.04/26.10 arm64:
# installs Docker Engine, grants docker/video group access, and checks the camera.
set -euo pipefail

EXPECTED_ARCH="aarch64"
CAMERA_DEVICE="/dev/video0"
CAMERA_SYSFS_NAME="/sys/class/video4linux/video0/name"
APT_LOCK_TIMEOUT_S=120
DOCKER_PACKAGES=("docker.io" "docker-compose-v2")
REQUIRED_GROUPS=("docker" "video")

os_name="unknown"
if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    os_name="${PRETTY_NAME:-unknown}"
fi

current_arch="$(uname -m)"
echo "Host OS: ${os_name}"
echo "Architecture: ${current_arch}"

if [[ "${current_arch}" != "${EXPECTED_ARCH}" ]]; then
    echo "WARNING: Expected ${EXPECTED_ARCH}, got ${current_arch}."
fi

# Use Ubuntu's native docker.io + docker-compose-v2 packages because they work
# reliably on both Ubuntu 26.04 LTS and interim releases like Ubuntu 26.10
# (stonking) where Docker's upstream apt repository may lack a codename.
needs_docker_install=false
if ! command -v docker >/dev/null 2>&1; then
    needs_docker_install=true
elif ! docker compose version >/dev/null 2>&1; then
    needs_docker_install=true
fi

if [[ "${needs_docker_install}" == "true" ]]; then
    echo "Installing ${DOCKER_PACKAGES[*]} (waiting up to ${APT_LOCK_TIMEOUT_S}s if apt is busy)..."
    sudo apt-get -o DPkg::Lock::Timeout="${APT_LOCK_TIMEOUT_S}" update
    sudo apt-get -o DPkg::Lock::Timeout="${APT_LOCK_TIMEOUT_S}" install -y "${DOCKER_PACKAGES[@]}"
else
    echo "Docker and Docker Compose v2 are already installed."
fi

sudo systemctl enable --now docker

# Grant docker (passwordless CLI) and video (/dev/video0 access) memberships.
current_groups=" $(id -nG "${USER}") "
added_any_group=false
for grp in "${REQUIRED_GROUPS[@]}"; do
    if [[ "${current_groups}" != *" ${grp} "* ]]; then
        echo "Adding ${USER} to group '${grp}'..."
        sudo usermod -aG "${grp}" "${USER}"
        added_any_group=true
    else
        echo "User ${USER} is already in group '${grp}'."
    fi
done

if [[ -e "${CAMERA_DEVICE}" ]]; then
    echo "Camera device found:"
    ls -l "${CAMERA_DEVICE}"
    if [[ -r "${CAMERA_SYSFS_NAME}" ]]; then
        echo "Camera name: $(cat "${CAMERA_SYSFS_NAME}")"
    fi
else
    echo "WARNING: No camera device found at ${CAMERA_DEVICE}."
fi

docker --version
docker compose version

if [[ "${added_any_group}" == "true" ]]; then
    echo "NOTE: Group membership updated; open a new SSH session for changes to apply."
fi
