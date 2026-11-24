# Overhead ROS 2 on Windows 11 via WSL2 Mirrored Networking

## Why WSL2 Mirrored + Static Peers Instead of Docker Desktop

Docker Desktop on Windows 11 runs containers inside a separate NAT/bridge VM. Fast DDS packets and advertised RTPS unicast locators from inside Docker Desktop do not map cleanly to the Windows Wi-Fi adapter, so the collector robots (`phineas`, `ferb`) and the overhead PC cannot reliably discover each other or exchange `/world/state` and `/<robot>/plan` over Wi-Fi.

Using a dedicated WSL2 distro (`tsa-ros`, imported from `tri-state-area/overhead:lyrical`) avoids that gap:

1. **WSL2 `networkingMode=mirrored`** mirrors the Windows host Wi-Fi interface directly into WSL2, so the distro uses the real Wi-Fi IP and DDS locators are reachable from the robots.
2. **Fast DDS `ROS_STATIC_PEERS`** (`10.217.177.13;10.217.177.198`, set only on the PC in `overhead/wsl/robots.env`) performs unicast discovery directly to both robots even when the campus Wi-Fi blocks multicast.

## Files

- `robots.env`: sets `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and `ROS_STATIC_PEERS`.
- `setup-wsl.ps1`: idempotent Windows setup, rollback (`-Remove`), and command runner (`-Run`). Dry-run by default; requires `-Apply` to make changes.
- `wsl-entrypoint.sh`: runs inside `tsa-ros`, sources ROS 2 Lyrical and `robots.env`, builds `interfaces/` and `overhead/` into `~/tsa-ws` (never inside the Windows repo), and executes the requested command.

## Step-by-Step Setup

1. **Build the Docker image** (from `overhead/docker` on Windows):

   ```powershell
   docker compose build
   ```

2. **Preview setup actions (dry run)**:

   ```powershell
   cd overhead\wsl
   .\setup-wsl.ps1
   ```

3. **Apply setup** in an **elevated PowerShell** (Run as Administrator, required for the Windows Firewall and Hyper-V firewall step):

   ```powershell
   .\setup-wsl.ps1 -Apply
   ```

   Pass `-InstallDir <path>` to override `%LOCALAPPDATA%\tsa-ros` or `-RobotSubnet <cidr>` to override `10.217.177.0/24`.

4. **Restart WSL** so `networkingMode=mirrored` in `~/.wslconfig` takes effect (unless you passed `-Restart` in step 3):

   ```powershell
   wsl --shutdown
   ```

5. **Verify the end-to-end link** with `robot_link_check`:

   ```powershell
   .\setup-wsl.ps1 -Run "ros2 run overhead_ros robot_link_check"
   ```

   To launch the overhead node once the camera and robots are active:

   ```powershell
   .\setup-wsl.ps1 -Run "ros2 launch overhead_ros overhead.launch.py"
   ```

   To open an interactive ROS 2 shell inside `tsa-ros`:

   ```powershell
   .\setup-wsl.ps1 -Run ""
   ```

## Troubleshooting

### Robots are discovered but no messages arrive

Symptom (seen on 2026-11-24): `ros2 topic list` shows `/phineas/status` and `/ferb/status`, but
`robot_link_check` reports `NO_PLAN_TOPIC` and `NO_STATUS_TOPIC`, and `ros2 topic info -v` shows
`_NODE_NAME_UNKNOWN_`. Cause: with Fast DDS builtin transports (shared memory plus UDP over every
interface) the robots never send data to the WSL participant. `wsl-entrypoint.sh` therefore exports
`FASTDDS_DEFAULT_PROFILES_FILE=overhead/wsl/fastdds_udp_only.xml` (UDPv4 only). Discovery needs
several seconds over Wi-Fi, so give any check at least 15 s. `ros2 topic echo` without an explicit
message type gives up after about 1 s; pass the type, for example
`ros2 topic echo --once --spin-time 15 /ferb/status tsa_interfaces/msg/RobotStatus`.


- **`ros2 node list` does not show `collector_node` for both robots:**
  - Verify inside WSL that `collector_node` appears for both robots (`.\setup-wsl.ps1 -Run "ros2 node list"`).
  - Confirm both PC and robots are on the same Wi-Fi subnet (`10.217.177.0/24`) and ping `10.217.177.13` and `10.217.177.198`.
  - Check `overhead/overhead_ros/config/robots.yaml` and `overhead/wsl/robots.env` if robot IPs changed.
- **Hyper-V firewall blocking inbound DDS packets:**
  - In Windows 11 mirrored mode, WSL2 inbound traffic is governed by the Hyper-V firewall even when a standard Windows Firewall rule exists. Verify in an elevated PowerShell:
    ```powershell
    Get-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
    ```
  - Confirm `Enabled : True` and `DefaultInboundAction : Allow`.
- **Windows network profile set to `Public` instead of `Private`:**
  - Windows often classifies campus Wi-Fi as `Public`, which drops unsolicited inbound traffic unless rules apply to `Any` profile. `setup-wsl.ps1` creates `'TSA ROS DDS'` on `Profile Any`, or you can inspect profiles with `Get-NetConnectionProfile`.
- **Robot web endpoint unreachable (`WEB_UNREACHABLE`):**
  - Test directly from Windows or WSL: `curl http://10.217.177.13:8081/api/status` and `curl http://10.217.177.198:8081/api/status`.

## Rollback

Preview rollback actions first (dry run):

```powershell
.\setup-wsl.ps1 -Remove
```

Apply rollback in an elevated PowerShell (removes the `'TSA ROS DDS'` firewall rule, unregisters `tsa-ros`, and restores the newest `~/.wslconfig.bak-*` backup):

```powershell
.\setup-wsl.ps1 -Remove -Apply
```

Pass `-Restart` (or run `wsl --shutdown`) afterward so WSL reloads the restored `.wslconfig`.
