<#
.SYNOPSIS
    Configures WSL2 mirrored networking, imports the overhead ROS 2 image, sets firewall rules,
    or runs commands inside the tsa-ros WSL2 distro on Windows 11.

.DESCRIPTION
    Dry-run by default: prints every action it would perform without modifying the system.
    Pass -Apply to execute setup or -Remove changes.
    Pass -Run '<command>' to execute a command inside tsa-ros via overhead/wsl/wsl-entrypoint.sh.
#>
[CmdletBinding()]
param(
    [switch]$Apply,
    [switch]$Remove,
    [switch]$Restart,
    [string]$InstallDir = (Join-Path $env:LOCALAPPDATA "tsa-ros"),
    [string]$RobotSubnet = "10.217.177.0/24",
    [string]$DistroName = "tsa-ros",
    [string]$DockerImage = "tri-state-area/overhead:lyrical",
    [string]$Run
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$FirewallRuleName = "TSA ROS DDS"
$FirewallPortRange = "7400-7700"
$HyperVVMId = "{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}"
$WslConfigPath = Join-Path $HOME ".wslconfig"

function Test-IsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = [Security.Principal.WindowsPrincipal]::new($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function ConvertTo-WslPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$WindowsPath
    )

    $resolved = (Resolve-Path -LiteralPath $WindowsPath).Path
    if ($resolved -match '^([A-Za-z]):\\(.*)$') {
        $drive = $Matches[1].ToLowerInvariant()
        $rest = $Matches[2] -replace '\\', '/'
        return "/mnt/$drive/$rest"
    }

    $prevUtf8 = $env:WSL_UTF8
    $env:WSL_UTF8 = "1"
    try {
        $converted = & wsl.exe -d $DistroName -- wslpath -u $resolved
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to convert Windows path to WSL path: $resolved"
        }
        return ($converted | Select-Object -First 1).Trim()
    }
    finally {
        if ($null -eq $prevUtf8) {
            Remove-Item Env:WSL_UTF8 -ErrorAction SilentlyContinue
        }
        else {
            $env:WSL_UTF8 = $prevUtf8
        }
    }
}

function Get-WslDistros {
    $prevUtf8 = $env:WSL_UTF8
    $env:WSL_UTF8 = "1"
    try {
        $raw = & wsl.exe --list --quiet 2>$null
        if ($LASTEXITCODE -ne 0 -or $null -eq $raw) {
            return @()
        }
        return @(
            $raw |
                ForEach-Object { ($_ -replace "`0", "").Trim() } |
                Where-Object { $_ -ne "" }
        )
    }
    finally {
        if ($null -eq $prevUtf8) {
            Remove-Item Env:WSL_UTF8 -ErrorAction SilentlyContinue
        }
        else {
            $env:WSL_UTF8 = $prevUtf8
        }
    }
}

function Invoke-WslEntrypoint {
    param(
        [string]$CommandText
    )

    $existing = Get-WslDistros
    if ($DistroName -notin $existing) {
        throw "WSL distro '$DistroName' does not exist. Run setup-wsl.ps1 -Apply first."
    }

    $entrypointWinPath = Join-Path $PSScriptRoot "wsl-entrypoint.sh"
    if (-not (Test-Path -LiteralPath $entrypointWinPath)) {
        throw "Entrypoint script not found: $entrypointWinPath"
    }

    $entrypointWslPath = ConvertTo-WslPath -WindowsPath $entrypointWinPath

    $prevUtf8 = $env:WSL_UTF8
    $prevOutEncoding = [Console]::OutputEncoding
    $env:WSL_UTF8 = "1"
    [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
    try {
        if ([string]::IsNullOrWhiteSpace($CommandText)) {
            & wsl.exe -d $DistroName -- bash $entrypointWslPath
        }
        else {
            & wsl.exe -d $DistroName -- bash $entrypointWslPath $CommandText
        }
        exit $LASTEXITCODE
    }
    finally {
        [Console]::OutputEncoding = $prevOutEncoding
        if ($null -eq $prevUtf8) {
            Remove-Item Env:WSL_UTF8 -ErrorAction SilentlyContinue
        }
        else {
            $env:WSL_UTF8 = $prevUtf8
        }
    }
}

function Update-WslConfigLines {
    param(
        [string[]]$Lines
    )

    $result = [System.Collections.Generic.List[string]]::new()
    $inWsl2Section = $false
    $foundWsl2Section = $false
    $updatedKey = $false

    foreach ($line in $Lines) {
        if ($line -match '^\s*\[([^\]]+)\]\s*(?:#.*)?$') {
            if ($inWsl2Section -and -not $updatedKey) {
                $result.Add("networkingMode=mirrored")
                $updatedKey = $true
            }
            $sectionName = $Matches[1].Trim().ToLowerInvariant()
            $inWsl2Section = ($sectionName -eq "wsl2")
            if ($inWsl2Section) {
                $foundWsl2Section = $true
            }
            $result.Add($line)
            continue
        }

        if ($inWsl2Section -and $line -match '^\s*networkingMode\s*=') {
            $result.Add("networkingMode=mirrored")
            $updatedKey = $true
            continue
        }

        $result.Add($line)
    }

    if ($inWsl2Section -and -not $updatedKey) {
        $result.Add("networkingMode=mirrored")
    }
    elseif (-not $foundWsl2Section) {
        if ($result.Count -gt 0 -and $result[$result.Count - 1].Trim() -ne "") {
            $result.Add("")
        }
        $result.Add("[wsl2]")
        $result.Add("networkingMode=mirrored")
    }

    return ,$result.ToArray()
}

function Invoke-StepWslConfig {
    $hasExisting = Test-Path -LiteralPath $WslConfigPath
    $existingLines = if ($hasExisting) {
        [System.IO.File]::ReadAllLines($WslConfigPath)
    }
    else {
        @()
    }

    $inWsl2 = $false
    $alreadyMirrored = $false
    foreach ($line in $existingLines) {
        if ($line -match '^\s*\[([^\]]+)\]\s*(?:#.*)?$') {
            $inWsl2 = ($Matches[1].Trim().ToLowerInvariant() -eq "wsl2")
        }
        elseif ($inWsl2 -and $line -match '^\s*networkingMode\s*=\s*mirrored\s*(?:#.*)?$') {
            $alreadyMirrored = $true
        }
    }

    if ($alreadyMirrored) {
        Write-Host "[SKIP] $WslConfigPath already sets [wsl2] networkingMode=mirrored."
        return $false
    }

    $timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
    $backupPath = "$WslConfigPath.bak-$timestamp"
    $updatedLines = Update-WslConfigLines -Lines $existingLines

    if (-not $Apply) {
        if ($hasExisting) {
            Write-Host "[DRY-RUN] Would back up $WslConfigPath to $backupPath"
        }
        Write-Host "[DRY-RUN] Would set [wsl2] networkingMode=mirrored in $WslConfigPath"
        return $true
    }

    if ($hasExisting) {
        Copy-Item -LiteralPath $WslConfigPath -Destination $backupPath -Force
        Write-Host "[OK] Backed up $WslConfigPath to $backupPath"
    }

    $utf8NoBom = [System.Text.UTF8Encoding]::new($false)
    [System.IO.File]::WriteAllLines($WslConfigPath, $updatedLines, $utf8NoBom)
    Write-Host "[OK] Set [wsl2] networkingMode=mirrored in $WslConfigPath"
    return $true
}

function Invoke-StepImportDistro {
    $existingDistros = Get-WslDistros
    if ($DistroName -in $existingDistros) {
        Write-Host "[SKIP] WSL distro '$DistroName' already exists."
        return
    }

    $tarPath = Join-Path $env:TEMP "tsa-ros-rootfs-$([guid]::NewGuid().ToString('N')).tar"

    if (-not $Apply) {
        Write-Host "[DRY-RUN] Would ensure install directory exists: $InstallDir"
        Write-Host "[DRY-RUN] Would run: docker create $DockerImage"
        Write-Host "[DRY-RUN] Would run: docker export -o $tarPath <container_id>"
        Write-Host "[DRY-RUN] Would run: docker rm <container_id>"
        Write-Host "[DRY-RUN] Would run: wsl --import $DistroName $InstallDir $tarPath --version 2"
        return
    }

    & docker image inspect $DockerImage *> $null
    if ($LASTEXITCODE -ne 0) {
        throw "Docker image '$DockerImage' was not found. Build it first from overhead/docker with: docker compose build"
    }

    New-Item -ItemType Directory -Force -Path $InstallDir | Out-Null

    $containerId = $null
    try {
        $containerId = (& docker create $DockerImage).Trim()
        if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($containerId)) {
            throw "Failed to create temporary container from $DockerImage."
        }

        & docker export -o $tarPath $containerId
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to export container filesystem from $DockerImage."
        }

        & wsl.exe --import $DistroName $InstallDir $tarPath --version 2
        if ($LASTEXITCODE -ne 0) {
            throw "wsl --import failed for distro '$DistroName'."
        }

        Write-Host "[OK] Imported WSL distro '$DistroName' into $InstallDir"
    }
    finally {
        if (-not [string]::IsNullOrWhiteSpace($containerId)) {
            & docker rm -f $containerId *> $null
        }
        if (Test-Path -LiteralPath $tarPath) {
            Remove-Item -LiteralPath $tarPath -Force -ErrorAction SilentlyContinue
        }
    }
}

function Test-FirewallRuleMatches {
    $rule = Get-NetFirewallRule -DisplayName $FirewallRuleName -ErrorAction SilentlyContinue
    if ($null -eq $rule) {
        return $false
    }

    $portFilter = $rule | Get-NetFirewallPortFilter
    $addrFilter = $rule | Get-NetFirewallAddressFilter

    return (
        "$($rule.Enabled)" -eq "True" -and
        "$($rule.Direction)" -eq "Inbound" -and
        "$($rule.Action)" -eq "Allow" -and
        "$($portFilter.Protocol)" -eq "UDP" -and
        (($portFilter.LocalPort -join ",") -eq $FirewallPortRange) -and
        (($addrFilter.RemoteAddress -join ",") -eq $RobotSubnet)
    )
}

function Test-HyperVFirewallMatches {
    try {
        $setting = Get-NetFirewallHyperVVMSetting -Name $HyperVVMId -ErrorAction Stop
    }
    catch {
        return $false
    }

    return (
        $null -ne $setting -and
        "$($setting.Enabled)" -eq "True" -and
        "$($setting.DefaultInboundAction)" -eq "Allow"
    )
}

function Invoke-StepFirewall {
    $ruleExists = $null -ne (Get-NetFirewallRule -DisplayName $FirewallRuleName -ErrorAction SilentlyContinue)
    $ruleMatches = Test-FirewallRuleMatches
    $hvMatches = Test-HyperVFirewallMatches

    if ($ruleMatches -and $hvMatches) {
        Write-Host "[SKIP] Firewall rule '$FirewallRuleName' and Hyper-V VM setting are already configured."
        return
    }

    $isAdmin = Test-IsAdministrator

    if (-not $Apply) {
        if (-not $ruleMatches) {
            $verb = if ($ruleExists) { "update" } else { "create" }
            Write-Host "[DRY-RUN] Would $verb Windows Firewall rule '$FirewallRuleName' (Inbound UDP $FirewallPortRange from $RobotSubnet, Action Allow, Profile Any)."
        }
        if (-not $hvMatches) {
            Write-Host "[DRY-RUN] Would run: Set-NetFirewallHyperVVMSetting -Name '$HyperVVMId' -Enabled True -DefaultInboundAction Allow"
        }
        if (-not $isAdmin) {
            Write-Host "[NOTE] Elevated PowerShell (Run as Administrator) is required for the firewall step when -Apply is used."
        }
        return
    }

    if (-not $isAdmin) {
        throw "Elevated PowerShell (Run as Administrator) is required for the firewall step. Re-run setup-wsl.ps1 -Apply from an Administrator shell."
    }

    if (-not $ruleMatches) {
        if ($ruleExists) {
            Set-NetFirewallRule `
                -DisplayName $FirewallRuleName `
                -Enabled True `
                -Direction Inbound `
                -Action Allow `
                -Profile Any `
                -Protocol UDP `
                -LocalPort $FirewallPortRange `
                -RemoteAddress $RobotSubnet
            Write-Host "[OK] Updated Windows Firewall rule '$FirewallRuleName'."
        }
        else {
            New-NetFirewallRule `
                -DisplayName $FirewallRuleName `
                -Enabled True `
                -Direction Inbound `
                -Action Allow `
                -Profile Any `
                -Protocol UDP `
                -LocalPort $FirewallPortRange `
                -RemoteAddress $RobotSubnet | Out-Null
            Write-Host "[OK] Created Windows Firewall rule '$FirewallRuleName'."
        }
    }

    if (-not $hvMatches) {
        Set-NetFirewallHyperVVMSetting `
            -Name $HyperVVMId `
            -Enabled True `
            -DefaultInboundAction Allow
        Write-Host "[OK] Updated Hyper-V firewall setting for WSL2 mirrored mode."
    }
}

function Invoke-StepRestart {
    param(
        [bool]$ConfigChanged
    )

    if ($Restart) {
        if (-not $Apply) {
            Write-Host "[DRY-RUN] Would run: wsl --shutdown"
            return
        }
        & wsl.exe --shutdown
        if ($LASTEXITCODE -ne 0) {
            throw "wsl --shutdown failed with exit code $LASTEXITCODE."
        }
        Write-Host "[OK] Shut down WSL so configuration changes take effect on next launch."
        return
    }

    if ($ConfigChanged) {
        Write-Host "[INFO] WSL must be restarted for mirrored networking mode to take effect. Run 'wsl --shutdown' (or re-run with -Restart)."
    }
}

function Invoke-Remove {
    $rule = Get-NetFirewallRule -DisplayName $FirewallRuleName -ErrorAction SilentlyContinue
    if ($null -eq $rule) {
        Write-Host "[SKIP] Windows Firewall rule '$FirewallRuleName' does not exist."
    }
    elseif (-not $Apply) {
        Write-Host "[DRY-RUN] Would remove Windows Firewall rule '$FirewallRuleName'."
        if (-not (Test-IsAdministrator)) {
            Write-Host "[NOTE] Elevated PowerShell (Run as Administrator) is required to remove the firewall rule when -Apply is used."
        }
    }
    else {
        if (-not (Test-IsAdministrator)) {
            throw "Elevated PowerShell (Run as Administrator) is required to remove firewall rule '$FirewallRuleName'."
        }
        Remove-NetFirewallRule -DisplayName $FirewallRuleName
        Write-Host "[OK] Removed Windows Firewall rule '$FirewallRuleName'."
    }

    $existingDistros = Get-WslDistros
    if ($DistroName -notin $existingDistros) {
        Write-Host "[SKIP] WSL distro '$DistroName' is not registered."
    }
    elseif (-not $Apply) {
        Write-Host "[DRY-RUN] Would run: wsl --unregister $DistroName"
    }
    else {
        & wsl.exe --unregister $DistroName
        if ($LASTEXITCODE -ne 0) {
            throw "wsl --unregister $DistroName failed with exit code $LASTEXITCODE."
        }
        Write-Host "[OK] Unregistered WSL distro '$DistroName'."
    }

    $wslConfigDir = Split-Path -Parent $WslConfigPath
    $newestBackup = Get-ChildItem -LiteralPath $wslConfigDir -Filter ".wslconfig.bak-*" -File -ErrorAction SilentlyContinue |
        Sort-Object -Property Name -Descending |
        Select-Object -First 1

    if ($null -eq $newestBackup) {
        Write-Host "[SKIP] No .wslconfig.bak-* backup found in $wslConfigDir."
    }
    elseif (-not $Apply) {
        Write-Host "[DRY-RUN] Would restore $($newestBackup.FullName) to $WslConfigPath"
    }
    else {
        Copy-Item -LiteralPath $newestBackup.FullName -Destination $WslConfigPath -Force
        Write-Host "[OK] Restored $($newestBackup.FullName) to $WslConfigPath"
    }

    Invoke-StepRestart -ConfigChanged ($null -ne $newestBackup)
}

if ($PSBoundParameters.ContainsKey("Run")) {
    Invoke-WslEntrypoint -CommandText $Run
}

if (-not $Apply) {
    Write-Host "=== DRY RUN (no changes will be made; pass -Apply to execute) ==="
}

if ($Remove) {
    Invoke-Remove
    exit 0
}

$didChangeConfig = Invoke-StepWslConfig
Invoke-StepImportDistro
Invoke-StepFirewall
Invoke-StepRestart -ConfigChanged $didChangeConfig
