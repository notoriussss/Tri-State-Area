<#
.SYNOPSIS
Deploys the repository archive via SSH/SCP to the Raspberry Pi and builds the overhead image.
#>
[CmdletBinding()]
param(
    [string]$PiHost = "192.168.137.2",
    [string]$PiUser = "perry",
    [string]$SshKey = "$HOME\.ssh\tsa_overhead",
    [string]$RemoteDir = "~/tri-state-area",
    [string]$GitRef = "HEAD",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

# Connection and remote staging constants
$SSH_CONNECT_TIMEOUT_S = 10
$REMOTE_ARCHIVE_PATH = "/tmp/tsa-repo.tar.gz"
$REMOTE_DOCKER_SUBDIR = "overhead/docker"

if (-not (Test-Path -LiteralPath $SshKey)) {
    throw "SSH key not found at '$SshKey'. Generate it before deploying."
}

$repoRoot = (git rev-parse --show-toplevel).Trim()
if ($LASTEXITCODE -ne 0 -or -not $repoRoot) {
    throw "Failed to locate git repository root."
}

$sshTarget = "${PiUser}@${PiHost}"
$sshCommonArgs = @(
    "-i", $SshKey,
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=$SSH_CONNECT_TIMEOUT_S"
)

$localArchive = Join-Path ([System.IO.Path]::GetTempPath()) ("tsa-deploy-{0}.tar.gz" -f ([guid]::NewGuid().ToString("N")))

try {
    Write-Host "Creating archive of $GitRef from $repoRoot..."
    Push-Location $repoRoot
    try {
        git archive --format=tar.gz -o $localArchive $GitRef
        if ($LASTEXITCODE -ne 0) {
            throw "git archive failed with exit code $LASTEXITCODE."
        }
    }
    finally {
        Pop-Location
    }

    Write-Host "Uploading archive to ${sshTarget}:${REMOTE_ARCHIVE_PATH}..."
    & scp @sshCommonArgs $localArchive "${sshTarget}:${REMOTE_ARCHIVE_PATH}"
    if ($LASTEXITCODE -ne 0) {
        throw "scp failed with exit code $LASTEXITCODE."
    }

    # Extract cleanly into RemoteDir and remove the staging tarball on the Pi
    $extractCmd = "mkdir -p $RemoteDir && tar -xzf $REMOTE_ARCHIVE_PATH -C $RemoteDir && rm -f $REMOTE_ARCHIVE_PATH"
    Write-Host "Extracting archive into $RemoteDir on $PiHost..."
    & ssh @sshCommonArgs $sshTarget $extractCmd
    if ($LASTEXITCODE -ne 0) {
        throw "Remote extraction failed with exit code $LASTEXITCODE."
    }

    if (-not $SkipBuild) {
        $buildCmd = "cd $RemoteDir/$REMOTE_DOCKER_SUBDIR && docker compose build"
        Write-Host "Running docker compose build on $PiHost..."
        & ssh @sshCommonArgs $sshTarget $buildCmd
        if ($LASTEXITCODE -ne 0) {
            throw "Remote docker compose build failed with exit code $LASTEXITCODE."
        }
    }

    Write-Host "Deploy completed successfully."
}
finally {
    # Always remove the temporary local archive even when transfer or build fails
    if (Test-Path -LiteralPath $localArchive) {
        Remove-Item -LiteralPath $localArchive -Force -ErrorAction SilentlyContinue
    }
}
