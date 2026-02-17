#!/usr/bin/env pwsh
# ============================================================================
# Kronk Pi Tool - Team 4539
# All-in-one Raspberry Pi management for the Kronk robot coprocessor.
# ============================================================================

param(
    [string]$PiAddress = "10.45.39.11",
    [string]$PiUser = "kaotic",
    [string]$PiPath = "/home/kaotic/kronk"
)

$ErrorActionPreference = "Stop"
$PiPassword = "4539"
$TargetPiIP = "10.45.39.11"
$piFolder = Join-Path $PSScriptRoot "pi"
$backupFolder = Join-Path $PSScriptRoot "pi-backup"

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

function Show-Header {
    param([string]$Title)
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host "  $Title" -ForegroundColor Cyan
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host ""
}

function Test-PiConnection {
    Write-Host "Testing connection to $PiUser@$PiAddress..." -ForegroundColor Cyan
    try {
        $result = ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$PiUser@$PiAddress" "echo 'OK'" 2>&1
        if ($result -match "OK") {
            Write-Host "[OK] Pi is reachable" -ForegroundColor Green
            return $true
        }
    } catch {}
    Write-Host "[FAIL] Cannot connect to Pi at $PiAddress" -ForegroundColor Red
    Write-Host "  - Is the Pi powered on?" -ForegroundColor Yellow
    Write-Host "  - Are you on the robot network (10.45.39.x)?" -ForegroundColor Yellow
    Write-Host "  - Is SSH enabled on the Pi?" -ForegroundColor Yellow
    return $false
}

function Confirm-Action {
    param([string]$Message)
    Write-Host "$Message" -ForegroundColor Yellow
    Write-Host "Press ENTER to continue or Ctrl+C to cancel..." -ForegroundColor Gray
    Read-Host
}

# ============================================================================
# 1) DEPLOY - Push pi/ folder to Pi
# ============================================================================

function Invoke-Deploy {
    Show-Header "Deploy Pi Code"

    if (-not (Test-Path $piFolder)) {
        Write-Host "[FAIL] pi/ folder not found at $piFolder" -ForegroundColor Red
        return
    }

    Write-Host "Source:  $piFolder" -ForegroundColor White
    Write-Host "Target:  $PiUser@$PiAddress`:$PiPath" -ForegroundColor White
    Write-Host ""

    if (-not (Test-PiConnection)) { return }

    # Create dir, copy files
    Write-Host ""
    Write-Host "Copying files..." -ForegroundColor Cyan
    ssh "$PiUser@$PiAddress" "mkdir -p $PiPath"
    scp -r "$piFolder/*" "$PiUser@$PiAddress`:$PiPath/"

    if ($LASTEXITCODE -ne 0) {
        Write-Host "[FAIL] File copy failed!" -ForegroundColor Red
        return
    }
    Write-Host "[OK] Files copied" -ForegroundColor Green

    # Permissions + deps
    Write-Host "Setting permissions..." -ForegroundColor Cyan
    ssh "$PiUser@$PiAddress" "chmod +x $PiPath/*.py"

    Write-Host "Installing dependencies..." -ForegroundColor Cyan
    ssh "$PiUser@$PiAddress" "cd $PiPath && pip3 install -r requirements.txt --quiet 2>&1" | Out-Null

    # Restart service
    Write-Host ""
    Write-Host "Restarting Pi service..." -ForegroundColor Cyan
    $piPid = ssh "$PiUser@$PiAddress" "pgrep -f pi_shooting.py 2>/dev/null"

    if ($piPid) {
        ssh "$PiUser@$PiAddress" "pkill -f pi_shooting.py; sleep 1; cd $PiPath && nohup python3 pi_shooting.py > /dev/null 2>&1 &"
        Write-Host "[OK] Service restarted (was PID $piPid)" -ForegroundColor Green
    } else {
        ssh "$PiUser@$PiAddress" "cd $PiPath && nohup python3 pi_shooting.py > /dev/null 2>&1 &"
        Write-Host "[OK] Service started (was not running)" -ForegroundColor Green
    }

    Write-Host ""
    Write-Host "[OK] Deployment complete!" -ForegroundColor Green
    Write-Host "  Check logs: ssh $PiUser@$PiAddress 'tail -f $PiPath/pi_shooting.log'" -ForegroundColor White
}

# ============================================================================
# 2) PULL - Download files from Pi
# ============================================================================

function Invoke-Pull {
    Show-Header "Pull Data from Pi"

    Write-Host "What do you want to pull?" -ForegroundColor White
    Write-Host "  [1] Training CSV only" -ForegroundColor White
    Write-Host "  [2] Log files" -ForegroundColor White
    Write-Host "  [3] Trained model (.pkl)" -ForegroundColor White
    Write-Host "  [4] Everything" -ForegroundColor White
    Write-Host ""
    Write-Host "Choice: " -ForegroundColor Yellow -NoNewline
    $pullChoice = Read-Host

    if (-not (Test-PiConnection)) { return }

    New-Item -ItemType Directory -Force -Path $piFolder | Out-Null
    New-Item -ItemType Directory -Force -Path $backupFolder | Out-Null

    switch ($pullChoice) {
        "1" { Pull-CSV }
        "2" { Pull-Logs }
        "3" { Pull-Model }
        "4" {
            Pull-CSV
            Pull-Logs
            Pull-Model
        }
        default {
            Write-Host "Invalid choice" -ForegroundColor Red
        }
    }
}

function Pull-CSV {
    Write-Host ""
    Write-Host "Pulling training CSV..." -ForegroundColor Cyan

    $csvLocations = @(
        "$PiPath/shooting_training_data.csv",
        "/home/lvuser/deploy/shooting_training_data.csv"
    )

    foreach ($csvPath in $csvLocations) {
        $exists = ssh "$PiUser@$PiAddress" "test -f $csvPath && echo 1 || echo 0"
        if ($exists -eq "1") {
            $localCsv = Join-Path $piFolder "shooting_training_data.csv"

            # Backup existing
            if (Test-Path $localCsv) {
                $timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
                Copy-Item $localCsv (Join-Path $backupFolder "training_data_$timestamp.csv")
                Write-Host "  Backed up existing CSV" -ForegroundColor Yellow
            }

            scp "$PiUser@$PiAddress`:$csvPath" $localCsv
            Write-Host "[OK] CSV downloaded from $csvPath" -ForegroundColor Green

            # Stats
            $lines = (Get-Content $localCsv | Measure-Object -Line).Lines
            $hits = (Select-String -Path $localCsv -Pattern ",HIT" | Measure-Object).Count
            $misses = (Select-String -Path $localCsv -Pattern ",MISS" | Measure-Object).Count
            Write-Host "  Total: $lines lines | HITs: $hits | MISSes: $misses" -ForegroundColor White
            return
        }
    }
    Write-Host "  No CSV found on Pi (no training data yet)" -ForegroundColor Yellow
}

function Pull-Logs {
    Write-Host ""
    Write-Host "Pulling log files..." -ForegroundColor Cyan
    $logs = ssh "$PiUser@$PiAddress" "ls $PiPath/*.log 2>/dev/null || echo ''"

    if ($logs) {
        $logDest = Join-Path $piFolder "logs"
        New-Item -ItemType Directory -Force -Path $logDest | Out-Null
        scp "$PiUser@$PiAddress`:$PiPath/*.log" "$logDest/"
        Write-Host "[OK] Logs downloaded to $logDest" -ForegroundColor Green
    } else {
        Write-Host "  No log files found" -ForegroundColor Yellow
    }
}

function Pull-Model {
    Write-Host ""
    Write-Host "Pulling trained model..." -ForegroundColor Cyan
    $modelPath = "$PiPath/shooting_model.pkl"
    $exists = ssh "$PiUser@$PiAddress" "test -f $modelPath && echo 1 || echo 0"

    if ($exists -eq "1") {
        $localModel = Join-Path $piFolder "shooting_model.pkl"

        if (Test-Path $localModel) {
            $timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
            Copy-Item $localModel (Join-Path $backupFolder "model_$timestamp.pkl")
            Write-Host "  Backed up existing model" -ForegroundColor Yellow
        }

        scp "$PiUser@$PiAddress`:$modelPath" $localModel
        Write-Host "[OK] Model downloaded" -ForegroundColor Green
    } else {
        Write-Host "  No trained model found (run training first)" -ForegroundColor Yellow
    }
}

# ============================================================================
# 3) CLEAR MODEL - Delete trained model to force fallback
# ============================================================================

function Invoke-ClearModel {
    Show-Header "Clear Trained Model"

    Write-Host "This will DELETE the trained model from the Pi." -ForegroundColor Yellow
    Write-Host "The Pi will fall back to calibration tables only." -ForegroundColor Yellow
    Confirm-Action ""

    if (-not (Test-PiConnection)) { return }

    $modelPath = "$PiPath/shooting_model.pkl"
    $exists = ssh "$PiUser@$PiAddress" "test -f $modelPath && echo 1 || echo 0"

    if ($exists -eq "1") {
        # Backup first
        $timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
        ssh "$PiUser@$PiAddress" "cp $modelPath ${modelPath}_backup_$timestamp"
        Write-Host "  Backed up model on Pi" -ForegroundColor Yellow

        ssh "$PiUser@$PiAddress" "rm $modelPath"
        Write-Host "[OK] Model deleted" -ForegroundColor Green

        # Try to restart service
        $svcActive = ssh "$PiUser@$PiAddress" "sudo systemctl is-active kronk-shooting 2>/dev/null || echo inactive"
        if ($svcActive -eq "active") {
            ssh "$PiUser@$PiAddress" "sudo systemctl restart kronk-shooting"
            Write-Host "[OK] Service restarted" -ForegroundColor Green
        } else {
            $piPid = ssh "$PiUser@$PiAddress" "pgrep -f pi_shooting.py 2>/dev/null"
            if ($piPid) {
                ssh "$PiUser@$PiAddress" "pkill -f pi_shooting.py; sleep 1; cd $PiPath && nohup python3 pi_shooting.py > /dev/null 2>&1 &"
                Write-Host "[OK] Process restarted" -ForegroundColor Green
            }
        }

        Write-Host ""
        Write-Host "Pi is now using ONLY calibration tables." -ForegroundColor Green
        Write-Host "Check: Pi/Status/model_loaded should be FALSE" -ForegroundColor White
    } else {
        Write-Host "No model file found -- Pi already uses calibration tables only." -ForegroundColor Yellow
    }
}

# ============================================================================
# 4) DOWNLOAD DEPS - Get Python wheels for offline install
# ============================================================================

function Invoke-DownloadDeps {
    Show-Header "Download Pi Dependencies (Offline)"

    $wheelsFolder = Join-Path $piFolder "wheels"
    $requirementsFile = Join-Path $piFolder "requirements.txt"

    if (-not (Test-Path $requirementsFile)) {
        Write-Host "[FAIL] requirements.txt not found at $requirementsFile" -ForegroundColor Red
        return
    }

    Write-Host "Requirements: $requirementsFile" -ForegroundColor White
    Write-Host "Download to:  $wheelsFolder" -ForegroundColor White
    Write-Host ""

    New-Item -ItemType Directory -Force -Path $wheelsFolder | Out-Null

    Write-Host "Downloading packages for Raspberry Pi (ARM)..." -ForegroundColor Cyan
    Write-Host "(This may take a few minutes)" -ForegroundColor Gray

    pip download `
        --platform linux_armv7l `
        --python-version 3.11 `
        --only-binary=:all: `
        --dest $wheelsFolder `
        --requirement $requirementsFile 2>&1 | Where-Object { $_ -notmatch "Requirement already satisfied" }

    pip download `
        --platform any `
        --dest $wheelsFolder `
        --requirement $requirementsFile 2>&1 | Where-Object { $_ -notmatch "Requirement already satisfied" }

    Write-Host ""

    $wheelFiles = Get-ChildItem $wheelsFolder -Filter "*.whl"
    $totalMB = [math]::Round(($wheelFiles | Measure-Object -Property Length -Sum).Sum / 1MB, 2)
    Write-Host "[OK] Downloaded $($wheelFiles.Count) packages ($totalMB MB total)" -ForegroundColor Green

    foreach ($whl in $wheelFiles) {
        $sizeMB = [math]::Round($whl.Length / 1MB, 2)
        Write-Host "  $($whl.Name) ($sizeMB MB)" -ForegroundColor White
    }
}

# ============================================================================
# 5) SETUP FRESH PI - Complete first-time setup
# ============================================================================

function Invoke-FreshSetup {
    Show-Header "Fresh Pi Setup"

    Write-Host "This will do a COMPLETE first-time setup:" -ForegroundColor White
    Write-Host "  1. Test connection" -ForegroundColor White
    Write-Host "  2. Install Python (if needed)" -ForegroundColor White
    Write-Host "  3. Copy all Pi code" -ForegroundColor White
    Write-Host "  4. Install dependencies" -ForegroundColor White
    Write-Host "  5. Create auto-start service" -ForegroundColor White
    Write-Host "  6. Verify everything works" -ForegroundColor White
    Write-Host ""
    Write-Host "NOTE: Pi needs internet for first-time setup." -ForegroundColor Yellow
    Write-Host "      After setup it works 100% offline." -ForegroundColor Yellow
    Write-Host ""

    # Optional IP override
    Write-Host "Pi address [$PiAddress]: " -ForegroundColor Yellow -NoNewline
    $inputAddr = Read-Host
    if (-not [string]::IsNullOrWhiteSpace($inputAddr)) {
        $script:PiAddress = $inputAddr
    }

    Confirm-Action "Ready to set up Pi at $PiAddress"

    if (-not (Test-Path $piFolder)) {
        Write-Host "[FAIL] pi/ folder not found" -ForegroundColor Red
        return
    }

    # Step 1: Connection
    Write-Host ""
    Write-Host "--- Step 1: Connection ---" -ForegroundColor Cyan
    if (-not (Test-PiConnection)) { return }

    # Step 2: Python
    Write-Host ""
    Write-Host "--- Step 2: Python ---" -ForegroundColor Cyan
    $pythonVersion = ssh "$PiUser@$PiAddress" "python3 --version 2>&1"
    if ($pythonVersion -match "Python 3") {
        Write-Host "[OK] $pythonVersion" -ForegroundColor Green
    } else {
        Write-Host "Installing Python..." -ForegroundColor Yellow
        ssh "$PiUser@$PiAddress" "sudo apt update -qq && sudo apt install -y python3 python3-pip -qq 2>&1 | grep -v 'Reading\|Building'"
        Write-Host "[OK] Python installed" -ForegroundColor Green
    }

    # Step 3: Copy files
    Write-Host ""
    Write-Host "--- Step 3: Copy Files ---" -ForegroundColor Cyan
    ssh "$PiUser@$PiAddress" "mkdir -p $PiPath"
    scp -r "$piFolder/*" "$PiUser@${PiAddress}:$PiPath/" 2>$null
    ssh "$PiUser@$PiAddress" "chmod +x $PiPath/*.py"
    Write-Host "[OK] Files copied" -ForegroundColor Green

    # Step 4: Dependencies
    Write-Host ""
    Write-Host "--- Step 4: Dependencies ---" -ForegroundColor Cyan
    Write-Host "(This may take several minutes on first install)" -ForegroundColor Gray
    ssh "$PiUser@$PiAddress" "cd $PiPath && pip3 install -r requirements.txt --break-system-packages 2>&1" | ForEach-Object {
        if ($_ -match "Successfully installed|Collecting|Downloading") {
            Write-Host "  $_" -ForegroundColor Gray
        } elseif ($_ -match "error|Error|ERROR") {
            Write-Host "  $_" -ForegroundColor Red
        }
    }
    Write-Host "[OK] Dependencies installed" -ForegroundColor Green

    # Step 5: Systemd service
    Write-Host ""
    Write-Host "--- Step 5: Auto-Start Service ---" -ForegroundColor Cyan

    $serviceContent = @"
[Unit]
Description=Kronk Shooting Coprocessor
After=network.target

[Service]
Type=simple
User=$PiUser
WorkingDirectory=$PiPath
ExecStart=/usr/bin/python3 $PiPath/pi_shooting.py
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
"@

    $serviceContent | ssh "$PiUser@$PiAddress" "sudo tee /etc/systemd/system/kronk-shooting.service > /dev/null"
    ssh "$PiUser@$PiAddress" "sudo systemctl daemon-reload && sudo systemctl enable kronk-shooting && sudo systemctl start kronk-shooting"
    Write-Host "[OK] Service created and started" -ForegroundColor Green

    # Step 6: Verify
    Write-Host ""
    Write-Host "--- Step 6: Verify ---" -ForegroundColor Cyan
    Start-Sleep -Seconds 2
    $status = ssh "$PiUser@$PiAddress" "sudo systemctl is-active kronk-shooting"
    if ($status -eq "active") {
        Write-Host "[OK] Service is running!" -ForegroundColor Green
    } else {
        Write-Host "[WARN] Service status: $status" -ForegroundColor Yellow
        ssh "$PiUser@$PiAddress" "sudo journalctl -u kronk-shooting -n 10 --no-pager"
    }

    Write-Host ""
    Write-Host "[OK] Setup complete!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Pi will now:" -ForegroundColor White
    Write-Host "  - Start automatically on boot" -ForegroundColor White
    Write-Host "  - Restart automatically if it crashes" -ForegroundColor White
    Write-Host "  - Work 100% offline" -ForegroundColor White
    Write-Host ""
    Write-Host "Useful commands:" -ForegroundColor Yellow
    Write-Host "  View logs:     ssh $PiUser@$PiAddress 'sudo journalctl -u kronk-shooting -f'" -ForegroundColor White
    Write-Host "  Stop service:  ssh $PiUser@$PiAddress 'sudo systemctl stop kronk-shooting'" -ForegroundColor White
    Write-Host "  Start service: ssh $PiUser@$PiAddress 'sudo systemctl start kronk-shooting'" -ForegroundColor White
}

# ============================================================================
# 6) STATUS - Quick Pi health check
# ============================================================================

function Invoke-Status {
    Show-Header "Pi Status Check"

    if (-not (Test-PiConnection)) { return }

    Write-Host ""

    # Service status
    Write-Host "--- Service ---" -ForegroundColor Cyan
    $svcStatus = ssh "$PiUser@$PiAddress" "sudo systemctl is-active kronk-shooting 2>/dev/null || echo 'not-installed'"
    if ($svcStatus -eq "active") {
        Write-Host "  Service: RUNNING" -ForegroundColor Green
    } elseif ($svcStatus -eq "not-installed") {
        # Check for manual process
        $piPid = ssh "$PiUser@$PiAddress" "pgrep -f pi_shooting.py 2>/dev/null"
        if ($piPid) {
            Write-Host "  Service: not installed (manual process running, PID $piPid)" -ForegroundColor Yellow
        } else {
            Write-Host "  Service: not installed, not running" -ForegroundColor Red
        }
    } else {
        Write-Host "  Service: $svcStatus" -ForegroundColor Yellow
    }

    # Python
    Write-Host ""
    Write-Host "--- Python ---" -ForegroundColor Cyan
    $pyVer = ssh "$PiUser@$PiAddress" "python3 --version 2>&1"
    Write-Host "  Version: $pyVer" -ForegroundColor White

    # Files
    Write-Host ""
    Write-Host "--- Files ---" -ForegroundColor Cyan
    $fileList = ssh "$PiUser@$PiAddress" "ls -la $PiPath/ 2>/dev/null || echo 'Directory not found'"
    $fileList -split "`n" | ForEach-Object { Write-Host "  $_" -ForegroundColor White }

    # Model
    Write-Host ""
    Write-Host "--- Model ---" -ForegroundColor Cyan
    $modelExists = ssh "$PiUser@$PiAddress" "test -f $PiPath/shooting_model.pkl && echo 1 || echo 0"
    if ($modelExists -eq "1") {
        $modelSize = ssh "$PiUser@$PiAddress" "ls -lh $PiPath/shooting_model.pkl | awk '{print `$5}'"
        Write-Host "  Model: EXISTS ($modelSize)" -ForegroundColor Green
    } else {
        Write-Host "  Model: NOT FOUND (using calibration tables)" -ForegroundColor Yellow
    }

    # CSV
    $csvExists = ssh "$PiUser@$PiAddress" "test -f $PiPath/shooting_training_data.csv && echo 1 || echo 0"
    if ($csvExists -eq "1") {
        $csvLines = ssh "$PiUser@$PiAddress" "wc -l < $PiPath/shooting_training_data.csv"
        Write-Host "  Training CSV: $csvLines lines" -ForegroundColor Green
    } else {
        Write-Host "  Training CSV: not found" -ForegroundColor Yellow
    }

    # Recent logs
    Write-Host ""
    Write-Host "--- Recent Logs (last 5 lines) ---" -ForegroundColor Cyan
    $logs = ssh "$PiUser@$PiAddress" "sudo journalctl -u kronk-shooting -n 5 --no-pager 2>/dev/null || echo '  (no systemd logs)'"
    $logs -split "`n" | ForEach-Object { Write-Host "  $_" -ForegroundColor White }
}

# ============================================================================
# 7) FIND & SET IP - Scan network for the Pi and update address
# ============================================================================

function Invoke-FindAndSetIP {
    Show-Header "Find & Set Pi IP"

    Write-Host "Current Pi address: $PiAddress" -ForegroundColor White
    Write-Host ""
    Write-Host "  [1] Scan robot network (10.45.39.x) for the Pi" -ForegroundColor White
    Write-Host "  [2] Enter IP manually" -ForegroundColor White
    Write-Host "  [3] Cancel" -ForegroundColor White
    Write-Host ""
    Write-Host "Choice: " -ForegroundColor Yellow -NoNewline
    $ipChoice = Read-Host

    switch ($ipChoice) {
        "1" { Find-PiOnNetwork }
        "2" { Set-PiManual }
        "3" { return }
        default {
            Write-Host "Invalid choice" -ForegroundColor Red
        }
    }
}

function Find-PiOnNetwork {
    $subnet = "10.45.39"
    Write-Host ""
    Write-Host "Scanning $subnet.1 - $subnet.254 ..." -ForegroundColor Cyan
    Write-Host "(This may take 30-60 seconds)" -ForegroundColor Gray
    Write-Host ""

    $found = @()

    # Ping sweep -- run pings in parallel using jobs for speed
    $jobs = @()
    for ($i = 1; $i -le 254; $i++) {
        $ip = "$subnet.$i"
        $jobs += Start-Job -ScriptBlock {
            param($ip)
            $result = ping -c 1 -W 1 $ip 2>$null
            if ($LASTEXITCODE -eq 0) { return $ip }
            # Fallback for Windows
            $result = ping -n 1 -w 1000 $ip 2>$null
            if ($LASTEXITCODE -eq 0) { return $ip }
            return $null
        } -ArgumentList $ip
    }

    # Wait for all pings (timeout 60s)
    $null = $jobs | Wait-Job -Timeout 60

    $reachable = @()
    foreach ($job in $jobs) {
        $result = Receive-Job $job -ErrorAction SilentlyContinue
        if ($result) { $reachable += $result }
        Remove-Job $job -Force -ErrorAction SilentlyContinue
    }

    if ($reachable.Count -eq 0) {
        Write-Host "[FAIL] No devices found on $subnet.x" -ForegroundColor Red
        Write-Host "  Are you on the robot network?" -ForegroundColor Yellow
        return
    }

    Write-Host "Found $($reachable.Count) device(s). Checking for Pi..." -ForegroundColor Cyan
    Write-Host ""

    # Try SSH to each reachable host to identify the Pi
    foreach ($ip in ($reachable | Sort-Object)) {
        Write-Host "  $ip - " -NoNewline -ForegroundColor White

        # Skip known roboRIO IPs
        if ($ip -eq "$subnet.2") {
            Write-Host "roboRIO (skipped)" -ForegroundColor Gray
            continue
        }

        try {
            $sshResult = ssh -o ConnectTimeout=3 -o StrictHostKeyChecking=no -o BatchMode=yes "$PiUser@$ip" "hostname 2>/dev/null; echo '---PICHECK---'" 2>&1
            if ($sshResult -match "PICHECK") {
                $hostname = ($sshResult | Select-Object -First 1).Trim()
                Write-Host "PI FOUND! (hostname: $hostname)" -ForegroundColor Green
                $found += $ip
            } else {
                Write-Host "SSH failed (not the Pi or wrong credentials)" -ForegroundColor Gray
            }
        } catch {
            Write-Host "no SSH response" -ForegroundColor Gray
        }
    }

    Write-Host ""

    if ($found.Count -eq 0) {
        Write-Host "[FAIL] No Pi found with user '$PiUser' on the network." -ForegroundColor Red
        Write-Host "  - Is the Pi powered on?" -ForegroundColor Yellow
        Write-Host "  - Is SSH enabled?" -ForegroundColor Yellow
        Write-Host "  - Is the username '$PiUser' correct?" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Enter IP manually? (y/n): " -ForegroundColor Yellow -NoNewline
        $manual = Read-Host
        if ($manual -eq "y") { Set-PiManual }
    } elseif ($found.Count -eq 1) {
        $newIP = $found[0]
        Write-Host "Use $newIP as the Pi address? (y/n): " -ForegroundColor Yellow -NoNewline
        $confirm = Read-Host
        if ($confirm -eq "y") {
            Apply-NewIP $newIP
        }
    } else {
        Write-Host "Multiple Pis found:" -ForegroundColor Yellow
        for ($i = 0; $i -lt $found.Count; $i++) {
            Write-Host "  [$($i+1)] $($found[$i])" -ForegroundColor White
        }
        Write-Host ""
        Write-Host "Which one? " -ForegroundColor Yellow -NoNewline
        $pick = Read-Host
        $idx = [int]$pick - 1
        if ($idx -ge 0 -and $idx -lt $found.Count) {
            Apply-NewIP $found[$idx]
        } else {
            Write-Host "Invalid choice" -ForegroundColor Red
        }
    }
}

function Set-PiManual {
    Write-Host ""
    Write-Host "Enter Pi IP address: " -ForegroundColor Yellow -NoNewline
    $newIP = Read-Host

    if ([string]::IsNullOrWhiteSpace($newIP)) {
        Write-Host "No IP entered, cancelled." -ForegroundColor Yellow
        return
    }

    # Quick ping test
    Write-Host "Testing $newIP..." -ForegroundColor Cyan
    $pingOk = $false
    try {
        $result = ping -c 1 -W 2 $newIP 2>$null
        if ($LASTEXITCODE -eq 0) { $pingOk = $true }
    } catch {}
    if (-not $pingOk) {
        try {
            $result = ping -n 1 -w 2000 $newIP 2>$null
            if ($LASTEXITCODE -eq 0) { $pingOk = $true }
        } catch {}
    }

    if ($pingOk) {
        Write-Host "[OK] $newIP responds to ping" -ForegroundColor Green
    } else {
        Write-Host "[WARN] $newIP does not respond to ping" -ForegroundColor Yellow
        Write-Host "Set it anyway? (y/n): " -ForegroundColor Yellow -NoNewline
        $confirm = Read-Host
        if ($confirm -ne "y") { return }
    }

    Apply-NewIP $newIP
}

function Apply-NewIP {
    param([string]$NewIP)

    $oldIP = $script:PiAddress
    $script:PiAddress = $NewIP

    Write-Host ""
    Write-Host "[OK] Pi address updated: $oldIP -> $NewIP" -ForegroundColor Green
    Write-Host ""

    # Also update this script's default so it persists
    $scriptPath = $PSCommandPath
    if ($scriptPath -and (Test-Path $scriptPath)) {
        $content = Get-Content $scriptPath -Raw
        $updated = $content -replace 'PiAddress = "[^"]*"', "PiAddress = `"$NewIP`""
        $updated = $updated -replace '\$TargetPiIP = "[^"]*"', "`$TargetPiIP = `"$NewIP`""
        Set-Content $scriptPath -Value $updated -NoNewline
        Write-Host "[OK] Default IP saved to pi-tool.ps1 (will persist)" -ForegroundColor Green
    }

    # Test SSH
    Write-Host ""
    Write-Host "Testing SSH to $PiUser@$NewIP..." -ForegroundColor Cyan
    if (Test-PiConnection) {
        Write-Host "[OK] Pi is reachable at new address!" -ForegroundColor Green
    } else {
        Write-Host "[WARN] Could not SSH to new address. Connection will be retried when needed." -ForegroundColor Yellow
    }
}

# ============================================================================
# MAIN MENU
# ============================================================================

function Show-Menu {
    Show-Header "Kronk Pi Tool - Team 4539"

    Write-Host "  Pi: $PiUser@$PiAddress" -ForegroundColor Gray
    Write-Host ""
    Write-Host "  [1] Deploy       - Push pi/ code to the Pi" -ForegroundColor White
    Write-Host "  [2] Pull         - Download data from Pi (CSV, logs, model)" -ForegroundColor White
    Write-Host "  [3] Clear Model  - Delete trained model (use calibration tables)" -ForegroundColor White
    Write-Host "  [4] Download Deps- Get Python wheels for offline install" -ForegroundColor White
    Write-Host "  [5] Fresh Setup  - Complete first-time Pi setup" -ForegroundColor White
    Write-Host "  [6] Status       - Quick Pi health check" -ForegroundColor White
    Write-Host "  [7] Find/Set IP  - Scan network or set Pi address" -ForegroundColor White
    Write-Host "  [Q] Quit" -ForegroundColor White
    Write-Host ""
    Write-Host "Choice: " -ForegroundColor Yellow -NoNewline
}

# Main loop
while ($true) {
    Show-Menu
    $choice = Read-Host

    switch ($choice.ToUpper()) {
        "1" { Invoke-Deploy }
        "2" { Invoke-Pull }
        "3" { Invoke-ClearModel }
        "4" { Invoke-DownloadDeps }
        "5" { Invoke-FreshSetup }
        "6" { Invoke-Status }
        "7" { Invoke-FindAndSetIP }
        "Q" {
            Write-Host ""
            Write-Host "Bye!" -ForegroundColor Cyan
            exit 0
        }
        default {
            Write-Host "Invalid choice. Pick 1-7 or Q." -ForegroundColor Red
        }
    }

    Write-Host ""
    Write-Host "Press ENTER to return to menu..." -ForegroundColor Gray
    Read-Host
}
