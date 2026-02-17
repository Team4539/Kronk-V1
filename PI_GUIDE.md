# Pi Coprocessor Guide - Team 4539

All Pi management is done through one script: `pi-tool.bat` (or `pi-tool.ps1`).

## Quick Start

```
.\pi-tool.bat
```

This opens a menu with all available actions:

```
  [1] Deploy       - Push pi/ code to the Pi
  [2] Pull         - Download data from Pi (CSV, logs, model)
  [3] Clear Model  - Delete trained model (use calibration tables)
  [4] Download Deps- Get Python wheels for offline install
  [5] Fresh Setup  - Complete first-time Pi setup
  [6] Status       - Quick Pi health check
  [Q] Quit
```

---

## First-Time Setup

1. Connect Pi to a network with internet access
2. Run `.\pi-tool.bat` and choose **[5] Fresh Setup**
3. Enter the Pi's current IP address when prompted
4. Script handles everything: Python, dependencies, auto-start service, file copy

After setup the Pi works **100% offline** and auto-starts on boot.

---

## Deploying Code Changes

1. Edit files in the `pi/` folder
2. Run `.\pi-tool.bat` and choose **[1] Deploy**
3. Files are copied, permissions set, service restarted

---

## Recording Training Data

Training uses a two-phase system:

**Phase 1 (automatic):** Robot snapshots all state when the turret feed fires.

**Phase 2 (operator confirms):** After watching the ball land, operator presses:
- **D-pad Up** = HIT (on operator controller)
- **D-pad Right** = MISS
- **D-pad Left** = Discard (bad data)

The Pi logs the data and auto-retrains on HITs.

### Tips
- Record 20+ HITs minimum before relying on the model
- Record from varied positions, angles, and while moving
- Record at different battery levels

---

## Pulling Data

Run `.\pi-tool.bat` and choose **[2] Pull**, then pick:
- **[1]** Training CSV only
- **[2]** Log files
- **[3]** Trained model
- **[4]** Everything

Existing files are automatically backed up to `pi-backup/` with timestamps.

---

## Troubleshooting

### Can't connect to Pi
- Check Pi is powered on
- Check you're on the robot network (10.45.39.x)
- Try: `ping 10.45.39.11`
- Username: `kaotic` / Password: `4539`

### Shots not accurate
Run `.\pi-tool.bat` and choose **[3] Clear Model** to force calibration-table-only mode.

### Check Pi status
Run `.\pi-tool.bat` and choose **[6] Status** for a full health check.

### Manual SSH access
```bash
ssh kaotic@10.45.39.11
sudo systemctl status kronk-shooting    # Check service
sudo journalctl -u kronk-shooting -f    # View logs
sudo systemctl restart kronk-shooting   # Restart
```

---

## Network Info

| Device | IP |
|--------|-----|
| roboRIO | 10.45.39.2 |
| Raspberry Pi | 10.45.39.11 |
| Robot Radio | 10.45.39.1 |

---

## SmartDashboard Keys

| Key | Meaning |
|-----|---------|
| `Pi/Connected` | Pi is connected via NetworkTables |
| `Pi/UsingFallback` | Robot is using local calibration tables |
| `Pi/Status/model_loaded` | ML model is active |
| `Pi/Training/training_points` | Number of HITs in model |

---

## File Locations

**On your computer:**
- `pi/` -- Source code for the Pi
- `pi/shooting_training_data.csv` -- Training data (after pulling)
- `pi-backup/` -- Automatic timestamped backups

**On the Pi:**
- `/home/kaotic/kronk/` -- All Pi files
- `/home/kaotic/kronk/shooting_model.pkl` -- Trained model
- `/home/kaotic/kronk/shooting_training_data.csv` -- Training CSV

**Service:** `kronk-shooting` (auto-starts on boot, auto-restarts on crash)
