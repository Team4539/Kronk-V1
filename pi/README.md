# Kronk Raspberry Pi Shooting Coprocessor

**All turret/shooter intelligence runs on the Raspberry Pi.**

The Pi autonomously:
- Loads `shooting_training_data.csv` from the deploy directory
- Trains a machine learning model on startup (if CSV exists)
- Calculates optimal turret angle & shooter speeds in real-time
- Records training data when robot sends snapshot confirmations
- Retrains automatically when new HITs are recorded

The roboRIO simply sends robot state and receives shooting commands.

## Quick Start: Deployment

**From your development computer (project root):**

```powershell
# Launch the Pi management tool
.\pi-tool.bat
```

This gives you a menu with options to deploy code, pull data, clear the model, and more.
See **PI_GUIDE.md** in the project root for the full deployment guide.

## Recording Training Data (Two-Phase System)

Training data is recorded using a **two-phase snapshot + confirm** workflow:

### Phase 1: Automatic Snapshot
When the robot fires a shot (via AutoShootCommand), the system automatically
captures a snapshot of the current robot state -- pose, velocity, turret angle,
shooter speeds, distance, etc. This is published to the `Pi/Snapshot` NetworkTables table.

### Phase 2: Operator Confirmation
After observing whether the shot scored, the **operator** presses:
- **D-pad Up** = HIT (shot scored -- used for ML training)
- **D-pad Down** = MISS (shot missed -- logged but not used for training)
- **D-pad Left** = DISCARD (bad data, not recorded at all)

The Pi receives the confirmation, combines it with the snapshot data, and appends
a row to `shooting_training_data.csv`. HIT shots trigger an automatic retrain.

### Why Two Phases?
The old system required the operator to press a button at the exact moment of
firing, which was unreliable and led to bad training data. The new system
captures state automatically at the moment of firing and lets the operator
calmly confirm the result afterward.

## Manual Setup on Raspberry Pi

### 1. Install Python & dependencies
```bash
sudo apt update && sudo apt install python3 python3-pip -y
cd /home/kaotic/kronk
pip3 install -r requirements.txt
```

### 2. Run the coprocessor
```bash
python3 pi_shooting.py
```

**That's it!** The Pi will:
- Look for `shooting_training_data.csv` in the pi/ directory or /home/lvuser/deploy/
- Auto-train a model if CSV exists and no model file is found
- Fall back to interpolation tables if no training data is available
- Continuously calculate shooting solutions at 50 Hz

### 3. (Optional) Train manually
If you want to train/retrain manually:
```bash
python3 train_model.py --evaluate
```

### 4. Auto-start on boot (recommended)
Create a systemd service:
```bash
sudo nano /etc/systemd/system/kronk-shooting.service
```

```ini
[Unit]
Description=Kronk Shooting Coprocessor
After=network.target

[Service]
Type=simple
User=kaotic
WorkingDirectory=/home/kaotic/kronk
ExecStart=/usr/bin/python3 /home/kaotic/kronk/pi_shooting.py
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable kronk-shooting
sudo systemctl start kronk-shooting

# View logs
sudo journalctl -u kronk-shooting -f
```

## Troubleshooting Connection Issues

If the Pi isn't connecting to NetworkTables:

1. **Run the connection diagnostic:**
   ```bash
   python3 test_connection.py
   ```

2. **Check you're on the robot network:**
   ```bash
   ip addr show
   # Should see 10.45.39.XXX (for team 4539)
   ```

3. **Try pinging the roboRIO:**
   ```bash
   ping roborio-4539-frc.local
   ```

4. **Check the robot code is running** (Driver Station should show "Robot Code" green)

**See TROUBLESHOOTING.md for complete diagnostic steps.**

## How It Works

### Recording Flow (Two-Phase)

1. **Robot fires a shot** -> AutoShootCommand detects the firing edge
2. **Snapshot captured** -> Robot state published to `Pi/Snapshot` table
3. **Operator confirms** -> D-pad Up (HIT), Down (MISS), or Left (DISCARD)
4. **Pi records** -> Appends row to CSV with result
5. **Auto-retrain** -> If result was HIT, model retrains within 10 seconds

### Data Flow
```
Robot (roboRIO)                    Raspberry Pi
-----------------                  ------------
                  NetworkTables
Robot pose      ---------------->  Pi reads inputs
Robot velocity  ---------------->  Calculates solution
Alliance color  ---------------->  (interpolation + ML model)
Target mode     ---------------->
Battery voltage ---------------->

                <----------------  turret_angle (degrees)
                <----------------  top_speed (0.0-1.0)
                <----------------  bottom_speed (0.0-1.0)
                <----------------  distance, confidence, etc.

Shot snapshot   ---------------->  (captured at moment of firing)
Confirm result  ---------------->  Records to CSV, retrains on HIT
```

### NetworkTables Layout

| Table | Key | Direction | Description |
|-------|-----|-----------|-------------|
| `Pi/Input` | `robot_x`, `robot_y` | Robot -> Pi | Robot position (meters) |
| `Pi/Input` | `robot_heading` | Robot -> Pi | Robot heading (degrees) |
| `Pi/Input` | `robot_vx`, `robot_vy` | Robot -> Pi | Velocity (m/s, field-relative) |
| `Pi/Input` | `robot_omega` | Robot -> Pi | Rotation rate (rad/s) |
| `Pi/Input` | `is_blue_alliance` | Robot -> Pi | Alliance color |
| `Pi/Input` | `target_mode` | Robot -> Pi | "HUB" or "TRENCH" |
| `Pi/Input` | `battery_voltage` | Robot -> Pi | Current voltage |
| `Pi/Input` | `enabled` | Robot -> Pi | Whether to calculate |
| `Pi/Snapshot` | `snapshot_ready` | Robot -> Pi | New snapshot available |
| `Pi/Snapshot` | `confirm_result` | Robot -> Pi | "HIT", "MISS", or "DISCARD" |
| `Pi/Snapshot` | `robot_x`, `robot_y`, ... | Robot -> Pi | Frozen state at shot time |
| `Pi/Output` | `turret_angle` | Pi -> Robot | Turret angle to command |
| `Pi/Output` | `top_speed` | Pi -> Robot | Top motor power |
| `Pi/Output` | `bottom_speed` | Pi -> Robot | Bottom motor power |
| `Pi/Output` | `distance` | Pi -> Robot | Distance to target |
| `Pi/Output` | `confidence` | Pi -> Robot | Prediction confidence |
| `Pi/Status` | `connected` | Pi -> Robot | Pi alive flag |
| `Pi/Status` | `heartbeat` | Pi -> Robot | Incrementing counter |
| `Pi/Status` | `model_loaded` | Pi -> Robot | Has trained model |

### Fallback Behavior

If the Pi doesn't send a heartbeat for 500ms, the robot automatically falls back to:
- **Turret angle**: Calculated locally from robot pose and target position
- **Shooter powers**: Interpolated from the Constants.java calibration tables
- **No lead angle** or training corrections in fallback mode

The fallback ensures the robot can still shoot (with reduced accuracy) even if the Pi
crashes, loses network, or is physically disconnected.

### Model Training Details

The Pi trains a **Gradient Boosting Regressor** that predicts:
- **Angle correction**: Additional turret angle adjustment beyond geometric calculation
- **Top power correction**: Adjustment to base interpolated top wheel power
- **Bottom power correction**: Adjustment to base interpolated bottom wheel power

**Features used** (13 total):
- Target type (HUB vs TRENCH)
- Distance to target
- Robot position (x, y, heading)
- Robot velocity (vx, vy, omega, speed)
- Whether robot is moving
- Turret angle and angle to target
- Battery voltage

The model learns from **HIT** shots only, continuously improving as you add more training data.

## Monitoring

### On SmartDashboard / Shuffleboard

The robot publishes Pi status to NetworkTables:

| Key | Description |
|-----|-------------|
| `Pi/Status/connected` | True if Pi is responding |
| `Pi/Status/heartbeat` | Incrementing counter (should change every cycle) |
| `Pi/Status/model_loaded` | True if ML model is trained and loaded |
| `Pi/Status/loop_time_ms` | Processing time per loop (should be <20ms) |
| `Pi/Training/training_points` | Number of HIT shots in training dataset |
| `Pi/Training/last_train_time` | Timestamp of last successful training |
| `PiShooting/PiConnected` | Robot's view of Pi connection status |
| `PiShooting/UsingFallback` | True if robot is using fallback calculations |

### On the Pi

**View live logs** (if running as service):
```bash
ssh kaotic@10.45.39.11 'journalctl -u kronk-shooting -f'
```

**View status** (if running manually):
The Pi prints status every 5 seconds:
```
[Pi] Loop #250 | NT connected: True | Enabled: True | Model: True | Loop: 8.3ms
```

### Troubleshooting

**Pi not connecting?**
- Check network: `ping 10.45.39.11`
- Check service: `ssh kaotic@10.45.39.11 'systemctl status kronk-shooting'`
- Check team number: Ensure `TEAM_NUMBER = 4539` in `pi_shooting.py`

**Model not loading?**
- Check CSV exists: `ssh kaotic@10.45.39.11 'ls -l ~/kronk/*.csv'`
- Check model file: `ssh kaotic@10.45.39.11 'ls -l ~/kronk/*.pkl'`
- Manually train: `ssh kaotic@10.45.39.11 'cd ~/kronk && python3 train_model.py --evaluate'`

**Fallback mode constantly active?**
- Pi heartbeat timeout is 500ms
- Check Pi loop time (should be < 20ms)
- Check network latency
- Increase `CONNECTION_TIMEOUT_SECONDS` in PiShootingHelper.java if needed

You can also request a retrain from the robot by setting `Pi/Training/retrain_requested` to `true`.

## Files

| File | Description |
|------|-------------|
| `pi_shooting.py` | Main coprocessor script (runs on Pi) |
| `train_model.py` | Trains ML model from CSV data |
| `test_connection.py` | NetworkTables connection diagnostic |
| `test_recording.py` | Recording system diagnostic |
| `requirements.txt` | Python dependencies |
| `shooting_model.pkl` | Trained model (auto-generated) |
| `shooting_training_data.csv` | Training data (copy from deploy) |

## Robot-Side Java Files

| File | Description |
|------|-------------|
| `util/PiShootingHelper.java` | NT communication, snapshot capture, fallback logic |
| `commands/AutoShootCommand.java` | Uses PiShootingHelper for aiming/shooting, triggers snapshots |
| `commands/calibrations/RecordShotCommand.java` | Sends HIT/MISS/DISCARD confirmations to Pi |
