# Kronk Raspberry Pi Shooting Coprocessor

**All turret/shooter intelligence runs on the Raspberry Pi.**

The Pi autonomously:
- Loads `shooting_training_data.csv` from the deploy directory
- Trains a machine learning model on startup (if CSV exists)
- Calculates optimal turret angle & shooter speeds in real-time
- Retrains automatically when requested via NetworkTables

The roboRIO simply sends robot state and receives shooting commands.

## Setup on Raspberry Pi

### 1. Install Python & dependencies
```bash
sudo apt update && sudo apt install python3 python3-pip -y
cd /home/pi/kronk  # or wherever you put this
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
User=pi
WorkingDirectory=/home/pi/kronk
ExecStart=/usr/bin/python3 /home/pi/kronk/pi_shooting.py
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

### Autonomous Training & Recording Flow

The Pi is **fully autonomous** and handles everything:

1. **On Startup**: 
   - Looks for `shooting_training_data.csv` 
   - If found but no model exists → auto-train
   - If no CSV exists → creates one and uses interpolation tables

2. **During Operation**:
   - Calculates shooting solutions at 50 Hz
   - Listens for shot recording requests from robot
   - When robot reports a shot result → appends to CSV automatically

3. **Auto-Retrain**:
   - Every 10 seconds, checks if CSV was modified
   - If modified (or new HIT recorded) → retrains model automatically
   - Can also be triggered manually via NetworkTables

4. **Continuous Learning**:
   - Each successful (HIT) shot improves the model
   - Model gets better over time without any manual intervention
   - Failed shots are recorded too (for analysis) but not used in training

### Recording Shots from Robot

The robot sends shot results to the Pi via NetworkTables:

```java
// In your robot code - after taking a shot
NetworkTableEntry recordShot = input_table.getEntry("record_shot");
NetworkTableEntry shotResult = input_table.getEntry("shot_result");

shotResult.setString("HIT");  // or "MISS" or "PARTIAL"
recordShot.setBoolean(true);  // Trigger recording
recordShot.setBoolean(false); // Reset for next shot
```

The Pi will:
1. Capture the last calculated shooting solution
2. Combine with robot state (position, velocity, etc.)
3. Append a line to `shooting_training_data.csv`
4. If result was "HIT" → trigger auto-retrain

### Data Flow
```
Robot (roboRIO)                    Raspberry Pi
─────────────                      ────────────
                  NetworkTables
Robot pose      ──────────────►  Pi reads inputs
Robot velocity  ──────────────►  Calculates solution
Alliance color  ──────────────►  (interpolation + ML model)
Target mode     ──────────────►
Battery voltage ──────────────►
                                 
                ◄──────────────  turret_angle (degrees)
                ◄──────────────  top_speed (0.0-1.0)
                ◄──────────────  bottom_speed (0.0-1.0)
                ◄──────────────  distance, confidence, etc.
```

### NetworkTables Layout
| Table | Key | Direction | Description |
|-------|-----|-----------|-------------|
| `Pi/Input` | `robot_x`, `robot_y` | Robot → Pi | Robot position (meters) |
| `Pi/Input` | `robot_heading` | Robot → Pi | Robot heading (degrees) |
| `Pi/Input` | `robot_vx`, `robot_vy` | Robot → Pi | Velocity (m/s, field-relative) |
| `Pi/Input` | `robot_omega` | Robot → Pi | Rotation rate (rad/s) |
| `Pi/Input` | `is_blue_alliance` | Robot → Pi | Alliance color |
| `Pi/Input` | `target_mode` | Robot → Pi | "HUB" or "TRENCH" |
| `Pi/Input` | `battery_voltage` | Robot → Pi | Current voltage |
| `Pi/Input` | `enabled` | Robot → Pi | Whether to calculate |
| `Pi/Input` | `record_shot` | Robot → Pi | Trigger shot recording |
| `Pi/Input` | `shot_result` | Robot → Pi | "HIT", "MISS", or "PARTIAL" |
| `Pi/Output` | `turret_angle` | Pi → Robot | Turret angle to command |
| `Pi/Output` | `top_speed` | Pi → Robot | Top motor power |
| `Pi/Output` | `bottom_speed` | Pi → Robot | Bottom motor power |
| `Pi/Output` | `distance` | Pi → Robot | Distance to target |
| `Pi/Output` | `confidence` | Pi → Robot | Prediction confidence |
| `Pi/Status` | `connected` | Pi → Robot | Pi alive flag |
| `Pi/Status` | `heartbeat` | Pi → Robot | Incrementing counter |
| `Pi/Status` | `model_loaded` | Pi → Robot | Has trained model |

### Fallback Behavior
If the Pi doesn't send a heartbeat for 500ms, the robot automatically falls back to:
- **Turret angle**: Calculated locally from robot pose and target position
- **Shooter powers**: Interpolated from the Constants.java calibration tables
- **No lead angle** or training corrections in fallback mode

The fallback ensures the robot can still shoot (with reduced accuracy) even if the Pi
crashes, loses network, or is physically disconnected.

## Training Workflow

### Autonomous Recording (New!)

The Pi now records shots automatically! No need to manually copy CSV files.

**On the robot** (you need to add this code):
```java
// After shooting, determine if shot was successful
String result = detectShotResult(); // "HIT", "MISS", or "PARTIAL"

// Send to Pi for recording
NetworkTable inputTable = NetworkTableInstance.getDefault().getTable("Pi/Input");
inputTable.getEntry("shot_result").setString(result);
inputTable.getEntry("record_shot").setBoolean(true);

// Reset trigger (important!)
try { Thread.sleep(50); } catch (Exception e) {}
inputTable.getEntry("record_shot").setBoolean(false);
```

**The Pi automatically**:
1. Captures the shooting solution it calculated
2. Records robot state (position, velocity, heading, etc.)
3. Appends to `shooting_training_data.csv`
4. If result was "HIT" → triggers retrain within 10 seconds

### Manual CSV Editing (Optional)

If you want to manually edit training data or import from elsewhere:

#### Quick Deploy
Use the deployment scripts to automatically sync code and data to the Pi:

**Windows (PowerShell):**
```powershell
.\deploy_to_pi.ps1 [pi_hostname_or_ip]
```

**Linux/Mac:**
```bash
./deploy_to_pi.sh [pi_hostname_or_ip]
```

This will:
- Copy all Python files to the Pi
- Copy the latest `shooting_training_data.csv` from the deploy directory
- Optionally install dependencies
- Restart the shooting service if it's running

### Old Manual Workflow (Not Needed Anymore)

For reference, the old workflow was:
1. Drive robot, manually adjust turret/shooter until shots hit
2. Press record button on SmartDashboard
3. Copy line to CSV manually
4. Re-deploy CSV to Pi

**Now the Pi does all of this automatically!** Just implement shot detection on the robot and send the result via NetworkTables.

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
ssh pi@raspberrypi.local 'journalctl -u kronk-shooting -f'
```

**View status** (if running manually):
The Pi prints status every 5 seconds:
```
[Pi] Loop #250 | NT connected: True | Enabled: True | Model: True | Loop: 8.3ms
```

### Troubleshooting

**Pi not connecting?**
- Check network: `ping raspberrypi.local`
- Check service: `ssh pi@raspberrypi.local 'systemctl status kronk-shooting'`
- Check team number: Ensure `TEAM_NUMBER = 4539` in `pi_shooting.py`

**Model not loading?**
- Check CSV exists: `ssh pi@raspberrypi.local 'ls -l ~/kronk/*.csv'`
- Check model file: `ssh pi@raspberrypi.local 'ls -l ~/kronk/*.pkl'`
- Manually train: `ssh pi@raspberrypi.local 'cd ~/kronk && python3 train_model.py --evaluate'`

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
| `requirements.txt` | Python dependencies |
| `shooting_model.pkl` | Trained model (auto-generated) |
| `shooting_training_data.csv` | Training data (copy from deploy) |

## Robot-Side Java Files

| File | Description |
|------|-------------|
| `util/PiShootingHelper.java` | NT communication + fallback logic |
| `commands/AutoShootCommand.java` | Uses PiShootingHelper for aiming/shooting |
