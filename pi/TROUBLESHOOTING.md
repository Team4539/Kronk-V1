# Pi NetworkTables Connection Troubleshooting

## Quick Diagnostics

### 1. Check if Pi is on the network
```bash
# Check Pi's IP address
ip addr show

# Should see something like 10.45.39.XXX (for team 4539)
# or 192.168.1.XXX if on a home network with the robot
```

### 2. Check if you can ping the roboRIO
```bash
# Try team hostname
ping roborio-4539-frc.local

# Or try static IP
ping 10.45.39.2

# Or try mDNS
ping roborio-4539-frc
```

### 3. Check if ntcore is installed
```bash
python3 -c "import ntcore; print('ntcore OK')"

# If error, install it:
pip3 install robotpy-ntcore
```

### 4. Run connection diagnostic
```bash
cd ~/kronk
python3 test_connection.py
```

This will tell you exactly what's wrong.

### 5. Check if robot code is running
The roboRIO must be running robot code for NetworkTables to work.
- Check Driver Station shows "Robot Code" green
- Or SSH to roboRIO and check: `systemctl status frcUserProgram`

## Common Issues

### Issue: "No NetworkTables connection"

**Cause 1: Pi not on robot network**
```bash
# Check IP - should be 10.TE.AM.XXX (e.g., 10.45.39.123 for team 4539)
ip addr show

# If wrong network, configure static IP:
sudo nano /etc/dhcpcd.conf
# Add:
#   interface eth0
#   static ip_address=10.45.39.50/24
#   static routers=10.45.39.1
sudo systemctl restart dhcpcd
```

**Cause 2: roboRIO not running**
- Check Driver Station
- Ensure robot code is deployed
- Restart roboRIO

**Cause 3: Wrong team number**
- Check `TEAM_NUMBER = 4539` in `pi_shooting.py`
- Should match your actual team number

**Cause 4: Firewall blocking**
```bash
# Disable firewall temporarily to test
sudo ufw disable

# If that fixes it, add rule:
sudo ufw allow from 10.45.39.0/24
sudo ufw enable
```

### Issue: "ntcore not found"

```bash
# Install ntcore
pip3 install robotpy-ntcore

# Or from requirements.txt
pip3 install -r requirements.txt
```

### Issue: "Connection keeps dropping"

**Check network cable:**
- Use Ethernet, not WiFi (more reliable)
- Check cable is firmly connected
- Try different cable

**Check network switch:**
- Make sure Pi and roboRIO are on same switch
- Power cycle the switch

**Check robot radio:**
- Ensure radio is powered
- Check radio firmware is up to date

### Issue: "Pi publishes but robot doesn't receive"

**Check robot code is reading from correct table:**
```java
// In robot code, should be:
NetworkTable table = NetworkTableInstance.getDefault().getTable("Pi/Output");
```

**Enable NT logging in robot code:**
```java
// In Robot.java robotInit()
NetworkTableInstance.getDefault().startServer();
System.out.println("NT Server started");
```

### Issue: "Permission denied" errors

```bash
# Make sure you're running as pi user, not root
whoami  # Should say "pi"

# Make sure files are owned by pi
sudo chown -R pi:pi ~/kronk
chmod +x ~/kronk/*.py
```

## Testing the Full Pipeline

### Step 1: Test Pi alone
```bash
cd ~/kronk
python3 test_connection.py
```
Should see "CONNECTED" within a few seconds.

### Step 2: Test Pi shooting code
```bash
cd ~/kronk
python3 pi_shooting.py
```
Watch for:
- "Connected to NetworkTables!"
- "Loop #250 | NT connected: True"

### Step 3: Test from robot
In Driver Station, enable SmartDashboard, check for:
- `Pi/Status/connected = true`
- `Pi/Status/heartbeat` increasing

### Step 4: Test shot recording
Use test script:
```bash
python3 test_recording.py
```

## Network Setup for Different Scenarios

### Competition/Practice Field (FMS)
```bash
# Pi should auto-configure via DHCP
# Verify you're on 10.TE.AM.XXX network
ip addr show
```

### Home Network with Robot
```bash
# Set static IP on Pi
sudo nano /etc/dhcpcd.conf
# Add:
interface eth0
static ip_address=10.45.39.50/24
static routers=10.45.39.1
static domain_name_servers=8.8.8.8

# Restart networking
sudo systemctl restart dhcpcd

# Verify
ping 10.45.39.2  # roboRIO
```

### Wireless (Not Recommended)
If you MUST use WiFi:
```bash
# Connect to robot radio WiFi
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
# Add:
network={
    ssid="4539"
    psk="your_password"
}

sudo systemctl restart wpa_supplicant
```

## Still Not Working?

### Get detailed logs
```bash
# Run Pi code with verbose output
cd ~/kronk
python3 -u pi_shooting.py 2>&1 | tee pi_log.txt
```

### Check from both sides

**On Pi:**
```bash
# Show what Pi is publishing
ntcore-client -t 4539 -l Pi/Status
```

**On development machine:**
```bash
# Show what roboRIO is publishing
# Use OutlineViewer or AdvantageScope
```

### Ask for help with this info:
1. Pi IP address: `ip addr show`
2. Can ping roboRIO?: `ping roborio-4539-frc.local`
3. Output of: `python3 test_connection.py`
4. Output of: `python3 pi_shooting.py` (first 50 lines)
5. SmartDashboard screenshot showing Pi/Status entries
