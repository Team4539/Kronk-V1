#!/usr/bin/env python3
"""
Test Pi shot recording functionality
Run this on your development machine to verify the Pi is recording shots correctly
"""

import time
import ntcore

TEAM_NUMBER = 4539

def main():
    print("=" * 60)
    print("  Pi Shot Recording Test")
    print("=" * 60)
    print()
    
    # Connect to NetworkTables
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.setServerTeam(TEAM_NUMBER)
    inst.setServer("localhost")  # For testing on the Pi itself, connect to localhost
    inst.startClient4("Pi Recording Test")
    
    print("Connecting to robot NetworkTables...")
    time.sleep(2)
    
    if not inst.isConnected():
        print("❌ Failed to connect to NetworkTables!")
        print(f"   Make sure robot/Pi is running on team {TEAM_NUMBER}")
        return
    
    print("✅ Connected to NetworkTables")
    print()
    
    # Get tables
    input_table = inst.getTable("Pi/Input")
    status_table = inst.getTable("Pi/Status")
    training_table = inst.getTable("Pi/Training")
    
    # Check Pi status
    print("Checking Pi status...")
    connected = status_table.getEntry("connected").getBoolean(False)
    model_loaded = status_table.getEntry("model_loaded").getBoolean(False)
    training_points = training_table.getEntry("training_points").getInteger(0)
    last_train = training_table.getEntry("last_train_time").getString("Never")
    
    print(f"  Pi Connected: {connected}")
    print(f"  Model Loaded: {model_loaded}")
    print(f"  Training Points: {training_points}")
    print(f"  Last Trained: {last_train}")
    print()
    
    if not connected:
        print("❌ Pi not connected! Start pi_shooting.py first.")
        return
    
    print("✅ Pi is running")
    print()
    
    # Test shot recording
    print("Testing shot recording...")
    print("This will record a test shot to the CSV")
    print()
    
    # Set some test data
    input_table.getEntry("robot_x").setDouble(5.0)
    input_table.getEntry("robot_y").setDouble(4.0)
    input_table.getEntry("robot_heading").setDouble(0.0)
    input_table.getEntry("robot_vx").setDouble(0.0)
    input_table.getEntry("robot_vy").setDouble(0.0)
    input_table.getEntry("robot_omega").setDouble(0.0)
    input_table.getEntry("is_blue_alliance").setBoolean(True)
    input_table.getEntry("target_mode").setString("HUB")
    input_table.getEntry("battery_voltage").setDouble(12.5)
    input_table.getEntry("enabled").setBoolean(True)
    
    time.sleep(0.1)  # Let Pi calculate
    
    # Record a test shot
    print("Recording TEST shot...")
    input_table.getEntry("shot_result").setString("HIT")
    input_table.getEntry("record_shot").setBoolean(True)
    time.sleep(0.1)
    input_table.getEntry("record_shot").setBoolean(False)
    
    print("✅ Shot recording triggered")
    print()
    
    # Wait and check if training points increased
    print("Waiting for Pi to process...")
    time.sleep(2)
    
    new_training_points = training_table.getEntry("training_points").getInteger(0)
    
    if new_training_points > training_points:
        print(f"✅ Success! Training points increased: {training_points} → {new_training_points}")
    else:
        print(f"⚠️  Training points unchanged: {training_points}")
        print("   This might be normal if Pi hasn't retrained yet (retrains every 10s)")
    
    print()
    print("To verify recording on Pi, SSH and run:")
    print("  ssh pi@raspberrypi.local 'tail ~/kronk/shooting_training_data.csv'")
    print()
    print("Done!")
    
    inst.stopClient()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
