#!/usr/bin/env python3
"""
Pi NetworkTables Connection Diagnostic
Run this on the Pi to test NetworkTables connection
"""

import time
import sys

# Check if ntcore is installed
try:
    import ntcore
    print("✅ ntcore is installed")
except ImportError:
    print("❌ ntcore not installed!")
    print("   Run: pip3 install robotpy-ntcore")
    sys.exit(1)

TEAM_NUMBER = 4539

def main():
    print("=" * 60)
    print("  Pi NetworkTables Connection Diagnostic")
    print(f"  Team: {TEAM_NUMBER}")
    print("=" * 60)
    print()
    
    # Create NetworkTables instance
    print("Creating NetworkTables instance...")
    inst = ntcore.NetworkTableInstance.getDefault()
    
    # Set team number (will auto-discover roboRIO)
    print(f"Setting team number to {TEAM_NUMBER}...")
    inst.setServer("localhost")  # For testing on the Pi itself, connect to localhost
    # inst.setServerTeam(TEAM_NUMBER)
    
    # Start client
    print("Starting NetworkTables client...")
    inst.startClient4("Pi Diagnostic")
    
    print()
    print("Waiting for connection to roboRIO...")
    print("(Press Ctrl+C to stop)")
    print()
    
    last_state = False
    connection_attempts = 0
    
    try:
        while True:
            connected = inst.isConnected()
            
            if connected != last_state:
                if connected:
                    print(f"✅ CONNECTED to NetworkTables! (after {connection_attempts} attempts)")
                    print()
                    
                    # Test reading/writing
                    print("Testing read/write...")
                    test_table = inst.getTable("Pi/Status")
                    pub = test_table.getDoubleTopic("test_value").publish()
                    pub.set(123.45)
                    print("✅ Published test value to Pi/Status/test_value")
                    
                    # Show server info
                    print()
                    print("Connection info:")
                    connections = inst.getConnections()
                    for conn in connections:
                        print(f"  Connected to: {conn.remote_id}")
                    
                else:
                    print(f"❌ Disconnected from NetworkTables")
                
                last_state = connected
            
            if not connected:
                connection_attempts += 1
                if connection_attempts % 5 == 0:
                    print(f"Still waiting... (attempt {connection_attempts})")
                    print("Make sure:")
                    print(f"  - roboRIO is on and running robot code")
                    print(f"  - roboRIO is on team {TEAM_NUMBER} network")
                    print(f"  - This Pi is on the same network (check 'ip addr')")
                    print(f"  - Can ping roboRIO: 'ping roborio-{TEAM_NUMBER}-frc.local'")
                    print()
            
            time.sleep(1)
    
    except KeyboardInterrupt:
        print()
        print("Stopping...")
    
    finally:
        inst.stopClient()
        print("Done!")

if __name__ == "__main__":
    main()
