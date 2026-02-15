#!/usr/bin/env python3
"""
Kronk Raspberry Pi Shooting Coprocessor
========================================
Runs on a Raspberry Pi connected to the robot network.
Receives robot state via NetworkTables, calculates optimal:
  - Turret angle (degrees)
  - Top shooter speed (0.0 - 1.0 duty cycle)
  - Bottom shooter speed (0.0 - 1.0 duty cycle)

Publishes results back to NetworkTables for the roboRIO to consume.

Training:
  Run `python3 train_model.py` to train/retrain the model from CSV data.
  The trained model is saved to `shooting_model.pkl` and loaded at startup.
  If no model exists, falls back to interpolation tables (same as Constants.java).

Usage:
  pip install -r requirements.txt
  python3 pi_shooting.py
"""

import time
import math
import os
import sys
import json
import signal
import ntcore
import numpy as np

# Try to load trained model, fall back to interpolation
try:
    import pickle
    HAS_PICKLE = True
except ImportError:
    HAS_PICKLE = False

# ============================================================================
# CONFIGURATION
# ============================================================================

# NetworkTables team number
TEAM_NUMBER = 4539

# Update rate (Hz)
UPDATE_RATE_HZ = 50
UPDATE_PERIOD = 1.0 / UPDATE_RATE_HZ

# NT table names
PI_TABLE = "Pi"
PI_INPUT_TABLE = "Pi/Input"
PI_OUTPUT_TABLE = "Pi/Output"
PI_STATUS_TABLE = "Pi/Status"
PI_TRAINING_TABLE = "Pi/Training"

# Field constants (must match Constants.java)
FIELD_LENGTH_METERS = 16.540988
FIELD_WIDTH_METERS = 8.069326

# Hub positions
BLUE_HUB_X = 4.971194
BLUE_HUB_Y = 4.034631
RED_HUB_X = FIELD_LENGTH_METERS - BLUE_HUB_X
RED_HUB_Y = FIELD_WIDTH_METERS - BLUE_HUB_Y

# Trench positions
BLUE_TRENCH_ROTATING_X = 4.586034
BLUE_TRENCH_ROTATING_Y = 0.639458
BLUE_TRENCH_FIXED_X = 4.586034
BLUE_TRENCH_FIXED_Y = 7.421594
RED_TRENCH_ROTATING_X = FIELD_LENGTH_METERS - BLUE_TRENCH_ROTATING_X
RED_TRENCH_ROTATING_Y = FIELD_WIDTH_METERS - BLUE_TRENCH_ROTATING_Y
RED_TRENCH_FIXED_X = FIELD_LENGTH_METERS - BLUE_TRENCH_FIXED_X
RED_TRENCH_FIXED_Y = FIELD_WIDTH_METERS - BLUE_TRENCH_FIXED_Y

# Turret offset from robot center (meters)
TURRET_X_OFFSET_INCHES = 6.25
TURRET_Y_OFFSET_INCHES = 6.25
ROBOT_LENGTH_INCHES = 28.0
ROBOT_WIDTH_INCHES = 27.0
TURRET_X_OFFSET = (TURRET_X_OFFSET_INCHES - (ROBOT_LENGTH_INCHES / 2.0)) * 0.0254
TURRET_Y_OFFSET = ((ROBOT_WIDTH_INCHES / 2.0) - TURRET_Y_OFFSET_INCHES) * 0.0254

# Shooting on the fly
ESTIMATED_BALL_SPEED_MPS = 15.0
MIN_MOVING_SPEED_MPS = 0.3
MAX_LEAD_ANGLE_DEG = 25.0

# Voltage compensation
REFERENCE_VOLTAGE = 12.5
MIN_VALID_VOLTAGE = 10.0
MAX_VOLTAGE_COMPENSATION = 1.25

# Hub shooting calibration table: distance (m) -> [top_power, bottom_power]
HUB_CALIBRATION = {
    1.5: [0.30, 0.40], 2.0: [0.35, 0.45], 2.5: [0.40, 0.50],
    3.0: [0.45, 0.55], 3.5: [0.50, 0.60], 4.0: [0.55, 0.65],
    4.5: [0.60, 0.70], 5.0: [0.65, 0.75], 5.5: [0.70, 0.80],
    6.0: [0.75, 0.85], 7.0: [0.85, 0.95], 8.0: [0.95, 1.00],
}

# Trench shooting calibration table
TRENCH_CALIBRATION = {
    2.0: [0.25, 0.50], 3.0: [0.30, 0.55], 4.0: [0.35, 0.60],
    5.0: [0.40, 0.65], 6.0: [0.45, 0.70], 7.0: [0.50, 0.75],
    8.0: [0.55, 0.80],
}

# Path to trained model file
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "shooting_model.pkl")

# Paths to training CSV (try multiple locations)
CSV_PATHS = [
    # Local to pi directory
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "shooting_training_data.csv"),
    # If deployed to robot, check deploy directory
    "/home/lvuser/deploy/shooting_training_data.csv",
    # Alternative robot path
    "/home/pi/deploy/shooting_training_data.csv",
]

def get_csv_path():
    """Find the first valid CSV path."""
    for path in CSV_PATHS:
        if os.path.exists(path):
            return path
    return CSV_PATHS[0]  # Default to first if none found

CSV_PATH = get_csv_path()

# ============================================================================
# INTERPOLATION TABLE (fallback when no trained model)
# ============================================================================

def interpolate_powers(distance: float, calibration: dict) -> tuple:
    """
    Interpolate shooter powers from a calibration table.
    Matches the Java ShooterSubsystem.interpolatePowers() logic.
    """
    keys = sorted(calibration.keys())
    
    if not keys:
        return (0.5, 0.5)
    
    # Below min distance
    if distance <= keys[0]:
        v = calibration[keys[0]]
        return (v[0], v[1])
    
    # Above max distance
    if distance >= keys[-1]:
        v = calibration[keys[-1]]
        return (v[0], v[1])
    
    # Find surrounding keys
    lower_key = keys[0]
    upper_key = keys[-1]
    for k in keys:
        if k <= distance:
            lower_key = k
        if k >= distance:
            upper_key = k
            break
    
    if lower_key == upper_key:
        v = calibration[lower_key]
        return (v[0], v[1])
    
    # Linear interpolation
    t = (distance - lower_key) / (upper_key - lower_key)
    lo = calibration[lower_key]
    hi = calibration[upper_key]
    return (lo[0] + (hi[0] - lo[0]) * t, lo[1] + (hi[1] - lo[1]) * t)


# ============================================================================
# GEOMETRY HELPERS
# ============================================================================

def get_turret_field_position(robot_x, robot_y, robot_heading_deg):
    """Get turret position in field coordinates, accounting for offset from robot center."""
    heading_rad = math.radians(robot_heading_deg)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    
    field_offset_x = TURRET_X_OFFSET * cos_h - TURRET_Y_OFFSET * sin_h
    field_offset_y = TURRET_X_OFFSET * sin_h + TURRET_Y_OFFSET * cos_h
    
    return (robot_x + field_offset_x, robot_y + field_offset_y)


def get_hub_position(is_blue: bool) -> tuple:
    """Get hub position for given alliance."""
    if is_blue:
        return (BLUE_HUB_X, BLUE_HUB_Y)
    return (RED_HUB_X, RED_HUB_Y)


def get_closest_trench(robot_x, robot_y, is_blue: bool) -> tuple:
    """Get the closest trench target for given alliance."""
    if is_blue:
        rot = (BLUE_TRENCH_ROTATING_X, BLUE_TRENCH_ROTATING_Y)
        fix = (BLUE_TRENCH_FIXED_X, BLUE_TRENCH_FIXED_Y)
    else:
        rot = (RED_TRENCH_ROTATING_X, RED_TRENCH_ROTATING_Y)
        fix = (RED_TRENCH_FIXED_X, RED_TRENCH_FIXED_Y)
    
    d_rot = math.sqrt((robot_x - rot[0])**2 + (robot_y - rot[1])**2)
    d_fix = math.sqrt((robot_x - fix[0])**2 + (robot_y - fix[1])**2)
    return rot if d_rot <= d_fix else fix


def calculate_distance_and_angle(turret_x, turret_y, target_x, target_y):
    """Calculate distance and field-relative angle from turret to target."""
    dx = target_x - turret_x
    dy = target_y - turret_y
    distance = math.sqrt(dx * dx + dy * dy)
    angle_deg = math.degrees(math.atan2(dy, dx))
    return distance, angle_deg


def field_angle_to_turret_angle(field_angle_deg, robot_heading_deg):
    """Convert field-relative angle to robot-relative turret angle."""
    turret_angle = field_angle_deg - robot_heading_deg
    # Normalize to -180 to +180
    turret_angle = ((turret_angle + 180) % 360 + 360) % 360 - 180
    return turret_angle


def calculate_lead_angle(distance, robot_vx, robot_vy, angle_to_target_deg):
    """
    Calculate lead angle for shooting on the fly.
    Matches ShootingTrainingManager.calculateLeadAngle().
    """
    flight_time = distance / ESTIMATED_BALL_SPEED_MPS
    angle_rad = math.radians(angle_to_target_deg)
    
    # Direction toward target
    target_dir_x = math.cos(angle_rad)
    target_dir_y = math.sin(angle_rad)
    
    # Perpendicular direction (right of target direction)
    perp_dir_x = -target_dir_y
    perp_dir_y = target_dir_x
    
    # Lateral velocity component
    lateral_velocity = robot_vx * perp_dir_x + robot_vy * perp_dir_y
    lateral_drift = lateral_velocity * flight_time
    
    lead_angle_deg = math.degrees(math.atan2(lateral_drift, distance))
    return max(-MAX_LEAD_ANGLE_DEG, min(MAX_LEAD_ANGLE_DEG, lead_angle_deg))


def get_voltage_compensation(battery_voltage):
    """Calculate voltage compensation factor."""
    if battery_voltage < MIN_VALID_VOLTAGE:
        battery_voltage = REFERENCE_VOLTAGE
    compensation = REFERENCE_VOLTAGE / battery_voltage
    return min(compensation, MAX_VOLTAGE_COMPENSATION)


# ============================================================================
# TRAINED MODEL (loaded from train_model.py output)
# ============================================================================

class TrainedModel:
    """Wrapper for the trained sklearn model."""
    
    def __init__(self, auto_train=True):
        self.model = None
        self.is_loaded = False
        self.auto_train = auto_train
        self.training_points = 0
        self.last_train_time = "Never"
        self.load()
        
        # Auto-train if no model exists but CSV does
        if not self.is_loaded and self.auto_train:
            self._auto_train()
    
    def load(self):
        """Try to load the trained model from disk."""
        if not HAS_PICKLE:
            print("[Pi] pickle not available, using interpolation fallback")
            return
        
        if not os.path.exists(MODEL_PATH):
            print(f"[Pi] No trained model at {MODEL_PATH}")
            return
        
        try:
            with open(MODEL_PATH, "rb") as f:
                data = pickle.load(f)
            self.model = data
            self.is_loaded = True
            print(f"[Pi] Loaded trained model from {MODEL_PATH}")
            if "info" in data:
                info = data['info']
                print(f"[Pi] Model info: {info}")
                self.training_points = info.get("hit_points", 0)
                self.last_train_time = info.get("trained_at", "Unknown")
        except Exception as e:
            print(f"[Pi] Failed to load model: {e}")
            self.is_loaded = False
    
    def _auto_train(self):
        """Automatically train if CSV data exists."""
        csv_path = get_csv_path()
        if not os.path.exists(csv_path):
            print(f"[Pi] No CSV at {csv_path}, skipping auto-train")
            print(f"[Pi] Will use interpolation tables until training data is provided")
            return
        
        print(f"[Pi] Found CSV at {csv_path}, auto-training model...")
        try:
            from train_model import train_model
            success = train_model(csv_path, evaluate=False)
            if success:
                self.load()  # Reload the newly trained model
        except Exception as e:
            print(f"[Pi] Auto-train failed: {e}")
            import traceback
            traceback.print_exc()
    
    def predict(self, target_type: str, distance: float, robot_x: float, robot_y: float,
                robot_heading: float, robot_vx: float, robot_vy: float, robot_omega: float,
                turret_angle: float, angle_to_target: float, battery_voltage: float):
        """
        Predict turret correction, top power correction, and bottom power correction.
        
        Returns: (angle_correction, top_power_correction, bottom_power_correction)
        """
        if not self.is_loaded or self.model is None:
            return (0.0, 0.0, 0.0)
        
        try:
            # Build feature vector matching train_model.py
            is_trench = 1.0 if target_type == "TRENCH" else 0.0
            robot_speed = math.sqrt(robot_vx**2 + robot_vy**2)
            is_moving = 1.0 if robot_speed >= MIN_MOVING_SPEED_MPS else 0.0
            
            features = np.array([[
                is_trench,
                distance,
                robot_x,
                robot_y,
                robot_heading,
                robot_vx,
                robot_vy,
                robot_omega,
                turret_angle,
                angle_to_target,
                robot_speed,
                is_moving,
                battery_voltage
            ]])
            
            # Model outputs: [angle_correction, top_power_correction, bottom_power_correction]
            prediction = self.model["regressor"].predict(features)[0]
            
            # Apply voltage compensation to power corrections
            v_comp = get_voltage_compensation(battery_voltage)
            angle_corr = float(prediction[0])
            top_corr = float(prediction[1]) * v_comp
            bottom_corr = float(prediction[2]) * v_comp
            
            return (angle_corr, top_corr, bottom_corr)
        
        except Exception as e:
            print(f"[Pi] Model prediction error: {e}")
            return (0.0, 0.0, 0.0)


# ============================================================================
# MAIN SHOOTING CALCULATOR
# ============================================================================

class ShootingCalculator:
    """
    Calculates turret angle, top shooter speed, and bottom shooter speed
    based on robot position, velocity, and target mode.
    """
    
    def __init__(self):
        self.model = TrainedModel()
        self.use_model = self.model.is_loaded
    
    def calculate(self, robot_x: float, robot_y: float, robot_heading_deg: float,
                  robot_vx: float, robot_vy: float, robot_omega: float,
                  is_blue_alliance: bool, target_mode: str,
                  battery_voltage: float,
                  hub_offset_x: float = 0.0, hub_offset_y: float = 0.0,
                  trench_offset_x: float = 0.0, trench_offset_y: float = 0.0):
        """
        Calculate shooting solution.
        
        Args:
            robot_x, robot_y: Robot position on field (meters)
            robot_heading_deg: Robot heading (degrees)
            robot_vx, robot_vy: Robot velocity (m/s, field-relative)
            robot_omega: Robot rotational velocity (rad/s)
            is_blue_alliance: True for blue, False for red
            target_mode: "HUB" or "TRENCH"
            battery_voltage: Current battery voltage
            hub/trench_offset_x/y: Tunable position offsets
        
        Returns:
            dict with keys:
                turret_angle: Turret angle to command (degrees, -180 to +180)
                top_speed: Top motor power (0.0 to 1.0)
                bottom_speed: Bottom motor power (0.0 to 1.0)
                distance: Distance to target (meters)
                is_moving: Whether robot is moving
                lead_angle: Lead angle applied (degrees)
                confidence: Prediction confidence (0.0 to 1.0)
        """
        is_trench = target_mode == "TRENCH"
        
        # Get turret field position
        turret_x, turret_y = get_turret_field_position(robot_x, robot_y, robot_heading_deg)
        
        # Get target position
        if is_trench:
            target_x, target_y = get_closest_trench(robot_x, robot_y, is_blue_alliance)
            target_x += trench_offset_x
            target_y += trench_offset_y
        else:
            target_x, target_y = get_hub_position(is_blue_alliance)
            target_x += hub_offset_x
            target_y += hub_offset_y
        
        # Calculate distance and angles
        distance, field_angle = calculate_distance_and_angle(turret_x, turret_y, target_x, target_y)
        turret_angle = field_angle_to_turret_angle(field_angle, robot_heading_deg)
        
        # Check if moving
        robot_speed = math.sqrt(robot_vx**2 + robot_vy**2)
        is_moving = robot_speed >= MIN_MOVING_SPEED_MPS
        
        # Calculate lead angle for shooting on the fly
        lead_angle = 0.0
        if is_moving:
            lead_angle = calculate_lead_angle(distance, robot_vx, robot_vy, field_angle)
            turret_angle += lead_angle
        
        # Get base shooter powers from interpolation table
        calibration = TRENCH_CALIBRATION if is_trench else HUB_CALIBRATION
        top_power, bottom_power = interpolate_powers(distance, calibration)
        
        # Apply trained model corrections if available
        confidence = 0.5  # Default confidence from calibration tables
        angle_correction = 0.0
        
        if self.use_model:
            target_type_str = "TRENCH" if is_trench else "HUB"
            angle_corr, top_corr, bottom_corr = self.model.predict(
                target_type_str, distance, robot_x, robot_y, robot_heading_deg,
                robot_vx, robot_vy, robot_omega, turret_angle, field_angle, battery_voltage
            )
            angle_correction = angle_corr
            turret_angle += angle_corr
            top_power += top_corr
            bottom_power += bottom_corr
            confidence = 0.85  # Higher confidence with trained model
        
        # Clamp powers
        top_power = max(0.0, min(1.0, top_power))
        bottom_power = max(0.0, min(1.0, bottom_power))
        
        # Normalize turret angle to -180 to +180
        turret_angle = ((turret_angle + 180) % 360 + 360) % 360 - 180
        
        return {
            "turret_angle": turret_angle,
            "top_speed": top_power,
            "bottom_speed": bottom_power,
            "distance": distance,
            "is_moving": is_moving,
            "lead_angle": lead_angle,
            "angle_correction": angle_correction,
            "confidence": confidence,
            "target_x": target_x,
            "target_y": target_y,
        }


# ============================================================================
# NETWORK TABLES COPROCESSOR
# ============================================================================

class PiShootingCoprocessor:
    """
    Main coprocessor loop. Reads robot state from NetworkTables,
    calculates shooting solution, publishes results.
    """
    
    def __init__(self, auto_retrain_on_csv_change=True):
        self.calculator = ShootingCalculator()
        self.running = True
        self.loop_count = 0
        self.auto_retrain_on_csv_change = auto_retrain_on_csv_change
        self.last_csv_mtime = 0
        self._check_csv_modified()  # Initialize CSV modification time
        
        # Setup NetworkTables
        self.nt_inst = ntcore.NetworkTableInstance.getDefault()
        self.nt_inst.setServerTeam(TEAM_NUMBER)
        self.nt_inst.setServer("localhost")
        self.nt_inst.startClient4("Pi Shooting Coprocessor")
        
        # Input table (robot publishes, Pi reads)
        input_table = self.nt_inst.getTable(PI_INPUT_TABLE)
        self.sub_robot_x = input_table.getDoubleTopic("robot_x").subscribe(0.0)
        self.sub_robot_y = input_table.getDoubleTopic("robot_y").subscribe(0.0)
        self.sub_robot_heading = input_table.getDoubleTopic("robot_heading").subscribe(0.0)
        self.sub_robot_vx = input_table.getDoubleTopic("robot_vx").subscribe(0.0)
        self.sub_robot_vy = input_table.getDoubleTopic("robot_vy").subscribe(0.0)
        self.sub_robot_omega = input_table.getDoubleTopic("robot_omega").subscribe(0.0)
        self.sub_is_blue = input_table.getBooleanTopic("is_blue_alliance").subscribe(True)
        self.sub_target_mode = input_table.getStringTopic("target_mode").subscribe("HUB")
        self.sub_battery_voltage = input_table.getDoubleTopic("battery_voltage").subscribe(12.5)
        self.sub_hub_offset_x = input_table.getDoubleTopic("hub_offset_x").subscribe(0.0)
        self.sub_hub_offset_y = input_table.getDoubleTopic("hub_offset_y").subscribe(0.0)
        self.sub_trench_offset_x = input_table.getDoubleTopic("trench_offset_x").subscribe(0.0)
        self.sub_trench_offset_y = input_table.getDoubleTopic("trench_offset_y").subscribe(0.0)
        self.sub_enabled = input_table.getBooleanTopic("enabled").subscribe(False)
        self.sub_robot_enabled = input_table.getBooleanTopic("robot_enabled").subscribe(False)
        
        # Training input (robot reports shot results)
        self.sub_record_shot = input_table.getBooleanTopic("record_shot").subscribe(False)
        self.sub_shot_result = input_table.getStringTopic("shot_result").subscribe("MISS")
        
        # Output table (Pi publishes, robot reads)
        output_table = self.nt_inst.getTable(PI_OUTPUT_TABLE)
        self.pub_turret_angle = output_table.getDoubleTopic("turret_angle").publish()
        self.pub_top_speed = output_table.getDoubleTopic("top_speed").publish()
        self.pub_bottom_speed = output_table.getDoubleTopic("bottom_speed").publish()
        self.pub_distance = output_table.getDoubleTopic("distance").publish()
        self.pub_is_moving = output_table.getBooleanTopic("is_moving").publish()
        self.pub_lead_angle = output_table.getDoubleTopic("lead_angle").publish()
        self.pub_confidence = output_table.getDoubleTopic("confidence").publish()
        self.pub_angle_correction = output_table.getDoubleTopic("angle_correction").publish()
        
        # Status table
        status_table = self.nt_inst.getTable(PI_STATUS_TABLE)
        self.pub_connected = status_table.getBooleanTopic("connected").publish()
        self.pub_heartbeat = status_table.getIntegerTopic("heartbeat").publish()
        self.pub_model_loaded = status_table.getBooleanTopic("model_loaded").publish()
        self.pub_update_rate = status_table.getDoubleTopic("update_rate_hz").publish()
        self.pub_loop_time_ms = status_table.getDoubleTopic("loop_time_ms").publish()
        
        # Training control (robot can request retrain)
        training_table = self.nt_inst.getTable(PI_TRAINING_TABLE)
        self.sub_retrain = training_table.getBooleanTopic("retrain_requested").subscribe(False)
        self.pub_retrain_ack = training_table.getBooleanTopic("retrain_requested").publish()
        self.pub_last_train_time = training_table.getStringTopic("last_train_time").publish()
        self.pub_training_points = training_table.getIntegerTopic("training_points").publish()
        
        # Training data recording
        self.last_shot_state = {
            "robot_x": 0.0, "robot_y": 0.0, "robot_heading": 0.0,
            "robot_vx": 0.0, "robot_vy": 0.0, "robot_omega": 0.0,
            "target_mode": "HUB", "battery_voltage": 12.5,
            "turret_angle": 0.0, "top_speed": 0.0, "bottom_speed": 0.0,
            "distance": 0.0, "angle_to_target": 0.0, "angle_correction": 0.0,
        }
        self.record_shot_last = False
        
        # Signal handler for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print(f"[Pi] Shooting coprocessor initialized for team {TEAM_NUMBER}")
        print(f"[Pi] Update rate: {UPDATE_RATE_HZ} Hz")
        print(f"[Pi] Model loaded: {self.calculator.use_model}")
        print(f"[Pi] Training points: {self.calculator.model.training_points}")
        print(f"[Pi] Last trained: {self.calculator.model.last_train_time}")
        
        # Publish initial training status
        self.pub_training_points.set(self.calculator.model.training_points)
        self.pub_last_train_time.set(self.calculator.model.last_train_time)
    
    def _signal_handler(self, sig, frame):
        print("\n[Pi] Shutting down...")
        self.running = False
    
    def _check_csv_modified(self):
        """Check if CSV has been modified since last check."""
        csv_path = get_csv_path()
        if os.path.exists(csv_path):
            mtime = os.path.getmtime(csv_path)
            if mtime > self.last_csv_mtime:
                self.last_csv_mtime = mtime
                return True
        return False
    
    def _record_shot(self, result: str):
        """Record a shot to the training CSV."""
        try:
            csv_path = get_csv_path()
            
            # Create CSV if it doesn't exist
            if not os.path.exists(csv_path):
                os.makedirs(os.path.dirname(csv_path), exist_ok=True)
                with open(csv_path, 'w') as f:
                    f.write("# Shooting training data - 15 field format\n")
                    f.write("# target_type,distance,robot_x,robot_y,robot_heading,")
                    f.write("robot_vx,robot_vy,robot_omega,turret_angle,angle_to_target,")
                    f.write("angle_correction,top_power_correction,bottom_power_correction,")
                    f.write("battery_voltage,result\n")
                print(f"[Pi] Created new CSV: {csv_path}")
            
            state = self.last_shot_state
            
            # Calculate power corrections (actual - base)
            target_mode = state["target_mode"]
            distance = state["distance"]
            calibration = TRENCH_CALIBRATION if target_mode == "TRENCH" else HUB_CALIBRATION
            base_top, base_bottom = interpolate_powers(distance, calibration)
            
            # Apply voltage normalization for corrections
            v_comp = get_voltage_compensation(state["battery_voltage"])
            top_correction = (state["top_speed"] - base_top) / v_comp
            bottom_correction = (state["bottom_speed"] - base_bottom) / v_comp
            
            # Build CSV line
            line = f"{target_mode},{distance:.4f},"
            line += f"{state['robot_x']:.4f},{state['robot_y']:.4f},{state['robot_heading']:.4f},"
            line += f"{state['robot_vx']:.4f},{state['robot_vy']:.4f},{state['robot_omega']:.4f},"
            line += f"{state['turret_angle']:.4f},{state['angle_to_target']:.4f},"
            line += f"{state['angle_correction']:.4f},{top_correction:.4f},{bottom_correction:.4f},"
            line += f"{state['battery_voltage']:.2f},{result.upper()}\n"
            
            # Append to CSV
            with open(csv_path, 'a') as f:
                f.write(line)
            
            print(f"[Pi] Recorded {result} shot at {distance:.2f}m to {csv_path}")
            
            # If it was a HIT, trigger retrain
            if result.upper() == "HIT":
                print(f"[Pi] HIT recorded! Will retrain on next cycle.")
                self.last_csv_mtime = 0  # Force retrain check
            
        except Exception as e:
            print(f"[Pi] Error recording shot: {e}")
            import traceback
            traceback.print_exc()
    
    def run(self):
        """Main processing loop."""
        print("[Pi] Starting main loop...")
        print(f"[Pi] Waiting for NetworkTables connection to team {TEAM_NUMBER}...")
        
        # Wait for initial connection
        connection_timeout = 10  # seconds
        start_time = time.monotonic()
        while not self.nt_inst.isConnected() and (time.monotonic() - start_time) < connection_timeout:
            print(f"[Pi] Still waiting for connection... ({int(time.monotonic() - start_time)}s)")
            time.sleep(1)
        
        if self.nt_inst.isConnected():
            print("[Pi] ✅ Connected to NetworkTables!")
        else:
            print("[Pi] ⚠️  No NetworkTables connection yet, will continue trying...")
            print("[Pi] Make sure:")
            print(f"[Pi]   - roboRIO is on and connected to team {TEAM_NUMBER} network")
            print("[Pi]   - This Pi is on the same network")
            print("[Pi]   - Team number is correct in pi_shooting.py")
        
        print("[Pi] Entering main loop...")
        
        last_time = time.monotonic()
        heartbeat = 0
        last_connection_state = False
        
        while self.running:
            loop_start = time.monotonic()
            
            # Monitor connection state changes
            current_connection = self.nt_inst.isConnected()
            if current_connection != last_connection_state:
                if current_connection:
                    print("[Pi] ✅ NetworkTables connected!")
                else:
                    print("[Pi] ❌ NetworkTables disconnected!")
                last_connection_state = current_connection
            
            # Read inputs
            enabled = self.sub_enabled.get()
            robot_enabled = self.sub_robot_enabled.get()
            
            if enabled:
                robot_x = self.sub_robot_x.get()
                robot_y = self.sub_robot_y.get()
                robot_heading = self.sub_robot_heading.get()
                robot_vx = self.sub_robot_vx.get()
                robot_vy = self.sub_robot_vy.get()
                robot_omega = self.sub_robot_omega.get()
                is_blue = self.sub_is_blue.get()
                target_mode = self.sub_target_mode.get()
                battery_voltage = self.sub_battery_voltage.get()
                hub_offset_x = self.sub_hub_offset_x.get()
                hub_offset_y = self.sub_hub_offset_y.get()
                trench_offset_x = self.sub_trench_offset_x.get()
                trench_offset_y = self.sub_trench_offset_y.get()
                
                # Always calculate shooting solution (even when disabled)
                # so values are ready the instant the robot enables
                result = self.calculator.calculate(
                    robot_x, robot_y, robot_heading,
                    robot_vx, robot_vy, robot_omega,
                    is_blue, target_mode, battery_voltage,
                    hub_offset_x, hub_offset_y,
                    trench_offset_x, trench_offset_y
                )
                
                # SAFETY: Only publish non-zero outputs when robot is ENABLED.
                # When disabled, publish zeros so roboRIO never reads stale values.
                if robot_enabled:
                    self.pub_turret_angle.set(result["turret_angle"])
                    self.pub_top_speed.set(result["top_speed"])
                    self.pub_bottom_speed.set(result["bottom_speed"])
                    self.pub_distance.set(result["distance"])
                    self.pub_is_moving.set(result["is_moving"])
                    self.pub_lead_angle.set(result["lead_angle"])
                    self.pub_confidence.set(result["confidence"])
                    self.pub_angle_correction.set(result["angle_correction"])
                else:
                    # Robot disabled - publish zeros for safety
                    self.pub_turret_angle.set(0.0)
                    self.pub_top_speed.set(0.0)
                    self.pub_bottom_speed.set(0.0)
                    self.pub_distance.set(result["distance"])  # Distance is informational, safe to publish
                    self.pub_is_moving.set(False)
                    self.pub_lead_angle.set(0.0)
                    self.pub_confidence.set(0.0)
                    self.pub_angle_correction.set(0.0)
                
                # Store shot state for potential recording
                target_x = result["target_x"]
                target_y = result["target_y"]
                turret_x, turret_y = get_turret_field_position(robot_x, robot_y, robot_heading)
                dx = target_x - turret_x
                dy = target_y - turret_y
                angle_to_target = math.degrees(math.atan2(dy, dx))
                
                self.last_shot_state = {
                    "robot_x": robot_x,
                    "robot_y": robot_y,
                    "robot_heading": robot_heading,
                    "robot_vx": robot_vx,
                    "robot_vy": robot_vy,
                    "robot_omega": robot_omega,
                    "target_mode": target_mode,
                    "battery_voltage": battery_voltage,
                    "turret_angle": result["turret_angle"],
                    "top_speed": result["top_speed"],
                    "bottom_speed": result["bottom_speed"],
                    "distance": result["distance"],
                    "angle_to_target": angle_to_target,
                    "angle_correction": result["angle_correction"],
                }
            
            # Check for shot recording request
            record_shot = self.sub_record_shot.get()
            if record_shot and not self.record_shot_last:
                # Rising edge detection
                shot_result = self.sub_shot_result.get()
                self._record_shot(shot_result)
            self.record_shot_last = record_shot
            
            # Publish status (always, even when disabled)
            heartbeat += 1
            self.pub_connected.set(True)
            self.pub_heartbeat.set(heartbeat)
            self.pub_model_loaded.set(self.calculator.use_model)
            self.pub_update_rate.set(UPDATE_RATE_HZ)
            
            loop_time_ms = (time.monotonic() - loop_start) * 1000
            self.pub_loop_time_ms.set(loop_time_ms)
            
            # Check for retrain request
            if self.sub_retrain.get():
                print("[Pi] Retrain requested via NetworkTables!")
                self.pub_retrain_ack.set(False)  # Clear the request
                self._retrain()
            
            # Auto-retrain if CSV has been updated
            if self.auto_retrain_on_csv_change and self.loop_count % (UPDATE_RATE_HZ * 10) == 0:
                if self._check_csv_modified():
                    print("[Pi] CSV updated, auto-retraining...")
                    self._retrain()
            
            # Log status periodically
            self.loop_count += 1
            if self.loop_count % (UPDATE_RATE_HZ * 5) == 0:  # Every 5 seconds
                connected = self.nt_inst.isConnected()
                output_status = "OUTPUTTING" if (enabled and robot_enabled) else "ZEROED"
                print(f"[Pi] Loop #{self.loop_count} | NT connected: {connected} | "
                      f"Enabled: {enabled} | Robot: {'ON' if robot_enabled else 'OFF'} | "
                      f"Output: {output_status} | Model: {self.calculator.use_model} | "
                      f"Loop: {loop_time_ms:.1f}ms")
            
            # Sleep to maintain update rate
            elapsed = time.monotonic() - loop_start
            sleep_time = UPDATE_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # Cleanup
        self.pub_connected.set(False)
        self.nt_inst.stopClient()
        print("[Pi] Shutdown complete")
    
    def _retrain(self):
        """Retrain the model by calling train_model.py."""
        try:
            csv_path = get_csv_path()
            if not os.path.exists(csv_path):
                print(f"[Pi] Cannot retrain: CSV not found at {csv_path}")
                return
            
            print(f"[Pi] Retraining from {csv_path}...")
            
            # Import and run training directly
            from train_model import train_model
            success = train_model(csv_path, evaluate=False)
            
            if success:
                # Reload model
                self.calculator.model.load()
                self.calculator.use_model = self.calculator.model.is_loaded
                self.pub_last_train_time.set(self.calculator.model.last_train_time)
                self.pub_training_points.set(self.calculator.model.training_points)
                print("[Pi] Model retrained and reloaded!")
            else:
                print("[Pi] Training failed!")
                
        except Exception as e:
            print(f"[Pi] Retrain error: {e}")
            import traceback
            traceback.print_exc()


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  Kronk Raspberry Pi Shooting Coprocessor")
    print(f"  Team {TEAM_NUMBER}")
    print("=" * 60)
    
    coprocessor = PiShootingCoprocessor()
    coprocessor.run()
