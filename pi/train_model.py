#!/usr/bin/env python3
"""
Kronk Shooting Model Trainer
==============================
Reads shooting_training_data.csv and trains a scikit-learn model
to predict turret angle correction, top power correction, and bottom power correction.

The trained model is saved to shooting_model.pkl for use by pi_shooting.py.

Usage:
    python3 train_model.py
    python3 train_model.py --csv /path/to/shooting_training_data.csv
    python3 train_model.py --evaluate  (train + print evaluation metrics)
"""

import os
import sys
import csv
import math
import argparse
import pickle
import time
import numpy as np
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.multioutput import MultiOutputRegressor
from sklearn.model_selection import cross_val_score
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline

# Constants matching pi_shooting.py / Constants.java
REFERENCE_VOLTAGE = 12.5
MIN_VALID_VOLTAGE = 10.0
MIN_MOVING_SPEED_MPS = 0.3

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Try multiple CSV locations
CSV_PATHS = [
    os.path.join(SCRIPT_DIR, "shooting_training_data.csv"),
    "/home/lvuser/deploy/shooting_training_data.csv",
    "/home/pi/deploy/shooting_training_data.csv",
]

def get_csv_path():
    """Find the first valid CSV path."""
    for path in CSV_PATHS:
        if os.path.exists(path):
            return path
    return CSV_PATHS[0]  # Default to first

DEFAULT_CSV = get_csv_path()
MODEL_OUTPUT = os.path.join(SCRIPT_DIR, "shooting_model.pkl")


def parse_csv(csv_path: str) -> list:
    """
    Parse training data CSV into list of dicts.
    Supports all legacy formats (6, 7, 12, 15 fields).
    """
    points = []
    
    if not os.path.exists(csv_path):
        print(f"[Train] CSV not found: {csv_path}")
        return points
    
    with open(csv_path, "r") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            
            parts = [p.strip() for p in line.split(",")]
            
            try:
                point = None
                
                # 15-field format (full velocity data)
                if len(parts) >= 15:
                    point = {
                        "target_type": parts[0].upper(),
                        "distance": float(parts[1]),
                        "robot_x": float(parts[2]),
                        "robot_y": float(parts[3]),
                        "robot_heading": float(parts[4]),
                        "robot_vx": float(parts[5]),
                        "robot_vy": float(parts[6]),
                        "robot_omega": float(parts[7]),
                        "turret_angle": float(parts[8]),
                        "angle_to_target": float(parts[9]),
                        "angle_correction": float(parts[10]),
                        "top_power_correction": float(parts[11]),
                        "bottom_power_correction": float(parts[12]),
                        "battery_voltage": float(parts[13]),
                        "result": parts[14].upper(),
                    }
                # 12-field format (pose, no velocity)
                elif len(parts) >= 12:
                    point = {
                        "target_type": parts[0].upper(),
                        "distance": float(parts[1]),
                        "robot_x": float(parts[2]),
                        "robot_y": float(parts[3]),
                        "robot_heading": float(parts[4]),
                        "robot_vx": 0.0,
                        "robot_vy": 0.0,
                        "robot_omega": 0.0,
                        "turret_angle": float(parts[5]),
                        "angle_to_target": float(parts[6]),
                        "angle_correction": float(parts[7]),
                        "top_power_correction": float(parts[8]),
                        "bottom_power_correction": float(parts[9]),
                        "battery_voltage": float(parts[10]),
                        "result": parts[11].upper(),
                    }
                # 7-field format (no pose, with voltage)
                elif len(parts) >= 7:
                    point = {
                        "target_type": parts[0].upper(),
                        "distance": float(parts[1]),
                        "robot_x": 0.0,
                        "robot_y": 0.0,
                        "robot_heading": 0.0,
                        "robot_vx": 0.0,
                        "robot_vy": 0.0,
                        "robot_omega": 0.0,
                        "turret_angle": 0.0,
                        "angle_to_target": 0.0,
                        "angle_correction": float(parts[2]),
                        "top_power_correction": float(parts[3]),
                        "bottom_power_correction": float(parts[4]),
                        "battery_voltage": float(parts[5]),
                        "result": parts[6].upper(),
                    }
                # 6-field format (legacy, no voltage)
                elif len(parts) >= 6:
                    point = {
                        "target_type": parts[0].upper(),
                        "distance": float(parts[1]),
                        "robot_x": 0.0,
                        "robot_y": 0.0,
                        "robot_heading": 0.0,
                        "robot_vx": 0.0,
                        "robot_vy": 0.0,
                        "robot_omega": 0.0,
                        "turret_angle": 0.0,
                        "angle_to_target": 0.0,
                        "angle_correction": float(parts[2]),
                        "top_power_correction": float(parts[3]),
                        "bottom_power_correction": float(parts[4]),
                        "battery_voltage": REFERENCE_VOLTAGE,
                        "result": parts[5].upper(),
                    }
                
                if point is not None:
                    points.append(point)
                else:
                    print(f"[Train] Line {line_num}: not enough fields ({len(parts)}), skipping")
            
            except (ValueError, IndexError) as e:
                print(f"[Train] Line {line_num}: parse error - {e}")
    
    return points


def build_features(point: dict) -> list:
    """Build feature vector from a training point."""
    robot_speed = math.sqrt(point["robot_vx"]**2 + point["robot_vy"]**2)
    is_moving = 1.0 if robot_speed >= MIN_MOVING_SPEED_MPS else 0.0
    is_trench = 1.0 if point["target_type"] == "TRENCH" else 0.0
    
    return [
        is_trench,
        point["distance"],
        point["robot_x"],
        point["robot_y"],
        point["robot_heading"],
        point["robot_vx"],
        point["robot_vy"],
        point["robot_omega"],
        point["turret_angle"],
        point["angle_to_target"],
        robot_speed,
        is_moving,
        point["battery_voltage"],
    ]


def build_targets(point: dict) -> list:
    """Build target vector (what we want to predict)."""
    # Normalize power corrections to reference voltage
    voltage_ratio = point["battery_voltage"] / REFERENCE_VOLTAGE
    
    return [
        point["angle_correction"],
        point["top_power_correction"] * voltage_ratio,      # Normalized
        point["bottom_power_correction"] * voltage_ratio,    # Normalized
    ]


def train_model(csv_path: str, evaluate: bool = False):
    """Train the shooting prediction model."""
    
    print(f"[Train] Loading data from {csv_path}")
    all_points = parse_csv(csv_path)
    
    if not all_points:
        print("[Train] No training data found! Cannot train model.")
        print("[Train] Add training data to shooting_training_data.csv first.")
        return False
    
    print(f"[Train] Loaded {len(all_points)} total points")
    
    # Filter to HIT results only (we only want to learn from successful shots)
    hit_points = [p for p in all_points if p["result"] == "HIT"]
    print(f"[Train] {len(hit_points)} HIT points (used for training)")
    
    hub_hits = len([p for p in hit_points if p["target_type"] == "HUB"])
    trench_hits = len([p for p in hit_points if p["target_type"] == "TRENCH"])
    moving_hits = len([p for p in hit_points 
                       if math.sqrt(p["robot_vx"]**2 + p["robot_vy"]**2) >= MIN_MOVING_SPEED_MPS])
    print(f"[Train] HUB hits: {hub_hits}, TRENCH hits: {trench_hits}, Moving hits: {moving_hits}")
    
    if len(hit_points) < 3:
        print("[Train] Need at least 3 HIT points to train a model.")
        print("[Train] Continue collecting training data and try again.")
        return False
    
    # Build feature matrix and target matrix
    X = np.array([build_features(p) for p in hit_points])
    y = np.array([build_targets(p) for p in hit_points])
    
    feature_names = [
        "is_trench", "distance", "robot_x", "robot_y", "robot_heading",
        "robot_vx", "robot_vy", "robot_omega", "turret_angle", "angle_to_target",
        "robot_speed", "is_moving", "battery_voltage"
    ]
    
    target_names = ["angle_correction", "top_power_correction", "bottom_power_correction"]
    
    print(f"[Train] Feature matrix: {X.shape}")
    print(f"[Train] Target matrix: {y.shape}")
    
    # Train a Gradient Boosting Regressor with multi-output
    # GBR is good for small datasets and handles non-linear relationships well
    print("[Train] Training Gradient Boosting model...")
    
    # Adjust complexity based on dataset size
    n_estimators = min(200, max(50, len(hit_points) * 5))
    max_depth = min(5, max(2, len(hit_points) // 10))
    
    base_regressor = GradientBoostingRegressor(
        n_estimators=n_estimators,
        max_depth=max_depth,
        learning_rate=0.1,
        min_samples_split=max(2, len(hit_points) // 10),
        min_samples_leaf=max(1, len(hit_points) // 20),
        subsample=0.8,
        random_state=42,
    )
    
    model = Pipeline([
        ("scaler", StandardScaler()),
        ("regressor", MultiOutputRegressor(base_regressor)),
    ])
    
    model.fit(X, y)
    
    # Evaluate
    if evaluate and len(hit_points) >= 5:
        print("\n[Train] Cross-validation evaluation:")
        for i, name in enumerate(target_names):
            # Train individual models for CV scoring
            single_model = Pipeline([
                ("scaler", StandardScaler()),
                ("regressor", GradientBoostingRegressor(
                    n_estimators=n_estimators, max_depth=max_depth,
                    learning_rate=0.1, random_state=42)),
            ])
            cv_folds = min(5, len(hit_points))
            scores = cross_val_score(single_model, X, y[:, i], cv=cv_folds, scoring="r2")
            print(f"  {name}: R² = {scores.mean():.3f} (+/- {scores.std():.3f})")
    
    # Show prediction stats on training data
    predictions = model.predict(X)
    print("\n[Train] Training set prediction statistics:")
    for i, name in enumerate(target_names):
        residuals = y[:, i] - predictions[:, i]
        print(f"  {name}:")
        print(f"    Mean absolute error: {np.mean(np.abs(residuals)):.4f}")
        print(f"    Max absolute error:  {np.max(np.abs(residuals)):.4f}")
        print(f"    Std of residuals:    {np.std(residuals):.4f}")
    
    # Save model
    model_data = {
        "regressor": model,
        "feature_names": feature_names,
        "target_names": target_names,
        "info": {
            "trained_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "total_points": len(all_points),
            "hit_points": len(hit_points),
            "hub_hits": hub_hits,
            "trench_hits": trench_hits,
            "moving_hits": moving_hits,
            "n_estimators": n_estimators,
            "max_depth": max_depth,
        }
    }
    
    with open(MODEL_OUTPUT, "wb") as f:
        pickle.dump(model_data, f)
    
    file_size_kb = os.path.getsize(MODEL_OUTPUT) / 1024
    print(f"\n[Train] Model saved to {MODEL_OUTPUT} ({file_size_kb:.1f} KB)")
    print("[Train] Training complete!")
    return True


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train Kronk shooting prediction model")
    parser.add_argument("--csv", default=DEFAULT_CSV, help="Path to training CSV file")
    parser.add_argument("--evaluate", action="store_true", help="Run cross-validation evaluation")
    args = parser.parse_args()
    
    print("=" * 60)
    print("  Kronk Shooting Model Trainer")
    print("=" * 60)
    
    success = train_model(args.csv, evaluate=args.evaluate)
    sys.exit(0 if success else 1)
