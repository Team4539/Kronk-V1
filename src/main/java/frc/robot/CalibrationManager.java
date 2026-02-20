package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.TreeMap;

/**
 * Live-tuning sliders for calibration sessions.
 * 
 * SmartDashboard sliders for tuning, record button, and code export.
 * 
 * WORKFLOW:
 *   1. Drive to known distance from target
 *   2. Adjust Tuning/Shooter/TopPower and BottomPower sliders until shots land
 *   3. Press "RecordPoint" button to save the distance+power pair
 *   4. Repeat for multiple distances
 *   5. Press "PrintTable" to get copy-paste code for Constants.java
 */
public class CalibrationManager {
    
    private static CalibrationManager instance;
    
    public static CalibrationManager getInstance() {
        if (instance == null) instance = new CalibrationManager();
        return instance;
    }
    
    // --- Calibration values (read from SmartDashboard each cycle) ---
    
    private double shooterTopRPM = 2000.0;
    private double shooterBottomRPM = 2500.0;
    private double turretAngleOffset = 0.0;
    private double turretP = Constants.Turret.PID_P;
    private double turretI = Constants.Turret.PID_I;
    private double turretD = Constants.Turret.PID_D;
    
    /** Live RPM offset added to calculated top motor RPM (positive = more power) */
    private double topRPMOffset = 0.0;
    /** Live RPM offset added to calculated bottom motor RPM (positive = more power) */
    private double bottomRPMOffset = 0.0;
    
    // --- Recorded data ---
    
    private final TreeMap<Double, double[]> recordedPoints = new TreeMap<>();
    private int recordCount = 0;
    
    /** Recorded turret rotation offset points: distance (m) -> angle offset (deg) */
    private final TreeMap<Double, Double> recordedRotationPoints = new TreeMap<>();
    private int rotationRecordCount = 0;
    
    /** Live turret rotation offset being tuned (degrees) */
    private double turretRotationOffset = 0.0;
    
    // --- Live measurement (set externally) ---
    
    private double currentDistance = 0.0;
    
    // ========================================================================
    // INIT
    // ========================================================================
    
    private CalibrationManager() {
        // Shooter tuning (RPM-based)
        SmartDashboard.putNumber("Tuning/Shooter/TopRPM", shooterTopRPM);
        SmartDashboard.putNumber("Tuning/Shooter/BottomRPM", shooterBottomRPM);
        
        // Live RPM offsets (applied on top of calibration table values)
        // Positive = more power, Negative = less power
        SmartDashboard.putNumber("Tuning/Shooter/TopRPMOffset", topRPMOffset);
        SmartDashboard.putNumber("Tuning/Shooter/BottomRPMOffset", bottomRPMOffset);
        
        // Turret tuning
        SmartDashboard.putNumber("Tuning/Turret/AngleOffset", turretAngleOffset);
        SmartDashboard.putNumber("Tuning/Turret/P", turretP);
        SmartDashboard.putNumber("Tuning/Turret/I", turretI);
        SmartDashboard.putNumber("Tuning/Turret/D", turretD);
        
        // Recording controls
        SmartDashboard.putBoolean("Tuning/RecordPoint", false);
        SmartDashboard.putBoolean("Tuning/PrintTable", false);
        SmartDashboard.putBoolean("Tuning/ClearData", false);
        SmartDashboard.putNumber("Tuning/RecordCount", 0);
        SmartDashboard.putString("Tuning/Status", "Ready");
        
        // Turret rotation offset calibration (per-distance turret angle offset)
        SmartDashboard.putNumber("Tuning/Turret/RotationOffset", turretRotationOffset);
        SmartDashboard.putBoolean("Tuning/RecordRotationPoint", false);
        SmartDashboard.putBoolean("Tuning/PrintRotationTable", false);
        SmartDashboard.putBoolean("Tuning/ClearRotationData", false);
        SmartDashboard.putNumber("Tuning/RotationRecordCount", 0);
    }
    
    // ========================================================================
    // UPDATE (call from robotPeriodic)
    // ========================================================================
    
    public void update() {
        // Read tuning values
        shooterTopRPM = SmartDashboard.getNumber("Tuning/Shooter/TopRPM", shooterTopRPM);
        shooterBottomRPM = SmartDashboard.getNumber("Tuning/Shooter/BottomRPM", shooterBottomRPM);
        topRPMOffset = SmartDashboard.getNumber("Tuning/Shooter/TopRPMOffset", topRPMOffset);
        bottomRPMOffset = SmartDashboard.getNumber("Tuning/Shooter/BottomRPMOffset", bottomRPMOffset);
        turretAngleOffset = SmartDashboard.getNumber("Tuning/Turret/AngleOffset", turretAngleOffset);
        turretP = SmartDashboard.getNumber("Tuning/Turret/P", turretP);
        turretI = SmartDashboard.getNumber("Tuning/Turret/I", turretI);
        turretD = SmartDashboard.getNumber("Tuning/Turret/D", turretD);
        turretRotationOffset = SmartDashboard.getNumber("Tuning/Turret/RotationOffset", turretRotationOffset);
        
        // Check button presses - Shooting calibration
        if (SmartDashboard.getBoolean("Tuning/RecordPoint", false)) {
            recordShootingPoint();
            SmartDashboard.putBoolean("Tuning/RecordPoint", false);
        }
        if (SmartDashboard.getBoolean("Tuning/PrintTable", false)) {
            printCalibrationTable();
            SmartDashboard.putBoolean("Tuning/PrintTable", false);
        }
        if (SmartDashboard.getBoolean("Tuning/ClearData", false)) {
            clearData();
            SmartDashboard.putBoolean("Tuning/ClearData", false);
        }
        
        // Check button presses - Rotation offset calibration
        if (SmartDashboard.getBoolean("Tuning/RecordRotationPoint", false)) {
            recordRotationPoint();
            SmartDashboard.putBoolean("Tuning/RecordRotationPoint", false);
        }
        if (SmartDashboard.getBoolean("Tuning/PrintRotationTable", false)) {
            printRotationCalibrationTable();
            SmartDashboard.putBoolean("Tuning/PrintRotationTable", false);
        }
        if (SmartDashboard.getBoolean("Tuning/ClearRotationData", false)) {
            clearRotationData();
            SmartDashboard.putBoolean("Tuning/ClearRotationData", false);
        }
    }
    
    // ========================================================================
    // RECORDING
    // ========================================================================
    
    private void recordShootingPoint() {
        if (currentDistance <= 0) {
            SmartDashboard.putString("Tuning/Status", "ERROR: No distance!");
            return;
        }
        
        double roundedDist = Math.round(currentDistance * 4.0) / 4.0;
        recordedPoints.put(roundedDist, new double[]{shooterTopRPM, shooterBottomRPM});
        recordCount++;
        
        String code = String.format("put(%.2f, new double[]{%.0f, %.0f});", roundedDist, shooterTopRPM, shooterBottomRPM);
        SmartDashboard.putNumber("Tuning/RecordCount", recordCount);
        SmartDashboard.putString("Tuning/Status", "Recorded #" + recordCount + ": " + code);
        System.out.println("[CAL] " + code);
    }
    
    private void printCalibrationTable() {
        if (recordedPoints.isEmpty()) {
            SmartDashboard.putString("Tuning/Status", "No data to print");
            return;
        }
        
        System.out.println("\n=== COPY TO Constants.Shooter.SHOOTING_CALIBRATION ===");
        System.out.println("public static final TreeMap<Double, double[]> SHOOTING_CALIBRATION = new TreeMap<>() {{");
        for (var entry : recordedPoints.entrySet()) {
            System.out.printf("  put(%.2f, new double[]{%.0f, %.0f});%n",
                entry.getKey(), entry.getValue()[0], entry.getValue()[1]);
        }
        System.out.println("}};");
        System.out.println("=====================================================\n");
        
        SmartDashboard.putString("Tuning/Status", "Printed " + recordedPoints.size() + " points to console");
    }
    
    private void clearData() {
        recordedPoints.clear();
        recordCount = 0;
        SmartDashboard.putNumber("Tuning/RecordCount", 0);
        SmartDashboard.putString("Tuning/Status", "Data cleared");
    }
    
    // ========================================================================
    // ROTATION OFFSET RECORDING
    // ========================================================================
    
    private void recordRotationPoint() {
        if (currentDistance <= 0) {
            SmartDashboard.putString("Tuning/Status", "ERROR: No distance for rotation point!");
            return;
        }
        
        double roundedDist = Math.round(currentDistance * 4.0) / 4.0;
        recordedRotationPoints.put(roundedDist, turretRotationOffset);
        rotationRecordCount++;
        
        String code = String.format("put(%.2f, %.2f);", roundedDist, turretRotationOffset);
        SmartDashboard.putNumber("Tuning/RotationRecordCount", rotationRecordCount);
        SmartDashboard.putString("Tuning/Status", "Rotation #" + rotationRecordCount + ": " + code);
        System.out.println("[CAL-ROT] " + code);
    }
    
    private void printRotationCalibrationTable() {
        if (recordedRotationPoints.isEmpty()) {
            SmartDashboard.putString("Tuning/Status", "No rotation data to print");
            return;
        }
        
        System.out.println("\n=== COPY TO Constants.Turret.ROTATION_CALIBRATION ===");
        System.out.println("public static final TreeMap<Double, Double> ROTATION_CALIBRATION = new TreeMap<>() {{");
        for (var entry : recordedRotationPoints.entrySet()) {
            System.out.printf("  put(%.2f, %.2f);%n", entry.getKey(), entry.getValue());
        }
        System.out.println("}};");
        System.out.println("=====================================================\n");
        
        SmartDashboard.putString("Tuning/Status", "Printed " + recordedRotationPoints.size() + " rotation points to console");
    }
    
    private void clearRotationData() {
        recordedRotationPoints.clear();
        rotationRecordCount = 0;
        SmartDashboard.putNumber("Tuning/RotationRecordCount", 0);
        SmartDashboard.putString("Tuning/Status", "Rotation data cleared");
    }
    
    // ========================================================================
    // EXTERNAL SETTERS
    // ========================================================================
    
    public void setCurrentDistance(double distance) { this.currentDistance = distance; }
    
    // ========================================================================
    // GETTERS (used by commands and subsystems)
    // ========================================================================
    
    public double getShooterTopRPM() { return shooterTopRPM; }
    public double getShooterBottomRPM() { return shooterBottomRPM; }
    public double getTopRPMOffset() { return topRPMOffset; }
    public double getBottomRPMOffset() { return bottomRPMOffset; }
    public double getTurretAngleOffset() { return turretAngleOffset; }
    public double getTurretRotationOffset() { return turretRotationOffset; }
    public double getTurretP() { return turretP; }
    public double getTurretI() { return turretI; }
    public double getTurretD() { return turretD; }
    public double getCurrentDistance() { return currentDistance; }
    public int getRecordCount() { return recordCount; }
}
