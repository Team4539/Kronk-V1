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
    
    private double shooterTopPower = 0.5;
    private double shooterBottomPower = 0.5;
    private double turretAngleOffset = 0.0;
    private double turretP = Constants.Turret.PID_P;
    private double turretI = Constants.Turret.PID_I;
    private double turretD = Constants.Turret.PID_D;
    
    // --- Recorded data ---
    
    private final TreeMap<Double, double[]> recordedPoints = new TreeMap<>();
    private int recordCount = 0;
    
    // --- Live measurement (set externally) ---
    
    private double currentDistance = 0.0;
    
    // ========================================================================
    // INIT
    // ========================================================================
    
    private CalibrationManager() {
        // Shooter tuning
        SmartDashboard.putNumber("Tuning/Shooter/TopPower", shooterTopPower);
        SmartDashboard.putNumber("Tuning/Shooter/BottomPower", shooterBottomPower);
        
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
    }
    
    // ========================================================================
    // UPDATE (call from robotPeriodic)
    // ========================================================================
    
    public void update() {
        // Read tuning values
        shooterTopPower = SmartDashboard.getNumber("Tuning/Shooter/TopPower", shooterTopPower);
        shooterBottomPower = SmartDashboard.getNumber("Tuning/Shooter/BottomPower", shooterBottomPower);
        turretAngleOffset = SmartDashboard.getNumber("Tuning/Turret/AngleOffset", turretAngleOffset);
        turretP = SmartDashboard.getNumber("Tuning/Turret/P", turretP);
        turretI = SmartDashboard.getNumber("Tuning/Turret/I", turretI);
        turretD = SmartDashboard.getNumber("Tuning/Turret/D", turretD);
        
        // Check button presses
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
        recordedPoints.put(roundedDist, new double[]{shooterTopPower, shooterBottomPower});
        recordCount++;
        
        String code = String.format("put(%.2f, new double[]{%.2f, %.2f});", roundedDist, shooterTopPower, shooterBottomPower);
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
            System.out.printf("  put(%.2f, new double[]{%.2f, %.2f});%n",
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
    // EXTERNAL SETTERS
    // ========================================================================
    
    public void setCurrentDistance(double distance) { this.currentDistance = distance; }
    
    // ========================================================================
    // GETTERS (used by commands and subsystems)
    // ========================================================================
    
    public double getShooterTopPower() { return shooterTopPower; }
    public double getShooterBottomPower() { return shooterBottomPower; }
    public double getTurretAngleOffset() { return turretAngleOffset; }
    public double getTurretP() { return turretP; }
    public double getTurretI() { return turretI; }
    public double getTurretD() { return turretD; }
    public double getCurrentDistance() { return currentDistance; }
    public int getRecordCount() { return recordCount; }
}
