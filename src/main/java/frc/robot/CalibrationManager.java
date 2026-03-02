package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.ShootingCalculator;

import java.util.ArrayList;
import java.util.List;

/**
 * Live-tuning sliders for calibration sessions.
 * 
 * SmartDashboard sliders for tuning, record button, and code export.
 * 
 * WORKFLOW:
 *   1. Drive to known distance from target
 *   2. Adjust Tuning/Shooter/RPM slider until shots land
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
    
    private double shooterRPM = 2000.0;
    
    /** Live RPM offset added to calculated motor RPM (positive = more power) */
    private double rpmOffset = 0.0;
    
    // --- Recorded data (unified: x, y, bearing, shooterRPM) ---
    
    /** Recorded unified calibration points: {robotX, robotY, bearingDeg, shooterRPM} */
    private final List<double[]> recordedPoints = new ArrayList<>();
    private int recordCount = 0;
    
    /** Whether a calibration session is active (offsets have been zeroed) */
    private boolean calibrationSessionActive = false;
    
    // --- Live measurement (set externally from ShootingCalculator) ---
    
    private double currentDistance = 0.0;
    
    /** Robot-relative bearing to target (degrees, -180 to +180). Set from ShootingCalculator. */
    private double currentBearing = 0.0;
    
    /** Robot X position relative to target (meters). Alliance-independent. */
    private double currentX = 0.0;
    
    /** Robot Y position relative to target (meters). Alliance-independent. */
    private double currentY = 0.0;
    
    // ========================================================================
    // INIT
    // ========================================================================
    
    private CalibrationManager() {
        // Shooter tuning (RPM-based)
        SmartDashboard.putNumber("Tuning/Shooter/RPM", shooterRPM);
        
        // Live RPM offset (applied on top of calibration table values)
        // Positive = more power, Negative = less power
        SmartDashboard.putNumber("Tuning/Shooter/RPMOffset", rpmOffset);
        
        // Recording controls (unified: records shooter data)
        SmartDashboard.putBoolean("Tuning/RecordPoint", false);
        SmartDashboard.putBoolean("Tuning/PrintTable", false);
        SmartDashboard.putBoolean("Tuning/ClearData", false);
        SmartDashboard.putNumber("Tuning/RecordCount", 0);
        SmartDashboard.putString("Tuning/Status", "Ready");
        
        // Start/End calibration buttons
        SmartDashboard.putBoolean("Tuning/StartCalibration", false);
        SmartDashboard.putBoolean("Tuning/EndCalibration", false);
        SmartDashboard.putBoolean("Tuning/CalibrationActive", false);
    }
    
    // ========================================================================
    // UPDATE (call from robotPeriodic)
    // ========================================================================
    
    public void update() {
        // --- Start Calibration button: zero ALL offsets + bypass baked-in for a clean session ---
        if (SmartDashboard.getBoolean("Tuning/StartCalibration", false)) {
            resetAllOffsets();
            SmartDashboard.putBoolean("Tuning/StartCalibration", false);
        }
        
        // --- End Calibration button: re-enable baked-in Constants offsets ---
        if (SmartDashboard.getBoolean("Tuning/EndCalibration", false)) {
            endCalibration();
            SmartDashboard.putBoolean("Tuning/EndCalibration", false);
        }
        
        // Read tuning values
        shooterRPM = SmartDashboard.getNumber("Tuning/Shooter/RPM", shooterRPM);
        rpmOffset = SmartDashboard.getNumber("Tuning/Shooter/RPMOffset", rpmOffset);
        
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
    // RESET ALL OFFSETS
    // ========================================================================
    
    /**
     * Zeros every live offset slider AND bypasses baked-in Constants offsets
     * so the next calibration session starts from a completely raw baseline.
     * 
     * Resets (live sliders):
     *   - RPM offset = 0
     *   - Recorded data (shooting points)
     * 
     * Bypasses (baked-in Constants):
     *   - Constants.Shooter.SHOOTING_CALIBRATION (unified pose-based calibration table)
     */
    private void resetAllOffsets() {
        // Zero all live offset values
        rpmOffset = 0.0;
        
        // Push zeroes back to SmartDashboard so the sliders visually reset
        SmartDashboard.putNumber("Tuning/Shooter/RPMOffset", 0.0);
        
        // Tell ShootingCalculator to bypass ALL baked-in Constants offsets
        ShootingCalculator.getInstance().setCalibrationMode(true);
        
        // Clear any previously recorded points so they don't mix with new session
        recordedPoints.clear();
        recordCount = 0;
        SmartDashboard.putNumber("Tuning/RecordCount", 0);
        
        // Mark session active
        calibrationSessionActive = true;
        SmartDashboard.putBoolean("Tuning/CalibrationActive", true);
        SmartDashboard.putString("Tuning/Status", "CALIBRATION STARTED - All offsets zeroed, baked-in offsets bypassed");
        System.out.println("[CAL] === CALIBRATION SESSION STARTED - All offsets reset, baked-in offsets bypassed ===");
    }
    
    /**
     * Ends the calibration session and re-enables baked-in Constants offsets.
     */
    private void endCalibration() {
        ShootingCalculator.getInstance().setCalibrationMode(false);
        
        calibrationSessionActive = false;
        SmartDashboard.putBoolean("Tuning/CalibrationActive", false);
        SmartDashboard.putString("Tuning/Status", "Calibration ended - baked-in offsets re-enabled");
        System.out.println("[CAL] === CALIBRATION SESSION ENDED - Baked-in offsets re-enabled ===");
    }
    
    // ========================================================================
    // RECORDING
    // ========================================================================
    
    private void recordShootingPoint() {
        if (currentDistance <= 0) {
            SmartDashboard.putString("Tuning/Status", "ERROR: No distance!");
            return;
        }
        
        // Record unified point: {relX, relY, bearingDeg, shooterRPM}
        double roundedX = Math.round(currentX * 100.0) / 100.0;
        double roundedY = Math.round(currentY * 100.0) / 100.0;
        double roundedBearing = Math.round(currentBearing);
        
        recordedPoints.add(new double[]{
            roundedX, roundedY, roundedBearing, shooterRPM
        });
        recordCount++;
        
        String code = String.format("add(new double[]{%.2f, %.2f, %.0f, %.0f});",
                roundedX, roundedY, roundedBearing, shooterRPM);
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
        System.out.println("public static final List<double[]> SHOOTING_CALIBRATION = new ArrayList<>() {{");
        for (double[] point : recordedPoints) {
            System.out.printf("  add(new double[]{%.2f, %.2f, %.0f, %.0f}); " +
                            "// relX=%.2f relY=%.2f bearing=%.0f\u00b0 rpm=%.0f%n",
                    point[0], point[1], point[2], point[3],
                    point[0], point[1], point[2], point[3]);
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
    
    /** Set the current robot-relative bearing to target (from ShootingCalculator.getRawBearing()) */
    public void setCurrentBearing(double bearing) { this.currentBearing = bearing; }
    
    /** Set the robot X position relative to target (from ShootingCalculator.getRelativeX()) */
    public void setCurrentX(double x) { this.currentX = x; }
    
    /** Set the robot Y position relative to target (from ShootingCalculator.getRelativeY()) */
    public void setCurrentY(double y) { this.currentY = y; }
    
    // ========================================================================
    // GETTERS (used by commands and subsystems)
    // ========================================================================
    
    public double getShooterRPM() { return shooterRPM; }
    public double getRPMOffset() { return rpmOffset; }
    public double getCurrentDistance() { return currentDistance; }
    public double getCurrentBearing() { return currentBearing; }
    public double getCurrentX() { return currentX; }
    public double getCurrentY() { return currentY; }
    public int getRecordCount() { return recordCount; }
}
