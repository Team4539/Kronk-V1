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
    
    // --- Recorded data (unified: x, y, bearing, turretOffset, topRPM, bottomRPM) ---
    
    /** Recorded unified calibration points: {robotX, robotY, bearingDeg, turretOffsetDeg, topRPM, bottomRPM} */
    private final List<double[]> recordedPoints = new ArrayList<>();
    private int recordCount = 0;
    
    /** Live turret rotation offset being tuned (degrees) */
    private double turretRotationOffset = 0.0;
    
    /** Whether a calibration session is active (offsets have been zeroed) */
    private boolean calibrationSessionActive = false;
    
    // --- Live measurement (set externally from ShootingCalculator) ---
    
    private double currentDistance = 0.0;
    
    /** Robot-relative bearing to target (degrees, -180 to +180). Set from ShootingCalculator. */
    private double currentBearing = 0.0;
    
    /** Turret X position relative to target (meters). Alliance-independent. */
    private double currentX = 0.0;
    
    /** Turret Y position relative to target (meters). Alliance-independent. */
    private double currentY = 0.0;
    
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
        
        // Recording controls (unified: records turret + shooter together)
        SmartDashboard.putBoolean("Tuning/RecordPoint", false);
        SmartDashboard.putBoolean("Tuning/PrintTable", false);
        SmartDashboard.putBoolean("Tuning/ClearData", false);
        SmartDashboard.putNumber("Tuning/RecordCount", 0);
        SmartDashboard.putString("Tuning/Status", "Ready");
        
        // Turret rotation offset (still live-tunable per-session)
        SmartDashboard.putNumber("Tuning/Turret/RotationOffset", turretRotationOffset);
        
        // Start/End calibration buttons
        // Start: zeroes ALL offsets + bypasses baked-in Constants offsets
        // End: re-enables baked-in Constants offsets for normal operation
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
        shooterTopRPM = SmartDashboard.getNumber("Tuning/Shooter/TopRPM", shooterTopRPM);
        shooterBottomRPM = SmartDashboard.getNumber("Tuning/Shooter/BottomRPM", shooterBottomRPM);
        topRPMOffset = SmartDashboard.getNumber("Tuning/Shooter/TopRPMOffset", topRPMOffset);
        bottomRPMOffset = SmartDashboard.getNumber("Tuning/Shooter/BottomRPMOffset", bottomRPMOffset);
        turretAngleOffset = SmartDashboard.getNumber("Tuning/Turret/AngleOffset", turretAngleOffset);
        turretP = SmartDashboard.getNumber("Tuning/Turret/P", turretP);
        turretI = SmartDashboard.getNumber("Tuning/Turret/I", turretI);
        turretD = SmartDashboard.getNumber("Tuning/Turret/D", turretD);
        turretRotationOffset = SmartDashboard.getNumber("Tuning/Turret/RotationOffset", turretRotationOffset);
        
        // Check button presses - Unified shooting + turret calibration
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
     * Call this BEFORE adjusting any sliders to avoid stacking offsets on top of old offsets.
     * 
     * Resets (live sliders):
     *   - Turret angle offset → 0
     *   - Turret rotation offset → 0
     *   - Top RPM offset → 0
     *   - Bottom RPM offset → 0
     *   - Recorded data (shooting + rotation points)
     * 
     * Bypasses (baked-in Constants — can't be zeroed, so ShootingCalculator skips them):
     *   - Constants.Turret.GLOBAL_ANGLE_OFFSET_DEG
     *   - Constants.Shooter.SHOOTING_CALIBRATION (unified pose-based calibration table)
     */
    private void resetAllOffsets() {
        // Zero all live offset values
        turretAngleOffset = 0.0;
        turretRotationOffset = 0.0;
        topRPMOffset = 0.0;
        bottomRPMOffset = 0.0;
        
        // Push zeroes back to SmartDashboard so the sliders visually reset
        SmartDashboard.putNumber("Tuning/Turret/AngleOffset", 0.0);
        SmartDashboard.putNumber("Tuning/Turret/RotationOffset", 0.0);
        SmartDashboard.putNumber("Tuning/Shooter/TopRPMOffset", 0.0);
        SmartDashboard.putNumber("Tuning/Shooter/BottomRPMOffset", 0.0);
        
        // Tell ShootingCalculator to bypass ALL baked-in Constants offsets
        // (GLOBAL_ANGLE_OFFSET_DEG, unified SHOOTING_CALIBRATION) so we're tuning from raw
        ShootingCalculator.getInstance().setCalibrationMode(true);
        
        // Clear any previously recorded points so they don't mix with new session
        recordedPoints.clear();
        recordCount = 0;
        SmartDashboard.putNumber("Tuning/RecordCount", 0);
        
        // Mark session active
        calibrationSessionActive = true;
        SmartDashboard.putBoolean("Tuning/CalibrationActive", true);
        SmartDashboard.putString("Tuning/Status", "✓ CALIBRATION STARTED — All offsets zeroed + baked-in bypassed");
        System.out.println("[CAL] === CALIBRATION SESSION STARTED — All offsets reset, baked-in offsets bypassed ===");
    }
    
    /**
     * Ends the calibration session and re-enables baked-in Constants offsets.
     * Call this when done calibrating to return to normal operation.
     */
    private void endCalibration() {
        // Re-enable baked-in Constants offsets in ShootingCalculator
        ShootingCalculator.getInstance().setCalibrationMode(false);
        
        calibrationSessionActive = false;
        SmartDashboard.putBoolean("Tuning/CalibrationActive", false);
        SmartDashboard.putString("Tuning/Status", "Calibration ended — baked-in offsets re-enabled");
        System.out.println("[CAL] === CALIBRATION SESSION ENDED — baked-in offsets re-enabled ===");
    }
    
    // ========================================================================
    // RECORDING
    // ========================================================================
    
    private void recordShootingPoint() {
        if (currentDistance <= 0) {
            SmartDashboard.putString("Tuning/Status", "ERROR: No distance!");
            return;
        }
        
        // Record unified point: {relX, relY, bearingDeg, turretOffsetDeg, topRPM, bottomRPM}
        // relX/relY are relative to the target (turret pos - target pos), so alliance-independent.
        double roundedX = Math.round(currentX * 100.0) / 100.0;
        double roundedY = Math.round(currentY * 100.0) / 100.0;
        double roundedBearing = Math.round(currentBearing);
        
        recordedPoints.add(new double[]{
            roundedX, roundedY, roundedBearing,
            turretRotationOffset, shooterTopRPM, shooterBottomRPM
        });
        recordCount++;
        
        String code = String.format("add(new double[]{%.2f, %.2f, %.0f, %.2f, %.0f, %.0f});",
                roundedX, roundedY, roundedBearing,
                turretRotationOffset, shooterTopRPM, shooterBottomRPM);
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
            System.out.printf("  add(new double[]{%.2f, %.2f, %.0f, %.2f, %.0f, %.0f}); " +
                            "// relX=%.2f relY=%.2f bearing=%.0f° offset=%.2f° top=%.0f bot=%.0f%n",
                    point[0], point[1], point[2], point[3], point[4], point[5],
                    point[0], point[1], point[2], point[3], point[4], point[5]);
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
    
    /** Set the turret X position relative to target (from ShootingCalculator.getRelativeX()) */
    public void setCurrentX(double x) { this.currentX = x; }
    
    /** Set the turret Y position relative to target (from ShootingCalculator.getRelativeY()) */
    public void setCurrentY(double y) { this.currentY = y; }
    
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
    public double getCurrentBearing() { return currentBearing; }
    public double getCurrentX() { return currentX; }
    public double getCurrentY() { return currentY; }
    public int getRecordCount() { return recordCount; }
}
