package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;

import java.util.TreeMap;

/**
 * Manages all calibration data for the robot.
 * Provides SmartDashboard sliders for live tuning and outputs ready-to-paste code for Constants.java.
 * All values are logged to NetworkTables for AdvantageScope.
 */
public class CalibrationManager {
    
    // Singleton
    
    private static CalibrationManager instance;
    
    public static CalibrationManager getInstance() {
        if (instance == null) {
            instance = new CalibrationManager();
        }
        return instance;
    }
    
    // Network Tables for AdvantageScope
    
    private final NetworkTable calibrationTable;
    private final NetworkTable shooterTable;
    private final NetworkTable turretTable;
    private final NetworkTable limelightTable;
    private final NetworkTable recordedTable;
    
    // Shooter calibration values
    
    /** Manual top motor power override (0.0 - 1.0) */
    private double shooterTopPower = 0.5;
    
    /** Manual bottom motor power override (0.0 - 1.0) */
    private double shooterBottomPower = 0.5;
    
    /** Global top motor power offset */
    private double shooterTopOffset = 0.0;
    
    /** Global bottom motor power offset */
    private double shooterBottomOffset = 0.0;
    
    /** Whether to use manual power override vs interpolated */
    private boolean useManualShooterPower = false;
    
    // Turret calibration values
    
    /** Manual turret angle override (degrees) */
    private double turretAngle = 0.0;
    
    /** Global turret angle offset (degrees) */
    private double turretAngleOffset = 0.0;
    
    /** Whether to use manual angle override vs auto-aim */
    private boolean useManualTurretAngle = false;
    
    /** Turret PID P gain */
    private double turretP = Constants.Turret.PID_P;
    
    /** Turret PID I gain */
    private double turretI = Constants.Turret.PID_I;
    
    /** Turret PID D gain */
    private double turretD = Constants.Turret.PID_D;
    
    // LIMELIGHT CALIBRATION VALUES
    
    /** Camera X offset from robot center (meters) */
    private double limelightXOffset = Constants.Limelight.CAMERA_X_OFFSET;
    
    /** Camera Y offset from robot center (meters) */
    private double limelightYOffset = Constants.Limelight.CAMERA_Y_OFFSET;
    
    /** Camera Z offset (height) from ground (meters) */
    private double limelightZOffset = Constants.Limelight.CAMERA_Z_OFFSET;
    
    /** Camera pitch angle (degrees) */
    private double limelightPitch = Constants.Limelight.CAMERA_PITCH_DEGREES;
    
    /** Vision X standard deviation */
    private double visionStdDevX = Constants.Limelight.VISION_STD_DEV_X;
    
    /** Vision Y standard deviation */
    private double visionStdDevY = Constants.Limelight.VISION_STD_DEV_Y;
    
    /** Vision rotation standard deviation */
    private double visionStdDevTheta = Constants.Limelight.VISION_STD_DEV_THETA;
    
    // Recorded calibration data
    private final TreeMap<Double, double[]> recordedShootingPoints = new TreeMap<>();
    private final TreeMap<Integer, Double> recordedTurretOffsets = new TreeMap<>();
    private int recordCount = 0;
    
    // Live measurement data
    private double currentDistance = 0.0;
    private int currentTagId = -1;
    private double currentTurretAngle = 0.0;
    private boolean hasTarget = false;
    
    // Constructor
    
    private CalibrationManager() {
        calibrationTable = NetworkTableInstance.getDefault().getTable("Calibration");
        shooterTable = calibrationTable.getSubTable("Shooter");
        turretTable = calibrationTable.getSubTable("Turret");
        limelightTable = calibrationTable.getSubTable("Limelight");
        recordedTable = calibrationTable.getSubTable("Recorded");
        
        // Initialize SmartDashboard controls
        initializeSmartDashboard();
    }
    
    // INITIALIZATION
    
    private void initializeSmartDashboard() {
        // ----- Shooter Calibration Controls -----
        SmartDashboard.putNumber("Cal/Shooter/TopPower", shooterTopPower);
        SmartDashboard.putNumber("Cal/Shooter/BottomPower", shooterBottomPower);
        SmartDashboard.putNumber("Cal/Shooter/TopOffset", shooterTopOffset);
        SmartDashboard.putNumber("Cal/Shooter/BottomOffset", shooterBottomOffset);
        SmartDashboard.putBoolean("Cal/Shooter/UseManual", useManualShooterPower);
        
        // ----- Turret Calibration Controls -----
        SmartDashboard.putNumber("Cal/Turret/ManualAngle", turretAngle);
        SmartDashboard.putNumber("Cal/Turret/AngleOffset", turretAngleOffset);
        SmartDashboard.putBoolean("Cal/Turret/UseManual", useManualTurretAngle);
        SmartDashboard.putNumber("Cal/Turret/P", turretP);
        SmartDashboard.putNumber("Cal/Turret/I", turretI);
        SmartDashboard.putNumber("Cal/Turret/D", turretD);
        
        // ----- Limelight Calibration Controls -----
        SmartDashboard.putNumber("Cal/Limelight/XOffset", limelightXOffset);
        SmartDashboard.putNumber("Cal/Limelight/YOffset", limelightYOffset);
        SmartDashboard.putNumber("Cal/Limelight/ZOffset", limelightZOffset);
        SmartDashboard.putNumber("Cal/Limelight/Pitch", limelightPitch);
        SmartDashboard.putNumber("Cal/Limelight/VisionStdDevX", visionStdDevX);
        SmartDashboard.putNumber("Cal/Limelight/VisionStdDevY", visionStdDevY);
        SmartDashboard.putNumber("Cal/Limelight/VisionStdDevTheta", visionStdDevTheta);
        
        // ----- Recording Controls -----
        SmartDashboard.putBoolean("Cal/RecordShootingPoint", false);
        SmartDashboard.putBoolean("Cal/RecordTurretOffset", false);
        SmartDashboard.putNumber("Cal/RecordCount", recordCount);
        
        // ----- Status Display -----
        SmartDashboard.putString("Cal/Status", "Ready");
        SmartDashboard.putString("Cal/LastRecorded", "None");
        
        // ----- Export Buttons -----
        SmartDashboard.putBoolean("Cal/PrintShooterTable", false);
        SmartDashboard.putBoolean("Cal/PrintTurretOffsets", false);
        SmartDashboard.putBoolean("Cal/PrintAllConstants", false);
        
        // ----- Data Management -----
        SmartDashboard.putBoolean("Cal/ClearAllData", false);
        
        // ----- Game State Controls -----
        SmartDashboard.putBoolean("Cal/ToggleShuttleMode", false);
        SmartDashboard.putBoolean("Cal/ToggleForceShoot", false);
        SmartDashboard.putBoolean("Cal/ClearShuttleOverride", false);
        SmartDashboard.putBoolean("Cal/ResetGameState", false);
    }
    
    // PERIODIC UPDATE
    
    /**
     * Call this from robotPeriodic() to update calibration values from SmartDashboard
     * and log to AdvantageScope.
     */
    public void update() {
        // ----- Read Shooter Values -----
        shooterTopPower = SmartDashboard.getNumber("Cal/Shooter/TopPower", shooterTopPower);
        shooterBottomPower = SmartDashboard.getNumber("Cal/Shooter/BottomPower", shooterBottomPower);
        shooterTopOffset = SmartDashboard.getNumber("Cal/Shooter/TopOffset", shooterTopOffset);
        shooterBottomOffset = SmartDashboard.getNumber("Cal/Shooter/BottomOffset", shooterBottomOffset);
        useManualShooterPower = SmartDashboard.getBoolean("Cal/Shooter/UseManual", useManualShooterPower);
        
        // ----- Read Turret Values -----
        turretAngle = SmartDashboard.getNumber("Cal/Turret/ManualAngle", turretAngle);
        turretAngleOffset = SmartDashboard.getNumber("Cal/Turret/AngleOffset", turretAngleOffset);
        useManualTurretAngle = SmartDashboard.getBoolean("Cal/Turret/UseManual", useManualTurretAngle);
        turretP = SmartDashboard.getNumber("Cal/Turret/P", turretP);
        turretI = SmartDashboard.getNumber("Cal/Turret/I", turretI);
        turretD = SmartDashboard.getNumber("Cal/Turret/D", turretD);
        
        // ----- Read Limelight Values -----
        limelightXOffset = SmartDashboard.getNumber("Cal/Limelight/XOffset", limelightXOffset);
        limelightYOffset = SmartDashboard.getNumber("Cal/Limelight/YOffset", limelightYOffset);
        limelightZOffset = SmartDashboard.getNumber("Cal/Limelight/ZOffset", limelightZOffset);
        limelightPitch = SmartDashboard.getNumber("Cal/Limelight/Pitch", limelightPitch);
        visionStdDevX = SmartDashboard.getNumber("Cal/Limelight/VisionStdDevX", visionStdDevX);
        visionStdDevY = SmartDashboard.getNumber("Cal/Limelight/VisionStdDevY", visionStdDevY);
        visionStdDevTheta = SmartDashboard.getNumber("Cal/Limelight/VisionStdDevTheta", visionStdDevTheta);
        
        // ----- Check for Recording Requests -----
        if (SmartDashboard.getBoolean("Cal/RecordShootingPoint", false)) {
            recordShootingPoint();
            SmartDashboard.putBoolean("Cal/RecordShootingPoint", false);
        }
        
        if (SmartDashboard.getBoolean("Cal/RecordTurretOffset", false)) {
            recordTurretOffset();
            SmartDashboard.putBoolean("Cal/RecordTurretOffset", false);
        }
        
        // ----- Check for Export Requests -----
        if (SmartDashboard.getBoolean("Cal/PrintShooterTable", false)) {
            printShooterCalibrationTable();
            SmartDashboard.putBoolean("Cal/PrintShooterTable", false);
        }
        
        if (SmartDashboard.getBoolean("Cal/PrintTurretOffsets", false)) {
            printTurretOffsets();
            SmartDashboard.putBoolean("Cal/PrintTurretOffsets", false);
        }
        
        if (SmartDashboard.getBoolean("Cal/PrintAllConstants", false)) {
            printAllConstants();
            SmartDashboard.putBoolean("Cal/PrintAllConstants", false);
        }
        
        // ----- Check for Data Management Requests -----
        if (SmartDashboard.getBoolean("Cal/ClearAllData", false)) {
            clearRecordedData();
            SmartDashboard.putBoolean("Cal/ClearAllData", false);
        }
        
        // ----- Check for Game State Requests -----
        GameStateManager gameState = GameStateManager.getInstance();
        
        if (SmartDashboard.getBoolean("Cal/ToggleShuttleMode", false)) {
            boolean newMode = !gameState.isShuttleMode();
            gameState.setShuttleMode(newMode);
            logInfo(newMode ? "Shuttle Mode ON" : "Hub Mode ON",
                newMode ? "Aiming at trench" : "Aiming at hub");
            SmartDashboard.putBoolean("Cal/ToggleShuttleMode", false);
        }
        
        if (SmartDashboard.getBoolean("Cal/ToggleForceShoot", false)) {
            boolean newForce = !gameState.isForceShootEnabled();
            gameState.setForceShootEnabled(newForce);
            if (newForce) {
                logWarning("Force Shoot ON", "Ignoring alliance timing!");
            } else {
                logInfo("Force Shoot OFF", "Normal timing restored");
            }
            SmartDashboard.putBoolean("Cal/ToggleForceShoot", false);
        }
        
        if (SmartDashboard.getBoolean("Cal/ClearShuttleOverride", false)) {
            gameState.clearShuttleModeOverride();
            logInfo("Auto-Shuttle Enabled", "Auto-switching based on position");
            SmartDashboard.putBoolean("Cal/ClearShuttleOverride", false);
        }
        
        if (SmartDashboard.getBoolean("Cal/ResetGameState", false)) {
            gameState.reset();
            logInfo("Game State Reset", "All modes cleared");
            SmartDashboard.putBoolean("Cal/ResetGameState", false);
        }
        
        // ----- Log to AdvantageScope -----
        logToAdvantageScope();
    }
    
    // ADVANTAGESCOPE LOGGING
    
    private void logToAdvantageScope() {
        // Shooter data
        shooterTable.getEntry("TopPower").setDouble(shooterTopPower);
        shooterTable.getEntry("BottomPower").setDouble(shooterBottomPower);
        shooterTable.getEntry("TopOffset").setDouble(shooterTopOffset);
        shooterTable.getEntry("BottomOffset").setDouble(shooterBottomOffset);
        shooterTable.getEntry("UseManual").setBoolean(useManualShooterPower);
        
        // Turret data
        turretTable.getEntry("ManualAngle").setDouble(turretAngle);
        turretTable.getEntry("AngleOffset").setDouble(turretAngleOffset);
        turretTable.getEntry("UseManual").setBoolean(useManualTurretAngle);
        turretTable.getEntry("P").setDouble(turretP);
        turretTable.getEntry("I").setDouble(turretI);
        turretTable.getEntry("D").setDouble(turretD);
        turretTable.getEntry("CurrentAngle").setDouble(currentTurretAngle);
        
        // Limelight data
        limelightTable.getEntry("XOffset").setDouble(limelightXOffset);
        limelightTable.getEntry("YOffset").setDouble(limelightYOffset);
        limelightTable.getEntry("ZOffset").setDouble(limelightZOffset);
        limelightTable.getEntry("Pitch").setDouble(limelightPitch);
        limelightTable.getEntry("VisionStdDevX").setDouble(visionStdDevX);
        limelightTable.getEntry("VisionStdDevY").setDouble(visionStdDevY);
        limelightTable.getEntry("VisionStdDevTheta").setDouble(visionStdDevTheta);
        
        // Live measurements
        calibrationTable.getEntry("CurrentDistance").setDouble(currentDistance);
        calibrationTable.getEntry("CurrentTagId").setDouble(currentTagId);
        calibrationTable.getEntry("HasTarget").setBoolean(hasTarget);
        calibrationTable.getEntry("RecordCount").setDouble(recordCount);
    }
    
    // RECORDING METHODS
    
    private void recordShootingPoint() {
        if (currentDistance <= 0) {
            String errorMsg = "Cannot record - no valid distance measurement";
            SmartDashboard.putString("Cal/Status", "ERROR: No valid distance!");
            logError("Recording Failed", errorMsg);
            return;
        }
        
        // Round distance to 0.25m increments for clean table
        double roundedDistance = Math.round(currentDistance * 4.0) / 4.0;
        
        // Record the point
        double[] powers = new double[]{shooterTopPower, shooterBottomPower};
        recordedShootingPoints.put(roundedDistance, powers);
        recordCount++;
        
        // Generate Java code
        String javaCode = String.format("put(%.2f, new double[]{%.2f, %.2f});", 
            roundedDistance, shooterTopPower, shooterBottomPower);
        
        SmartDashboard.putNumber("Cal/RecordCount", recordCount);
        SmartDashboard.putString("Cal/LastRecorded", javaCode);
        SmartDashboard.putString("Cal/Status", "Recorded point at " + roundedDistance + "m");
        
        // Log to console and Elastic
        String description = String.format("Distance: %.2fm | Top: %.2f | Bottom: %.2f", 
            roundedDistance, shooterTopPower, shooterBottomPower);
        logInfo("Calibration Point #" + recordCount, description);
        
        System.out.println("=== CALIBRATION POINT RECORDED ===");
        System.out.println(javaCode);
        System.out.println("Distance: " + currentDistance + "m (rounded to " + roundedDistance + "m)");
        System.out.println("Top Power: " + shooterTopPower);
        System.out.println("Bottom Power: " + shooterBottomPower);
        
        // Log to AdvantageScope
        recordedTable.getEntry("Point" + recordCount + "_Distance").setDouble(roundedDistance);
        recordedTable.getEntry("Point" + recordCount + "_TopPower").setDouble(shooterTopPower);
        recordedTable.getEntry("Point" + recordCount + "_BottomPower").setDouble(shooterBottomPower);
        recordedTable.getEntry("LastRecordedCode").setString(javaCode);
    }
    
    private void recordTurretOffset() {
        if (currentTagId < 0) {
            String errorMsg = "Cannot record turret offset - no tag visible";
            SmartDashboard.putString("Cal/Status", "ERROR: No tag visible!");
            logError("Recording Failed", errorMsg);
            return;
        }
        
        recordedTurretOffsets.put(currentTagId, turretAngleOffset);
        recordCount++;
        
        String javaCode = String.format("put(%d, %.2f);", currentTagId, turretAngleOffset);
        
        SmartDashboard.putNumber("Cal/RecordCount", recordCount);
        SmartDashboard.putString("Cal/LastRecorded", "Tag " + currentTagId + " offset: " + turretAngleOffset);
        SmartDashboard.putString("Cal/Status", "Recorded offset for Tag " + currentTagId);
        
        // Log to console and Elastic
        String description = String.format("Tag %d offset: %.2f degrees", currentTagId, turretAngleOffset);
        logInfo("Turret Offset Recorded", description);
        
        System.out.println("=== TURRET OFFSET RECORDED ===");
        System.out.println("Tag ID: " + currentTagId);
        System.out.println("Offset: " + turretAngleOffset + " degrees");
        System.out.println(javaCode);
        
        // Log to AdvantageScope
        recordedTable.getEntry("TurretOffset_Tag" + currentTagId).setDouble(turretAngleOffset);
    }
    
    // EXPORT METHODS
    
    private void printShooterCalibrationTable() {
        if (recordedShootingPoints.isEmpty()) {
            logWarning("No Data", "No shooting calibration points recorded yet");
            return;
        }
        
        System.out.println("\n=============================================================");
        System.out.println("SHOOTER CALIBRATION TABLE - Copy to Constants.Shooter");
        System.out.println("=============================================================");
        System.out.println("public static final TreeMap<Double, double[]> SHOOTING_CALIBRATION = new TreeMap<>() {{");
        
        StringBuilder tableData = new StringBuilder();
        for (var entry : recordedShootingPoints.entrySet()) {
            double distance = entry.getKey();
            double[] powers = entry.getValue();
            String line = String.format("  put(%.2f, new double[]{%.2f, %.2f});", distance, powers[0], powers[1]);
            System.out.println(line);
            tableData.append(line).append("\n");
        }
        
        System.out.println("}};");
        System.out.println("=============================================================\n");
        
        // Log to AdvantageScope
        calibrationTable.getEntry("Export/ShooterTable").setString(tableData.toString());
        
        logSuccess("Shooter Table Exported", 
            recordedShootingPoints.size() + " points printed to console");
    }
    
    private void printTurretOffsets() {
        if (recordedTurretOffsets.isEmpty()) {
            logWarning("No Data", "No turret offset calibration points recorded yet");
            return;
        }
        
        System.out.println("\n=============================================================");
        System.out.println("TURRET ANGLE OFFSETS - Copy to Constants.Turret");
        System.out.println("=============================================================");
        System.out.println("public static final Map<Integer, Double> TAG_ANGLE_OFFSETS = new HashMap<Integer, Double>() {{");
        
        StringBuilder offsetData = new StringBuilder();
        for (var entry : recordedTurretOffsets.entrySet()) {
            String line = String.format("  put(%d, %.2f);", entry.getKey(), entry.getValue());
            System.out.println(line);
            offsetData.append(line).append("\n");
        }
        
        System.out.println("}};");
        System.out.println("=============================================================\n");
        
        // Log to AdvantageScope
        calibrationTable.getEntry("Export/TurretOffsets").setString(offsetData.toString());
        
        logSuccess("Turret Offsets Exported", 
            recordedTurretOffsets.size() + " offsets printed to console");
    }
    
    public void printAllConstants() {
        logInfo("Exporting Constants", "Printing all calibration values to console...");
        
        System.out.println("\n");
        System.out.println("+===========================================================+");
        System.out.println("|          ALL CALIBRATION VALUES FOR Constants.java        |");
        System.out.println("+===========================================================+");
        System.out.println("|  Copy these values to your Constants.java file            |");
        System.out.println("+===========================================================+");
        
        // Build the full export string for AdvantageScope
        StringBuilder allConstants = new StringBuilder();
        
        // Shooter
        System.out.println("\n// === SHOOTER (Constants.Shooter) ===");
        allConstants.append("// === SHOOTER ===\n");
        String line = String.format("public static final double TOP_MOTOR_POWER_OFFSET = %.3f;", shooterTopOffset);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double BOTTOM_MOTOR_POWER_OFFSET = %.3f;", shooterBottomOffset);
        System.out.println(line);
        allConstants.append(line).append("\n");
        
        // Turret
        System.out.println("\n// === TURRET (Constants.Turret) ===");
        allConstants.append("\n// === TURRET ===\n");
        line = String.format("public static final double PID_P = %.4f;", turretP);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double PID_I = %.4f;", turretI);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double PID_D = %.4f;", turretD);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double GLOBAL_ANGLE_OFFSET_DEG = %.2f;", turretAngleOffset);
        System.out.println(line);
        allConstants.append(line).append("\n");
        
        // Limelight
        System.out.println("\n// === LIMELIGHT (Constants.Limelight) ===");
        allConstants.append("\n// === LIMELIGHT ===\n");
        line = String.format("public static final double CAMERA_X_OFFSET = %.3f;", limelightXOffset);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double CAMERA_Y_OFFSET = %.3f;", limelightYOffset);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double CAMERA_Z_OFFSET = %.3f;", limelightZOffset);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double CAMERA_PITCH_DEGREES = %.2f;", limelightPitch);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double VISION_STD_DEV_X = %.3f;", visionStdDevX);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double VISION_STD_DEV_Y = %.3f;", visionStdDevY);
        System.out.println(line);
        allConstants.append(line).append("\n");
        line = String.format("public static final double VISION_STD_DEV_THETA = %.3f;", visionStdDevTheta);
        System.out.println(line);
        allConstants.append(line).append("\n");
        
        // Shooting table if we have points
        if (!recordedShootingPoints.isEmpty()) {
            printShooterCalibrationTable();
        }
        
        // Turret offsets if we have them
        if (!recordedTurretOffsets.isEmpty()) {
            printTurretOffsets();
        }
        
        System.out.println("\n=============================================================");
        
        // Log to AdvantageScope
        calibrationTable.getEntry("Export/AllConstants").setString(allConstants.toString());
        
        logSuccess("Export Complete", "All constants printed to console - check output!");
    }
    
    // Live data setters
    
    public void setCurrentDistance(double distance) {
        this.currentDistance = distance;
    }
    
    public void setCurrentTagId(int tagId) {
        this.currentTagId = tagId;
    }
    
    public void setCurrentTurretAngle(double angle) {
        this.currentTurretAngle = angle;
    }
    
    public void setHasTarget(boolean hasTarget) {
        this.hasTarget = hasTarget;
    }
    
    // Getters for calibration values
    
    public double getShooterTopPower() {
        return shooterTopPower;
    }
    
    public double getShooterBottomPower() {
        return shooterBottomPower;
    }
    
    public double getShooterTopOffset() {
        return shooterTopOffset;
    }
    
    public double getShooterBottomOffset() {
        return shooterBottomOffset;
    }
    
    public boolean useManualShooterPower() {
        return useManualShooterPower;
    }
    
    // ----- Turret -----
    
    public double getTurretManualAngle() {
        return turretAngle;
    }
    
    public double getTurretAngleOffset() {
        return turretAngleOffset;
    }
    
    public boolean useManualTurretAngle() {
        return useManualTurretAngle;
    }
    
    public double getTurretP() {
        return turretP;
    }
    
    public double getTurretI() {
        return turretI;
    }
    
    public double getTurretD() {
        return turretD;
    }
    
    // ----- Limelight -----
    
    public double getLimelightXOffset() {
        return limelightXOffset;
    }
    
    public double getLimelightYOffset() {
        return limelightYOffset;
    }
    
    public double getLimelightZOffset() {
        return limelightZOffset;
    }
    
    public double getLimelightPitch() {
        return limelightPitch;
    }
    
    public double getVisionStdDevX() {
        return visionStdDevX;
    }
    
    public double getVisionStdDevY() {
        return visionStdDevY;
    }
    
    public double getVisionStdDevTheta() {
        return visionStdDevTheta;
    }
    
    // ----- Status -----
    
    public double getCurrentDistance() {
        return currentDistance;
    }
    
    public int getCurrentTagId() {
        return currentTagId;
    }
    
    public boolean hasTarget() {
        return hasTarget;
    }
    
    public int getRecordCount() {
        return recordCount;
    }
    
    /**
     * Get all recorded shooting points.
     * @return TreeMap of distance -> {topPower, bottomPower}
     */
    public TreeMap<Double, double[]> getRecordedShootingPoints() {
        return new TreeMap<>(recordedShootingPoints);
    }
    
    /**
     * Clear all recorded data (for starting fresh).
     */
    public void clearRecordedData() {
        recordedShootingPoints.clear();
        recordedTurretOffsets.clear();
        recordCount = 0;
        SmartDashboard.putNumber("Cal/RecordCount", 0);
        SmartDashboard.putString("Cal/Status", "Cleared all recorded data");
        logWarning("Data Cleared", "All recorded calibration data has been cleared");
        System.out.println("=== CALIBRATION DATA CLEARED ===");
    }
    
    // Logging helpers
    
    /**
     * Log an info message to console, Elastic, and AdvantageScope.
     */
    public void logInfo(String title, String description) {
        // Console
        System.out.println("[CAL INFO] " + title + ": " + description);
        
        // Elastic notification (green info popup)
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.INFO)
            .withTitle(title)
            .withDescription(description)
            .withDisplaySeconds(3.0));
        
        // AdvantageScope logging
        calibrationTable.getEntry("Log/LastInfo").setString(title + ": " + description);
        calibrationTable.getEntry("Log/InfoCount").setDouble(
            calibrationTable.getEntry("Log/InfoCount").getDouble(0) + 1);
    }
    
    /**
     * Log a warning message to console, Elastic, and AdvantageScope.
     * @param title Short title for the warning
     * @param description Detailed description
     */
    public void logWarning(String title, String description) {
        // Console
        System.out.println("[CAL WARNING] " + title + ": " + description);
        
        // Elastic notification (yellow warning popup)
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.WARNING)
            .withTitle(title)
            .withDescription(description)
            .withDisplaySeconds(5.0));
        
        // AdvantageScope logging
        calibrationTable.getEntry("Log/LastWarning").setString(title + ": " + description);
        calibrationTable.getEntry("Log/WarningCount").setDouble(
            calibrationTable.getEntry("Log/WarningCount").getDouble(0) + 1);
    }
    
    /**
     * Log an error message to console, Elastic, and AdvantageScope.
     * @param title Short title for the error
     * @param description Detailed description
     */
    public void logError(String title, String description) {
        // Console
        System.err.println("[CAL ERROR] " + title + ": " + description);
        
        // Elastic notification (red error popup)
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.ERROR)
            .withTitle(title)
            .withDescription(description)
            .withDisplaySeconds(8.0));
        
        // AdvantageScope logging
        calibrationTable.getEntry("Log/LastError").setString(title + ": " + description);
        calibrationTable.getEntry("Log/ErrorCount").setDouble(
            calibrationTable.getEntry("Log/ErrorCount").getDouble(0) + 1);
    }
    
    /**
     * Log a success message (uses INFO level with checkmark).
     * @param title Short title
     * @param description Detailed description
     */
    public void logSuccess(String title, String description) {
        // Console
        System.out.println("[CAL SUCCESS] " + title + ": " + description);
        
        // Elastic notification
        Elastic.sendNotification(new Elastic.Notification()
            .withLevel(NotificationLevel.INFO)
            .withTitle("OK: " + title)
            .withDescription(description)
            .withDisplaySeconds(3.0));
        
        // AdvantageScope logging
        calibrationTable.getEntry("Log/LastSuccess").setString(title + ": " + description);
    }
}
