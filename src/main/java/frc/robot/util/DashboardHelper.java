package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Organizes SmartDashboard into 3 clean folders:
 *   MATCH/   - Driver-facing status (shoot unlocked, target mode, etc.)
 *   TUNING/  - Calibration sliders and commands (pit crew / testing)
 *   DEBUG/   - Diagnostic data (only when troubleshooting)
 * 
 * Subsystem telemetry (Turret/, Shooter/, etc.) goes unprefixed
 * so it stays at the top level and is easy to find.
 */
public class DashboardHelper {
    
    public enum Category {
        MATCH("Match/"),     // Key driver info during matches
        TUNING("Tuning/"),   // Calibration controls and commands
        DEBUG("Debug/");     // Diagnostic telemetry

        private final String prefix;
        Category(String prefix) { this.prefix = prefix; }
        public String getPrefix() { return prefix; }
    }
    
    // --- Put methods ---
    
    public static void putNumber(Category category, String key, double value) {
        SmartDashboard.putNumber(category.getPrefix() + key, value);
    }
    
    public static void putBoolean(Category category, String key, boolean value) {
        SmartDashboard.putBoolean(category.getPrefix() + key, value);
    }
    
    public static void putString(Category category, String key, String value) {
        SmartDashboard.putString(category.getPrefix() + key, value);
    }
    
    public static void putData(Category category, String key, Command value) {
        SmartDashboard.putData(category.getPrefix() + key, value);
    }
    
    public static void putData(Category category, String key, edu.wpi.first.util.sendable.Sendable value) {
        SmartDashboard.putData(category.getPrefix() + key, value);
    }
    
    // --- Get methods ---
    
    public static double getNumber(Category category, String key, double defaultValue) {
        return SmartDashboard.getNumber(category.getPrefix() + key, defaultValue);
    }
    
    public static boolean getBoolean(Category category, String key, boolean defaultValue) {
        return SmartDashboard.getBoolean(category.getPrefix() + key, defaultValue);
    }
    
    public static String getString(Category category, String key, String defaultValue) {
        return SmartDashboard.getString(category.getPrefix() + key, defaultValue);
    }
}
