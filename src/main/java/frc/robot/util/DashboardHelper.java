package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Helper class to organize SmartDashboard entries into folders based on robot state.
 * Automatically prefixes entries with the appropriate category folder.
 */
public class DashboardHelper {
    
    /**
     * Categories for organizing dashboard entries
     */
    public enum Category {
        PRE_MATCH("Pre Match/"),
        AUTO("Auto/"),
        TELEOP("Teleop/"),
        POST_MATCH("Post Match/"),
        HOME("Home/"),
        PRACTICE("Practice/"),
        SETTINGS("Settings/"),
        TEST("Test/"),
        DEBUG("Debug/"),
        MISC("Misc/"),
        SPINDEXER("Spindexer/"),
        TURRET_FEED("Turret Feed/");
        
        private final String prefix;
        
        Category(String prefix) {
            this.prefix = prefix;
        }
        
        public String getPrefix() {
            return prefix;
        }
    }
    
    /**
     * Put a number value to SmartDashboard with category prefix
     */
    public static void putNumber(Category category, String key, double value) {
        SmartDashboard.putNumber(category.getPrefix() + key, value);
    }
    
    /**
     * Put a boolean value to SmartDashboard with category prefix
     */
    public static void putBoolean(Category category, String key, boolean value) {
        SmartDashboard.putBoolean(category.getPrefix() + key, value);
    }
    
    /**
     * Put a string value to SmartDashboard with category prefix
     */
    public static void putString(Category category, String key, String value) {
        SmartDashboard.putString(category.getPrefix() + key, value);
    }
    
    /**
     * Put a command to SmartDashboard with category prefix
     */
    public static void putData(Category category, String key, Command value) {
        SmartDashboard.putData(category.getPrefix() + key, value);
    }
    
    /**
     * Put a Sendable object to SmartDashboard with category prefix
     */
    public static void putData(Category category, String key, edu.wpi.first.util.sendable.Sendable value) {
        SmartDashboard.putData(category.getPrefix() + key, value);
    }
    
    /**
     * Get a number value from SmartDashboard with category prefix
     */
    public static double getNumber(Category category, String key, double defaultValue) {
        return SmartDashboard.getNumber(category.getPrefix() + key, defaultValue);
    }
    
    /**
     * Get a boolean value from SmartDashboard with category prefix
     */
    public static boolean getBoolean(Category category, String key, boolean defaultValue) {
        return SmartDashboard.getBoolean(category.getPrefix() + key, defaultValue);
    }
    
    /**
     * Get a string value from SmartDashboard with category prefix
     */
    public static String getString(Category category, String key, String defaultValue) {
        return SmartDashboard.getString(category.getPrefix() + key, defaultValue);
    }
    
    /**
     * Automatically determine the appropriate category based on robot state
     */
    public static Category getAutomaticCategory() {
        if (DriverStation.isDisabled()) {
            // Check if we're before match start
            if (DriverStation.getMatchTime() < 0 || DriverStation.getMatchTime() > 135) {
                return Category.PRE_MATCH;
            } else {
                return Category.POST_MATCH;
            }
        } else if (DriverStation.isAutonomous()) {
            return Category.AUTO;
        } else if (DriverStation.isTeleop()) {
            return Category.TELEOP;
        } else if (DriverStation.isTest()) {
            return Category.TEST;
        } else {
            return Category.MISC;
        }
    }
    
    /**
     * Put a value using automatic category detection
     */
    public static void putNumberAuto(String key, double value) {
        putNumber(getAutomaticCategory(), key, value);
    }
    
    /**
     * Put a boolean using automatic category detection
     */
    public static void putBooleanAuto(String key, boolean value) {
        putBoolean(getAutomaticCategory(), key, value);
    }
    
    /**
     * Put a string using automatic category detection
     */
    public static void putStringAuto(String key, String value) {
        putString(getAutomaticCategory(), key, value);
    }
}
