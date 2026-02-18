// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;

/**
 * Main robot class - init() methods run once on mode entry, periodic() runs every 20ms.
 */
public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    notify("Robot Initialized", "All systems starting up");
  }
  
  @Override
  public void robotPeriodic() {
    m_robotContainer.updateVisionPose();
    CommandScheduler.getInstance().run();
    LEDSubsystem leds = m_robotContainer.getLEDs();
    if (RobotController.getBatteryVoltage() < 7.0) {
      if (leds != null) leds.setAction(LEDSubsystem.ActionState.BROWNOUT);
    } else {
      if (leds != null && leds.getAction() == LEDSubsystem.ActionState.BROWNOUT) {
        leds.clearAction();
      }
    }
  }

  @Override
  public void disabledInit() {
    // SAFETY: Stop all motors immediately on disable.
    // Each subsystem's periodic() also sends zero power when disabled,
    // but this ensures an immediate stop at the transition.
    m_robotContainer.stopAllMotors();
    
    setLEDState(LEDState.DISABLED);
    LEDSubsystem leds = m_robotContainer.getLEDs();
    if (leds != null) leds.clearAction();
    notify("Robot Disabled", "Safe to approach");
    
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    setLEDState(LEDState.AUTO);
    
    String autoName = m_autonomousCommand != null ? m_autonomousCommand.getName() : "None";
    notify("AUTO STARTED", "Running: " + autoName);
    
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    setLEDState(LEDState.TELEOP);
    notify("TELEOP STARTED", "Driver control active");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
  
  // Helper methods
  
  private void setLEDState(LEDState state) {
    LEDSubsystem leds = m_robotContainer.getLEDs();
    if (leds != null) leds.setState(state);
  }
  
  private void notify(String title, String description) {
    Elastic.sendNotification(new Elastic.Notification()
        .withLevel(NotificationLevel.INFO)
        .withTitle(title)
        .withDescription(description)
        .withDisplaySeconds(2.0));
  }
}
