// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  private double lastChange;
  private boolean on = true;
  private Color m_EyeColor = Color.kGreen;
  private Color m_BackgroundColor = Color.kPurple;
  private int m_Length;
  private int m_eyePosition = 0;
  private int m_scanDirection = 1;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    switch (Constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    Logger.start();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    // PWM port 0
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(0);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(30);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    m_led.setData(m_ledBuffer);
  }

  public void green() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 255, 0);
    }
  }

  public void blue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
  }

  public void purple() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 255);
    }
  }

  public void blinking() {
    double timestamp = Timer.getFPGATimestamp();
    if (timestamp - lastChange > 0.1) {
      on = !on;
      lastChange = timestamp;
    }
    if (on) {
      green();
    } else {
      blue();
    }
  }
  /* public Command blinkGreenBlue() {
  }*/
  public void scanner() {
    int bufferLength = m_ledBuffer.getLength();
    double intensity;
    double red;
    double green;
    double blue;
    double distanceFromEye;
    for (int index = 0; index < bufferLength; index++) {
      distanceFromEye = MathUtil.clamp(Math.abs(m_eyePosition - index), 0, 2);
      intensity = 1 - distanceFromEye / 2;
      red = MathUtil.interpolate(m_BackgroundColor.red, m_EyeColor.red, intensity);
      green = MathUtil.interpolate(m_BackgroundColor.green, m_EyeColor.green, intensity);
      blue = MathUtil.interpolate(m_BackgroundColor.blue, m_EyeColor.blue, intensity);
      m_ledBuffer.setLED(index, new Color(red, green, blue));
    }
    if (m_eyePosition == 0) {
      m_scanDirection = 1;
    } else if (m_eyePosition == bufferLength - 1) {
      m_scanDirection = -1;
    }
    m_eyePosition += m_scanDirection;
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // if (!SmartFeed.IndexerSensorHasNote()) {
    //   blinking();
    // } else {
    //   scanner();
    // }
    m_led.setData(m_ledBuffer);
  }
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Command autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    m_robotContainer.intake.zero();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
