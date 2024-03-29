// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;

public class Sensor extends SubsystemBase {
  /** Creates a new Sensor. */
  public static boolean noteDetected = false;

  private boolean noteInitialDetect = false;

  private static DigitalInput sensor;
  private static Intake intake;

  public Sensor() {
    sensor = new DigitalInput(1);
  }

  public static boolean isDetected() {
    return !sensor.get();
  }

  public static void getNoteDetected() {
    if (noteDetected) {
      System.out.println("Note detected ");
    }
  }

  public static void stopIntake() {
    if (isDetected()) {
      intake.zero();
    }
  }

  public void detectOff() {
    noteInitialDetect = false;
  }

  @Override
  public void periodic() {
    noteDetected = isDetected();
    // getNoteDetected();
    if (noteDetected && !noteInitialDetect) {
      RobotContainer.leds.blinkLEDsPurple();
      noteInitialDetect = true;
    }
  }
}
