// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensor extends SubsystemBase {
  /** Creates a new Sensor. */
  public static boolean noteDetected = false;

  private static DigitalInput sensor;

  public Sensor() {
    sensor = new DigitalInput(0);
  }

  // public boolean getNoteDetected() {
  //   if (sensor.get()) {
  //     intake.zero();
  //     led.blinkCommand().withTimeout(3);
  //     return true;
  //   }
  //   return false;
  // }

  public static boolean isDetected() {
    return !sensor.get();
  }

  public static void getNoteDetected() {
    if (noteDetected) {
      System.out.println("Note detected ");
    }
  }

  @Override
  public void periodic() {
    noteDetected = isDetected();
    getNoteDetected();
  }
}
