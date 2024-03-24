// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;

public class Sensor extends SubsystemBase {
  /** Creates a new Sensor. */
  private static Intake intake;

  private static LEDs led;

  private DigitalInput sensor;

  public Sensor() {
    intake = new Intake();
    sensor = new DigitalInput(LEDConstants.k_DIOPort);
    led = new LEDs();
  }

  public boolean getNoteDetected() {
    if (sensor.get()) {
      intake.zero();
      led.blinkCommand().withTimeout(3);
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
