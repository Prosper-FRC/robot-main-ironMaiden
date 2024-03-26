// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SmartFeed extends Command {
  /** Creates a new SmartFeed. */
  private Intake intake;

  private static DigitalInput indexerSensor;

  public SmartFeed(Intake intake) {
    this.intake = intake;

    indexerSensor = new DigitalInput(0);
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(intake);
  }

  public static boolean IndexerSensorHasNote() {
    return indexerSensor.get();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (indexerSensor.get()) {
      intake.intake();

    } else {
      intake.zero();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
