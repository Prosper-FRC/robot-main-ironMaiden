// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SmartFeed extends Command {
  /** Creates a new SmartFeed. */
  private Intake indexerIntake;

  private static DigitalInput indexerSensor;

  public SmartFeed(Intake indexerIntake) {
    this.indexerIntake = indexerIntake;

    indexerSensor = new DigitalInput(1);
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(indexerIntake);
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
    if (!indexerSensor.get()) {
      indexerIntake.zero();

    } else {
      indexerIntake.intake();
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
