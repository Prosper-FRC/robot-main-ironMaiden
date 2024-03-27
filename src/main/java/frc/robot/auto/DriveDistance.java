// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveDistance extends Command {
  /** Creates a new DriveDistance. */
  private Drive drive;

  public DriveDistance(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies. Speed * time = distance

    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.runVelocity(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            0.0, Units.feetToMeters(5.0), 0.0, drive.gyroValue()));
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
