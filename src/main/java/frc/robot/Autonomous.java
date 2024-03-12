// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Autonomous extends SubsystemBase {
  /** Creates a new DriveAutonomous. */
  private final Drive drive;

  private final Intake intake;
  private final Shooter shooter;
  private final Arm arm;

  public Autonomous(Drive drive, Intake intake, Shooter shooter, Arm arm) {

    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.arm = arm;
  }

  public Command shoot() {
    return new SequentialCommandGroup(shootSpeaker(), wait(1.0), cancel());
  }

  public Command shoot_mobility() {
    return new SequentialCommandGroup(shootSpeaker(), wait(1.0), cancel());
  }

  public Command cancel() {
    return new ParallelCommandGroup(zeroShoot(), zeroIntake());
  }

  public Command shootSpeaker() {
    return shooter.shootSpeaker().withTimeout(15);
  }

  public Command wait(double seconds) {
    return new WaitCommand(seconds);
  }

  public Command zeroShoot() {
    return new InstantCommand(() -> shooter.zero());
  }

  public Command zeroIntake() {
    return new InstantCommand(() -> intake.zero());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
