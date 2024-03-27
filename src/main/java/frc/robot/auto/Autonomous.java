// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Autonomous extends SubsystemBase {
  /** Creates a new DriveAutonomous. */
  private final Intake intake;

  private final Shooter shooter;
  private final Drive drive;

  public Autonomous(Intake intake, Shooter shooter, Drive drive) {

    this.intake = intake;
    this.shooter = shooter;
    this.drive = drive;
  }

  // ---------------------------------------------------------------[Auton Path
  // Configuration]--------------------------------------------------------

  public Command test() {
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test Path"));
  }
  // Pre-Loaded Shot
  public Command PL() {
    return new SequentialCommandGroup(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("To Shoot1")), SHOOT());
  }
  // PreLoad-Mobility
  public Command PL_MB() {
    return new SequentialCommandGroup(
        PL(), AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot1-Note1")));
  }

  // PreLoad-Mobility-Note1
  public Command PL_MB_1P(int side) {
    switch (side) {
      case 0:
        return new SequentialCommandGroup(
            PL_MB(), AutoBuilder.followPath(PathPlannerPath.fromPathFile("Note1-Shoot1")), SHOOT());
      default:
        return new SequentialCommandGroup(
            PL_MB(), AutoBuilder.followPath(PathPlannerPath.fromPathFile("Note1-Shoot1")), SHOOT());
    }
  }

  public Command PL_MB_1P_L() {
    return new SequentialCommandGroup(
        PL_MB_1P(0), AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot1-Note2")));
  }

  public Command PL_MB_2P() {
    return new SequentialCommandGroup(
        PL_MB_1P_L(),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Note2-Shoot2")),
        SHOOT());
  }

  public Command MOBILITY() {
    return new SequentialCommandGroup(cancel(), moveField(2.0, 0.0, 0.5));
  }

  public Command SHOOT_MOBILITY() {
    return new SequentialCommandGroup(
        cancel(),
        moveField(2.0, 0.0, 0.5),
        cancelDrive(),
        SHOOT(),
        cancel(),
        moveField(2.0, 0.0, 0.5),
        cancelDrive());
  }

  public Command SHOOT_MOBILITY_LOAD() {
    return new ParallelCommandGroup(
        new SequentialCommandGroup(
            cancel(),
            moveField(2.0, 0.0, 0.5),
            cancelDrive(),
            SHOOT(),
            cancel(),
            rotate(-0.05, 0.585),
            moveField(0.908, 1.782, 1.8),
            moveField(-0.908, -1.782, 1.8),
            rotate(0.05, 0.585),
            cancelDrive(),
            SHOOT()),
        new SequentialCommandGroup(
            wait(6.0), runIntake(), wait(1.5), runRetract(), wait(0.75), zeroIntake()));
  }

  // ---------------------------------------------------------------[Commands]--------------------------------------------------------
  public Command SHOOT() {
    return new SequentialCommandGroup(shootSpeaker(), wait(1.5), cancel());
  }

  public Command cancel() {
    return new ParallelCommandGroup(zeroShoot(), zeroIntake());
  }

  public Command cancelDrive() {
    return DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 0.0).withTimeout(1);
  }

  public Command move() {
    return new InstantCommand(
        () ->
            drive.runVelocity(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    0.0, Units.feetToMeters(5.0), 0.0, drive.gyroValue())));
  }

  public Command moveField(double x, double y, double sec) {
    return DriveCommands.joystickDrive(
            drive, () -> Units.feetToMeters(x), () -> Units.feetToMeters(y), () -> 0.0)
        .withTimeout(sec);
  }

  public Command rotate(double rot, double sec) {
    return DriveCommands.joystickDrive(
            drive,
            () -> Units.feetToMeters(0.0),
            () -> Units.feetToMeters(0.0),
            () ->
                rot
                    * (Units.feetToMeters(17.1)
                        / (Math.hypot(
                            Units.inchesToMeters(25.0) / 2.0, Units.inchesToMeters(25.0) / 2.0))))
        .withTimeout(sec);
  }

  public Command shootSpeaker() {
    return shooter.shootSpeaker();
  }

  public Command wait(double seconds) {
    return new WaitCommand(seconds);
  }

  public Command runIntake() {
    return new InstantCommand(() -> intake.intake());
  }

  public Command runRetract() {
    return new InstantCommand(() -> intake.retract());
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
