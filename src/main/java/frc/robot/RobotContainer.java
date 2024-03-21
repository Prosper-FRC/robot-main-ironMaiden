// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.Autonomous;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  private static final CommandXboxController driver =
      new CommandXboxController(Constants.k_driverID);
  private static final CommandXboxController operator =
      new CommandXboxController(Constants.k_operatorID);

  private static final LEDs leds = new LEDs();
  public static final Arm arm = new Arm(operator.getHID(), leds);

  private static Intake intake;
  private static Shooter shooter;
  private static Autonomous autonomous;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    intake = new Intake();
    shooter = new Shooter(intake);

    // Manual:
    arm.setDefaultCommand(new ArmCommand(() -> operator.getRightY(), arm));

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    autonomous = new Autonomous(intake, shooter);

    // Set up auto routines
    NamedCommands.registerCommand("Run Shoot", autonomous.SHOOT());

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up autonomous pathplanner routines
    autoChooser.addOption("Auto: SHOOT", autonomous.SHOOT());
    // autoChooser.addOption("Auto: PL-MB", autonomous.PL_MB());
    // autoChooser.addOption("Auto: PL-MB-1P", autonomous.PL_MB_1P(0));
    // autoChooser.addOption("Auto: PL-MB-1P-L", autonomous.PL_MB_1P_L());
    // autoChooser.addOption("Auto: PL-MB-2P", autonomous.PL_MB_2P());
    // autoChooser.addOption("Auto: Auto PL-MB-1L", AutoBuilder.buildAuto("PL-M-1L"));
    autoChooser.addOption("Auto: Test", autonomous.test());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  // -------------------------------------------------------------------[Button
  // Bindings]-----------------------------------------------------------------------

  private void configureButtonBindings() {

    /* New operator bindings:
    - left bumper = intake
    - left trigger = outtake
    - right bumper = shoot (starts intake and shooter wheels)
    - X = amp pos
    - B = speaker pos
    - Right joystick = manual (in ArmCommand)
    */


    // right bumper intakes note and retracts to move note away from shooter wheels
    operator
        .leftBumper()
        .whileTrue(
            new InstantCommand(
                () -> {
                  intake.intake();
                }))
        .onFalse(intake.retract());

    // 'a' button outtakes note
    operator
        .leftTrigger()
        .whileTrue(new InstantCommand(() -> intake.outtake()))
        .onFalse(new InstantCommand(() -> intake.zero()));

    // Left bumper shoots Speaker, now shootSpeaker also includes moving intake also.
    operator
        .rightBumper()
        .whileTrue(shooter.shootSpeaker())
        .onFalse(
            new InstantCommand(
                () -> {
                  shooter.zero();
                  intake.zero();
                }));

    // Button x = arm to Amp Position, starts shooter wheels
    operator
        .x()
        .whileTrue(new ParallelCommandGroup(
          new InstantCommand(() -> arm.goToAmpPos()),
          new InstantCommand(() -> shooter.shootAmp())
        ))
        .onFalse(
            new InstantCommand(
                () -> {
                  shooter.zero();
                  // intake.zero();
                }));

    // Sets speaker wheels to reverse
    operator
        .povUp()
        .whileTrue(new InstantCommand(() -> shooter.setSpeedReverse()))
        .onFalse(
            new InstantCommand(
                () -> {
                  shooter.zero();
                  intake.zero();
                }));

    // Button b = arm to Speaker position, starts shooter wheels
    operator
        .b()
        .onTrue(
            new ParallelCommandGroup(
              new InstantCommand(() -> arm.goToShootPos()),
              new InstantCommand(() -> shooter.shootSpeaker())
            ));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    driver.leftBumper().onTrue(leds.toggleOrange());

    driver.rightBumper().onTrue(leds.toggleBlue());

    driver.rightTrigger().onTrue(new InstantCommand(() -> arm.climbOff()));

    /*
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller.b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.a()
        .whileTrue(
            Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));
    */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
