// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class ArmCommand extends Command {
  /** Creates a new ArmCommand. */
  private Arm arm;

  private DoubleSupplier verticalSup;

  public ArmCommand(DoubleSupplier verticalSup, Arm arm) {

    this.arm = arm;
    this.verticalSup = verticalSup;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double verticalVal =
        MathUtil.applyDeadband(verticalSup.getAsDouble(), ArmConstants.k_armDeadband) * -1.0;

    arm.runArm(verticalVal);
  }
}
