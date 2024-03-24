package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConstants {
  public static final int k_armMotorID = 18;
  public static final int k_smartCurrentLimit = 50;
  public static final double k_armSpeed = 0.7;
  public static final double k_climbSpeed = 0.3;
  public static final double k_speedZero = 0.0;
  public static final boolean k_isInverted = false;

  public static final double k_armGearRatio =
      (1.0 / 60.0)
          * (28.0 / 50.0)
          * (16.0 / 64.0); // was 1/25, but another maxplanetary was added to make it 1/125.
  public static final double k_positionConversionFactor = k_armGearRatio;
  public static final double k_velocityConversionFactor = k_armGearRatio / 60.0;

  public static final double k_armP = 25.0; // 2.5 on REV ION
  public static final double k_armI = 0.0;
  public static final double k_armD = 0.0;

  public static final double k_maxVelocity = 1; // use 0 to 1
  public static final double k_maxAcceleration = 0.3; // use 0 to 1

  public static final Rotation2d k_armEncoderOffset = Rotation2d.fromRotations(0.007469); // 1.342
  public static final double k_armFreeSpeed = 5676.0 * k_velocityConversionFactor;

  // Feed Forward, PID, and setpoint constants - TO BE CONFIGURED: 12,0 / k_armFreeSpeed
  public static final ArmFeedforward k_armFeedforward =
      new ArmFeedforward(0.0, 1.0, 12 / k_armFreeSpeed, 0.0);
  public static final Constraints k_trapezoidalConstraints =
      new TrapezoidProfile.Constraints(1.0, 2.0);
  public static final ProfiledPIDController k_armPID =
      new ProfiledPIDController(0.2, 0.0, 0.0, k_trapezoidalConstraints);

  public static final double k_climbUpSetpoint = 0.23;
  // public static final double k_climbDownSetpoint = 0.0;
  public static final double k_ampSetpoint = 0.18; // 0.18
  public static final double k_shootSetpoint = 0.0; // 0189998
  public static final double k_tolerance = 0.1;

  public static final double k_armDeadband = 0.1;
  // public static final double k_upperBoundAmp = 0.18000; // + k_armEncoderOffset.getRotations();

  public static final double k_upperBound = 0.23; // 0.194179
  public static final double k_upperBoundNormal = 0.966363;
  public static final double k_lowerBound = 0.0; // + k_armEncoderOffset.getRotations();
}
