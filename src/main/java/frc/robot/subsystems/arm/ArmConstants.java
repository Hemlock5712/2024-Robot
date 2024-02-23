package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public final class ArmConstants {
  public static final ArmPositions backAmp =
      new ArmPositions(Rotation2d.fromDegrees(80), Rotation2d.fromDegrees(-45));
  public static final ArmPositions frontAmp =
      new ArmPositions(Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(35));
  public static final ArmPositions intake =
      new ArmPositions(Rotation2d.fromDegrees(-25), Rotation2d.fromDegrees(-38));
  public static final ArmPositions shoot =
      new ArmPositions(Rotation2d.fromDegrees(-25), Rotation2d.fromDegrees(-45));

  public static final MotorFeedbackController armControlConstants =
      switch (Constants.getRobot()) {
        case SIMBOT -> new MotorFeedbackController(0, 0, 0, 0);
        case COMPBOT -> new MotorFeedbackController(0, 0, 0, 0);
      };

  public static final MotorFeedbackController wristControlConstants =
      switch (Constants.getRobot()) {
        case SIMBOT -> new MotorFeedbackController(0, 0, 0, 0);
        case COMPBOT -> new MotorFeedbackController(0, 0, 0, 0);
      };

  public record ArmPositions(Rotation2d arm, Rotation2d wrist) {}

  public record MotorFeedbackController(double kP, double kI, double kD, double kFF) {}
}
