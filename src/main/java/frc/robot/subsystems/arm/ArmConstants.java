package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public final class ArmConstants {
  public static final ArmPositions backAmp =
      new ArmPositions(Rotation2d.fromDegrees(23), Rotation2d.fromDegrees(65));
  public static final ArmPositions frontAmp =
      new ArmPositions(Rotation2d.fromDegrees(31), Rotation2d.fromDegrees(-60));
  public static final ArmPositions intake =
      new ArmPositions(Rotation2d.fromDegrees(-23), Rotation2d.fromDegrees(66));
  public static final ArmPositions shoot =
      new ArmPositions(Rotation2d.fromDegrees(-23), Rotation2d.fromDegrees(65));
  public static final ArmPositions preclimb =
      new ArmPositions(Rotation2d.fromDegrees(53), Rotation2d.fromDegrees(-48));
  public static final ArmPositions trap =
      new ArmPositions(Rotation2d.fromDegrees(32), Rotation2d.fromDegrees(33));
  public static final ArmPositions emergencyIntake =
      new ArmPositions(Rotation2d.fromDegrees(7), Rotation2d.fromDegrees(25));
  public static final ArmPositions feed =
      new ArmPositions(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));

  public static final ArmPositions manualShot =
      new ArmPositions(Rotation2d.fromDegrees(-23), Rotation2d.fromDegrees(58.75));

  public static final MotorFeedbackController armControlConstants =
      switch (Constants.getRobot()) {
        case SIMBOT -> new MotorFeedbackController(0, 0, 0, 0);
        case COMPBOT -> new MotorFeedbackController(160, 0, 0, 0.3);
          // case COMPBOT -> new MotorFeedbackController(0, 0, 0, 0);
      };

  public static final MotorFeedbackController wristControlConstants =
      switch (Constants.getRobot()) {
        case SIMBOT -> new MotorFeedbackController(0, 0, 0, 0);
        case COMPBOT -> new MotorFeedbackController(60, 0, .005, 0.0);
          // case COMPBOT -> new MotorFeedbackController(0, 0, 0, 0);
      };

  public static final int ARM_GEAR_RATIO = 20;
  public static final int WRIST_GEAR_RATIO = 15;

  public record ArmPositions(Rotation2d arm, Rotation2d wrist) {}

  public record MotorFeedbackController(double kP, double kI, double kD, double kG) {}
}
