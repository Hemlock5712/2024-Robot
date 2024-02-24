package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public final class IntakeConstants {

  public static final MotorFeedbackController FeedbackController =
      switch (Constants.getRobot()) {
        case SIMBOT -> new MotorFeedbackController(0, 0, 0, 0);
        case COMPBOT -> new MotorFeedbackController(0, 0, 0, 0);
      };

  public record MotorFeedbackController(double kP, double kI, double kD, double kFF) {}

  public static final double intakeSpeed = 1000;

  public static final IntakePositions floorPosition =
      new IntakePositions(Rotation2d.fromDegrees(225));
  public static final IntakePositions bumperPosition =
      new IntakePositions(Rotation2d.fromDegrees(190));
  public static final IntakePositions upPosition = new IntakePositions(Rotation2d.fromDegrees(90));

  public record IntakePositions(Rotation2d angle) {}
}
