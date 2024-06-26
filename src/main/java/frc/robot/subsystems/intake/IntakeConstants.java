package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public final class IntakeConstants {

  public record MotorFeedbackController(double kP, double kI, double kD, double kFF) {}

  public static final IntakePositions floorPosition =
      new IntakePositions(Rotation2d.fromDegrees(184));
  public static final IntakePositions upPosition = new IntakePositions(Rotation2d.fromDegrees(91));

  public record IntakePositions(Rotation2d angle) {}
}
