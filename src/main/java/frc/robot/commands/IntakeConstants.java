package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  public static final double intakeSpeed = 1000;

  public static final IntakePositions floorPosition =
      new IntakePositions(Rotation2d.fromDegrees(200));
  public static final IntakePositions bumperPosition =
      new IntakePositions(Rotation2d.fromDegrees(190));
  public static final IntakePositions upPosition = new IntakePositions(Rotation2d.fromDegrees(90));

  public record IntakePositions(Rotation2d angle) {}
}
