package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;

public final class ArmConstants {
  public static final ArmPositions backAmp =
      new ArmPositions(Rotation2d.fromDegrees(80), Rotation2d.fromDegrees(-45));
  public static final ArmPositions frontAmp =
      new ArmPositions(Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(35));
  public static final ArmPositions intake =
      new ArmPositions(Rotation2d.fromDegrees(-25), Rotation2d.fromDegrees(-38));
  public static final ArmPositions shoot =
      new ArmPositions(Rotation2d.fromDegrees(-80), Rotation2d.fromDegrees(0));

  public record ArmPositions(Rotation2d arm, Rotation2d wrist) {}
}
