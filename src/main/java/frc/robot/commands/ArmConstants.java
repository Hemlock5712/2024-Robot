package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class ArmConstants {
  public static final ArmPositions backAmp = new ArmPositions(80, -45);
  public static final ArmPositions frontAmp = new ArmPositions(50, 35);
  public static final ArmPositions intake = new ArmPositions(-25, -38);
  public static final ArmPositions shoot = new ArmPositions(-80, 0);

  public record ArmPositions(double arm, double wrist) {
    public double getArmRadians() {
      return Units.degreesToRadians(arm);
    }

    public double getWristRadians() {
      return Units.degreesToRadians(wrist);
    }

    public Rotation2d getArmRotation() {
      return new Rotation2d(getArmRadians());
    }

    public Rotation2d getWristRotation() {
      return new Rotation2d(getWristRadians());
    }
  }
}
