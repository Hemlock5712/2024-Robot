package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public final class ArmConstants {
  public static final ArmPositions backAmp = new ArmPositions(80, -45);
  public static final ArmPositions frontAmp = new ArmPositions(50, 35);
  public static final ArmPositions intake = new ArmPositions(-25, -38);
  public static final ArmPositions podium = new ArmPositions(-25, 45);

  public static final LinearInterpolationTable linearInterpolationTable =
      new LinearInterpolationTable(
          new Point2D.Double(0, Units.degreesToRadians(0)),
          new Point2D.Double(5, Units.degreesToRadians(5)),
          new Point2D.Double(8, Units.degreesToRadians(20)),
          new Point2D.Double(10, Units.degreesToRadians(30)));

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
