package frc.robot.commands.flywheel;

import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class FlywheelConstants {
  public static final FlywheelSpeeds podium = new FlywheelSpeeds(3000);

  public static final LinearInterpolationTable table =
      new LinearInterpolationTable(
          new Point2D.Double(0, 8000),
          new Point2D.Double(5, 12000),
          new Point2D.Double(8, 16000),
          new Point2D.Double(10, 18000));

  public record FlywheelSpeeds(double speed) {}
}
