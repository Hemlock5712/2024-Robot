package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class ArmConstants {

  public static final LinearInterpolationTable linearInterpolationTable =
      new LinearInterpolationTable(
          new Point2D.Double(0, Units.degreesToRadians(0)),
          new Point2D.Double(5, Units.degreesToRadians(5)),
          new Point2D.Double(8, Units.degreesToRadians(20)),
          new Point2D.Double(10, Units.degreesToRadians(30)));

  // arm.setArmTarget(Units.degreesToRadians(-25));
  // arm.setWristTarget(Units.degreesToRadians(-38));
  public static final double armTargetPostionIntakeMode = Units.degreesToRadians(-25);
  public static final double wristTargetPositionIntakeMode = Units.degreesToRadians(-38);
}
