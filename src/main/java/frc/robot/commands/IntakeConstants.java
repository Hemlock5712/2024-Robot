package frc.robot.commands;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  public static final double intakeSpeed = 1000;

  public static final IntakePositions floorPosition = new IntakePositions(225);
  public static final IntakePositions bumperPosition = new IntakePositions(190);
  public static final IntakePositions upPosition = new IntakePositions(90);

  public record IntakePositions(double degrees) {
    public double toRadians() {
      return Units.degreesToRadians(degrees);
    }
  }
}
