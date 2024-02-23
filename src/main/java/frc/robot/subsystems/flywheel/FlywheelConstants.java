package frc.robot.subsystems.flywheel;

import frc.robot.Constants;

public final class FlywheelConstants {

  public static final MotorFeedbackController FeedbackController =
      switch (Constants.getRobot()) {
        case SIMBOT -> new MotorFeedbackController(0, 0, 0, 0);
        case COMPBOT -> new MotorFeedbackController(0, 0, 0, 0);
      };

  public record MotorFeedbackController(double kP, double kI, double kD, double kFF) {}
}
