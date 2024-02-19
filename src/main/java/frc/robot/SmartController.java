package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.ArmConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The SmartController class represents a controller for the robot's system. It provides methods to
 * control the robot.
 */
public class SmartController {
  private static SmartController instance;

  private DriveModeType driveModeType = DriveModeType.SPEAKER;
  private AimingParameters targetAimingParameters =
      new AimingParameters(Rotation2d.fromDegrees(90), 0.0, 2500, ArmConstants.shoot.wrist());
  private boolean smartControl = false;

  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

  private SmartController() {
    shooterSpeedMap.put(Units.inchesToMeters(60), 5000.0);
    shooterSpeedMap.put(Units.inchesToMeters(80), 6000.0);
    shooterSpeedMap.put(Units.inchesToMeters(100), 7000.0);
    shooterSpeedMap.put(Units.inchesToMeters(120), 8000.0);
    shooterSpeedMap.put(Units.inchesToMeters(140), 9000.0);
    shooterSpeedMap.put(Units.inchesToMeters(160), 10000.0);
    shooterSpeedMap.put(Units.inchesToMeters(180), 11000.0);

    // Units: radians
    shooterAngleMap.put(Units.inchesToMeters(60), Units.degreesToRadians(-45));
    shooterAngleMap.put(Units.inchesToMeters(80), Units.degreesToRadians(-38));
    shooterAngleMap.put(Units.inchesToMeters(100), Units.degreesToRadians(-34));
    shooterAngleMap.put(Units.inchesToMeters(120), Units.degreesToRadians(-32));
    shooterAngleMap.put(Units.inchesToMeters(140), Units.degreesToRadians(-30));
    shooterAngleMap.put(Units.inchesToMeters(160), Units.degreesToRadians(-29));
    shooterAngleMap.put(Units.inchesToMeters(180), Units.degreesToRadians(-28));

    // Units: seconds
    flightTimeMap.put(Units.inchesToMeters(40), 0.5);
    flightTimeMap.put(Units.inchesToMeters(150), 0.8);
  }

  public static SmartController getInstance() {
    if (instance == null) {
      instance = new SmartController();
    }
    Logger.recordOutput("SmartController/smartControl", instance.smartControl);
    return instance;
  }

  /**
   * Gets the shooter speed for a given distance.
   *
   * @param distance The distance to the target in meters.
   * @return The shooter speed for the given distance.
   */
  public Double getShooterSpeed(double distance) {
    return shooterSpeedMap.get(distance);
  }

  /**
   * Gets the shooter angle for a given distance.
   *
   * @param distance The distance to the target in inches.
   * @return The shooter angle for the given distance.
   */
  public Double getShooterAngle(double distance) {
    return shooterAngleMap.get(Units.inchesToMeters(distance));
  }

  /**
   * Checks if the heading is being controlled.
   *
   * @return True if the heading is being controlled, false otherwise.
   */
  public boolean isSmartControlEnabled() {
    return this.smartControl;
  }

  /**
   * Gets the current drive mode.
   *
   * @return The supplier that provides the current drive mode.
   */
  public DriveModeType getDriveModeType() {
    return this.driveModeType;
  }

  /**
   * Sets the drive mode.
   *
   * @param driveModeType The drive mode to set.
   */
  public void setDriveMode(DriveModeType driveModeType) {
    this.driveModeType = driveModeType;
  }

  /** Toggles the drive mode between AMP and SPEAKER. */
  public void toggleDriveMode() {
    if (this.driveModeType == DriveModeType.AMP) {
      setDriveMode(SmartController.DriveModeType.SPEAKER);
    } else {
      setDriveMode(SmartController.DriveModeType.AMP);
    }
  }

  /** Enables heading control based on the current drive mode. */
  public void enableSmartControl() {
    this.smartControl = true;
  }

  /** Disables heading control (heading control is disabled). */
  public void disableSmartControl() {
    this.smartControl = false;
  }

  public void calculateSpeaker(Pose2d fieldRelativePose, Translation2d fieldRelativeVelocity) {
    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
    double distanceToSpeaker = fieldRelativePose.getTranslation().getDistance(speakerPose);
    double shotTime = flightTimeMap.get(distanceToSpeaker);
    Translation2d movingGoalLocation = speakerPose.minus(fieldRelativeVelocity.times(shotTime));
    Translation2d toTestGoal = movingGoalLocation.minus(fieldRelativePose.getTranslation());
    double newDistanceToSpeaker = toTestGoal.getNorm();
    double newShotTime = flightTimeMap.get(newDistanceToSpeaker);
    for (int i = 0; i < 5 && Math.abs(newShotTime - shotTime) > 0.01; i++) {
      shotTime = newShotTime;
      movingGoalLocation = speakerPose.minus(fieldRelativeVelocity.times(shotTime));
      toTestGoal = movingGoalLocation.minus(fieldRelativePose.getTranslation());
      newDistanceToSpeaker = toTestGoal.getNorm();
      newShotTime = flightTimeMap.get(newDistanceToSpeaker);
    }
    Rotation2d setpointAngle =
        movingGoalLocation.minus(fieldRelativePose.getTranslation()).getAngle();
    double angleDifference = setpointAngle.minus(fieldRelativePose.getRotation()).getRadians();

    // Assuming a constant linear velocity (you can adjust this)
    // double assumedLinearVelocity = fieldRelativeVelocity.getNorm();

    // Calculate tangential velocity using linear velocity and angle difference

    // double tangentialVelocity = assumedLinearVelocity * Math.sin(angleDifference);

    // Now, calculate angular velocity using tangential velocity and newDistanceToSpeaker

    // double radialVelocity = tangentialVelocity / newDistanceToSpeaker;
    double radialVelocity = 0.0;
    Logger.recordOutput("ShotCalculator/effectiveDistanceToSpeaker", newDistanceToSpeaker);
    Logger.recordOutput(
        "ShotCalculator/effectiveAimingPose", new Pose2d(movingGoalLocation, setpointAngle));
    Logger.recordOutput("ShotCalculator/angleDifference", angleDifference);
    Logger.recordOutput("ShotCalculator/radialVelocity", radialVelocity);
    setTargetAimingParameters(
        new AimingParameters(
            setpointAngle,
            radialVelocity,
            shooterSpeedMap.get(newDistanceToSpeaker),
            new Rotation2d(shooterAngleMap.get(newDistanceToSpeaker))));
  }

  public void calculateAmp() {
    setTargetAimingParameters(
        new AimingParameters(Rotation2d.fromDegrees(90), 0.0, 1000, ArmConstants.frontAmp.wrist()));
  }

  public void setTargetAimingParameters(AimingParameters targetAimingParameters) {
    this.targetAimingParameters = targetAimingParameters;
  }

  public AimingParameters getTargetAimingParameters() {
    return targetAimingParameters;
  }

  public record AimingParameters(
      Rotation2d robotAngle, double radialVelocity, double shooterSpeed, Rotation2d shooterAngle) {}

  /** Possible Drive Modes. */
  public enum DriveModeType {
    AMP,
    SPEAKER,
  }
}
