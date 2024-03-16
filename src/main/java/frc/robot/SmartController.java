package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The SmartController class represents a controller for the robot's system. It provides methods to
 * control the robot.
 */
public class SmartController {
  private static SmartController instance;

  private DriveModeType driveModeType = DriveModeType.SAFE;
  private AimingParameters targetAimingParameters =
      new AimingParameters(Rotation2d.fromDegrees(90), 0.0, 40.5, ArmConstants.shoot.wrist());

  private boolean smartControl = false;
  private boolean emergencyIntakeMode = false;

  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

  private SmartController() {
    // NEW
    // shooterSpeedMap.put(1.4, 40.5);
    // shooterSpeedMap.put(1.57, 40.5);
    // shooterSpeedMap.put(2.195, 40.5);
    // shooterSpeedMap.put(2.361, 40.5);
    // shooterSpeedMap.put(2.633, 40.5);
    // shooterSpeedMap.put(2.709, 40.5);
    // shooterSpeedMap.put(6.2, 40.5);
    shooterSpeedMap.put(1.177, 25.0);
    shooterSpeedMap.put(1.464, 26.0);
    shooterSpeedMap.put(1.734, 27.0);
    shooterSpeedMap.put(2.135, 28.0);
    shooterSpeedMap.put(2.469, 29.0);
    shooterSpeedMap.put(2.889, 30.0);
    shooterSpeedMap.put(3.174, 31.0);
    shooterSpeedMap.put(3.635, 32.0);
    shooterSpeedMap.put(3.913, 34.0);
    shooterSpeedMap.put(4.079, 36.0);

    // Units: radians
    // Old
    // shooterAngleMap.put(1.4, Units.degreesToRadians(80));
    // shooterAngleMap.put(1.57, Units.degreesToRadians(75));
    // shooterAngleMap.put(2.195, Units.degreesToRadians(69));
    // shooterAngleMap.put(2.361, Units.degreesToRadians(67));
    // shooterAngleMap.put(2.633, Units.degreesToRadians(63));
    // shooterAngleMap.put(2.709, Units.degreesToRadians(61.5));
    // shooterAngleMap.put(5.2, Units.degreesToRadians(53));
    // shooterAngleMap.put(6.2, Units.degreesToRadians(49));
    shooterAngleMap.put(1.177, Units.degreesToRadians(80));
    shooterAngleMap.put(1.464, Units.degreesToRadians(78));
    shooterAngleMap.put(1.734, Units.degreesToRadians(74));
    shooterAngleMap.put(2.135, Units.degreesToRadians(69));
    shooterAngleMap.put(2.469, Units.degreesToRadians(61));
    shooterAngleMap.put(2.889, Units.degreesToRadians(59));
    shooterAngleMap.put(3.174, Units.degreesToRadians(56));
    shooterAngleMap.put(3.635, Units.degreesToRadians(53));
    shooterAngleMap.put(3.913, Units.degreesToRadians(52));
    shooterAngleMap.put(4.079, Units.degreesToRadians(51));

    flightTimeMap.put(1.2, 0.3);
    flightTimeMap.put(2.709, 0.75);
  }

  public static SmartController getInstance() {
    if (instance == null) {
      instance = new SmartController();
    }
    Logger.recordOutput("SmartController/smartControl", instance.smartControl);
    Logger.recordOutput("SmartController/driveModeType", instance.driveModeType.toString());
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

  public void setEmergencyIntakeMode(boolean emergencyMode) {
    this.emergencyIntakeMode = emergencyMode;
  }

  public boolean getEmergencyIntakeMode() {
    return this.emergencyIntakeMode;
  }

  public void toggleEmergencyIntakeMode() {
    this.emergencyIntakeMode = !emergencyIntakeMode;
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

  /** Enables heading control based on the current drive mode. */
  public void enableSmartControl() {
    this.smartControl = true;
  }

  /** Disables heading control (heading control is disabled). */
  public void disableSmartControl() {
    this.smartControl = false;
  }

  public void calculateSpeaker(
      Pose2d fieldRelativePose,
      Translation2d fieldRelativeVelocity,
      Translation2d fieldRelativeAcceleration) {
    Logger.recordOutput("ShotCalculator/fieldRelativePose", fieldRelativePose);
    Logger.recordOutput("ShotCalculator/fieldRelativeVelocity", fieldRelativeVelocity);
    Logger.recordOutput("ShotCalculator/fieldRelativeAcceleration", fieldRelativeAcceleration);
    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
    double distanceToSpeaker = fieldRelativePose.getTranslation().getDistance(speakerPose);
    double shotTime = flightTimeMap.get(distanceToSpeaker);
    Translation2d speedAccComp = fieldRelativeVelocity.plus(fieldRelativeAcceleration.times(0.025));
    Translation2d movingGoalLocation = speakerPose.minus(speedAccComp.times(shotTime));
    Translation2d toTestGoal = movingGoalLocation.minus(fieldRelativePose.getTranslation());
    double effectiveDistanceToSpeaker = toTestGoal.getNorm();
    double newShotTime = flightTimeMap.get(effectiveDistanceToSpeaker);
    for (int i = 0; i < 5 && Math.abs(newShotTime - shotTime) > 0.01; i++) {
      shotTime = newShotTime;
      speedAccComp = fieldRelativeVelocity.plus(fieldRelativeAcceleration.times(0.025));
      movingGoalLocation = speakerPose.minus(speedAccComp.times(shotTime));
      toTestGoal = movingGoalLocation.minus(fieldRelativePose.getTranslation());
      effectiveDistanceToSpeaker = toTestGoal.getNorm();
      newShotTime = flightTimeMap.get(effectiveDistanceToSpeaker);
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
    Logger.recordOutput("ShotCalculator/effectiveDistanceToSpeaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "ShotCalculator/effectiveAimingPose", new Pose2d(movingGoalLocation, setpointAngle));
    Logger.recordOutput("ShotCalculator/angleDifference", angleDifference);
    Logger.recordOutput("ShotCalculator/radialVelocity", radialVelocity);
    setTargetAimingParameters(
        new AimingParameters(
            setpointAngle,
            radialVelocity,
            shooterSpeedMap.get(effectiveDistanceToSpeaker),
            new Rotation2d(shooterAngleMap.get(effectiveDistanceToSpeaker))));
  }

  public void calculateAmp() {
    setTargetAimingParameters(
        new AimingParameters(Rotation2d.fromDegrees(90), 0.0, 10, ArmConstants.frontAmp.wrist()));
  }

  public void calculateFeed(Pose2d fieldRelativePose, Translation2d fieldRelativeVelocity) {
    Translation2d feedLocation = AllianceFlipUtil.apply(FieldConstants.cornerFeedLocation);
    double distanceToFeedLocation = fieldRelativePose.getTranslation().getDistance(feedLocation);
    double shotTime = flightTimeMap.get(distanceToFeedLocation);
    Translation2d movingGoalLocation = feedLocation.minus(fieldRelativeVelocity.times(shotTime));
    Translation2d toTestGoal = movingGoalLocation.minus(fieldRelativePose.getTranslation());
    double effectiveDistanceToFeedLocation = toTestGoal.getNorm();
    Rotation2d setpointAngle =
        movingGoalLocation.minus(fieldRelativePose.getTranslation()).getAngle();
    double angleDifference = setpointAngle.minus(fieldRelativePose.getRotation()).getRadians();
    double radialVelocity = 0.0;
    Logger.recordOutput(
        "ShotCalculator/effectiveDistanceToFeedLocation", effectiveDistanceToFeedLocation);
    Logger.recordOutput(
        "ShotCalculator/effectiveAimingPose", new Pose2d(movingGoalLocation, setpointAngle));
    Logger.recordOutput("ShotCalculator/angleDifference", angleDifference);
    Logger.recordOutput("ShotCalculator/radialVelocity", radialVelocity);
    setTargetAimingParameters(
        new AimingParameters(setpointAngle, radialVelocity, 40.5, new Rotation2d(0)));
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
    SAFE,
    CLIMBER,
    FEED
  }
}
