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
      new AimingParameters(Rotation2d.fromDegrees(90), 0.0, 40.5, ArmConstants.shoot.wrist(), 2, 0);

  private boolean smartControl = false;
  private boolean emergencyIntakeMode = false;

  public static double prerollDistance = 7.002;

  private boolean isFasterToFlipWrist = false;

  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap wristErrorMap = new InterpolatingDoubleTreeMap();

  private final InterpolatingDoubleTreeMap feederSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feederAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feederFlightTimeMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feederWristErrorMap = new InterpolatingDoubleTreeMap();

  private SmartController() {

    // Units: RPS
    shooterSpeedMap.put(1.24036, 30.0);
    shooterSpeedMap.put(1.509, 32.5);
    shooterSpeedMap.put(2.006, 35.0);
    shooterSpeedMap.put(2.5018, 37.5);
    shooterSpeedMap.put(3.002, 40.0);
    shooterSpeedMap.put(3.5116, 42.5);
    shooterSpeedMap.put(4.002, 45.0);

    // // Units: Degress
    shooterAngleMap.put(1.24036, Units.degreesToRadians(78.5));
    shooterAngleMap.put(1.509, Units.degreesToRadians(73.0));
    shooterAngleMap.put(2.006, Units.degreesToRadians(65.0));
    shooterAngleMap.put(2.5018, Units.degreesToRadians(60.0));
    shooterAngleMap.put(3.002, Units.degreesToRadians(56.5));
    shooterAngleMap.put(3.5116, Units.degreesToRadians(53.75));
    shooterAngleMap.put(4.002, Units.degreesToRadians(50.75));

    flightTimeMap.put(1.2, 0.2);
    flightTimeMap.put(4.002, 0.8);

    wristErrorMap.put(1.2, 2.0);
    wristErrorMap.put(4.002, 0.25);

    // Feed Maps
    feederSpeedMap.put(9.071, 30.0);
    feederSpeedMap.put(5.4, 15.0);

    feederAngleMap.put(9.071, Units.degreesToRadians(50.5 + 23));
    feederAngleMap.put(5.4, Units.degreesToRadians(50.5 + 23));

    feederFlightTimeMap.put(30.0, 3.0); // Way further than we should ever be shooting
    feederFlightTimeMap.put(9.071, 1.9);
    feederFlightTimeMap.put(5.4, 0.9);
    feederFlightTimeMap.put(0.0, 0.0); // Way less than we should ever be shooting

    feederWristErrorMap.put(9.071, 2.0);
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

  private boolean calculateIsFlipFaster(
      Pose2d fieldRelativePose, Translation2d targetPose, boolean isSpeakerShot) {
    if (isSpeakerShot && fieldRelativePose.getTranslation().getDistance(targetPose) > 1.8) {
      return false;
    }
    if (Math.abs(
            targetPose
                .minus(fieldRelativePose.getTranslation())
                .getAngle()
                .minus(fieldRelativePose.getRotation())
                .getDegrees())
        > 90) {
      return true;
    }
    return false;
  }

  public boolean isFlipFaster() {
    return isFasterToFlipWrist;
  }

  public void calculateSpeaker(
      Pose2d fieldRelativePose,
      Translation2d fieldRelativeVelocity,
      Translation2d fieldRelativeAcceleration) {
    Logger.recordOutput("ShotCalculator/fieldRelativePose", fieldRelativePose);
    Logger.recordOutput("ShotCalculator/fieldRelativeVelocity", fieldRelativeVelocity);
    Logger.recordOutput("ShotCalculator/fieldRelativeAcceleration", fieldRelativeAcceleration);
    SmartController.prerollDistance = 7.002;
    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
    isFasterToFlipWrist = calculateIsFlipFaster(fieldRelativePose, speakerPose, true);
    Logger.recordOutput("ShotCalculator/isFasterToFlipWrist", isFasterToFlipWrist);
    double distanceToSpeaker = fieldRelativePose.getTranslation().getDistance(speakerPose);
    if (isFasterToFlipWrist) {
      // Add distance to where flywheel actually is to account for it not being centered in the
      // robot
      distanceToSpeaker += Units.inchesToMeters(13);
    }
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
    if (!isFasterToFlipWrist) {
      setTargetAimingParameters(
          new AimingParameters(
              setpointAngle,
              radialVelocity,
              shooterSpeedMap.get(effectiveDistanceToSpeaker),
              new Rotation2d(shooterAngleMap.get(effectiveDistanceToSpeaker)),
              wristErrorMap.get(effectiveDistanceToSpeaker),
              effectiveDistanceToSpeaker));
    } else {
      setTargetAimingParameters(
          new AimingParameters(
              setpointAngle.rotateBy(Rotation2d.fromDegrees(180)),
              radialVelocity,
              shooterSpeedMap.get(effectiveDistanceToSpeaker),
              new Rotation2d(shooterAngleMap.get(effectiveDistanceToSpeaker)),
              wristErrorMap.get(effectiveDistanceToSpeaker),
              effectiveDistanceToSpeaker));
    }
  }

  public void calculateAmp() {
    setTargetAimingParameters(
        new AimingParameters(
            Rotation2d.fromDegrees(90), 0.0, 20, ArmConstants.frontAmp.wrist(), 1, 0));
  }

  public void calculateFeed(Pose2d fieldRelativePose, Translation2d fieldRelativeVelocity) {
    SmartController.prerollDistance = 9.071;
    Translation2d feedLocation = AllianceFlipUtil.apply(FieldConstants.cornerFeedLocation);
    isFasterToFlipWrist = calculateIsFlipFaster(fieldRelativePose, feedLocation, false);
    double distanceToFeedLocation = fieldRelativePose.getTranslation().getDistance(feedLocation);
    if (isFasterToFlipWrist) {
      // Add distance to where flywheel actually is to account for it not being centered in the
      // robot
      distanceToFeedLocation += Units.inchesToMeters(13);
    }
    double shotTime = feederFlightTimeMap.get(distanceToFeedLocation);
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

    if (!isFasterToFlipWrist) {
      setTargetAimingParameters(
          new AimingParameters(
              setpointAngle,
              radialVelocity,
              feederSpeedMap.get(effectiveDistanceToFeedLocation),
              new Rotation2d(feederAngleMap.get(effectiveDistanceToFeedLocation)),
              feederWristErrorMap.get(effectiveDistanceToFeedLocation),
              effectiveDistanceToFeedLocation));
    } else {
      setTargetAimingParameters(
          new AimingParameters(
              setpointAngle.rotateBy(Rotation2d.fromDegrees(180)),
              radialVelocity,
              feederSpeedMap.get(effectiveDistanceToFeedLocation),
              new Rotation2d(feederAngleMap.get(effectiveDistanceToFeedLocation)),
              feederWristErrorMap.get(effectiveDistanceToFeedLocation),
              effectiveDistanceToFeedLocation));
    }
  }

  public void setTargetAimingParameters(AimingParameters targetAimingParameters) {
    this.targetAimingParameters = targetAimingParameters;
  }

  public AimingParameters getTargetAimingParameters() {
    return targetAimingParameters;
  }

  public record AimingParameters(
      Rotation2d robotAngle,
      double radialVelocity,
      double shooterSpeed,
      Rotation2d shooterAngle,
      double wristError,
      double effectiveDistanceToTarget) {}

  /** Possible Drive Modes. */
  public enum DriveModeType {
    AMP,
    SPEAKER,
    SAFE,
    CLIMBER,
    FEED
  }
}
