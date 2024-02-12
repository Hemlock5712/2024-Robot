package frc.robot.subsystems.drive;

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
 * The DriveController class represents a controller for the robot's drive system. It provides
 * methods to control the heading and drive mode of the robot.
 */
public class SmartController {
  private static SmartController instance;

  private DriveModeType driveModeType = DriveModeType.SPEAKER;
  private AimingParameters targetAimingParameters =
      new AimingParameters(
          Rotation2d.fromDegrees(90),
          0.0,
          1000,
          Rotation2d.fromDegrees(ArmConstants.frontAmp.wrist()));
  private boolean smartControl = false;

  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

  private SmartController() {
    shooterSpeedMap.put(Units.inchesToMeters(60), 10.0);
    shooterSpeedMap.put(Units.inchesToMeters(80), 17.0);
    shooterSpeedMap.put(Units.inchesToMeters(100), 9.0);
    shooterSpeedMap.put(Units.inchesToMeters(120), 30.0);
    shooterSpeedMap.put(Units.inchesToMeters(140), 19.0);
    shooterSpeedMap.put(Units.inchesToMeters(160), 88.0);
    shooterSpeedMap.put(Units.inchesToMeters(180), 10.0);

    // Units: radians
    shooterAngleMap.put(Units.inchesToMeters(60), 1.0);
    shooterAngleMap.put(Units.inchesToMeters(80), 0.0);
    shooterAngleMap.put(Units.inchesToMeters(100), 3.5);
    shooterAngleMap.put(Units.inchesToMeters(120), 3.0);
    shooterAngleMap.put(Units.inchesToMeters(140), 1.9);
    shooterAngleMap.put(Units.inchesToMeters(160), 8.0);
    shooterAngleMap.put(Units.inchesToMeters(180), 11.0);

    // Units: seconds
    flightTimeMap.put(Units.inchesToMeters(40), 0.5);
    flightTimeMap.put(Units.inchesToMeters(150), 0.8);
  }

  public static SmartController getInstance() {
    if (instance == null) {
      instance = new SmartController();
    }
    Logger.recordOutput("DriveController/smartControl", instance.smartControl);
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

  public void calculateSpeaker(
      Translation2d fieldRelativePose, Translation2d fieldRelativeVelocity) {
    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
    double distanceToSpeaker = fieldRelativePose.getDistance(speakerPose);
    double shotTime = flightTimeMap.get(distanceToSpeaker);
    Translation2d movingGoalLocation = speakerPose.minus(fieldRelativeVelocity.times(shotTime));
    Translation2d toTestGoal = movingGoalLocation.minus(fieldRelativePose);
    double newDistanceToSpeaker = toTestGoal.getNorm();
    double newShotTime = flightTimeMap.get(newDistanceToSpeaker);
    for (int i = 0; i < 5 && Math.abs(newShotTime - shotTime) > 0.01; i++) {
      shotTime = newShotTime;
      distanceToSpeaker = fieldRelativePose.getDistance(speakerPose);
      shotTime = flightTimeMap.get(distanceToSpeaker);
      movingGoalLocation = speakerPose.minus(fieldRelativeVelocity.times(shotTime));
      toTestGoal = movingGoalLocation.minus(fieldRelativePose);
      newDistanceToSpeaker = toTestGoal.getNorm();
      newShotTime = flightTimeMap.get(newDistanceToSpeaker);
    }
    Rotation2d setpointAngle = movingGoalLocation.minus(fieldRelativePose).getAngle();
    // double tangentialVelocity =
    // -fieldRelativeVelocity.rotateBy(setpointAngle.unaryMinus()).getY();
    // double radialVelocity = tangentialVelocity / newDistanceToSpeaker;
    double radialVelocity = 0.0;
    Logger.recordOutput("ShotCalculator/effectiveDistanceToSpeaker", newDistanceToSpeaker);
    Logger.recordOutput(
        "ShotCalculator/effectiveAimingPose", new Pose2d(movingGoalLocation, new Rotation2d()));
    Logger.recordOutput("ShotCalculator/robotAngle", setpointAngle);
    setTargetAimingParameters(
        new AimingParameters(
            setpointAngle,
            radialVelocity,
            shooterSpeedMap.get(newDistanceToSpeaker),
            new Rotation2d(shooterAngleMap.get(newDistanceToSpeaker))));
  }

  public void calculateAmp() {
    setTargetAimingParameters(
        new AimingParameters(
            Rotation2d.fromDegrees(90),
            0.0,
            1000,
            Rotation2d.fromDegrees(ArmConstants.frontAmp.wrist())));
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