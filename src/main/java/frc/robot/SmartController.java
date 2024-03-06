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
    shooterSpeedMap.put(1.4, 40.5);
    shooterSpeedMap.put(1.57, 40.5);
    shooterSpeedMap.put(2.195, 40.5);
    shooterSpeedMap.put(2.361, 40.5);
    shooterSpeedMap.put(2.633, 40.5);
    shooterSpeedMap.put(2.709, 40.5);
    // OLD
    // shooterSpeedMap.put(1.45, 40.5);
    // shooterSpeedMap.put(2.087, 40.5);
    // shooterSpeedMap.put(2.24, 40.5);
    // shooterSpeedMap.put(2.74, 40.5);
    // shooterSpeedMap.put(3.0, 40.5);
    // shooterSpeedMap.put(3.211, 40.5);

    // Units: radians
    // NEW
    shooterAngleMap.put(1.4, Units.degreesToRadians(80));
    shooterAngleMap.put(1.57, Units.degreesToRadians(75));
    shooterAngleMap.put(2.195, Units.degreesToRadians(70));
    shooterAngleMap.put(2.361, Units.degreesToRadians(67));
    shooterAngleMap.put(2.633, Units.degreesToRadians(63));
    shooterAngleMap.put(2.709, Units.degreesToRadians(61.5));

    // OLD
    // shooterAngleMap.put(1.45, Units.degreesToRadians(80));
    // shooterAngleMap.put(1.7, Units.degreesToRadians(75));
    // shooterAngleMap.put(2.24, Units.degreesToRadians(72));
    // shooterAngleMap.put(2.41, Units.degreesToRadians(67));
    // shooterAngleMap.put(3.0, Units.degreesToRadians(63));
    // shooterAngleMap.put(3.211, Units.degreesToRadians(62.5));

    flightTimeMap.put(1.2, 0.2);
    flightTimeMap.put(4.0, 0.5);
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
    double effectiveDistanceToSpeaker = toTestGoal.getNorm();
    double newShotTime = flightTimeMap.get(effectiveDistanceToSpeaker);
    for (int i = 0; i < 5 && Math.abs(newShotTime - shotTime) > 0.01; i++) {
      shotTime = newShotTime;
      movingGoalLocation = speakerPose.minus(fieldRelativeVelocity.times(shotTime));
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
  }
}
