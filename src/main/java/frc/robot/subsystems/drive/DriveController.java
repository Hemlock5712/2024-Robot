package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The DriveController class represents a controller for the robot's drive system. It provides
 * methods to control the heading and drive mode of the robot.
 */
public class DriveController {
  private static DriveController instance;

  private DriveModeType driveModeType = DriveModeType.SPEAKER;
  private AimingParameters targetAimingParameters;
  private boolean headingSupplier = false;

  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

  private DriveController() {
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

  public static DriveController getInstance() {
    if (instance == null) {
      instance = new DriveController();
    }
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
  public boolean isHeadingControlled() {
    return this.headingSupplier;
  }

  /**
   * Gets the current drive mode.
   *
   * @return The supplier that provides the current drive mode.
   */
  public Supplier<DriveModeType> getDriveModeType() {
    return () -> this.driveModeType;
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
    if (getDriveModeType().get() == DriveModeType.AMP) {
      setDriveMode(DriveController.DriveModeType.SPEAKER);
    } else {
      setDriveMode(DriveController.DriveModeType.AMP);
    }
  }

  /** Enables heading control based on the current drive mode. */
  public void enableHeadingControl() {
    this.headingSupplier = true;
  }

  /** Disables heading control (heading control is disabled). */
  public void disableHeadingControl() {
    this.headingSupplier = false;
  }

  public void calculate(Translation2d fieldRelativePose, Translation2d fieldRelativeVelocity) {
    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
    Translation2d effectiveAimingPose = new Translation2d();
    double effectiveDistanceToSpeaker = 0.01;
    double distanceToSpeaker = fieldRelativePose.getDistance(speakerPose);
    double shotTime = flightTimeMap.get(distanceToSpeaker);
    for (int i = 0; i < 5; i++) {
      effectiveAimingPose = fieldRelativePose.plus(fieldRelativeVelocity.times(distanceToSpeaker));
      effectiveDistanceToSpeaker = effectiveAimingPose.getDistance(speakerPose);
      double newShotTime = flightTimeMap.get(effectiveDistanceToSpeaker);
      double flightDif = Math.abs(shotTime - newShotTime);
      if (flightDif < 0.01) {
        break;
      } else {
        shotTime = newShotTime;
      }
    }
    Rotation2d setpointAngle = speakerPose.minus(effectiveAimingPose).getAngle();
    double tangentialVelocity = -fieldRelativeVelocity.rotateBy(setpointAngle.unaryMinus()).getY();
    double radialVelocity = tangentialVelocity / effectiveDistanceToSpeaker;
    Logger.recordOutput("ShotCalculator/effectiveDistanceToSpeaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "ShotCalculator/effectiveAimingPose", new Pose2d(effectiveAimingPose, new Rotation2d()));
    Logger.recordOutput("ShotCalculator/robotAngle", setpointAngle);
    setTargetAimingParameters(
        new AimingParameters(
            setpointAngle,
            radialVelocity,
            shooterSpeedMap.get(effectiveDistanceToSpeaker),
            new Rotation2d(shooterAngleMap.get(effectiveDistanceToSpeaker))));
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
