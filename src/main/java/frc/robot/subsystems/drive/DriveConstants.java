package frc.robot.subsystems.drive;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  public static DrivetrainConfig drivetrainConfig =
      switch (Constants.getRobot()) {
        /** Values of path planner and potential other drivetrain changes.
         * 60 As follows:
         * 4.608, 6.627, 11.277, 33.436
         * 
         * 40A As follows:
         * 4.608, 4.418, 11.277 (646), 22.290 (1277)
         * 
         * PathPlanner default values:
         * "maxVelocity": 2.5,
         * "maxAcceleration": 3.0,
         * "maxAngularVelocity": 540.0,
         * "maxAngularAcceleration": 720.0
         * 
        */
        default ->
            new DrivetrainConfig(
                Units.inchesToMeters(2.0),
                Units.inchesToMeters(22.75),
                Units.inchesToMeters(22.75),
                4.608,
                4.418,
                11.277,
                22.290);
      };
  public static final double wheelRadius = Units.inchesToMeters(2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0)
      };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);
  public static final double odometryFrequency =
      switch (Constants.getRobot()) {
        case SIMBOT -> 50.0;
        case COMPBOT -> 250.0;
      };
  public static final Matrix<N3, N1> stateStdDevs =
      switch (Constants.getRobot()) {
        default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
      };
  public static final double xyStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01;
      };
  public static final double thetaStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01;
      };

  public static final int gyroID = 13;

  // Turn to "" for no canbus name
  public static final String canbus = "chassis";

  public static ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConfig[] {
              // 0.454346
              new ModuleConfig(1, 2, 9, Rotation2d.fromRotations(-0.041504 + .5), true),
              // -0.305420
              new ModuleConfig(3, 4, 10, Rotation2d.fromRotations(-0.310547 + .5), true),
              // -0.486084
              new ModuleConfig(5, 6, 11, Rotation2d.fromRotations(-0.487061), true),
              // -0.052002
              new ModuleConfig(7, 8, 12, Rotation2d.fromRotations(0.336914), true)
            };
        case SIMBOT -> {
          ModuleConfig[] configs = new ModuleConfig[4];
          for (int i = 0; i < configs.length; i++)
            configs[i] = new ModuleConfig(0, 0, 0, new Rotation2d(0), false);
          yield configs;
        }
      };

  public static final ModuleConstants moduleConstants =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConstants(
                0.1,
                0.13,
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L4.reduction,
                Mk4iReductions.TURN.reduction);
        case SIMBOT ->
            new ModuleConstants(
                0.014,
                0.134,
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L4.reduction,
                Mk4iReductions.TURN.reduction);
      };

  public static HeadingControllerConstants headingControllerConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new HeadingControllerConstants(5.0, 0.0);
        case SIMBOT -> new HeadingControllerConstants(7, 0.0);
      };

  public static final PIDConstants PPtranslationConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PIDConstants(5, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0);
      };

  public static final PIDConstants PProtationConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PIDConstants(5, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0);
      };

  public record DrivetrainConfig(
      double wheelRadius,
      double trackwidthX,
      double trackwidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackwidthX / 2.0, trackwidthY / 2.0);
    }
  }

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  public record ModuleConstants(
      double ffKs,
      double ffKv,
      double driveKp,
      double drivekD,
      double turnKp,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  public record HeadingControllerConstants(double Kp, double Kd) {}

  private enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    L4((50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
