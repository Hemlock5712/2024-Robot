// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.SmartController;
import frc.robot.SmartController.AimingParameters;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final LoggedTunableNumber autoAimFieldVelocityDeadband =
      new LoggedTunableNumber("autoaim/fieldvelocitydeadband", 0.5);

  private static Translation2d lastFieldRelativeVelocity = new Translation2d();

  private static long lastTime = Logger.getRealTimestamp();

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static final Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    ProfiledPIDController aimController =
        new ProfiledPIDController(
            headingControllerConstants.Kp(),
            0,
            headingControllerConstants.Kd(),
            new TrapezoidProfile.Constraints(
                drivetrainConfig.maxAngularVelocity(), drivetrainConfig.maxAngularAcceleration()),
            Constants.loopPeriodMs);
    aimController.reset(drive.getRotation().getRadians());
    aimController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          if (SmartController.getInstance().isSmartControlEnabled()) {
            linearMagnitude = Math.min(linearMagnitude, 0.75);
          }

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Get robot relative vel
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          Optional<Rotation2d> targetGyroAngle = Optional.empty();
          Rotation2d measuredGyroAngle = drive.getRotation();
          double feedForwardRadialVelocity = 0.0;

          double robotRelativeXVel = linearVelocity.getX() * drivetrainConfig.maxLinearVelocity();
          double robotRelativeYVel = linearVelocity.getY() * drivetrainConfig.maxLinearVelocity();

          Translation2d deadbandFieldRelativeVelocity =
              (drive.getFieldRelativeVelocity().getNorm() < autoAimFieldVelocityDeadband.get())
                  ? new Translation2d(0, 0)
                  : drive.getFieldRelativeVelocity();
          long currentTime = Logger.getRealTimestamp();
          Translation2d deadbandFieldRelativeAcceleration =
              deadbandFieldRelativeVelocity
                  .minus(lastFieldRelativeVelocity)
                  .div((currentTime - lastTime) * 1e-6);
          lastFieldRelativeVelocity = deadbandFieldRelativeVelocity;
          lastTime = currentTime;

          // Amp Mode
          if (SmartController.getInstance().isSmartControlEnabled()
              && SmartController.getInstance().getDriveModeType() == DriveModeType.AMP) {
            SmartController.getInstance().calculateAmp();
            AimingParameters calculatedAim =
                SmartController.getInstance().getTargetAimingParameters();
            targetGyroAngle = Optional.of(calculatedAim.robotAngle());
          }
          // Feed Mode
          else if (SmartController.getInstance().isSmartControlEnabled()
              && SmartController.getInstance().getDriveModeType() == DriveModeType.FEED) {
            SmartController.getInstance()
                .calculateFeed(drive.getPose(), deadbandFieldRelativeVelocity);
            AimingParameters calculatedAim =
                SmartController.getInstance().getTargetAimingParameters();
            targetGyroAngle = Optional.of(calculatedAim.robotAngle());
            feedForwardRadialVelocity = calculatedAim.radialVelocity();
          } else {
            SmartController.getInstance()
                .calculateSpeaker(
                    drive.getPose(),
                    deadbandFieldRelativeVelocity,
                    deadbandFieldRelativeAcceleration);
            AimingParameters calculatedAim =
                SmartController.getInstance().getTargetAimingParameters();
            targetGyroAngle = Optional.of(calculatedAim.robotAngle());
            feedForwardRadialVelocity = calculatedAim.radialVelocity();
          }
          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  robotRelativeXVel,
                  robotRelativeYVel,
                  SmartController.getInstance().isSmartControlEnabled()
                          && targetGyroAngle.isPresent()
                      ? feedForwardRadialVelocity
                          + aimController.calculate(
                              measuredGyroAngle.getRadians(), targetGyroAngle.get().getRadians())
                      : omega * drivetrainConfig.maxAngularVelocity(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());

          // Convert to field relative speeds & send command
          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }
}
