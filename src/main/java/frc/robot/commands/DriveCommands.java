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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static DriveController driveMode = new DriveController();

  static {
    driveMode.disableHeadingSupplier();
  }

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DriveController driveController,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    ProfiledPIDController thetaController =
        new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(8, 8));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(1.5));

    return Commands.run(
        () -> {
          double omega = 0;
          if (driveController.isHeadingControlled()) {
            final var thata = driveController.getHeadingAngle();

            omega =
                thetaController.calculate(
                    drive.getPoseEstimatorPose().getRotation().getRadians(), thata.getRadians());
            if (thetaController.atGoal()) {
              omega = 0;
            }
            omega = Math.copySign(Math.min(1, Math.abs(omega)), omega);

          } else {
            omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          }

          omega = Math.copySign(omega * omega, omega);

          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drivetrainConfig.maxLinearVelocity(),
                  linearVelocity.getY() * drivetrainConfig.maxLinearVelocity(),
                  omega * drivetrainConfig.maxAngularVelocity(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  // public static Command DriveWithAngle(
  // Drive drive,
  // DoubleSupplier xSupplier,
  // DoubleSupplier ySupplier,
  // Supplier<Rotation2d> thetaSupplier) {

  // return Commands.run(
  // () -> {
  // final var theta = thetaSupplier.get();
  // double omega;
  // // Calculate the angular rate for the robot to turn

  // Logger.recordOutput("Drive/Rad",
  // drive.getPoseEstimatorPose().getRotation().getRadians());

  // Logger.recordOutput("Drive/omega", omega);

  // Logger.recordOutput("Drive/DisiredPos", theta.getRadians());

  // // Apply deadband
  // double linearMagnitude =
  // MathUtil.applyDeadband(
  // Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
  // Rotation2d linearDirection =
  // new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
  // // Square values
  // linearMagnitude = linearMagnitude * linearMagnitude;
  // omega = Math.copySign(omega * omega, omega);
  // Logger.recordOutput("Drive/omegaSign", omega);

  // // omega = 0;

  // // Calcaulate new linear velocity
  // Translation2d linearVelocity =
  // new Pose2d(new Translation2d(), linearDirection)
  // .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
  // .getTranslation();

  // // Convert to field relative speeds & send command
  // drive.runVelocity(
  // ChassisSpeeds.fromFieldRelativeSpeeds(
  // linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
  // linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
  // omega * drive.getMaxAngularSpeedRadPerSec(),
  // drive.getRotation()));
  // },
  // drive);
  // }
}
