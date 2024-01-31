// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class DistanceTrackWithArm extends Command {
  Drive drive;
  Arm arm;

  public DistanceTrackWithArm(Arm arm, Drive drive) {
    this.drive = drive;
    this.arm = arm;
    addRequirements(arm);
  }

  LinearInterpolationTable table = new LinearInterpolationTable(
      new Point2D.Double(0, Units.degreesToRadians(0)),
      new Point2D.Double(5, Units.degreesToRadians(5)),
      new Point2D.Double(8, Units.degreesToRadians(20)),
      new Point2D.Double(10, Units.degreesToRadians(30)));

  Pose3d targetRed = new Pose3d(16.3, 5.54, 2, new Rotation3d());
  Pose3d targetBlue = new Pose3d(0.3, 5.54, 2, new Rotation3d());

  Pose3d stageRed = new Pose3d(11.74, 4.11, 0, new Rotation3d());
  Pose3d stageBlue = new Pose3d(4.9, 4.11, 0, new Rotation3d());

  Pose3d target;

  double stageRadius = 1.8;
  double midpoint = Units.inchesToMeters(325.6);
  boolean isRed = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      target = targetRed;
      isRed = true;
    } else {
      target = targetBlue;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    double distance = robotPose.getTranslation().getDistance(target.getTranslation().toTranslation2d());
    double heightDifference = target.getTranslation().getZ() - Units.inchesToMeters(24);
    // Calculate angle to target from distance and height difference
    double angle = Math.atan2(heightDifference, distance);

    // If within 1.5m of stage, move arm to stage position
    double stageRedDistance = robotPose.getTranslation().getDistance(stageRed.getTranslation().toTranslation2d());
    double stageBlueDistance = robotPose.getTranslation().getDistance(stageBlue.getTranslation().toTranslation2d());

    if (stageRedDistance < stageRadius || stageBlueDistance < stageRadius) {
      arm.setArmTarget(Units.degreesToRadians(-10));
      arm.setWristTarget(Units.degreesToRadians(0));
      return;
    }

    // If on other side of midpoint, move arm to intake position
    if ((isRed && robotPose.getTranslation().getX() <= midpoint)
        || (!isRed && robotPose.getTranslation().getX() > midpoint)) {
      arm.setArmTarget(Units.degreesToRadians(-25));
      arm.setWristTarget(Units.degreesToRadians(-38));
      return;
    }

    double additionalAngle = table.getOutput(distance);

    arm.setArmTarget(Units.degreesToRadians(-25));
    arm.setWristTarget(-angle - additionalAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
