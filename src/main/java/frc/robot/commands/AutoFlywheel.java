// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class AutoFlywheel extends Command {
  Flywheel flywheel;
  Drive drive;

  LinearInterpolationTable table = new LinearInterpolationTable(
      new Point2D.Double(0, 8000),
      new Point2D.Double(5, 12000),
      new Point2D.Double(8, 16000),
      new Point2D.Double(10, 18000));

  Pose3d targetRed = new Pose3d(16.3, 5.54, 2, new Rotation3d());
  Pose3d targetBlue = new Pose3d(0.3, 5.54, 2, new Rotation3d());

  Pose3d stageRed = new Pose3d(11.74, 4.11, 0, new Rotation3d());
  Pose3d stageBlue = new Pose3d(4.9, 4.11, 0, new Rotation3d());

  Pose3d target;
  boolean isRed = false;

  /** Creates a new AutoFlywheel. */
  public AutoFlywheel(Flywheel flywheel, Drive drive) {
    this.flywheel = flywheel;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
  }

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

    flywheel.setSpeedRPM(table.getOutput(distance));
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
