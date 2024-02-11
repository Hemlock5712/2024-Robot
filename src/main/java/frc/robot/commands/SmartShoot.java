// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.lineBreak.LineBreak;
import frc.robot.subsystems.magazine.Magazine;
import java.util.function.Supplier;

public class SmartShoot extends Command {
  Supplier<Pose2d> pose;
  Arm arm;
  Flywheel flywheel;
  Magazine magazine;
  LineBreak lineBreak;

  /** Creates a new Shoot. */
  public SmartShoot(
      Arm arm, Flywheel flywheel, Magazine magazine, LineBreak lineBreak, Supplier<Pose2d> pose) {
    this.arm = arm;
    this.flywheel = flywheel;
    this.magazine = magazine;
    this.pose = pose;
    this.lineBreak = lineBreak;
    addRequirements(magazine);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveController.getInstance().enableSmartControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriveController.getInstance().isSmartControlEnabled()
        && arm.isArmWristInTargetPose()
        && Math.abs(
                pose.get()
                    .getRotation()
                    .minus(DriveController.getInstance().getTargetAimingParameters().robotAngle())
                    .getRadians())
            < 0.1
        && flywheel.atTargetSpeed()) {
      magazine.forward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveController.getInstance().disableSmartControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return !lineBreak.hasGamePiece();
  }
}
