// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.drive.DriveController.DriveModeType;
import frc.robot.subsystems.lineBreak.LineBreak;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;

public class SmartArm extends Command {
  Arm arm;
  Supplier<Pose2d> pose;
  Supplier<DriveModeType> driveModeType;
  LineBreak lineBreak;

  /** Creates a new moveArm. */
  public SmartArm(
      Arm arm, LineBreak lineBreak, Supplier<DriveModeType> driveModeType, Supplier<Pose2d> pose) {
    this.arm = arm;
    this.pose = pose;
    this.driveModeType = driveModeType;
    this.lineBreak = lineBreak;
    addRequirements(arm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lineBreak.isShooterLoaded() || lineBreak.isShooterLong()) {
      if (driveModeType.get() == DriveModeType.AMP) {
        arm.setArmTarget(ArmConstants.frontAmp.getArmRadians());
        arm.setWristTarget(ArmConstants.frontAmp.getWristRadians());
      } else {
        double distance =
            pose.get()
                .getTranslation()
                .getDistance(FieldConstants.Speaker.centerSpeakerOpening3d.toTranslation2d());
        double heightDifference =
            FieldConstants.Speaker.centerSpeakerOpening3d.getZ() - Units.inchesToMeters(24);
        double angle = Math.atan2(heightDifference, distance);

        double additionalAngle = DriveController.getInstance().getShooterAngle(distance);

        arm.setArmTarget(ArmConstants.shoot.getArmRadians());
        arm.setWristTarget(-angle - additionalAngle);
      }
    } else {
      arm.setArmTarget(ArmConstants.intake.getArmRadians());
      arm.setWristTarget(ArmConstants.intake.getArmRadians());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
