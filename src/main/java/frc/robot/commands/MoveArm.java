// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.DriveController.DriveModeType;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;

public class MoveArm extends Command {
  Arm arm;
  Supplier<Pose2d> pose;
  Supplier<DriveModeType> driveModeType;

  /** Creates a new moveArm. */
  public MoveArm(Arm arm, Supplier<DriveModeType> driveModeType, Supplier<Pose2d> pose) {
    this.arm = arm;
    this.pose = pose;
    this.driveModeType = driveModeType;
    addRequirements(arm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (driveModeType.get()) {
      case AMP:
        arm.setArmTarget(Units.degreesToRadians(80));
        arm.setWristTarget(Units.degreesToRadians(-45));
        break;
      case SPEAKER:
        double distance =
            pose.get()
                .getTranslation()
                .getDistance(FieldConstants.Speaker.centerSpeakerOpening3d.toTranslation2d());
        double heightDifference =
            FieldConstants.Speaker.centerSpeakerOpening3d.getZ() - Units.inchesToMeters(24);
        double angle = Math.atan2(heightDifference, distance);

        double additionalAngle = ArmConstants.linearInterpolationTable.getOutput(distance);

        arm.setArmTarget(Units.degreesToRadians(-25));
        arm.setWristTarget(-angle - additionalAngle);

        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmWristInTargetPose();
  }
}
