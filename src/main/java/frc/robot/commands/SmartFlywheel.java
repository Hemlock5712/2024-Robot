// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;

public class SmartFlywheel extends Command {
  Flywheel flywheel;
  Supplier<Pose2d> pose;

  Pose2d target = FieldConstants.Speaker.centerSpeakerOpening;

  /** Creates a new AutoFlywheel. */
  public SmartFlywheel(Flywheel flywheel, Supplier<Pose2d> pose) {
    this.flywheel = flywheel;
    this.pose = pose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = AllianceFlipUtil.apply(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!DriveController.getInstance().isHeadingControlled()) {
      flywheel.setSpeedRPM(0);
    } else {
      flywheel.setSpeedRPM(
          DriveController.getInstance().getTargetAimingParameters().shooterSpeed());
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
