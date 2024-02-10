// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.drive.DriveController.DriveModeType;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.lineBreak.LineBreak;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;

public class AutoFlywheel extends Command {
  Flywheel flywheel;
  LineBreak lineBreak;
  Supplier<Pose2d> pose;

  Pose2d target = FieldConstants.Speaker.centerSpeakerOpening;

  /** Creates a new AutoFlywheel. */
  public AutoFlywheel(Flywheel flywheel, LineBreak lineBreak, Supplier<Pose2d> pose) {
    this.flywheel = flywheel;
    this.lineBreak = lineBreak;
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
    if (!(lineBreak.isShooterLoaded() || lineBreak.isShooterLong())) {
      flywheel.setSpeedRPM(0);
    } else if (DriveController.getInstance().getDriveModeType().get() == DriveModeType.AMP) {
      flywheel.setSpeedRPM(3000);
    } else {
      double distance = pose.get().getTranslation().getDistance(target.getTranslation());
      flywheel.setSpeedRPM(DriveController.getInstance().getShooterSpeed(distance));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flywheel.atTargetSpeed();
  }
}
