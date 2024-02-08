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
import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;
import java.util.function.Supplier;

public class AutoFlywheel extends Command {
  Flywheel flywheel;
  Supplier<Pose2d> pose;
  DriveController driveController;

  LinearInterpolationTable table =
      new LinearInterpolationTable(
          new Point2D.Double(0, 8000),
          new Point2D.Double(5, 12000),
          new Point2D.Double(8, 16000),
          new Point2D.Double(10, 18000));

  Pose2d target = FieldConstants.Speaker.centerSpeakerOpening;

  /** Creates a new AutoFlywheel. */
  public AutoFlywheel(Flywheel flywheel, DriveController driveController, Supplier<Pose2d> pose) {
    this.flywheel = flywheel;
    this.pose = pose;
    this.driveController = driveController;
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

    switch (driveController.getDriveModeType().get()) {
      case AMP:
        flywheel.setSpeedRPM(3000);
        break;
      case SPEAKER:
        double distance = pose.get().getTranslation().getDistance(target.getTranslation());

        flywheel.setSpeedRPM(table.getOutput(distance));
        break;
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
