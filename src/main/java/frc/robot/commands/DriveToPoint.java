// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.SmartController;
import frc.robot.subsystems.drive.SmartController.DriveModeType;
import frc.robot.util.AllianceFlipUtil;

public class DriveToPoint extends Command {
  // THIS IS JUST A DOCUMENT FOR TESTING pathfindToPose. I RECOMMEND YOU USE pathfindThenFollowPath
  // INSTEAD
  private Command scoreCommand;
  private Command pathRun;
  private DriveModeType driveMode;
  private final Pose2d targetPose;

  /** Creates a new ShootPoint. */
  public DriveToPoint(Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    // For shooting you would also want to pass in your shooter subsystem
    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runNewAutonPath();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveModeType currentDriveMode = SmartController.getInstance().getDriveModeType();
    if (driveMode != currentDriveMode) {
      scoreCommand.cancel();
      runNewAutonPath();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    scoreCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This should be the last command in the sequence
    return pathRun.isFinished();
  }

  public void runNewAutonPath() {
    pathRun =
        AutoBuilder.pathfindToPose(
            AllianceFlipUtil.apply(targetPose),
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0,
            0.0);
    scoreCommand = Commands.sequence(pathRun);
    scoreCommand.schedule();
  }
}
