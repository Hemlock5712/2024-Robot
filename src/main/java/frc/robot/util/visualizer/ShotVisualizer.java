// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.visualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;

public class ShotVisualizer extends Command {

  Drive drive;
  Arm arm;
  Flywheel flywheel;

  /** Creates a new ShotVisualizer. */
  public ShotVisualizer(Drive drive, Arm arm, Flywheel flywheel) {
    this.drive = drive;
    this.arm = arm;
    this.flywheel = flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPosition = drive.getPose();

    Pose3d robotPosition3d =
        new Pose3d(
            robotPosition.getTranslation().getX(),
            robotPosition.getTranslation().getY(),
            0.5,
            new Rotation3d(0, 0, robotPosition.getRotation().getRadians()));

    NoteVisualizer.shootNote(
        robotPosition3d, -arm.getWristAngleAbsolute(), flywheel.getVelocityMetersPerSec());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NoteVisualizer.updateNotePosition(0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !NoteVisualizer.isTrajectoryActive();
  }
}
