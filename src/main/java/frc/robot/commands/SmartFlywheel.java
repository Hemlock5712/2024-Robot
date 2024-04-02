// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.linebreak.LineBreak;

public class SmartFlywheel extends Command {
  Flywheel flywheel;
  LineBreak lineBreak;

  /** Creates a new SmartFlywheel. */
  public SmartFlywheel(Flywheel flywheel, LineBreak lineBreak) {
    this.flywheel = flywheel;
    this.lineBreak = lineBreak;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* Not needed */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE) {
      flywheel.stop();
      return;
    }
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.CLIMBER) {
      flywheel.setSpeedRotPerSec(4);
      return;
    }
    if (SmartController.getInstance().isSmartControlEnabled()) {
      flywheel.setSpeedRotPerSec(
          SmartController.getInstance().getTargetAimingParameters().shooterSpeed());
      return;
    } else if ((SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER
            || SmartController.getInstance().getDriveModeType() == DriveModeType.FEED)
        && SmartController.getInstance().getTargetAimingParameters().effectiveDistanceToTarget()
            < SmartController.prerollDistence
        && (lineBreak.isShooterLoaded() || lineBreak.isShooterLong())) {
      flywheel.setSpeedRotPerSec(
          SmartController.getInstance().getTargetAimingParameters().shooterSpeed());
      return;
    } else {
      flywheel.stop();
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* No end to function */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
