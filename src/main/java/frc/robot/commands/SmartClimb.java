// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.climber.Climber;

public class SmartClimb extends Command {
  Climber climber;

  /** Creates a new SmartClimb. */
  public SmartClimb(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.isCalibrated()) {
      if ((SmartController.getInstance().getDriveModeType() == DriveModeType.CLIMBER)
          && !climber.isRequestingClimb()) {
        climber.setCustomPosition(5.2, 0);
      } else if ((SmartController.getInstance().getDriveModeType() == DriveModeType.CLIMBER)
          && climber.isRequestingClimb()) {
        climber.setCustomPosition(0.0, 1);

      } else if ((SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE)) {
        return;
      } else {
        climber.setCustomPosition(0, 0);
      }
    } else {
      if (!climber.isLimitSwitchTriggered()) {
        climber.voltageControl(-2);
      } else {
        climber.resetClimberPositionToZero();
        climber.voltageControl(0);
      }
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
