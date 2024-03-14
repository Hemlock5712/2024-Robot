// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class CalibrateClimber extends Command {

  private Climber climber;

  /** Creates a new CalibrateClimber. */
  public CalibrateClimber(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!climber.isLimitSwitchTriggered()) {
      climber.voltageControl(-2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.resetClimberPositionToZero();
    climber.voltageControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isLimitSwitchTriggered();
  }
}
