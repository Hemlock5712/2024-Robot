// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ManualClimber extends Command {
  private Climber climber;
  private double targetPosition;
  private int slot;

  /** Creates a new moveArm. */
  public ManualClimber(Climber climber, double targetPosition, int slot) {
    this.climber = climber;
    this.targetPosition = targetPosition;
    this.slot = slot;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setCustomPosition(targetPosition, slot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
