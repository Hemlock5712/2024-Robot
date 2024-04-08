// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sim;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.linebreak.LineBreak;

public class SimulatePreload extends Command {
  LineBreak lineBreak;

  /** Creates a new SimulatePreload. */
  public SimulatePreload(LineBreak lineBreak) {
    this.lineBreak = lineBreak;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.getMode() == Constants.Mode.SIM) {
      lineBreak.setGamePiece(false, false, false, false, true, false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
