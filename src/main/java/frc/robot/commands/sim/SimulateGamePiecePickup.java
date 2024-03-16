// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sim;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.linebreak.LineBreak;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimulateGamePiecePickup extends SequentialCommandGroup {
  /** Creates a new SimulateGamePiecePickup. */
  public SimulateGamePiecePickup(LineBreak lineBreak, Arm arm) {

    // In sim mode, bump the game piece until it's in the shooter
    if (Constants.getMode() == Constants.Mode.SIM) {
      addCommands(
          Commands.deadline(
              Commands.waitUntil(lineBreak::isShooterLoaded),
              Commands.repeatingSequence(
                  new InstantCommand(lineBreak::bumpGamePiece), new WaitCommand(0.2))));
    } else {
      // If not in sim mode, do nothing
      addCommands();
    }
  }
}
