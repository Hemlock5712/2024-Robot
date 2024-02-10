// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePositions;
import frc.robot.subsystems.lineBreak.LineBreak;
import java.util.function.BooleanSupplier;

public class SmartIntake extends Command {

  Intake intake;
  Boolean runArmDown;
  BooleanSupplier isArmWristInIntakePosition;
  LineBreak lineBreak;

  /** Creates a new IntakeDown. */
  public SmartIntake(
      Intake intake, LineBreak lineBreak, BooleanSupplier isArmWristInIntakePosition) {
    this.intake = intake;
    this.isArmWristInIntakePosition = isArmWristInIntakePosition;
    this.lineBreak = lineBreak;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getDriverRequestIntakeDown() && !lineBreak.hasGamePiece()) {
      intake.setIntakeMode(IntakePositions.FLOOR);
      intake.setSpeed(IntakeConstants.intakeSpeed);
    } else if (intake.getDriverRequestIntakeDown()
        && lineBreak.hasGamePiece()
        && lineBreak.inLowerIntake()) {
      intake.setIntakeMode(IntakePositions.BUMPER);
      intake.setSpeed(IntakeConstants.intakeSpeed);
    } else {
      intake.setIntakeMode(IntakePositions.UP);
      if (isArmWristInIntakePosition.getAsBoolean()
          && !(lineBreak.isShooterLoaded() || lineBreak.isShooterLong())
          && lineBreak.hasGamePiece()) {
        intake.setSpeed(1000);
      } else {
        intake.setSpeed(0);
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