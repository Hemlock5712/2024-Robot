// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePositions;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.magazine.Magazine;
import java.util.function.BooleanSupplier;

public class SmartIntake extends Command {

  Intake intake;
  Magazine magazine;
  Boolean runArmDown;
  BooleanSupplier isArmWristInIntakePosition;
  LineBreak lineBreak;

  /** Creates a new IntakeDown. */
  public SmartIntake(
      Intake intake,
      LineBreak lineBreak,
      Magazine magazine,
      BooleanSupplier isArmWristInIntakePosition) {
    this.intake = intake;
    this.isArmWristInIntakePosition = isArmWristInIntakePosition;
    this.lineBreak = lineBreak;
    this.magazine = magazine;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE) {
      intake.setIntakeMode(IntakePositions.UP);
      intake.stop();
      return;
    }
    if (magazine.isShooting()) {
      intake.intakeSlow();
      return;
    }
    if (intake.getIntakeRequest() && lineBreak.hasNoGamePiece()) {
      intake.setIntakeMode(IntakePositions.FLOOR);
      intake.intake();
    } else {
      intake.setIntakeMode(IntakePositions.UP);
      if (!(lineBreak.isShooterLoaded() || lineBreak.isShooterLong()) && lineBreak.hasGamePiece()) {
        intake.intake();
      } else {
        intake.stop();
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
