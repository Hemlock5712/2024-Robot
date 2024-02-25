// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.lineBreak.LineBreak;
import frc.robot.subsystems.magazine.Magazine;

public class SmartMagazine extends Command {
  Magazine magazine;
  LineBreak lineBreak;
  boolean upperInt2Sensor = false;
  double multiplier = 1.0;

  /** Creates a new SmartMagazine. */
  public SmartMagazine(Magazine magazine, LineBreak lineBreak) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magazine = magazine;
    this.lineBreak = lineBreak;
    addRequirements(magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE) {
      magazine.stop();
      return;
    }

    if (lineBreak.hasNoGamePiece()) {
      multiplier = 1.0;
      upperInt2Sensor = false;
    }

    if (lineBreak.isupperIntake2Sensor()) {
      upperInt2Sensor = true;
    }

    if (upperInt2Sensor && !lineBreak.isupperIntake2Sensor()) {
      if (lineBreak.isShooterLong()) {
        magazine.slowBackward(multiplier);
        multiplier *= 1.001;
      } else {
        magazine.stop();
      }
      return;
    }

    if (lineBreak.hasGamePiece() && !(lineBreak.isShooterLong() || lineBreak.isShooterLoaded())) {
      magazine.forward();
    } else {
      if (lineBreak.isShooterLong()) {
        magazine.slowBackward(multiplier);
        multiplier *= 1.001;
      } else {
        magazine.stop();
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
