// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.magazine.Magazine;

public class ManualShoot extends Command {
  Arm arm;
  Flywheel flywheel;
  Magazine magazine;
  LineBreak lineBreak;
  Timer timer;
  Timer flywheelTimer;
  double forceShootTimeout;

  /** Creates a new Shoot. */
  public ManualShoot(
      Arm arm,
      Flywheel flywheel,
      Magazine magazine,
      LineBreak lineBreak,
      double forceShootTimeout) {
    this.arm = arm;
    this.flywheel = flywheel;
    this.magazine = magazine;
    this.lineBreak = lineBreak;
    timer = new Timer();
    flywheelTimer = new Timer();
    this.forceShootTimeout = forceShootTimeout;
    addRequirements(magazine, arm, flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartController.getInstance().enableSmartControl();
    // if (Constants.getMode() == Constants.Mode.SIM) timer.restart();
    // flywheel.setSpeedRotPerSec(30);
    flywheel.setSpeedRotPerSec(20);
    arm.setArmAndWristTarget(
        ArmConstants.manualShot.arm().getRadians(), ArmConstants.manualShot.wrist().getRadians());
    flywheelTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((arm.isArmWristInTargetPose() && (flywheel.atTargetSpeed() || flywheelTimer.hasElapsed(2)))
        || flywheelTimer.hasElapsed(forceShootTimeout)) {
      magazine.shoot();
      if (Constants.getMode() == Constants.Mode.SIM && timer.hasElapsed(0.75)) {
        lineBreak.shootGamePiece();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    magazine.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lineBreak.hasNoGamePiece() && lineBreak.timeSinceLastGamePiece() > 0.05;
  }
}
