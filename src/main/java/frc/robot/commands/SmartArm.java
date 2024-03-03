// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.lineBreak.LineBreak;

public class SmartArm extends Command {
  Arm arm;
  LineBreak lineBreak;

  /** Creates a new moveArm. */
  public SmartArm(Arm arm, LineBreak lineBreak) {
    this.arm = arm;
    this.lineBreak = lineBreak;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setArmTarget(arm.getArmAngleRelative());
    arm.setWristTarget(arm.getWristAngleRelative());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE) {
      return;
    }
    if (SmartController.getInstance().isSmartControlEnabled()
        && SmartController.getInstance().getDriveModeType() == DriveModeType.AMP) {
      arm.setArmTarget(ArmConstants.frontAmp.arm().getRadians());
      arm.setWristTarget(ArmConstants.frontAmp.wrist().getRadians());
      return;
    }
    if (lineBreak.isShooterLoaded() || lineBreak.isShooterLong()) {
      // Is in Speaker mode and
      // Is within distance and loaded
      if (SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER
          && (SmartController.getInstance().isSmartControlEnabled()
              || (SmartController.getInstance()
                          .getTargetAimingParameters()
                          .effectiveDistanceToSpeaker()
                      < SmartController.getInstance().getMaxDistance()
                  && lineBreak.isShooterLoaded()))) {
        arm.setArmTarget(ArmConstants.shoot.arm().getRadians());
        arm.setWristTarget(
            SmartController.getInstance().getTargetAimingParameters().shooterAngle().getRadians());
      }
    } else {
      arm.setArmTarget(ArmConstants.intake.arm().getRadians());
      arm.setWristTarget(ArmConstants.intake.wrist().getRadians());
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
