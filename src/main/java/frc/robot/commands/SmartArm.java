// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.lineBreak.LineBreak;

public class SmartArm extends Command {
  Arm arm;
  LineBreak lineBreak;
  Climber climber;

  /** Creates a new moveArm. */
  public SmartArm(Arm arm, LineBreak lineBreak, Climber climber) {
    this.arm = arm;
    this.lineBreak = lineBreak;
    this.climber = climber;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setArmAndWristTarget(arm.getArmAngleRelative(), arm.getWristAngleRelative());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE) {
      arm.stop();
      return;
    }
    if (SmartController.getInstance().isSmartControlEnabled()
        && SmartController.getInstance().getDriveModeType() == DriveModeType.AMP) {
      arm.setArmAndWristTarget(
          ArmConstants.frontAmp.arm().getRadians(), ArmConstants.frontAmp.wrist().getRadians());
      return;
    }
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.CLIMBER) {
      if (climber.getPosition() < 3.5) {
        arm.setArmAndWristTarget(
            (climber.getPosition() / 3.5) * ArmConstants.trap.arm().getRadians(),
            ArmConstants.trap.wrist().getRadians());
      } else {
        arm.setArmAndWristTarget(
            ArmConstants.trap.arm().getRadians(), ArmConstants.trap.wrist().getRadians());
      }
      return;
    }
    if (lineBreak.isShooterLoaded() || lineBreak.isShooterLong()) {
      if (SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER
          && SmartController.getInstance().isSmartControlEnabled()) {
        arm.setArmAndWristTarget(
            ArmConstants.shoot.arm().getRadians(),
            SmartController.getInstance().getTargetAimingParameters().shooterAngle().getRadians());
      } else {
        arm.setArmAndWristTarget(
            ArmConstants.intake.arm().getRadians(), ArmConstants.intake.wrist().getRadians());
      }
    } else {
      arm.setArmAndWristTarget(
          ArmConstants.intake.arm().getRadians(), ArmConstants.intake.wrist().getRadians());
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
