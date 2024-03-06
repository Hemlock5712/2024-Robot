// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmPositions;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.lineBreak.LineBreak;

public class ManualArm extends Command {
  Arm arm;
  Flywheel flywheel;
  ArmPositions armPosition;

  /** Creates a new moveArm. */
  public ManualArm(Arm arm, Flywheel flywheel, ArmPositions armPosition) {
    this.arm = arm;
    this.flywheel = flywheel;
    this.armPosition = armPosition;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setArmAndWristTarget(armPosition.arm().getRadians(), armPosition.wrist().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
