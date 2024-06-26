// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.climber.Climber;

public class MoveArmForClimbing extends Command {
  private Climber climber;
  private Arm arm;

  /** Creates a new MoveArmForClimbing. */
  public MoveArmForClimbing(Arm arm, Climber climber) {
    this.arm = arm;
    this.climber = climber;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (climber.isCalibrated()) {
      arm.setArmAndWristTarget(
          ArmConstants.trap.arm().getRadians(), ArmConstants.trap.wrist().getRadians());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.getPosition() < 3.5) {
      arm.setArmAndWristTarget(
          (climber.getPosition() / 3.5) * ArmConstants.trap.arm().getRadians(),
          ArmConstants.trap.wrist().getRadians());
    } else {
      arm.setArmAndWristTarget(
          ArmConstants.trap.arm().getRadians(), ArmConstants.trap.wrist().getRadians());
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
