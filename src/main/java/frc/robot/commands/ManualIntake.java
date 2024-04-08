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
import frc.robot.subsystems.linebreak.LineBreak;
import java.util.function.DoubleSupplier;

public class ManualIntake extends Command {
  Arm arm;
  Flywheel flywheel;
  ArmPositions armPosition;
  DoubleSupplier flywheelSpeed;
  LineBreak lineBreak;

  /** Creates a new moveArm. */
  public ManualIntake(
      Arm arm,
      Flywheel flywheel,
      ArmPositions armPosition,
      DoubleSupplier flywheelSpeed,
      LineBreak lineBreak) {
    this.arm = arm;
    this.flywheel = flywheel;
    this.armPosition = armPosition;
    this.flywheelSpeed = flywheelSpeed;
    this.lineBreak = lineBreak;
    addRequirements(arm, flywheel);
  }

  @Override
  public void initialize() {
    arm.setArmAndWristTarget(armPosition.arm().getRadians(), armPosition.wrist().getRadians());
    if (flywheelSpeed.getAsDouble() == 0.0) {
      flywheel.stop();
    } else {
      flywheel.setSpeedRotPerSec(flywheelSpeed.getAsDouble());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    arm.setArmAndWristTarget(
        ArmConstants.intake.arm().getRadians(), ArmConstants.intake.arm().getRadians());
    SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lineBreak.isShooterLoaded();
  }
}
