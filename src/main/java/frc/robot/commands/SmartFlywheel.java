// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.BooleanSupplier;

public class SmartFlywheel extends Command {
  Flywheel flywheel;
  BooleanSupplier isLoaded;

  /** Creates a new SmartFlywheel. */
  public SmartFlywheel(Flywheel flywheel, BooleanSupplier isLoaded) {
    this.flywheel = flywheel;
    this.isLoaded = isLoaded;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE) {
      // 40.5 SHOOT SPEEED
      flywheel.stop();
      return;
    }
    // Is Smart Control Enabled or 
    // In Speaker mode and shooter is loaded and within distance
    if (SmartController.getInstance().isSmartControlEnabled()
        || (SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER
            && (SmartController.getInstance()
                        .getTargetAimingParameters()
                        .effectiveDistanceToSpeaker()
                    < SmartController.getInstance().getMaxDistance()
                && isLoaded.getAsBoolean()))) {
      flywheel.setSpeedRotPerSec(
          SmartController.getInstance().getTargetAimingParameters().shooterSpeed());

    } else {
      flywheel.stop();
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
