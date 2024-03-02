// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SmartController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.lineBreak.LineBreak;
import frc.robot.subsystems.magazine.Magazine;
import java.util.function.Supplier;

public class SmartShoot extends Command {
  Supplier<Pose2d> pose;
  Arm arm;
  Flywheel flywheel;
  Magazine magazine;
  LineBreak lineBreak;
  Timer timer;
  Timer flywheelTimer;
  double forceShootTimeout;

  /** Creates a new Shoot. */
  public SmartShoot(
      Arm arm,
      Flywheel flywheel,
      Magazine magazine,
      LineBreak lineBreak,
      Supplier<Pose2d> pose,
      double forceShootTimeout) {
    this.arm = arm;
    this.flywheel = flywheel;
    this.magazine = magazine;
    this.pose = pose;
    this.lineBreak = lineBreak;
    timer = new Timer();
    flywheelTimer = new Timer();
    this.forceShootTimeout = forceShootTimeout;
    addRequirements(magazine);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartController.getInstance().enableSmartControl();
    if (Constants.getMode() == Constants.Mode.SIM) timer.restart();
    flywheelTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((SmartController.getInstance().isSmartControlEnabled()
            && arm.isArmWristInTargetPose()
            && Math.abs(
                    pose.get()
                        .getRotation()
                        .minus(
                            SmartController.getInstance().getTargetAimingParameters().robotAngle())
                        .getRadians())
                < Units.degreesToRadians(1.5)
            && (flywheel.atTargetSpeed() || flywheelTimer.hasElapsed(2)))
        || flywheelTimer.hasElapsed(forceShootTimeout)) {
      magazine.forward();
      if (Constants.getMode() == Constants.Mode.SIM && timer.hasElapsed(0.75)) {
        lineBreak.shootGamePiece();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartController.getInstance().disableSmartControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lineBreak.hasNoGamePiece() && lineBreak.timeSinceLastGamePiece() > 0.5;
  }
}
