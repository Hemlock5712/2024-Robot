// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SmartController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.magazine.Magazine;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
    boolean isSmartControlEnabled = SmartController.getInstance().isSmartControlEnabled();
    boolean isWristInTargetPose = isWristInTargetPose();
    boolean isDriveAngleInTarget = isDriveAngleInTarget();
    boolean isFlywheelAtTargetSpeed = isFlywheelAtTargetSpeed();
    boolean isShooting = false;
    if ((isSmartControlEnabled
            && isWristInTargetPose
            && isDriveAngleInTarget
            && isFlywheelAtTargetSpeed)
        || flywheelTimer.hasElapsed(forceShootTimeout)) {
      magazine.forward();
      isShooting = true;
      if (Constants.getMode() == Constants.Mode.SIM && timer.hasElapsed(0.75)) {
        lineBreak.shootGamePiece();
      }
    }
    Logger.recordOutput("SmartShoot/IsShooting", isShooting);
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

  public boolean isWristInTargetPose() {
    Logger.recordOutput("SmartShoot/IsWristInTargetPose", arm.isArmWristInTargetPose());
    Logger.recordOutput("SmartShoot/WristAngle", arm.getWristAngleRelative());
    Logger.recordOutput("SmartShoot/TargetWristAngle", arm.getRelativeWristTarget());
    return arm.isArmWristInTargetPose();
  }

  public boolean isFlywheelAtTargetSpeed() {
    Logger.recordOutput("SmartShoot/IsFlywheelAtTargetSpeed", flywheel.atTargetSpeed());
    Logger.recordOutput("SmartShoot/FlywheelSpeed", flywheel.getVelocityRotPerSec());
    Logger.recordOutput(
        "SmartShoot/TargetFlywheelSpeed",
        SmartController.getInstance().getTargetAimingParameters().shooterSpeed());
    return flywheel.atTargetSpeed();
  }

  public boolean isDriveAngleInTarget() {
    Rotation2d targetAngle = SmartController.getInstance().getTargetAimingParameters().robotAngle();
    Rotation2d robotAngle = this.pose.get().getRotation();
    boolean isDriveAngleInTarget =
        Math.abs(robotAngle.minus(targetAngle).getRadians()) < Units.degreesToRadians(1.5);
    Logger.recordOutput("SmartShoot/IsDriveAngleInTarget", isDriveAngleInTarget);
    Logger.recordOutput("SmartShoot/DriveAngle", robotAngle.getDegrees());
    Logger.recordOutput("SmartShoot/TargetDriveAngle", targetAngle.getDegrees());
    return isDriveAngleInTarget;
  }
}
