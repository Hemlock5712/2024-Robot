package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SmartController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  ArmVisualizer visualizerMeasured;
  ArmVisualizer visualizerSetpoint;

  private double armTarget = ArmConstants.intake.arm().getRadians();
  private double wristTarget = ArmConstants.intake.wrist().getRadians();

  public Arm(ArmIO io) {
    this.io = io;
    visualizerMeasured = new ArmVisualizer("ArmMeasured", null);
    visualizerSetpoint = new ArmVisualizer("ArmSetpoint", new Color8Bit(Color.kOrange));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    visualizerMeasured.update(inputs.armRelativePositionRad, inputs.wristRelativePositionRad);
  }

  public void setArmTarget(double armTarget) {
    this.armTarget = armTarget;
    io.setArmTarget(armTarget);
    Logger.recordOutput("Arm/ArmTargetPositionRad", armTarget);
    visualizerSetpoint.update(this.armTarget, this.wristTarget);
  }

  public void setWristTarget(double wristTarget) {
    this.wristTarget = wristTarget;
    io.setWristTarget(wristTarget, inputs.wristAbsolutePositionRad);
    Logger.recordOutput("Arm/WristTargetPositionRad", wristTarget);
    visualizerSetpoint.update(this.armTarget, this.wristTarget);
  }

  public void setArmAndWristTarget(double armTarget, double wristTarget) {
    this.wristTarget = wristTarget;
    this.armTarget = armTarget;

    io.setWristTarget(wristTarget, inputs.wristAbsolutePositionRad);
    Logger.recordOutput("Arm/WristTargetPositionRad", wristTarget);

    io.setArmTarget(armTarget);
    Logger.recordOutput("Arm/ArmTargetPositionRad", armTarget);

    visualizerSetpoint.update(this.armTarget, this.wristTarget);
  }

  public void setArmAndWristTargetReversed(double armTarget, double wristTarget) {
    this.armTarget = armTarget;

    double angleOffVertical = (Math.PI / 2) - wristTarget - armTarget;
    this.wristTarget = wristTarget + (2 * angleOffVertical);

    io.setWristTarget(this.wristTarget, inputs.wristAbsolutePositionRad);
    Logger.recordOutput("Arm/WristTargetPositionRad", this.wristTarget);

    io.setArmTarget(armTarget);
    Logger.recordOutput("Arm/ArmTargetPositionRad", this.armTarget);

    visualizerSetpoint.update(this.armTarget, this.wristTarget);
  }

  public void stop() {
    io.stop();
  }

  public double getWristAngleRelative() {
    return inputs.wristRelativePositionRad;
  }

  public double getWristAngleAbsolute() {
    return inputs.wristAbsolutePositionRad;
  }

  public double getArmAngleAbsolute() {
    return inputs.armAbsolutePositionRad;
  }

  public double getArmAngleRelative() {
    return inputs.armRelativePositionRad;
  }

  public double getRelativeWristTarget() {
    return wristTarget;
  }

  public double getRelativeArmTarget() {
    return armTarget;
  }

  public Transform3d getFlywheelPosition() {
    return new Transform3d(
        new Pose3d() {},
        visualizerMeasured.getWristPose(
            inputs.armRelativePositionRad, inputs.wristRelativePositionRad));
  }

  @AutoLogOutput(key = "Arm/isArmWristInIntakePosition")
  public boolean isArmWristInIntakePosition() {
    return (Math.abs(ArmConstants.intake.arm().getRadians() - getArmAngleRelative())
            < (Units.degreesToRadians(1)))
        && (Math.abs(ArmConstants.intake.wrist().getRadians() - getWristAngleRelative())
            < (Units.degreesToRadians(1)));
  }

  @AutoLogOutput(key = "Arm/isArmWristInTargetPose")
  public boolean isArmWristInTargetPose() {
    return (Math.abs(armTarget - getArmAngleRelative()) < (Units.degreesToRadians(3)))
        && (Math.abs(getRelativeWristTarget() - getWristAngleRelative())
            < (Units.degreesToRadians(
                SmartController.getInstance().getTargetAimingParameters().wristError())));
  }
}
