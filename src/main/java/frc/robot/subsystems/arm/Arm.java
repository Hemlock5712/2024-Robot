package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  ArmVisualizer visualizerMeasured;
  ArmVisualizer visualizerSetpoint;

  private double armTarget = getArmAngleRelative();
  private double wristTarget = getWristAngleRelative();

  public Arm(ArmIO io) {
    this.io = io;
    visualizerMeasured = new ArmVisualizer("ArmMeasured", null);
    visualizerSetpoint = new ArmVisualizer("ArmSetpoint", new Color8Bit(Color.kOrange));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double realWristTarget = getRelativeWristTarget();
    double realArmTarget = getRelativeArmTarget();
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/ArmTargetPositionRad", realArmTarget);
    Logger.recordOutput("Arm/WristTargetPositionRad", realWristTarget);
    io.setArmTarget(realArmTarget);
    io.setWristTarget(realWristTarget);
    // This method will be called once per scheduler run
    visualizerMeasured.update(inputs.armRelativePositionRad, inputs.wristRelativePositionRad);
    visualizerSetpoint.update(realArmTarget, realWristTarget);
  }

  public void setArmTarget(double target) {
    armTarget = target;
  }

  public void setWristTarget(double target) {
    wristTarget = target;
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
    return (Math.abs(armTarget - getArmAngleRelative()) < (Units.degreesToRadians(1.5)))
        && (Math.abs(getRelativeWristTarget() - getWristAngleRelative())
            < (Units.degreesToRadians(1)));
  }
}
