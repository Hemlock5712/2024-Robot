package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  ArmVisualizer visualizerMeasured;
  ArmVisualizer visualizerSetpoint;

  private static final LoggedTunableNumber armkP =
      new LoggedTunableNumber("Arm/kP", ArmConstants.armControlConstants.kP());
  private static final LoggedTunableNumber armkI =
      new LoggedTunableNumber("Arm/kI", ArmConstants.armControlConstants.kI());
  private static final LoggedTunableNumber armkD =
      new LoggedTunableNumber("Arm/kD", ArmConstants.armControlConstants.kD());
  private static final LoggedTunableNumber armkG =
      new LoggedTunableNumber("Arm/kG", ArmConstants.armControlConstants.kG());

  private static final LoggedTunableNumber wristkP =
      new LoggedTunableNumber("Wrist/kP", ArmConstants.wristControlConstants.kP());
  private static final LoggedTunableNumber wristkI =
      new LoggedTunableNumber("Wrist/kI", ArmConstants.wristControlConstants.kI());
  private static final LoggedTunableNumber wristkD =
      new LoggedTunableNumber("Wrist/kD", ArmConstants.wristControlConstants.kD());
  private static final LoggedTunableNumber wristkG =
      new LoggedTunableNumber("Wrist/kG", ArmConstants.wristControlConstants.kG());

  private double armTarget = 0;
  private double wristTarget = 0;

  public Arm(ArmIO io) {
    this.io = io;
    io.setPIDArm(armkP.get(), armkI.get(), armkD.get(), armkG.get());
    io.setPIDWrist(wristkP.get(), wristkI.get(), wristkD.get(), wristkG.get());

    visualizerMeasured = new ArmVisualizer("ArmMeasured", null);
    visualizerSetpoint = new ArmVisualizer("ArmSetpoint", new Color8Bit(Color.kOrange));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double realWristTarget = getRelativeWristTarget();
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/ArmTargetPositionRad", armTarget);
    Logger.recordOutput("Arm/WristTargetPositionRad", realWristTarget);
    io.setArmTarget(armTarget);
    io.setWristTarget(realWristTarget);
    // This method will be called once per scheduler run
    visualizerMeasured.update(inputs.armRelativePositionRad, inputs.wristRelativePositionRad);
    visualizerSetpoint.update(armTarget, realWristTarget);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPIDArm(armkP.get(), armkI.get(), armkD.get(), armkG.get()),
        armkP,
        armkI,
        armkD,
        armkG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPIDWrist(wristkP.get(), wristkI.get(), wristkD.get(), wristkG.get()),
        wristkP,
        wristkI,
        wristkD,
        wristkG);
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

  public double getArmAngle() {
    return inputs.armRelativePositionRad;
  }

  public double getRelativeWristTarget() {
    return wristTarget;
  }

  public Transform3d getFlywheelPosition() {
    return new Transform3d(
        new Pose3d() {},
        visualizerMeasured.getWristPose(
            inputs.armRelativePositionRad, inputs.wristRelativePositionRad));
  }

  @AutoLogOutput(key = "Arm/isArmWristInIntakePosition")
  public boolean isArmWristInIntakePosition() {
    return (Math.abs(ArmConstants.intake.arm().getRadians() - getArmAngle())
            < (Units.degreesToRadians(1)))
        && (Math.abs(ArmConstants.intake.wrist().getRadians() - getWristAngleAbsolute())
            < (Units.degreesToRadians(1)));
  }

  @AutoLogOutput(key = "Arm/isArmWristInTargetPose")
  public boolean isArmWristInTargetPose() {
    return (Math.abs(armTarget - getArmAngle()) < (Units.degreesToRadians(1)))
        && (Math.abs(getRelativeWristTarget() - getWristAngleRelative())
            < (Units.degreesToRadians(1)));
  }
}
