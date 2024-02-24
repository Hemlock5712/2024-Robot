package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeActuatorIOSpark implements IntakeActuatorIO {
  CANSparkMax leader = new CANSparkMax(19, MotorType.kBrushless);

  public IntakeActuatorIOSpark() {
    leader.getPIDController().setP(0);
    leader.getPIDController().setI(0);
    leader.getPIDController().setD(0);
    leader.getPIDController().setFF(0);

    resetAngleToUp();
  }

  public void resetAngleToUp() {}

  @Override
  public void updateInputs(IntakeActuatorIOInputs inputs) {
    inputs.angle = leader.getEncoder().getPosition();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
    inputs.tempCelcius = new double[] {leader.getMotorTemperature()};
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double kFF) {
    leader.getPIDController().setP(kP);
    leader.getPIDController().setI(kI);
    leader.getPIDController().setD(kD);
    leader.getPIDController().setFF(kFF);
  }

  @Override
  public void setIntakeAngle(double angleRad) {
    leader.getPIDController().setReference(angleRad, ControlType.kPosition);
  }
}
