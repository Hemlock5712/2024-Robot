package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeWheelIOTalonFX implements IntakeWheelsIO {
  TalonFX leader = new TalonFX(30);

  public IntakeWheelIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leader.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeWheelsIOInputs inputs) {
    inputs.velocityRadPerSec = leader.getVelocity().refresh().getValue() * (Math.PI * 2.0);
    inputs.appliedVolts = leader.getMotorVoltage().refresh().getValue();
    inputs.currentAmps = new double[] {leader.getSupplyCurrent().refresh().getValue()};
  }

  @Override
  public void runVoltage(double voltage) {
    leader.setVoltage(voltage);
  }
}
