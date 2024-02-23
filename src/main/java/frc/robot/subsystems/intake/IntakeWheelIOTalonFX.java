package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeWheelIOTalonFX implements IntakeWheelsIO {
  TalonFX leader = new TalonFX(30);

  public IntakeWheelIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    var slot0Configs = config.Slot0;
    slot0Configs.kP = 9.2;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    config.Feedback.SensorToMechanismRatio = 3.0;
    leader.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeWheelsIOInputs inputs) {

    inputs.velocityRadPerSec = leader.getVelocity().refresh().getValue() * (Math.PI * 2.0);
    inputs.appliedVolts = leader.getMotorVoltage().refresh().getValue();
    inputs.currentAmps = new double[] {leader.getSupplyCurrent().refresh().getValue()};
  }

  @Override
  public void runRPM(double speedRPM) {
    double speedRPS = speedRPM / 60.0;
    leader.setControl(
        new VelocityVoltage(0)
            .withVelocity(speedRPS)
            .withEnableFOC(true)
            .withFeedForward(speedRPS * 0.0135));
  }

  public void runVoltage(double voltage) {
    leader.setVoltage(voltage);
  }
}
