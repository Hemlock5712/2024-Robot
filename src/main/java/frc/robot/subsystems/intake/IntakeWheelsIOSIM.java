package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeWheelsIOSIM implements IntakeWheelsIO {
  TalonFX leader = new TalonFX(45);
  TalonFXSimState leaderSim = leader.getSimState();
  FlywheelSim flywheelSim;

  public IntakeWheelsIOSIM() {
    leaderSim = leader.getSimState();
    flywheelSim = new FlywheelSim(DCMotor.getNeo550(1), 3.0, 0.01);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 3.0;
    leader.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeWheelsIOInputs inputs) {
    flywheelSim.setInput(leaderSim.getMotorVoltage());
    flywheelSim.update(0.02);
    leaderSim.setRotorVelocity(flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2));
    leaderSim.addRotorPosition(0.02 * flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2));

    inputs.velocityRadPerSec = leader.getVelocity().refresh().getValue() * (Math.PI * 2.0);
    inputs.appliedVolts = leaderSim.getMotorVoltage();
    inputs.currentAmps = new double[] {leaderSim.getSupplyCurrent()};
  }

  @Override
  public void runVoltage(double voltage) {
    leader.setVoltage(voltage);
  }
}
