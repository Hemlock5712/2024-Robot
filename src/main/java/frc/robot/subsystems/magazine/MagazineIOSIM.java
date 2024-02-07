package frc.robot.subsystems.magazine;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class MagazineIOSIM implements MagazineIO {
  TalonFX leader = new TalonFX(45);
  TalonFXSimState leaderSim = leader.getSimState();
  FlywheelSim flywheelSim;

  public MagazineIOSIM() {
    leaderSim = leader.getSimState();
    flywheelSim = new FlywheelSim(DCMotor.getNeo550(1), 3.0, 0.01);
    TalonFXConfiguration config = new TalonFXConfiguration();
    var slot0Configs = config.Slot0;
    slot0Configs.kP = 9.2;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    config.Feedback.SensorToMechanismRatio = 3.0;
    leader.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    flywheelSim.setInput(leaderSim.getMotorVoltage());
    flywheelSim.update(0.02);
    leaderSim.setRotorVelocity(flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2));
    leaderSim.addRotorPosition(0.02 * flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2));

    inputs.velocityRadPerSec = leader.getVelocity().getValue() * (Math.PI * 2.0);
    inputs.appliedVolts = leaderSim.getMotorVoltage();
    inputs.currentAmps = new double[] {leaderSim.getSupplyCurrent()};
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
}
