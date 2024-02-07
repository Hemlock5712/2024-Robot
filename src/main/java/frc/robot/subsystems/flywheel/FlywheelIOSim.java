package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {

  TalonFX leader = new TalonFX(40);
  TalonFX follower = new TalonFX(41);
  TalonFXSimState leaderSim = leader.getSimState();
  TalonFXSimState followerSim = follower.getSimState();

  FlywheelSim flywheelSim;

  public FlywheelIOSim() {
    leaderSim = leader.getSimState();
    followerSim = follower.getSimState();

    follower.setControl(new Follower(leader.getDeviceID(), false));

    flywheelSim = new FlywheelSim(DCMotor.getKrakenX60Foc(2), 1.0 / 3.0, 0.01);

    TalonFXConfiguration config = new TalonFXConfiguration();

    var slot0Configs = config.Slot0;
    slot0Configs.kP = 2.2;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    config.Feedback.SensorToMechanismRatio = 1.0 / 3.0;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    flywheelSim.setInput(leaderSim.getMotorVoltage());
    flywheelSim.update(0.02);
    leaderSim.setRotorVelocity(flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2));
    leaderSim.addRotorPosition(0.02 * flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2));

    inputs.velocityRadPerSec = leader.getVelocity().getValue() * (Math.PI * 2.0);
    inputs.appliedVolts = leaderSim.getMotorVoltage();
    inputs.currentAmps =
        new double[] {leaderSim.getSupplyCurrent(), followerSim.getSupplyCurrent()};
  }

  @Override
  public void setSpeedRPM(double speedRPM) {
    double speedRPS = speedRPM / 60.0;
    leader.setControl(
        new VelocityVoltage(0)
            .withVelocity(speedRPS)
            .withEnableFOC(true)
            .withFeedForward(speedRPS * 0.0135));
  }
}
