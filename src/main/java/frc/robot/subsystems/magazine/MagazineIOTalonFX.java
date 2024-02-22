package frc.robot.subsystems.magazine;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class MagazineIOTalonFX implements MagazineIO {
  TalonFX leader = new TalonFX(45);

  public MagazineIOTalonFX() {
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
    inputs.velocityRadPerSec = leader.getVelocity().refresh().getValue() * (Math.PI * 2.0);
    inputs.appliedVolts = leader.getMotorVoltage().getValue();
    inputs.currentAmps = new double[] {leader.getSupplyCurrent().getValue()};
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
