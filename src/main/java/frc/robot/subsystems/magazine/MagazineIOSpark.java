package frc.robot.subsystems.magazine;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class MagazineIOSpark implements MagazineIO {
  CANSparkMax leader;

  public MagazineIOSpark() {
    leader = new CANSparkMax(18, MotorType.kBrushless);
    leader.setSmartCurrentLimit(30);
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.velocityRadPerSec = leader.getEncoder().getVelocity();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
  }

  @Override
  public void runVoltage(double voltage) {
    leader.setVoltage(voltage);
  }
}
