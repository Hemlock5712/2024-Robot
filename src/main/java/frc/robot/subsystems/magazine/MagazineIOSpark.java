package frc.robot.subsystems.magazine;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

public class MagazineIOSpark implements MagazineIO {
  CANSparkMax leader = new CANSparkMax(18, MotorType.kBrushless);

  public MagazineIOSpark() {
    leader.getPIDController().setP(0);
    leader.getPIDController().setI(0);
    leader.getPIDController().setD(0);
    leader.getPIDController().setFF(0);
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.velocityRadPerSec = leader.getEncoder().getVelocity();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
  }

  @Override
  public void runRPM(double speedRPM) {
    leader
        .getPIDController()
        .setReference(speedRPM, ControlType.kVelocity, 0, 0.0, ArbFFUnits.kVoltage);
  }

  public void runVoltage(double voltage) {
    leader.setVoltage(voltage);
  }
}
