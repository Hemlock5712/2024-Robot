package frc.robot.subsystems.magazine;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class MagazineIOSpark implements MagazineIO {
  CANSparkMax leader = new CANSparkMax(1, MotorType.kBrushless);

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
    leader.getPIDController().setReference(
      speedRPM,
      ControlType.kVelocity,
      0,
      0.0,
      ArbFFUnits.kVoltage);
  }
}
