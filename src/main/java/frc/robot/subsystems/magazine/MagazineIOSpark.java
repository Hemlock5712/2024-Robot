package frc.robot.subsystems.magazine;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.CANSparkMax;

public class MagazineIOSpark implements MagazineIO {
  CANSparkMax leader;
  PIDController pidController;
  SimpleMotorFeedforward feedForward;

  public MagazineIOSpark() {
    leader = new CANSparkMax(18, MotorType.kBrushless);
    leader.setSmartCurrentLimit(30);

    pidController = new PIDController(1.0, 0, 0);

    feedForward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.velocityRadPerSec = leader.getEncoder().getVelocity();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
  }

  @Override
  public void setSpeedRadPerSec(double velocityRadPerSec) {
    double pidSpeed =
        pidController.calculate(
                leader.getEncoder().getVelocity(),
                velocityRadPerSec)
            + feedForward.calculate(
                leader.getEncoder().getVelocity(),
                velocityRadPerSec);
    leader.set(pidSpeed);
  }
}
