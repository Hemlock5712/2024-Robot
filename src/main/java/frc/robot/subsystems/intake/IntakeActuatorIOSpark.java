package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class IntakeActuatorIOSpark implements IntakeActuatorIO {
  CANSparkMax motor;
  PIDController pidController;
  ArmFeedforward feedForward;
  IdleMode currentIdleMode = IdleMode.kCoast;

  public IntakeActuatorIOSpark() {
    motor = new CANSparkMax(19, MotorType.kBrushless);
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kCoast);
    motor.setSmartCurrentLimit(30);
    pidController = new PIDController(0.2, 0, 0);
    pidController.enableContinuousInput(0, 2 * Math.PI);
    // Create a new ArmFeedforward with gains kS, kG, kV, and kA
    feedForward = new ArmFeedforward(0, 0.06, 0, 0);
  }

  @Override
  public void updateInputs(IntakeActuatorIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
    inputs.targetAngle = pidController.getSetpoint();
    inputs.upLimitSwitchTriggered =
        motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    inputs.downLimitSwitchTriggered =
        motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void brakeMode() {
    if (currentIdleMode != IdleMode.kBrake) {
      motor.setIdleMode(IdleMode.kBrake);
      currentIdleMode = IdleMode.kBrake;
    }
  }

  @Override
  public void coastMode() {
    if (currentIdleMode != IdleMode.kCoast) {
      motor.setIdleMode(IdleMode.kCoast);
      currentIdleMode = IdleMode.kCoast;
    }
  }
}
