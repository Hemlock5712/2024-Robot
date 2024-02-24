package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;

public class IntakeActuatorIOSpark implements IntakeActuatorIO {
  CANSparkMax motor;
  PIDController pidController;
  CANcoder intakeEncoder;

  public IntakeActuatorIOSpark() {
    motor = new CANSparkMax(19, MotorType.kBrushless);
    intakeEncoder = new CANcoder(0);
    pidController = new PIDController(0, 0, 0);
    pidController.enableContinuousInput(-Math.PI, Math.PI);

    CANcoderConfiguration intakeEncoderConfig = new CANcoderConfiguration();
    intakeEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    intakeEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    for (int i = 0; i < 4; i++) {
      boolean statusOK =
          intakeEncoder.getConfigurator().apply(intakeEncoderConfig) == StatusCode.OK;
      if (statusOK) break;
    }
  }

  @Override
  public void updateInputs(IntakeActuatorIOInputs inputs) {
    inputs.angle = motor.getEncoder().getPosition();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
  }

  @Override
  public void setIntakeAngle(double angleRad) {
    double pidSpeed = pidController.calculate(intakeEncoder.getPosition().getValue(), angleRad);
    motor.set(pidSpeed);
  }
}
