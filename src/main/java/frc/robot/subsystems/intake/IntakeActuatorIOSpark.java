package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeActuatorIOSpark implements IntakeActuatorIO {
  CANSparkMax motor;
  PIDController pidController;
  CANcoder intakeEncoder;
  ArmFeedforward feedForward;

  public IntakeActuatorIOSpark() {
    motor = new CANSparkMax(19, MotorType.kBrushless);
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kCoast);
    motor.setSmartCurrentLimit(30);
    intakeEncoder = new CANcoder(0);
    pidController = new PIDController(0.13, 0, 0);
    pidController.enableContinuousInput(0, 2 * Math.PI);
    // Create a new ArmFeedforward with gains kS, kG, kV, and kA
    feedForward = new ArmFeedforward(0, 0.06, 0, 0);

    CANcoderConfiguration intakeEncoderConfig = new CANcoderConfiguration();
    intakeEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    intakeEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    intakeEncoderConfig.MagnetSensor.MagnetOffset = -0.715 + 0.25;
    // intakeEncoderConfig.MagnetSensor.MagnetOffset = 0;
    for (int i = 0; i < 4; i++) {
      boolean statusOK =
          intakeEncoder.getConfigurator().apply(intakeEncoderConfig) == StatusCode.OK;
      if (statusOK) break;
    }
  }

  @Override
  public void updateInputs(IntakeActuatorIOInputs inputs) {
    inputs.angle = intakeEncoder.getAbsolutePosition().refresh().getValue() * (Math.PI * 2.0);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
    inputs.targetAngle = pidController.getSetpoint();
  }

  @Override
  public void setIntakeAngle(double angleRad) {
    double pidSpeed =
        pidController.calculate(
                Rotation2d.fromRotations(intakeEncoder.getAbsolutePosition().refresh().getValue())
                    .getRadians(),
                angleRad)
            + feedForward.calculate(
                Rotation2d.fromRotations(intakeEncoder.getAbsolutePosition().refresh().getValue())
                    .getRadians(),
                angleRad);
    motor.set(pidSpeed);
  }
}
