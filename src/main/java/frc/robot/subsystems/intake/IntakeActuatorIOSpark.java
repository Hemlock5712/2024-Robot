package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class IntakeActuatorIOSpark implements IntakeActuatorIO {
  CANSparkMax motor;
  PIDController pidController;
  CANcoder intakeEncoder;
  ArmFeedforward feedForward;
  CANcoderConfiguration intakeEncoderConfig;
  IdleMode currentIdleMode = IdleMode.kCoast;

  public IntakeActuatorIOSpark() {
    motor = new CANSparkMax(19, MotorType.kBrushless);
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kCoast);
    motor.setSmartCurrentLimit(30);
    intakeEncoder = new CANcoder(0);
    pidController = new PIDController(0.2, 0, 0);
    pidController.enableContinuousInput(0, 2 * Math.PI);
    // Create a new ArmFeedforward with gains kS, kG, kV, and kA
    feedForward = new ArmFeedforward(0, 0.06, 0, 0);

    intakeEncoderConfig = new CANcoderConfiguration();
    intakeEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    intakeEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
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
    inputs.upLimitSwitchTriggered =
        motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    inputs.downLimitSwitchTriggered =
        motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  @Override
  public void setIntakeAngle(double angleRad) {
    double intakeRad =
        Rotation2d.fromRotations(intakeEncoder.getAbsolutePosition().refresh().getValue())
            .getRadians();
    double pidSpeed =
        pidController.calculate(intakeRad, angleRad)
            + feedForward.calculate(
                Rotation2d.fromRotations(intakeEncoder.getAbsolutePosition().refresh().getValue())
                    .getRadians(),
                angleRad);
    motor.set(pidSpeed);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void resetEncoder() {
    if (Math.abs(
            intakeEncoder.getAbsolutePosition().refresh().getValueAsDouble()
                - Units.degreesToRotations(90))
        > Units.degreesToRotations(3)) {
      intakeEncoderConfig.MagnetSensor.MagnetOffset = 0;
      intakeEncoder.getConfigurator().apply(intakeEncoderConfig);
      intakeEncoderConfig.MagnetSensor.MagnetOffset =
          -intakeEncoder.getAbsolutePosition().refresh().getValueAsDouble() + 0.25;
      intakeEncoder.getConfigurator().apply(intakeEncoderConfig);
    }
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
