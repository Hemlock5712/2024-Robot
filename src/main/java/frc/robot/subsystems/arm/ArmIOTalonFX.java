package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

public class ArmIOTalonFX implements ArmIO {

  TalonFX armMotor;
  TalonFX wristMotor;

  CANcoder armEncoder;
  CANcoder wristEncoder;
  double armkFF;
  double wristkFF;

  public ArmIOTalonFX() {
    armMotor = new TalonFX(50);
    wristMotor = new TalonFX(51);

    armEncoder = new CANcoder(52);
    wristEncoder = new CANcoder(53);

    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = -.180908;
    armEncoder.getConfigurator().apply(armEncoderConfig);

    CANcoderConfiguration wristEncoderConfig = new CANcoderConfiguration();
    wristEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    wristEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    wristEncoderConfig.MagnetSensor.MagnetOffset = 0.224121;
    wristEncoder.getConfigurator().apply(wristEncoderConfig);

    var armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    armConfig.Feedback.SensorToMechanismRatio = 1;
    armConfig.Feedback.RotorToSensorRatio = ArmConstants.ARM_GEAR_RATIO;

    armMotor.getConfigurator().apply(armConfig);

    var wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristConfig.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristConfig.Feedback.SensorToMechanismRatio = 1;
    wristConfig.Feedback.RotorToSensorRatio = ArmConstants.WRIST_GEAR_RATIO;

    wristMotor.getConfigurator().apply(wristConfig);
  }

  public void updateInputs(ArmIOInputs inputs) {

    inputs.armAbsolutePositionRad =
        armEncoder.getPosition().refresh().getValue() * Math.PI * 2; // Units.rotationsToRadians
    inputs.armRelativePositionRad = armEncoder.getPosition().refresh().getValue() * Math.PI * 2;
    inputs.armVelocityRadPerSec = armEncoder.getVelocity().refresh().getValue();
    inputs.armCurrentAmps = new double[] {armMotor.getSupplyCurrent().refresh().getValue()};
    inputs.armTempCelcius = new double[] {armMotor.getDeviceTemp().refresh().getValue()};
    inputs.wristAbsolutePositionRad =
        (wristEncoder.getPosition().refresh().getValue() * Math.PI * 2)
            - inputs.armRelativePositionRad;
    inputs.wristRelativePositionRad = wristEncoder.getPosition().refresh().getValue() * Math.PI * 2;
    inputs.wristVelocityRadPerSec = wristEncoder.getVelocity().refresh().getValue() * Math.PI * 2;
    inputs.wristCurrentAmps = new double[] {wristMotor.getSupplyCurrent().refresh().getValue()};
    inputs.wristTempCelcius = new double[] {wristMotor.getDeviceTemp().refresh().getValue()};
  }

  public void setArmTarget(double target) {
    var control = new PositionVoltage(0);
    armMotor.setControl(
        control.withPosition(Units.radiansToRotations(target)).withSlot(0).withEnableFOC(true));
  }

  public void setWristTarget(double target) {
    var control = new PositionVoltage(0);
    wristMotor.setControl(
        control.withPosition(Units.radiansToRotations(target)).withSlot(0).withEnableFOC(true));
  }

  public void setBrakeMode(boolean armBrake, boolean wristBrake) {
    armMotor.setNeutralMode(armBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    wristMotor.setNeutralMode(wristBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public void setPIDArm(double kP, double kI, double kD, double kFF) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    config.kG = kFF;
    config.GravityType = GravityTypeValue.Arm_Cosine;
    armMotor.getConfigurator().apply(config);
  }

  public void setPIDWrist(double kP, double kI, double kD, double kFF) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    config.kG = kFF;
    config.GravityType = GravityTypeValue.Arm_Cosine;
    wristMotor.getConfigurator().apply(config);
  }
}
