package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

  CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
  CANcoderConfiguration wristEncoderConfig = new CANcoderConfiguration();

  TalonFXConfiguration armConfig = new TalonFXConfiguration();
  TalonFXConfiguration wristConfig = new TalonFXConfiguration();

  private final StatusSignal<Double> armAbsolutePosition;
  private final StatusSignal<Double> armSpeed;
  private final StatusSignal<Double> armSupplyCurrent;
  private final StatusSignal<Double> armTemp;

  private final StatusSignal<Double> wristAbsolutePosition;
  private final StatusSignal<Double> wristSpeed;
  private final StatusSignal<Double> wristSupplyCurrent;
  private final StatusSignal<Double> wristTemp;

  public ArmIOTalonFX() {
    armMotor = new TalonFX(50);
    wristMotor = new TalonFX(51);

    armEncoder = new CANcoder(52);
    wristEncoder = new CANcoder(53);

    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = -.180908;
    armEncoder.getConfigurator().apply(armEncoderConfig);

    wristEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    wristEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    wristEncoderConfig.MagnetSensor.MagnetOffset = 0.224121;
    wristEncoder.getConfigurator().apply(wristEncoderConfig);

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    armConfig.Feedback.SensorToMechanismRatio = 1;
    armConfig.Feedback.RotorToSensorRatio = ArmConstants.ARM_GEAR_RATIO;

    armMotor.getConfigurator().apply(armConfig);

    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristConfig.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristConfig.Feedback.SensorToMechanismRatio = 1;
    wristConfig.Feedback.RotorToSensorRatio = ArmConstants.WRIST_GEAR_RATIO;

    wristMotor.getConfigurator().apply(wristConfig);

    armAbsolutePosition = armEncoder.getAbsolutePosition();
    armSpeed = armEncoder.getVelocity();
    armSupplyCurrent = armMotor.getSupplyCurrent();
    armTemp = armMotor.getDeviceTemp();

    wristAbsolutePosition = wristEncoder.getAbsolutePosition();
    wristSpeed = wristEncoder.getVelocity();
    wristSupplyCurrent = wristMotor.getSupplyCurrent();
    wristTemp = wristMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        armAbsolutePosition,
        armSpeed,
        armSupplyCurrent,
        armTemp,
        wristAbsolutePosition,
        wristSpeed,
        wristSupplyCurrent,
        wristTemp);

    armMotor.optimizeBusUtilization();
    wristMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armAbsolutePosition,
        armSpeed,
        armSupplyCurrent,
        armTemp,
        wristAbsolutePosition,
        wristSpeed,
        wristSupplyCurrent,
        wristTemp);

    inputs.armAbsolutePositionRad =
        armAbsolutePosition.getValue() * Math.PI * 2; // Units.rotationsToRadians
    inputs.armRelativePositionRad = armAbsolutePosition.getValue() * Math.PI * 2;
    inputs.armVelocityRadPerSec = armSpeed.getValue() * Math.PI * 2;
    inputs.armCurrentAmps = new double[] {armSupplyCurrent.getValue()};
    inputs.armTempCelcius = new double[] {armTemp.getValue()};

    inputs.wristAbsolutePositionRad = wristAbsolutePosition.getValue() * Math.PI * 2;
    inputs.wristRelativePositionRad =
        inputs.wristAbsolutePositionRad - inputs.armRelativePositionRad;
    inputs.wristVelocityRadPerSec = wristSpeed.getValue() * Math.PI * 2;
    inputs.wristCurrentAmps = new double[] {wristSupplyCurrent.getValue()};
    inputs.wristTempCelcius = new double[] {wristTemp.refresh().getValue()};
  }

  @Override
  public void setArmTarget(double target) {
    var control = new PositionVoltage(0);
    armMotor.setControl(
        control.withPosition(Units.radiansToRotations(target)).withSlot(0).withEnableFOC(true));
  }

  @Override
  public void setWristTarget(double target) {
    var control = new PositionVoltage(0);
    wristMotor.setControl(
        control.withPosition(Units.radiansToRotations(target)).withSlot(0).withEnableFOC(true));
  }

  @Override
  public void setBrakeMode(boolean armBrake, boolean wristBrake) {
    armMotor.setNeutralMode(armBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    wristMotor.setNeutralMode(wristBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPIDArm(double kP, double kI, double kD, double kFF) {
    armConfig.Slot0.kP = kP;
    armConfig.Slot0.kI = kI;
    armConfig.Slot0.kD = kD;
    armConfig.Slot0.kG = kFF;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armMotor.getConfigurator().apply(armConfig);
  }

  @Override
  public void setPIDWrist(double kP, double kI, double kD, double kFF) {
    wristConfig.Slot0.kP = kP;
    wristConfig.Slot0.kI = kI;
    wristConfig.Slot0.kD = kD;
    wristConfig.Slot0.kG = kFF;
    wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    wristMotor.getConfigurator().apply(wristConfig);
  }
}
