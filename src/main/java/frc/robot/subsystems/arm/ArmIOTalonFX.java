package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
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

    wristEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    wristEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    wristEncoderConfig.MagnetSensor.MagnetOffset = 0.050244;

    for (int i = 0; i < 4; i++) {
      boolean statusOK = armEncoder.getConfigurator().apply(armEncoderConfig, 0.1) == StatusCode.OK;
      statusOK =
          statusOK
              && wristEncoder.getConfigurator().apply(wristEncoderConfig, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    armConfig.Feedback.SensorToMechanismRatio = 1;

    armConfig.Slot0.kP = ArmConstants.armControlConstants.kP();
    armConfig.Slot0.kI = ArmConstants.armControlConstants.kI();
    armConfig.Slot0.kD = ArmConstants.armControlConstants.kD();
    armConfig.Slot0.kG = ArmConstants.armControlConstants.kG();

    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    wristConfig.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristConfig.Feedback.SensorToMechanismRatio = 1;

    // -.2 //.2
    wristConfig.Slot0.kP = ArmConstants.wristControlConstants.kP();
    wristConfig.Slot0.kI = ArmConstants.wristControlConstants.kI();
    wristConfig.Slot0.kD = ArmConstants.wristControlConstants.kD();
    wristConfig.Slot0.kG = ArmConstants.wristControlConstants.kG();
    wristConfig.Slot0.kS = 0.216;

    for (int i = 0; i < 4; i++) {
      boolean statusOK = wristMotor.getConfigurator().apply(wristConfig, 0.1) == StatusCode.OK;
      statusOK = statusOK && armMotor.getConfigurator().apply(armConfig, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }

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

    inputs.armRelativePositionRad = armAbsolutePosition.getValue() * Math.PI * 2;
    inputs.armAbsolutePositionRad =
        armAbsolutePosition.getValue() * Math.PI * 2; // Units.rotationsToRadians
    inputs.armVelocityRadPerSec = armSpeed.getValue() * Math.PI * 2;
    inputs.armCurrentAmps = new double[] {armSupplyCurrent.getValue()};
    inputs.armTempCelcius = new double[] {armTemp.getValue()};

    inputs.wristRelativePositionRad = wristAbsolutePosition.getValue() * Math.PI * 2;
    inputs.wristAbsolutePositionRad =
        inputs.wristRelativePositionRad + inputs.armRelativePositionRad;
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
  public void setWristTarget(double target, double wristAbsolutePosition) {
    var control = new PositionVoltage(0);
    wristMotor.setControl(
        control
            .withPosition(Units.radiansToRotations(target))
            .withSlot(0)
            .withEnableFOC(true)
            .withFeedForward(Rotation2d.fromRadians(wristAbsolutePosition).getCos() * 0.173));
  }

  @Override
  public void setBrakeMode(boolean armBrake, boolean wristBrake) {
    armMotor.setNeutralMode(armBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    wristMotor.setNeutralMode(wristBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void stop() {
    armMotor.stopMotor();
    wristMotor.stopMotor();
  }
}
