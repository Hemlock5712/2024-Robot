package frc.robot.subsystems.arm;

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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOTalonFX implements ArmIO {


  TalonFX armMotor;
  TalonFX wristMotor;

  CANcoder armEncoder;
  CANcoder wristEncoder;

  public ArmIOTalonFX() {
    armMotor = new TalonFX(50);
    wristMotor = new TalonFX(51);

    armEncoder = new CANcoder(52);
    wristEncoder = new CANcoder(53);


    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    armEncoder.getConfigurator().apply(armEncoderConfig);

    CANcoderConfiguration wristEncoderConfig = new CANcoderConfiguration();
    wristEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    wristEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    wristEncoder.getConfigurator().apply(wristEncoderConfig);

    var armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var slot0Configs = armConfig.Slot0;
    slot0Configs.kP = 6;
    slot0Configs.kI = 0.2;
    slot0Configs.kD = 1;
    slot0Configs.kS = 0.4;
    slot0Configs.kV = 0.01;
    slot0Configs.kA = 0.001;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    armConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    armConfig.Feedback.SensorToMechanismRatio = 1;
    armConfig.Feedback.RotorToSensorRatio = 10;

    var motionMagicConfig = armConfig.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 0.5;
    motionMagicConfig.MotionMagicAcceleration = 1;
    motionMagicConfig.MotionMagicJerk = 5;

    armMotor.getConfigurator().apply(armConfig);

    var wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var slot0ConfigsWrist = wristConfig.Slot0;
    slot0ConfigsWrist.kP = 8;
    slot0ConfigsWrist.kD = 0.3;
    slot0ConfigsWrist.kS = 0.3;
    slot0ConfigsWrist.kV = 0.01;
    slot0ConfigsWrist.kA = 0.001;

    wristConfig.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristConfig.Feedback.SensorToMechanismRatio = 1;
    wristConfig.Feedback.RotorToSensorRatio = 10;

    wristMotor.getConfigurator().apply(wristConfig);

    // var motionMagicConfig = armConfig.MotionMagic;
    // motionMagicConfig.MotionMagicCruiseVelocity = 40;
    // motionMagicConfig.MotionMagicAcceleration = 90;
    // motionMagicConfig.MotionMagicJerk = 1200;


  }

  public void updateInputs(ArmIOInputs inputs) {

    inputs.armAbsolutePositionRad = armEncoder.getPosition().refresh().getValue() * Math.PI * 2; // Units.rotationsToRadians
    inputs.armRelativePositionRad = armEncoder.getPosition().refresh().getValue() * Math.PI * 2;
    inputs.armVelocityRadPerSec = armEncoder.getVelocity().refresh().getValue();
    inputs.armCurrentAmps = new double[] {armMotor.getSupplyCurrent().getValue()};
    inputs.armTempCelcius = new double[] {armMotor.getDeviceTemp().getValue()};
    inputs.wristAbsolutePositionRad =
        (wristEncoder.getPosition().refresh().getValue() * Math.PI * 2)
            - inputs.armRelativePositionRad;
    inputs.wristRelativePositionRad = wristEncoder.getPosition().refresh().getValue() * Math.PI * 2;
    inputs.wristVelocityRadPerSec = wristEncoder.getVelocity().refresh().getValue() * Math.PI * 2;
    inputs.wristCurrentAmps = new double[] {wristMotor.getSupplyCurrent().getValue()};
    inputs.wristTempCelcius = new double[] {wristMotor.getDeviceTemp().getValue()};
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
}
