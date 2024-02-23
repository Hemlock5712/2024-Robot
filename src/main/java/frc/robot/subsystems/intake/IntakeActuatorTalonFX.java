package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class IntakeActuatorTalonFX implements IntakeActuatorIO {

  TalonFX intakeMotor;
  CANcoder intakeEncoder;
  double kG = 0.0;

  TalonFXConfiguration intakeConfig;

  public IntakeActuatorTalonFX() {
    intakeMotor = new TalonFX(60);
    intakeEncoder = new CANcoder(61);

    CANcoderConfiguration intakeEncoderConfig = new CANcoderConfiguration();
    intakeEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    intakeEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    intakeEncoder.getConfigurator().apply(intakeEncoderConfig);

    intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var slot0Configs = intakeConfig.Slot0;
    slot0Configs.kP = -9;
    slot0Configs.kI = 0;
    slot0Configs.kD = 1;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    intakeConfig.ClosedLoopGeneral.ContinuousWrap = true;

    intakeConfig.Feedback.FeedbackRemoteSensorID = intakeEncoder.getDeviceID();
    intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    intakeConfig.Feedback.SensorToMechanismRatio = 1;
    intakeConfig.Feedback.RotorToSensorRatio = 50;
    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  @Override
  public void updateInputs(IntakeActuatorIOInputs inputs) {
    inputs.angle = Rotation2d.fromRotations(intakeEncoder.getPosition().getValue()).getRadians();
    inputs.targetAngle =
        Rotation2d.fromRotations(intakeMotor.getClosedLoopReference().getValue()).getRadians();
    inputs.currentAmps = new double[] {intakeMotor.getSupplyCurrent().getValue()};
    inputs.tempCelcius = new double[] {intakeMotor.getDeviceTemp().getValue()};
  }

  @Override
  public void setIntakeAngle(double angleRad) {
    var control = new PositionVoltage(0);
    intakeMotor.setControl(
        control
            .withPosition(Units.radiansToRotations(-angleRad + (Math.PI / 2)))
            .withEnableFOC(true));
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double kFF) {
    intakeConfig.Slot0.kP = kP;
    intakeConfig.Slot0.kI = kI;
    intakeConfig.Slot0.kD = kD;
    intakeConfig.Slot0.kG = kFF;
    intakeConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    intakeMotor.getConfigurator().apply(intakeConfig);
  }
}
