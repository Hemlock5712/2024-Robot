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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeActuatorSim implements IntakeActuatorIO {

  SingleJointedArmSim intakeSim;

  TalonFX intakeMotor;
  TalonFXSimState intakeMotorSim;
  CANcoder intakeEncoder;

  public IntakeActuatorSim() {
    intakeMotor = new TalonFX(60);
    intakeEncoder = new CANcoder(61);

    intakeMotorSim = intakeMotor.getSimState();
    intakeMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;

    CANcoderConfiguration intakeEncoderConfig = new CANcoderConfiguration();
    intakeEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    intakeEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    intakeEncoder.getConfigurator().apply(intakeEncoderConfig);

    var intakeConfig = new TalonFXConfiguration();
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

    intakeSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            50,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), Units.lbsToKilograms(8)),
            Units.inchesToMeters(10),
            Units.degreesToRadians(-180),
            Units.degreesToRadians(180),
            false,
            Units.degreesToRadians(90));
  }

  public void updateInputs(IntakeActuatorIOInputs inputs) {
    intakeSim.setInput(intakeMotorSim.getMotorVoltage());
    intakeSim.update(0.02);
    intakeEncoder.getSimState().setRawPosition(intakeSim.getAngleRads() / (Math.PI * 2));
    intakeEncoder.getSimState().setVelocity(intakeSim.getVelocityRadPerSec() / (Math.PI * 2));

    inputs.angle = Units.rotationsToRadians(intakeEncoder.getPosition().getValue());
    inputs.targetAngle = Units.rotationsToRadians(intakeMotor.getClosedLoopReference().getValue());
  }

  public void setIntakeAngle(double angleRad) {
    var control = new PositionVoltage(0);
    intakeMotor.setControl(
        control
            .withPosition(Units.radiansToRotations(-angleRad + (Math.PI / 2)))
            .withFeedForward(0)
            .withEnableFOC(true));
  }
}
