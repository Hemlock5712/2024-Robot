// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeActuatorIO actuatorIO;
  private final IntakeWheelsIO wheelsIO;

  private final IntakeActuatorIOInputsAutoLogged actuatorInputs =
      new IntakeActuatorIOInputsAutoLogged();
  private final IntakeWheelsIOInputsAutoLogged wheelsInputs = new IntakeWheelsIOInputsAutoLogged();
  private double targetVoltage = 0;
  private boolean intakeRequest = false;
  private IntakePositions intakePositions = IntakePositions.BUMPER;

  IntakeVisualizer visualizerMeasured = new IntakeVisualizer("IntakeMeasured");
  IntakeVisualizer visualizerSetpoint = new IntakeVisualizer("IntakeSetpoint");

  /** Creates a new Intake. */
  public Intake(IntakeActuatorIO actuatorIO, IntakeWheelsIO wheelsIO) {
    this.actuatorIO = actuatorIO;
    this.wheelsIO = wheelsIO;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    actuatorIO.updateInputs(actuatorInputs);
    wheelsIO.updateInputs(wheelsInputs);
    Logger.processInputs("IntakeActuator", actuatorInputs);
    Logger.processInputs("IntakeWheels", wheelsInputs);

    wheelsIO.runVoltage(getTargetVoltage());
    switch (intakePositions) {
      case BUMPER:
        actuatorIO.setIntakeAngle(IntakeConstants.bumperPosition.angle().getRadians());
        break;
      case FLOOR:
        actuatorIO.setIntakeAngle(IntakeConstants.floorPosition.angle().getRadians());
        break;
      case UP:
        actuatorIO.setIntakeAngle(IntakeConstants.upPosition.angle().getRadians());
        break;
    }

    visualizerMeasured.update(actuatorInputs.angle);
    visualizerSetpoint.update(0);
  }

  public void intake() {
    targetVoltage = 8;
  }

  public void outtake() {
    targetVoltage = -8;
  }

  public void stopIntake() {
    targetVoltage = 0;
  }

  public void setIntakeMode(IntakePositions intakePositions) {
    this.intakePositions = intakePositions;
  }

  public void enableIntakeRequest() {
    this.intakeRequest = true;
  }

  public void disableIntakeRequest() {
    this.intakeRequest = false;
  }

  @AutoLogOutput(key = "Intake/IntakeRequest")
  public boolean getIntakeRequest() {
    return intakeRequest;
  }

  @AutoLogOutput(key = "Intake/TargetVoltage")
  public double getTargetVoltage() {
    return targetVoltage;
  }

  public enum IntakePositions {
    FLOOR,
    UP,
    BUMPER
  }
}
