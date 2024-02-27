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
  private double targetSpeed = 0;
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

    wheelsIO.setSpeedRotPerSec(getTargetRot());
    switch (intakePositions) {
      case BUMPER:
        actuatorIO.setIntakeAngle(IntakeConstants.floorPosition.angle().getRadians());
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
    setSpeedRotPerSec(15);
  }

  public void outtake() {
    setSpeedRotPerSec(-15);
  }

  /** Stops the intake. */
  public void stop() {
    setSpeedRotPerSec(0);
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

  public void setSpeedRotPerSec(double speedRotPerSec) {
    targetSpeed = speedRotPerSec;
  }

  /** Returns the current velocity in Rot Per Sec. */
  @AutoLogOutput
  public double getVelocityRotPerSec() {
    return wheelsInputs.velocityRotPerSec;
  }

  @AutoLogOutput(key = "Intake/TargetSpeed")
  public double getTargetRot() {
    return targetSpeed;
  }

  public boolean atTargetSpeed() {
    return Math.abs(wheelsInputs.velocityRotPerSec - getTargetRot()) < 0.5;
  }

  public enum IntakePositions {
    FLOOR,
    UP,
    BUMPER
  }
}
