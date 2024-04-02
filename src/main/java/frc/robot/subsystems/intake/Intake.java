// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
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

    visualizerMeasured.update(actuatorInputs.angle);
    visualizerSetpoint.update(0);
  }

  public void intake() {
    setSpeedRotPerSec(60);
  }

  public void outtake() {
    setSpeedRotPerSec(-30);
  }

  /** Stops the intake. */
  public void stop() {
    wheelsIO.stop();
  }

  public void setIntakeMode(IntakePositions intakePositions) {
    if (intakePositions == IntakePositions.FLOOR) {
      if (actuatorInputs.downLimitSwitchTriggered) {
        actuatorIO.setVoltage(0.01);
      } else {
        actuatorIO.setVoltage(5.4);
      }
      actuatorIO.coastMode();
    } else {
      if (actuatorInputs.upLimitSwitchTriggered) {
        actuatorIO.setVoltage(-0.01);
      } else {
        actuatorIO.setVoltage(-5.2);
      }
      actuatorIO.brakeMode();
    }
  }

  public void enableIntakeRequest() {
    this.intakeRequest = true;
  }

  public void disableIntakeRequest() {
    this.intakeRequest = false;
  }

  public boolean isAtOrAbovePosition() {
    return (actuatorInputs.targetAngle - actuatorInputs.angle) + Units.degreesToRadians(15) > 0;
  }

  @AutoLogOutput(key = "Intake/IntakeRequest")
  public boolean getIntakeRequest() {
    return intakeRequest;
  }

  public void setSpeedRotPerSec(double speedRotPerSec) {
    targetSpeed = speedRotPerSec;
    wheelsIO.setSpeedRotPerSec(targetSpeed);
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
    UP
  }
}
