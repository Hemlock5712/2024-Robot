// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intake.IntakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeActuatorIO actuatorIO;
  private final IntakeWheelsIO wheelsIO;
  private final IntakeActuatorIOInputsAutoLogged actuatorInputs =
      new IntakeActuatorIOInputsAutoLogged();
  private final IntakeWheelsIOInputsAutoLogged wheelsInputs = new IntakeWheelsIOInputsAutoLogged();
  private double targetSpeed = 0;
  private boolean driverRequestIntakeDown = false;
  private IntakePositions intakePositions = IntakePositions.UP;

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

    wheelsIO.runRPM(getTargetSpeed());
    switch (intakePositions) {
      case BUMPER:
        actuatorIO.setIntakeAngle(IntakeConstants.bumperPosition.toRadians());
        break;
      case FLOOR:
        actuatorIO.setIntakeAngle(IntakeConstants.floorPosition.toRadians());
        break;
      case UP:
        actuatorIO.setIntakeAngle(IntakeConstants.upPosition.toRadians());
        break;
    }

    visualizerMeasured.update(actuatorInputs.angle);
    visualizerSetpoint.update(0);
  }

  public void intake() {
    targetSpeed = 1000;
  }

  public void outtake() {
    targetSpeed = -1000;
  }

  public void stopIntake() {
    targetSpeed = 0;
  }

  public void setSpeed(double speedRPM) {
    targetSpeed = speedRPM;
  }

  public void setIntakeMode(IntakePositions intakePositions) {
    this.intakePositions = intakePositions;
  }

  public void setDriverRequestIntakeDown() {
    this.driverRequestIntakeDown = true;
  }

  public void setDriverRequestIntakeUp() {
    this.driverRequestIntakeDown = false;
  }

  @AutoLogOutput(key = "Intake/DriverRequestIntakeDown")
  public boolean getDriverRequestIntakeDown() {
    return driverRequestIntakeDown;
  }

  @AutoLogOutput(key = "Intake/TargetSpeed")
  public double getTargetSpeed() {
    return targetSpeed;
  }

  public enum IntakePositions {
    FLOOR,
    UP,
    BUMPER
  }
}
