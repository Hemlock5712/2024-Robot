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
  private boolean isDown = false;

  IntakeVisualizer visualizerMeasured = new IntakeVisualizer("IntakeMeasured");
  IntakeVisualizer visualizerSetpoint = new IntakeVisualizer("IntakeSetpoint");

  /** Creates a new Intake. */
  public Intake(IntakeActuatorIO actuatorIO, IntakeWheelsIO wheelsIO) {
    this.actuatorIO = actuatorIO;
    this.wheelsIO = wheelsIO;
    intakeUp();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    actuatorIO.updateInputs(actuatorInputs);
    wheelsIO.updateInputs(wheelsInputs);
    Logger.processInputs("IntakeActuator", actuatorInputs);
    Logger.processInputs("IntakeWheels", wheelsInputs);

    wheelsIO.runRPM(getTargetSpeed());
    if (getIsDown()) {
      actuatorIO.intakeDown();
    } else {
      actuatorIO.intakeUp();
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

  public void intakeDown() {
    isDown = true;
  }

  public void intakeUp() {
    isDown = false;
  }

  public void intakeToggle() {
    if (isDown) {
      isDown = false;
    } else {
      isDown = true;
    }
  }

  @AutoLogOutput(key = "Intake/TargetSpeed")
  public double getTargetSpeed() {
    return targetSpeed;
  }

  @AutoLogOutput(key = "Intake/IsDown")
  public boolean getIsDown() {
    return isDown;
  }
}
