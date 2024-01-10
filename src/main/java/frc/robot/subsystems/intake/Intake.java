// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeActuatorIO actuatorIO;
  private final IntakeWheelsIO wheelsIO;
  private final IntakeActuatorIOInputsAutoLogged actuatorInputs =
      new IntakeActuatorIOInputsAutoLogged();
  private final IntakeWheelsIOInputsAutoLogged wheelsInputs = new IntakeWheelsIOInputsAutoLogged();
  private double targetSpeed = 0;
  private boolean isDown = False;
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
    if(getIsDown()) {
      actuatorIO.intakeDown();
    }
    else {
      actuatorIO.intakeUp();
    }
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
    isDown = True;
  }

  public void intakeUp() {
    isDown = Flase;
  }

  public void intakeToggle() {
    if (isDown){
      isDown = False;
    }
    else {
      isDown = True;
    }
  }

  @AutoLogOutput(key = "Intake/TargetSpeed")
  public double getTargetSpeed() {
    return targetSpeed;
  }

  @AutoLogOutput(key = "Intake/IsDown")
  public double getIsDown() {
    return isDown;
  }

}
