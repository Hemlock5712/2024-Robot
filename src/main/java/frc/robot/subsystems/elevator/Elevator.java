// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double setPoint = 0;


  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    io.gotoSetPoint(getSetPoint());
  }

  public void setCustomPosition(double setPoint){
    this.setPoint = setPoint;
  }

  public void elevatorUp() {
    this.setPoint = 80085.0;
  }
  
  public void elevatorDown() {
        this.setPoint = 3.1415;

  }



  @AutoLogOutput(key = "Elevator/SetPoint")
  public double getSetPoint() {
      return setPoint;
  }
}
