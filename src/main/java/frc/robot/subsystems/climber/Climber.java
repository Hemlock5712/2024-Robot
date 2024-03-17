// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private double targetPosition = 0;
  private boolean isCalibrated = false;
  private boolean isRequestingClimb = false;

  /** Creates a new Elevator. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void setCustomPosition(double targetPosition, int slot) {
    this.targetPosition = targetPosition;
    io.setCustomPosition(targetPosition, slot);
  }

  public void stop() {
    io.stop();
  }

  public void voltageControl(double voltage) {
    io.voltageControl(voltage);
  }

  public void resetClimberPositionToZero() {
    io.resetPositionToZero();
    isCalibrated = true;
  }

  public boolean isCalibrated() {
    return isCalibrated;
  }

  public double getPosition() {
    return inputs.position;
  }

  @AutoLogOutput(key = "Elevator/SetPoint")
  public double getSetPoint() {
    return targetPosition;
  }

  @AutoLogOutput(key = "Elevator/LimitSwitch")
  public boolean isLimitSwitchTriggered() {
    return inputs.limitSwitchTriggered;
  }

  public void setRequestingClimb(boolean requesting) {
    isRequestingClimb = requesting;
  }

  public boolean isRequestingClimb() {
    return isRequestingClimb;
  }
}
