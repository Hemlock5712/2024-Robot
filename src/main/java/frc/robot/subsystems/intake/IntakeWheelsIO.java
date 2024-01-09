// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.IntakeActuatorIO.IntakeActuatorIOInputs;

public interface IntakeWheelsIO {
  @AutoLog
  public static class IntakeWheelsIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeActuatorIOInputs inputs) {}

  public default void run(double speed) {}
}