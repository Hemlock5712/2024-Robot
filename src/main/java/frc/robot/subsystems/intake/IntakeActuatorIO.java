// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeActuatorIO {
  @AutoLog
  public static class IntakeActuatorIOInputs {
    public boolean isDown = false;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeActuatorIOInputs inputs) {}

  public default void intakeUp() {}

  public default void intakeDown() {}
}
