// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private double targetSpeed = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public void setSpeedRotPerSec(double speedRotPerSec) {
    targetSpeed = speedRotPerSec;
    io.setSpeedRotPerSec(targetSpeed);
  }

  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in Rot Per Sec. */
  @AutoLogOutput
  public double getVelocityRotPerSec() {
    return inputs.velocityRotPerSec;
  }

  @AutoLogOutput(key = "Flywheel/TargetSpeed")
  public double getTargetRot() {
    return targetSpeed;
  }

  public boolean atTargetSpeed() {
    return Math.abs(inputs.velocityRotPerSec - getTargetRot()) < 0.5;
  }
}
