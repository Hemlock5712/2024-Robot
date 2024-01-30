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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  private double targetSpeed = 0.0;
  private boolean voltageMode = false;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    if(!voltageMode) {
      io.setSpeedRPM(targetSpeed);
    } else {
      io.setVoltage(targetSpeed);
    }
  }

  public void setSpeedRPM(double speedRPM) {
    voltageMode = false;
    targetSpeed = speedRPM;
  }

  public void runVolts(double volts) {
    voltageMode = true;
    targetSpeed = volts;
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  @AutoLogOutput(key = "Flywheel/TargetSpeed")
  public double getTargetSpeed() {
    return targetSpeed;
  }
}
