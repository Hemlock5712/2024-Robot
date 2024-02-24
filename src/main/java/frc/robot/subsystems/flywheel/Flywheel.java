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
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  // private final SysIdRoutine sysId;

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Flywheel/kP", FlywheelConstants.FeedbackController.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Flywheel/kI", FlywheelConstants.FeedbackController.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Flywheel/kD", FlywheelConstants.FeedbackController.kD());

  private double targetSpeed = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    io.configurePID(kP.get(), kI.get(), kD.get());
  }

  // Configure SysId
  // sysId =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             null,
  //             null,
  //             null,
  //             state -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
  //         new SysIdRoutine.Mechanism(voltage -> runVolts(voltage.in(Volts)), null, this));

  @Override
  // 12V is 100RPS
  // 3V/50RPS .12 = kV = output V/targetinput RPS

  // kv = .17 this was found by supplying motor 3V and converting rad/s to rot/s and doing
  // 3V/(rot/s)
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    io.setSpeedRotPerSec(targetSpeed);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.configurePID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
  }

  public void setSpeedRotPerSec(double speedRotPerSec) {
    targetSpeed = speedRotPerSec;
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  // /** Returns a command to run a quasistatic test in the specified direction. */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysId.quasistatic(direction);
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysId.dynamic(direction);
  // }

  /** Returns the current velocity in Rot Per Sec. */
  @AutoLogOutput
  public double getVelocityRotPerSec() {
    return inputs.velocityRotPerSec;
  }

  @AutoLogOutput(key = "Flywheel/TargetSpeed")
  public double getTargetSpeed() {
    return targetSpeed;
  }

  public boolean atTargetSpeed() {
    return Math.abs(inputs.velocityRotPerSec - getTargetSpeed()) < 100;
  }
}
