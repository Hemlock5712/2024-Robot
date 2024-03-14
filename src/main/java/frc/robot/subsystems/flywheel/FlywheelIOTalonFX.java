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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX leader = new TalonFX(53);
  private final TalonFX follower = new TalonFX(54);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Double> followerCurrent = follower.getSupplyCurrent();

  final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

  TalonFXConfiguration config = new TalonFXConfiguration();

  public FlywheelIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // set slot 0 gains

    // kV assumes linear however get to operating velocity

    config.Slot0.kS = 0.31; // Add 0.25 V output to overcome static friction
    config.Slot0.kV = 7.25 / 50.0; // A velocity target of 1 rps results in 0.12 V output
    config.Slot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    config.Slot0.kP = 0.9; // An error of 1 rps results in 0.11 V output
    config.Slot0.kI = 0.0; // no output for integrated error
    config.Slot0.kD = 0.001; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = config.MotionMagic;
    // Some value that is achievable
    motionMagicConfigs.MotionMagicAcceleration =
        50.0; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 500; // Target jerk of 4000 rps/s/s (0.1 seconds)

    for (int i = 0; i < 4; i++) {
      boolean statusOK = leader.getConfigurator().apply(config, 0.1) == StatusCode.OK;
      statusOK = statusOK && follower.getConfigurator().apply(config, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }
    follower.setControl(new Follower(leader.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.velocityRotPerSec = leaderVelocity.getValueAsDouble();
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setSpeedRotPerSec(double velocityRotPerSec) {
    leader.setControl(request.withVelocity(velocityRotPerSec));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
