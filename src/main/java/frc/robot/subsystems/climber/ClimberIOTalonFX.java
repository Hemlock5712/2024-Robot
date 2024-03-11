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

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX leader = new TalonFX(42);

  private final DigitalInput limitSwitch = new DigitalInput(0);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getSupplyCurrent();

  final MotionMagicVoltage request = new MotionMagicVoltage(0);

  TalonFXConfiguration config = new TalonFXConfiguration();

  public ClimberIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 25;

    // kV assumes linear however get to operating velocity

    config.Slot0.kS = 0.228; // Add 0.25 V output to overcome static friction
    config.Slot0.kV = 1.0 / 6.7; // A velocity target of 1 rps results in 0.12 V output
    config.Slot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    config.Slot0.kP = 6.0; // An error of 1 rps results in 0.11 V output
    config.Slot0.kI = 0.0; // no output for integrated error
    config.Slot0.kD = 0.0; // no output for error derivative

    config.Slot1.kS = 0.228;
    config.Slot1.kV = 1.0 / 6.7;
    config.Slot1.kA = 0.0;
    config.Slot1.kP = 8.0;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = 0.0;

    config.MotionMagic.MotionMagicCruiseVelocity = 8;
    config.MotionMagic.MotionMagicAcceleration = 20;
    config.MotionMagic.MotionMagicJerk = 200;

    for (int i = 0; i < 4; i++) {
      boolean statusOK = leader.getConfigurator().apply(config, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    leader.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    inputs.position = leaderPosition.getValueAsDouble();
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {leaderCurrent.getValueAsDouble()};
    inputs.limitSwitchTriggered = limitSwitch.get();
  }

  @Override
  public void setCustomPosition(double rotPosition, int slot) {
    leader.setControl(request.withPosition(rotPosition).withSlot(slot));
  }

  @Override
  public void resetPositionToZero() {
    leader.setPosition(0);
  }

  public void voltageControl(double voltage) {
    VoltageOut control = new VoltageOut(voltage);
    leader.setControl(control);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
