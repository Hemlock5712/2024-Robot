// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.magazine;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Magazine extends SubsystemBase {
  private final MagazineIO magazineIO;
  private final MagazineIOInputsAutoLogged inputs = new MagazineIOInputsAutoLogged();
  private double targetVoltage = 0;
  private boolean isShooting = false;

  /** Creates a new Magazine. */
  public Magazine(MagazineIO magazineIO) {
    this.magazineIO = magazineIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    magazineIO.updateInputs(inputs);
    Logger.processInputs("Magazine", inputs);
    isShooting();
  }

  public void forward() {
    setSpeedRadPerSec(6);
    isShooting = false;
  }

  public void shoot() {
    setSpeedRadPerSec(7);
    isShooting = true;
  }

  public void backward() {
    setSpeedRadPerSec(-5);
    isShooting = false;
  }

  public void slowForward() {
    setSpeedRadPerSec(1.8);
    isShooting = false;
  }

  public void slowBackward() {
    setSpeedRadPerSec(-1.8);
    isShooting = false;
  }

  public void stop() {
    magazineIO.stop();
    isShooting = false;
  }

  @AutoLogOutput(key = "Magazine/isShooting")
  public boolean isShooting() {
    return isShooting;
  }

  public void setSpeedRadPerSec(double speedRadPerSec) {
    targetVoltage = speedRadPerSec;
    magazineIO.runVoltage(targetVoltage);
  }

  @AutoLogOutput(key = "Magazine/TargetRadPerSec")
  public double getTargetVoltage() {
    return targetVoltage;
  }

  @AutoLogOutput(key = "Magazine/VelocitRadPerSec")
  public double velocitRadsPerSec() {
    // Convert Radians per second to Meters per second
    return inputs.velocityRadPerSec;
  }
}
