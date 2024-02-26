// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.magazine;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Magazine extends SubsystemBase {
  private final MagazineIO magazineIO;
  private final MagazineIOInputsAutoLogged inputs = new MagazineIOInputsAutoLogged();
  private double targetSpeed = 0;

  /** Creates a new Magazine. */
  public Magazine(MagazineIO magazineIO) {
    this.magazineIO = magazineIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    magazineIO.updateInputs(inputs);
    magazineIO.setSpeedRadPerSec(getTargetRad());
  }

  public void forward() {
    targetSpeed = 6.283;
  }

  public void slowForward(double multiplier) {
    targetSpeed = 2 * multiplier;
  }

  public void slowBackward(double multiplier) {
    targetSpeed = -2 * multiplier;
  }

  public void backward() {
    targetSpeed = -6.283;
  }

  public void stop() {
    targetSpeed = 0;
  }

  @AutoLogOutput(key = "Magazine/TargetSpeed")
  public double getTargetRad() {
    return targetSpeed;
  }

  @AutoLogOutput(key = "Magazine/VelocitRadPerSec")
  public double velocitRadsPerSec() {
    return inputs.velocityRadPerSec;
  }
}
