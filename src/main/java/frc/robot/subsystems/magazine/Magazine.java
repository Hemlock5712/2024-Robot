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
  private double targetSpeed = 0;

  /** Creates a new Magazine. */
  public Magazine(MagazineIO magazineIO) {
    this.magazineIO = magazineIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    magazineIO.updateInputs(inputs);
    Logger.processInputs("Magazine", inputs);
  }

  public void forward() {
    setSpeedRadPerSec(5);
  }

  public void backward() {
    setSpeedRadPerSec(-5);
  }

  public void slowForward() {
    setSpeedRadPerSec(1.8);
  }

  public void slowBackward() {
    setSpeedRadPerSec(-1.8);
  }

  public void stop() {
    magazineIO.stop();
  }

  public void setSpeedRadPerSec(double speedRadPerSec) {
    targetSpeed = speedRadPerSec;
    magazineIO.runRadPerSec(targetSpeed);
  }

  @AutoLogOutput(key = "Magazine/TargetRadPerSec")
  public double getTargetRadPerSec() {
    return targetSpeed;
  }

  @AutoLogOutput(key = "Magazine/VelocitRadPerSec")
  public double velocitRadsPerSec() {
    // Convert Radians per second to Meters per second
    return inputs.velocityRadPerSec;
  }
}
