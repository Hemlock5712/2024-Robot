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
    magazineIO.runRPM(getTargetSpeed());
  }

  public void forward() {
    targetSpeed = 1000;
  }

  public void backward() {
    targetSpeed = -1000;
  }

  public void stop() {
    targetSpeed = 0;
  }

  public void setSpeed(double speedRPM) {
    targetSpeed = speedRPM;
  }

  @AutoLogOutput(key = "Magazine/TargetSpeed")
  public double getTargetSpeed() {
    return targetSpeed;
  }

  @AutoLogOutput(key = "Magazine/VelocitRotPerSec")
  public double velocitRadsPerSec() {
    // Convert Radians per second to Meters per second
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}
