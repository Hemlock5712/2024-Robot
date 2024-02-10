// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lineBreak;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lineBreak.LineBreakIO.LineBreakIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;

public class LineBreak extends SubsystemBase {
  private final LineBreakIO lineBreakIO;
  private final LineBreakIOInputs inputs = new LineBreakIOInputs();

  /** Creates a new LineBreak. */
  public LineBreak(LineBreakIO lineBreakIO) {
    this.lineBreakIO = lineBreakIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lineBreakIO.updateInputs(inputs);
  }

  @AutoLogOutput(key = "/LineBreak/hasGamePiece")
  public boolean hasGamePiece() {
    return inputs.lineBreakValues.hasGamePiece();
  }

  @AutoLogOutput(key = "/LineBreak/hasGamePieceIntake")
  public boolean hasGamePieceIntake() {
    return inputs.lineBreakValues.hasGamePieceIntake();
  }

  @AutoLogOutput(key = "/LineBreak/notInLowerIntake")
  public boolean notInLowerIntake() {
    return inputs.lineBreakValues.notInLowerIntake();
  }

  @AutoLogOutput(key = "/LineBreak/inLowerIntake")
  public boolean inLowerIntake() {
    return inputs.lineBreakValues.InLowerIntake();
  }

  @AutoLogOutput(key = "/LineBreak/isShooterLoaded")
  public boolean isShooterLoaded() {
    return inputs.lineBreakValues.isShooterLoaded();
  }

  @AutoLogOutput(key = "/LineBreak/isShooterLong")
  public boolean isShooterLong() {
    return inputs.lineBreakValues.isShooterLong();
  }
}