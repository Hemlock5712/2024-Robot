// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.linebreak;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.linebreak.LineBreakIO.LineBreakIOInputs;
import frc.robot.util.visualizer.RobotGamePieceVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;

public class LineBreak extends SubsystemBase {
  private final LineBreakIO lineBreakIO;
  private final LineBreakIOInputs inputs = new LineBreakIOInputs();
  private double lastGamePieceSeenTime;

  /** Creates a new LineBreak. */
  public LineBreak(LineBreakIO lineBreakIO) {
    this.lineBreakIO = lineBreakIO;
    lastGamePieceSeenTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lineBreakIO.updateInputs(inputs);
    if (inputs.lineBreakValues.hasGamePiece()) {
      lastGamePieceSeenTime = Timer.getFPGATimestamp();
    }
  }

  @AutoLogOutput(key = "/LineBreak/timeSinceLastGamePiece")
  public double timeSinceLastGamePiece() {
    return Timer.getFPGATimestamp() - lastGamePieceSeenTime;
  }

  @AutoLogOutput(key = "/LineBreak/hasNoGamePiece")
  public boolean hasNoGamePiece() {
    return !hasGamePiece();
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
    return inputs.lineBreakValues.inLowerIntake();
  }

  @AutoLogOutput(key = "/LineBreak/isShooterLoaded")
  public boolean isShooterLoaded() {
    return inputs.lineBreakValues.isShooterLoaded();
  }

  @AutoLogOutput(key = "/LineBreak/isShooterLong")
  public boolean isShooterLong() {
    return inputs.lineBreakValues.isShooterLong();
  }

  @AutoLogOutput(key = "/LineBreak/magazine3Sensor")
  public boolean isMagazine3Sensor() {
    return inputs.lineBreakValues.magazine3();
  }

  @AutoLogOutput(key = "/LineBreak/magazine2Sensor")
  public boolean isMagazine2Sensor() {
    return inputs.lineBreakValues.magazine2();
  }

  @AutoLogOutput(key = "/LineBreak/magazine1Sensor")
  public boolean isMagazine1Sensor() {
    return inputs.lineBreakValues.magazine1();
  }

  @AutoLogOutput(key = "/LineBreak/upperIntake1Sensor")
  public boolean isupperIntake1Sensor() {
    return inputs.lineBreakValues.upperIntake1();
  }

  @AutoLogOutput(key = "/LineBreak/upperInt2Sensor")
  public boolean isupperIntake2Sensor() {
    return inputs.lineBreakValues.upperIntake2();
  }

  public void bumpGamePiece() {
    lineBreakIO.bumpGamePiece();
    RobotGamePieceVisualizer.drawGamePieces();
  }

  public void shootGamePiece() {
    lineBreakIO.shootGamePiece();
  }

  public void setGamePiece(
      boolean intake,
      boolean upperIntake1,
      boolean upperIntake2,
      boolean magazine1,
      boolean magazine2,
      boolean magazine3) {
    lineBreakIO.setGamePiece(intake, upperIntake1, upperIntake2, magazine1, magazine2, magazine3);
    RobotGamePieceVisualizer.drawGamePieces();
  }
}
