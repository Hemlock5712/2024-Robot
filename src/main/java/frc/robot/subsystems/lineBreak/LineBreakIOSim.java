package frc.robot.subsystems.linebreak;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.linebreak.LineBreakHelper.LineBreakValues;

public class LineBreakIOSim implements LineBreakIO {
  NetworkTable table;
  NetworkTableEntry lowerIntakeSensor;
  NetworkTableEntry upperIntake1Sensor;
  NetworkTableEntry upperIntake2Sensor;
  NetworkTableEntry magazine1Sensor;
  NetworkTableEntry magazine2Sensor;
  NetworkTableEntry magazine3Sensor;

  public LineBreakIOSim() {
    table = NetworkTableInstance.getDefault().getTable("LineBreak");
    lowerIntakeSensor = table.getEntry("LowerIntakeSensor");
    upperIntake1Sensor = table.getEntry("UpperIntake1Sensor");
    upperIntake2Sensor = table.getEntry("UpperIntake2Sensor");
    magazine1Sensor = table.getEntry("Magazine1Sensor");
    magazine2Sensor = table.getEntry("Magazine2Sensor");
    magazine3Sensor = table.getEntry("Magazine3Sensor");
    lowerIntakeSensor.setBoolean(false);
    upperIntake1Sensor.setBoolean(false);
    upperIntake2Sensor.setBoolean(false);
    magazine1Sensor.setBoolean(false);
    magazine2Sensor.setBoolean(false);
    magazine3Sensor.setBoolean(false);
  }

  @Override
  /** Bumps the game piece to the next sensor. */
  public void bumpGamePiece() {
    if (lowerIntakeSensor.getBoolean(false)) {
      lowerIntakeSensor.setBoolean(false);
      upperIntake1Sensor.setBoolean(true);
    } else if (upperIntake1Sensor.getBoolean(false)) {
      upperIntake1Sensor.setBoolean(false);
      upperIntake2Sensor.setBoolean(true);
    } else if (upperIntake2Sensor.getBoolean(false)) {
      upperIntake2Sensor.setBoolean(false);
      magazine1Sensor.setBoolean(true);
    } else if (magazine1Sensor.getBoolean(false)) {
      magazine1Sensor.setBoolean(false);
      magazine2Sensor.setBoolean(true);
    } else if (magazine2Sensor.getBoolean(false)) {
      magazine2Sensor.setBoolean(false);
      magazine3Sensor.setBoolean(true);
    } else if (magazine3Sensor.getBoolean(false)) {
      magazine3Sensor.setBoolean(false);
      lowerIntakeSensor.setBoolean(true);
    } else {
      lowerIntakeSensor.setBoolean(true);
    }
  }

  @Override
  /** Shoots the game piece (empty magazines) */
  public void shootGamePiece() {
    lowerIntakeSensor.setBoolean(false);
    upperIntake1Sensor.setBoolean(false);
    upperIntake2Sensor.setBoolean(false);
    magazine1Sensor.setBoolean(false);
    magazine2Sensor.setBoolean(false);
    magazine3Sensor.setBoolean(false);
  }

  @Override
  /** Sets the game piece sensors */
  public void setGamePiece(
      boolean lowerIntake,
      boolean upperIntake1,
      boolean upperIntake2,
      boolean magazine1,
      boolean magazine2,
      boolean magazine3) {
    lowerIntakeSensor.setBoolean(lowerIntake);
    upperIntake1Sensor.setBoolean(upperIntake1);
    upperIntake2Sensor.setBoolean(upperIntake2);
    magazine1Sensor.setBoolean(magazine1);
    magazine2Sensor.setBoolean(magazine2);
    magazine3Sensor.setBoolean(magazine3);
  }

  public void updateInputs(LineBreakIOInputs inputs) {
    inputs.lineBreakValues =
        new LineBreakValues(
            lowerIntakeSensor.getBoolean(false),
            upperIntake1Sensor.getBoolean(false),
            upperIntake2Sensor.getBoolean(false),
            magazine1Sensor.getBoolean(false),
            magazine2Sensor.getBoolean(false),
            magazine3Sensor.getBoolean(false));
  }
}
