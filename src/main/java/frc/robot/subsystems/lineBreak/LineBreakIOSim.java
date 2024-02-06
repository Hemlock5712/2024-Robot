package frc.robot.subsystems.lineBreak;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.lineBreak.LineBreakHelper.LineBreakValues;

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
