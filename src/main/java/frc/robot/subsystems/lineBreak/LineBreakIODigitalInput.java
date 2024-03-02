package frc.robot.subsystems.lineBreak;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.lineBreak.LineBreakHelper.LineBreakValues;

public class LineBreakIODigitalInput implements LineBreakIO {
  DigitalInput lowerIntakeSensor = new DigitalInput(5);
  DigitalInput upperIntake1Sensor = new DigitalInput(3);
  AnalogInput upperIntake2Sensor = new AnalogInput(3);
  AnalogInput magazine1Sensor = new AnalogInput(2);
  AnalogInput magazine2Sensor = new AnalogInput(1);
  AnalogInput magazine3Sensor = new AnalogInput(0);

  public void updateInputs(LineBreakIOInputs inputs) {
    inputs.lineBreakValues =
        new LineBreakValues(
            !lowerIntakeSensor.get(),
            !upperIntake1Sensor.get(),
            upperIntake2Sensor.getVoltage() > 1,
            magazine1Sensor.getVoltage() > 1,
            magazine2Sensor.getVoltage() > 1,
            magazine3Sensor.getVoltage() > 1);
  }
}
