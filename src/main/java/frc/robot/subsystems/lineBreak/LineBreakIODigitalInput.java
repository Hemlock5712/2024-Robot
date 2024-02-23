package frc.robot.subsystems.lineBreak;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.lineBreak.LineBreakHelper.LineBreakValues;

public class LineBreakIODigitalInput implements LineBreakIO {
  DigitalInput lowerIntakeSensor = new DigitalInput(5);
  DigitalInput upperIntake1Sensor = new DigitalInput(4);
  DigitalInput upperIntake2Sensor = new DigitalInput(3);
  DigitalInput magazine1Sensor = new DigitalInput(2);
  DigitalInput magazine2Sensor = new DigitalInput(1);
  DigitalInput magazine3Sensor = new DigitalInput(0);

  public void updateInputs(LineBreakIOInputs inputs) {
    inputs.lineBreakValues =
        new LineBreakValues(
            lowerIntakeSensor.get(),
            upperIntake1Sensor.get(),
            upperIntake2Sensor.get(),
            magazine1Sensor.get(),
            magazine2Sensor.get(),
            magazine3Sensor.get());
  }
}
