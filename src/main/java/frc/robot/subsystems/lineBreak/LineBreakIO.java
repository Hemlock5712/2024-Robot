package frc.robot.subsystems.lineBreak;

import frc.robot.subsystems.lineBreak.LineBreakHelper.LineBreakValues;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LineBreakIO {
  class LineBreakIOInputs implements LoggableInputs {

    LineBreakValues lineBreakValues = new LineBreakValues(false, false, false, false, false, false);

    @Override
    public void toLog(LogTable table) {
table.put("LineBreaker/lowerIntake", lineBreakValues.lowerIntake());
      table.put("LineBreaker/upperIntake1", lineBreakValues.upperIntake1());
      table.put("LineBreaker/upperIntake2", lineBreakValues.upperIntake2());
      table.put("LineBreaker/magazine1", lineBreakValues.magazine1());
      table.put("LineBreaker/magazine2", lineBreakValues.magazine2());
      table.put("LineBreaker/magazine3", lineBreakValues.magazine3());
    }

    @Override
    public void fromLog(LogTable table) {
table.get("LineBreaker/lowerIntake", lineBreakValues.lowerIntake());
      table.get("LineBreaker/upperIntake1", lineBreakValues.upperIntake1());
      table.get("LineBreaker/upperIntake2", lineBreakValues.upperIntake2());
      table.get("LineBreaker/magazine1", lineBreakValues.magazine1());
      table.get("LineBreaker/magazine2", lineBreakValues.magazine2());
      table.get("LineBreaker/magazine3", lineBreakValues.magazine3());
    }
  }

  public default void bumpGamePiece() {}

  public default void shootGamePiece() {}

  public default void setGamePiece(
      boolean lowerIntake,
      boolean upperIntake1,
      boolean upperIntake2,
      boolean magazine1,
      boolean magazine2,
      boolean magazine3) {}

  default void updateInputs(LineBreakIOInputs inputs) {}
}
