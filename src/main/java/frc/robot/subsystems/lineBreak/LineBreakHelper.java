package frc.robot.subsystems.linebreak;

public class LineBreakHelper {
  public record LineBreakValues(
      boolean lowerIntake,
      boolean upperIntake1,
      boolean upperIntake2,
      boolean magazine1,
      boolean magazine2,
      boolean magazine3) {
    // Gets the value of the digital input.  Returns true if the circuit is open.
    // True = game piece

    public boolean hasGamePiece() {
      return lowerIntake || upperIntake1 || upperIntake2 || magazine1 || magazine2 || magazine3;
    }

    public boolean notInLowerIntake() {
      return !(lowerIntake || upperIntake1)
          && (upperIntake2 || magazine1 || magazine2 || magazine3);
    }

    public boolean inLowerIntake() {
      return (upperIntake2 || upperIntake1);
    }

    public boolean hasGamePieceIntake() {
      return lowerIntake || upperIntake1 || upperIntake2;
    }

    public boolean isShooterLoaded() {
      return (magazine1 && magazine2 && magazine3);
    }

    public boolean isShooterLong() {
      return (magazine2 || magazine3) && !magazine1;
    }
  }
}
