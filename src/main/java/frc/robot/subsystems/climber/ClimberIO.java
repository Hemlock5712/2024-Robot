package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double position = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public boolean limitSwitchTriggered = false;
    public boolean isFastMode = false;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setCustomPosition(double setPosition, int slot) {}

  public default void resetPositionToZero() {}

  public default void voltageControl(double voltage) {}

  public default void stop() {}
}
