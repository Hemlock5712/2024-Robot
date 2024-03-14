package frc.robot.subsystems.magazine;

import org.littletonrobotics.junction.AutoLog;

public interface MagazineIO {
  @AutoLog
  public static class MagazineIOInputs {
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MagazineIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void stop() {}
}
