package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armAbsolutePositionRad = 0.0;
    public double armRelativePositionRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};

    public double wristAbsolutePositionRad = 0.0;
    public double wristRelativePositionRad = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double[] wristCurrentAmps = new double[] {};
    public double[] wristTempCelcius = new double[] {};
  }

  public default void setArmTarget(double target) {}

  public default void setWristTarget(double target, double wristAbsolutePosition) {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setBrakeMode(boolean armBrake, boolean wristBrake) {}

  public default void stop() {}
}
