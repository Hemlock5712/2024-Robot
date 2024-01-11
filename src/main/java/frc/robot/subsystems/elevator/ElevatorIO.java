package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.IntakeWheelsIO.IntakeWheelsIOInputs;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double position = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void gotoSetPoint(double setPosition) {}

    public default void stop() {}

}
