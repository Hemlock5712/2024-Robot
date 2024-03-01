package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;

public interface LedsIO {
    default void setAnimation(Animation animation) {}
}
