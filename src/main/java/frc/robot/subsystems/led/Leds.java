package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  private final LedsIO io;

  public Leds(LedsIO io) {
    this.io = io;
  }

  public void setAnimation(Animation animation) {
    io.setAnimation(animation);
  }
}
