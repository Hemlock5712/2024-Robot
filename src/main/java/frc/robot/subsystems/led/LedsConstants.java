package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public final class LedsConstants {
  public static final int LED_CONTROLLER_ID = 33;
  public static final Animation DISABLED_ANIMATION = new FireAnimation(1, 0.5, -1, 0.5, 0.5);
  public static final Animation HAS_GAME_PIECE_ANIMATION = new SingleFadeAnimation(255, 165, 0);
  public static final Animation EMPTY_ANIMATION = new SingleFadeAnimation(0, 0, 100);
}
