package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class LedsIOCANdle implements LedsIO {
  private final CANdle ledController;

  public LedsIOCANdle() {
    ledController = new CANdle(LedsConstants.LED_CONTROLLER_ID);
    ledController.animate(LedsConstants.DISABLED_ANIMATION);
    ledController.configLEDType(LEDStripType.GRB);
    ledController.configV5Enabled(true);
  }

  @Override
  public void setAnimation(Animation animation) {
    ledController.animate(animation);
  }
}
