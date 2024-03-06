// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.vision.AprilTagVision;

public class LedController extends SubsystemBase {

  CANdle candle;
  AprilTagVision aprilTags;

  boolean hasGamePiece = false;
  boolean safeMode = false;
  boolean speakerMode = false;
  int stripLength = 33;
  int startOffset = 8;

  /** Creates a new LedController. */
  public LedController(AprilTagVision aprilTags) {
    candle = new CANdle(33);
    candle.configLEDType(LEDStripType.GRB);
    candle.configV5Enabled(true);
    this.aprilTags = aprilTags;
  }

  public void setHasGamePiece(boolean hasGamePiece) {
    this.hasGamePiece = hasGamePiece;
  }

  public void setSafeMode(boolean safeMode) {
    this.safeMode = safeMode;
  }

  public void setSpeakerMode(boolean speakerMode) {
    this.speakerMode = speakerMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      if (aprilTags.getPoseEstimationCount() == 0) {
        candle.animate(new StrobeAnimation(0, 255, 0, 0, 0.25, stripLength, startOffset));
        return;
      }
      // candle.animate(new FireAnimation(1, 0.5, -1, 0.5, 0.5));
      if (DriverStation.getAlliance().isEmpty()) {
        candle.animate(new RainbowAnimation(0.5, 0.5, stripLength, false, startOffset));
        return;
      }
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        candle.animate(
            new LarsonAnimation(150, 0, 0, 0, 0.05, stripLength, BounceMode.Front, 7, 8));
      } else {
        candle.animate(
            new LarsonAnimation(0, 0, 150, 0, 0.05, stripLength, BounceMode.Front, 7, 8));
      }
      return;
    }
    boolean isSpeakerMode =
        SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER;
    boolean isSafeMode = SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE;
    if (isSafeMode) {
      candle.animate(
          new TwinkleAnimation(
              100, 100, 100, 0, 0.5, stripLength, TwinklePercent.Percent64, startOffset));
      return;
    }
    if (hasGamePiece) {
      candle.animate(new StrobeAnimation(255, 50, 0, 0, 1, stripLength, startOffset));
    } else {
      candle.animate(new SingleFadeAnimation(0, 0, 100, 0, 1, stripLength, startOffset));
    }
  }
}
