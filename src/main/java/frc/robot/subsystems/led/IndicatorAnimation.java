// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.Constants;
import java.util.function.Consumer;

public enum IndicatorAnimation {
  /*
   * DefaultConfig(
   * new LarsonAnimation(
   * 255,
   * 0,
   * 255,
   * 1,
   * Constants.LEDConstants.kLEDSpeed,
   * Constants.LEDConstants.kLEDLength,
   * LarsonAnimation.BounceMode.Center,
   * 6)),
   * IntakeIsOpenAndRunningConfig(
   * new SingleFadeAnimation(
   * 255, 150, 0, 1, Constants.LEDConstants.kLEDSpeed / 2,
   * Constants.LEDConstants.kLEDLength)),
   */
  RobotAligned(new RainbowAnimation()),
  Default(new LarsonAnimation(255, 0, 255, 1, 0.50, 264, LarsonAnimation.BounceMode.Center, 6)),
  // Default(new RainbowAnimation(0.78, 0.24, 264)),
  IntakeIsOpenAndRunning(new StrobeAnimation(255, 0, 0, 127, 0.1, 264)),
  NoteIntaken(new StrobeAnimation(0, 255, 0, 127, 0.1, 264)),
  SpeakerScore(new StrobeAnimation(255, 165, 0, 127, 0.1, 264)),
  AmpScore(new StrobeAnimation(255, 165, 0, 127, 0.1, 264)),
  AzimuthRan(new StrobeAnimation(255, 255, 0, 127, 0.3, 264)),
  BeamBreakTriggered(new SingleFadeAnimation(0, 100, 0, 5, 0.5, 264));

  public final Consumer<CANdle> value;

  private IndicatorAnimation(Consumer<CANdle> animation) {
    this.value = animation;
  }

  private IndicatorAnimation(Animation animation) {
    this.value =
        (CANdle candle) -> {
          ErrorCode error = candle.animate(animation);
          if (error != ErrorCode.OK && Constants.FeatureFlags.kDebugEnabled) {
            System.out.println("CANdle Error (Unable to set animation). Error Code: " + error);
            System.out.println("Traceback (most recent call last)");
            for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
              System.out.println(ste + "\n");
            }
          }
        };
  }
}
