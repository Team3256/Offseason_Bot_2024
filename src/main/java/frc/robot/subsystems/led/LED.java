// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class LED extends SubsystemBase implements Loggable {
  public IndicatorAnimation currentAnimation = IndicatorAnimation.Default;
  private CANdle candle;

  public void DEFAULT() {}

  public LED() {
    candle = new CANdle(LEDConstants.kLEDCANID, "rio");
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = CANdle.LEDStripType.RGB;
    config.brightnessScalar = 0.25;
    candle.configAllSettings(config);
    // candle.animate(new LarsonAnimation(255, 0, 255, 1, 0.54, 64,
    // LarsonAnimation.BounceMode.Center, 6));

    // RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.67, 64);
    // TwinkleAnimation twinkleAnim = new TwinkleAnimation(255, 0, 255);
    // StrobeAnimation strobeAnim = new StrobeAnimation(255, 255, 255, 255, 0.1,
    // 64);

    this.animate(currentAnimation);
  }

  public void reset() {
    currentAnimation = IndicatorAnimation.Default;
    this.animate(currentAnimation);
  }

  public void animate(IndicatorAnimation animation) {
    currentAnimation = animation;
    currentAnimation.value.accept(candle);
    // candle.animate(IndicatorAnimation.Default.value);
  }

  public Command solidAnim(int r, int g, int b) {
    return new InstantCommand(() -> candle.setLEDs(r, g, b));
  }

  public void setLedColor(int r, int g, int b, int w, int index, int length) {
    candle.setLEDs(r, g, b, w, index, length);
  }
}
