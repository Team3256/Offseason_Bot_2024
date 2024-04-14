// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led.commands;

import static frc.robot.subsystems.led.LEDConstants.kLEDHeight;
import static frc.robot.subsystems.led.LEDConstants.kLEDWidth;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.led.IndicatorAnimation;
import frc.robot.subsystems.led.LED;

public class CoordinatesButItsMultiple extends DebugCommandBase {
  LED led;

  int[][] coordinates;

  private int r = 0;
  private int g = 0;
  private int b = 0;
  private int w = 0;

  public CoordinatesButItsMultiple(LED led, int[][] coordinates, int r, int g, int b, int w) {
    this.led = led;
    this.coordinates = coordinates;
    this.r = r;
    this.g = g;
    this.b = b;
    this.w = w;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    for (int[] coordinate : coordinates) {
      led.setLedColor(r, g, b, w, getIndex(coordinate[0], coordinate[1]), 1);
    }
    super.initialize();
  }

  private int getIndex(int x, int y) {
    x -= 1;
    if (x % 2 == 0) {
      return (kLEDWidth - x) * kLEDHeight - y + 1;
    } else {
      return (kLEDWidth - x - 1) * kLEDHeight + y + 1;
    }
  }

  @Override
  public void end(boolean interrupted) {
    led.animate(IndicatorAnimation.Default);
    super.end(interrupted);
  }
}
