// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.led.IndicatorAnimation;
import frc.robot.subsystems.led.LED;

public class SetLEDsFromBinaryString extends DebugCommandBase {
  private LED led;
  private String[] ledStates;
  private int r;
  private int g;
  private int b;
  private int w;

  public SetLEDsFromBinaryString(LED led, String[] ledStates, int r, int g, int b, int w) {
    this.led = led;
    this.ledStates = ledStates;
    this.r = r;
    this.g = g;
    this.b = b;
    this.w = w;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    super.initialize();
    new CoordinatesButItsMultiple(led, getCoordinates(ledStates), r, g, b, w).schedule();
  }

  @Override
  public void execute() {
    super.execute();
  }

  private int[][] getCoordinates(String[] ledStates) {

    int[][] coordinates = new int[ledStates.length][2];
    int height = ledStates.length - 1;
    for (int y = 0; y < ledStates.length; y++) {
      String row = ledStates[y];
      for (int x = 0; x < row.length(); x++) {
        if (row.charAt(x) == '1') {
          coordinates[y] = new int[] {x, height - y};
        }
      }
    }
    return coordinates;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    led.animate(IndicatorAnimation.Default);
  }
}
