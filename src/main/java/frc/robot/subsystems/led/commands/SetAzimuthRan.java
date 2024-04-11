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

public class SetAzimuthRan extends DebugCommandBase {
  LED led;

  public SetAzimuthRan(LED led) {
    this.led = led;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    led.animate(IndicatorAnimation.AzimuthRan);
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    led.animate(IndicatorAnimation.Default);
    super.end(interrupted);
  }
}
