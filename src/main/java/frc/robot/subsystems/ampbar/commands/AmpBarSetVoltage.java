// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.ampbar.AmpBar;

public class AmpBarSetVoltage extends DebugCommandBase {
  private AmpBar ampBar;
  private final double voltage;

  public AmpBarSetVoltage(AmpBar ampBar, double voltage) {
    this.ampBar = ampBar;
    this.voltage = voltage;
    addRequirements(ampBar);
  }

  public void initialize() {
    ampBar.setOutputVoltage(voltage);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
