// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.ampbar.AmpBar;
import frc.robot.subsystems.ampbar.AmpBarConstants;

public class StowPosition extends DebugCommandBase {

  private AmpBar ampBar;
  private double init;

  public StowPosition(AmpBar ampBar) {
    this.ampBar = ampBar;
    addRequirements(ampBar);
  }

  public void initialize() {
    ampBar.setOutputVoltage(AmpBarConstants.kAmpBarStowVoltage);
    init = System.currentTimeMillis();
  }

  @Override
  public boolean isFinished() {
    return ampBar.isCurrentSpiking();
  }
}
