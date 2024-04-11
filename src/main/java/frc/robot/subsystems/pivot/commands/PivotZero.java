// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.pivot.PivotIntake;

public class PivotZero extends DebugCommandBase {
  private PivotIntake pivotIntake;

  public PivotZero(PivotIntake pivotIntake) {
    this.pivotIntake = pivotIntake;
    // addRequirements(pivotIntake);
  }

  public void initialize() {
    pivotIntake.zero();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
