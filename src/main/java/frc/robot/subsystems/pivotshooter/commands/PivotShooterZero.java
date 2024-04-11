// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.pivotshooter.PivotShooter;

public class PivotShooterZero extends DebugCommandBase {
  private PivotShooter pivotShooter;

  public PivotShooterZero(PivotShooter pivotShooter) {
    this.pivotShooter = pivotShooter;
    // addRequirements(pivotIntake);
  }

  public void initialize() {
    pivotShooter.zero();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
