// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.climb.Climb;

public class UpClimb extends DebugCommandBase {
  private Climb climber;

  public UpClimb(Climb climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setLeftMotor(150);
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(climber.getLeftCurrent()) > 20) {
      // climber.off();
      return true;
    }
    return false;
  }
}
