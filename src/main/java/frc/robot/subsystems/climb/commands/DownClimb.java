// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb.commands;

import static frc.robot.subsystems.climb.ClimbConstants.gearRatio;
import static frc.robot.subsystems.climb.ClimbConstants.kClimbDownPosition;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.climb.Climb;

public class DownClimb extends DebugCommandBase {
  private Climb climber;

  public DownClimb(Climb climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    // L: times multiplied by -1 = 2
    // R: times multiplied by -1 = 2
    climber.setLeftMotor(kClimbDownPosition * gearRatio); // 80
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    // if (Math.abs(climber.getLeftCurrent()) > 80) {
    // // climber.off();
    // return true;
    // }
    return false;
  }
}
