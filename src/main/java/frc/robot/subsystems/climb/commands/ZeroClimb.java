// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.climb.Climb;

public class ZeroClimb extends DebugCommandBase {
  private Climb climber;

  public ZeroClimb(Climb climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    super.initialize();
    System.out.println("climb zeroed");
    climber.leftArmZero();
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
