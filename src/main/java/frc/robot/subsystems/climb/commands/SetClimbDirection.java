// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.climb.Climb;

public class SetClimbDirection extends DebugCommandBase {
  private Climb climber;

  boolean leftClimbDone = false;
  boolean rightClimbDone = false;
  double leftClimbInitial;
  double rightClimbInitial;
  double initialTime;

  public SetClimbDirection(Climb climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    leftClimbInitial = climber.getLeftPosition();
    climber.setLeftOutputVoltage(2);

    initialTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    super.execute();
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    if (Math.abs(climber.getLeftPosition() - leftClimbInitial) < 5) {
      climber.flipLeftUpDirection();
    }
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - initialTime > 250;
  }
}
