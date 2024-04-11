// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.climb.Climb;

public class SetClimbPosition extends DebugCommandBase {
  private Climb climber;
  private double position;

  public SetClimbPosition(Climb climber, double position) {
    this.climber = climber;
    this.position = position;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setLeftMotor(position);
  }

  @Override
  public void execute() {
    super.execute();
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    climber.setLeftOutputVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return climber.getLeftPosition() > this.position;
  }
}
