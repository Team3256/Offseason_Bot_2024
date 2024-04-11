// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.intake.Intake;

public class IntakeOff extends DebugCommandBase {
  private final Intake intakeSubsystem;

  public IntakeOff(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.off();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
