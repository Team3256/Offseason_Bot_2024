// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.commands;

import static frc.robot.subsystems.intake.IntakeConstants.kIntakeIntakeVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kPassthroughIntakeVoltage;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.intake.Intake;

public class IntakeAndPassthrough extends DebugCommandBase {
  private final Intake intakeSubsystem;

  public IntakeAndPassthrough(Intake intake) {
    this.intakeSubsystem = intake;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.setOutputVoltagePassthrough(kPassthroughIntakeVoltage);
    intakeSubsystem.setOutputVoltageIntake(kIntakeIntakeVoltage);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
