// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.intake.Intake;

public class IntakeSetVoltage extends DebugCommandBase {
  private final Intake intakeSubsystem;
  private final double volts;

  public IntakeSetVoltage(Intake intake, double volts) {
    this.intakeSubsystem = intake;
    this.volts = volts;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.setOutputVoltageIntake(volts);
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
