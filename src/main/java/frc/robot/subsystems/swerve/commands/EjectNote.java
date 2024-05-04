// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.swerve.SwerveDrive;

public class EjectNote extends DebugCommandBase {
  private SwerveDrive swerveSubsystem;
  private PivotIntake pivot;
  private Intake intake;

  public EjectNote(SwerveDrive swerveSubsystem, PivotIntake pivot, Intake intake) {
    this.swerveSubsystem = swerveSubsystem;
    this.pivot = pivot;
    this.intake = intake;
  }

  @Override
  public void execute() {
    // new ParallelCommandGroup(new IntakeIn(intake), new PivotIntakeSlamAndVoltage(pivot));
    intake.intakeIn().withTimeout(2);
    swerveSubsystem.setAngularVelocity(10); // 10 rads/sec rotation
  }
}
