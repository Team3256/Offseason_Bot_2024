// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.ampbar.AmpBar;
import frc.robot.subsystems.ampbar.commands.StowPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeInTeleop;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.pivotintake.PivotIntakeConstants;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.commands.PivotShooterSlamAndVoltage;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ShootSpeaker;

public class IntakeSequence extends SequentialCommandGroup {
  Intake intakeSubsystem;
  PivotIntake pivotIntake;
  PivotShooter pivotShooter;
  Shooter shooter;
  AmpBar ampBar;

  public IntakeSequence(
      Intake intake, PivotIntake pivot, PivotShooter shooterPivot, Shooter shoot, AmpBar ampBar) {
    intakeSubsystem = intake;
    pivotIntake = pivot;
    pivotShooter = shooterPivot;
    shooter = shoot;
    this.ampBar = ampBar;
    addRequirements(pivotIntake, pivotShooter, ampBar);
    addCommands(
        new ParallelCommandGroup(
            pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos).withTimeout(0.75),
            new IntakeInTeleop(intakeSubsystem),
            new PivotShooterSlamAndVoltage(pivotShooter)),
        pivotIntake.slamAndPID(),
        // new PivotShootSubwoofer(shooterPivot).withTimeout(0.5),
        new ParallelCommandGroup(
            new StowPosition(ampBar),
            new ScheduleCommand(new ParallelCommandGroup(new ShootSpeaker(shoot)))));
  }
}
