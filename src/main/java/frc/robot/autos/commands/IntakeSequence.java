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
import frc.robot.subsystems.pivotintake.PivotIntakeConstants;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.pivotintake.commands.PivotIntakeSetAngle;
import frc.robot.subsystems.pivotintake.commands.PivotIntakeSlamAndVoltage;
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
    // addCommands(
    // new ParallelCommandGroup(
    // new PivotSetAngle(pivotSubsystem, PivotConstants.kPivotGroundAngleDeg)
    // .withTimeout(0.75),
    // new IntakeIn(intakeSubsystem)),
    // new ParallelDeadlineGroup(
    // new PivotSlamAndVoltage(pivotSubsystem).withTimeout(0.75),
    // new IntakeSetVoltage(intakeSubsystem, 8)),
    // new ScheduleCommand(new ShootSpeaker(shooterSubsystem)));
    addCommands(
        new ParallelCommandGroup(
            new PivotIntakeSetAngle(pivotIntake, PivotIntakeConstants.kPivotGroundAngleDeg).withTimeout(0.75),
            new IntakeInTeleop(intakeSubsystem),
            new PivotShooterSlamAndVoltage(pivotShooter)),
        new PivotIntakeSlamAndVoltage(pivotIntake).withTimeout(1),
        // new PivotShootSubwoofer(shooterPivot).withTimeout(0.5),
        new ParallelCommandGroup(
            new StowPosition(ampBar),
            new ScheduleCommand(new ParallelCommandGroup(new ShootSpeaker(shoot)))));
  }
}

// // @Override
// // public void initialize() {
// // pivotDown =
// // new PivotSetAngle(pivotSubsystem,
// // PivotConstants.kPivotGroundAngleDeg).withTimeout(0.75);
// // stow = new PivotSlamAndVoltage(pivotSubsystem).withTimeout(0.75);
// // suckkk = new IntakeIn(intakeSubsystem);
// // // spit = new IntakeOut(intakeSubsystem).withTimeout(1);
// // rumptumptumptump = new ShootSpeaker(shooterSubsystem);
// //
// // intakeSequence = Commands.sequence(pivotDown, suckkk,
// stow,rumptumptumptump);
// // intakeSequence.schedule();
// //

// // verride
// // blic void execute() {}
// //
// // verride
// // public void end(boolean interrupted) {
// // super.end(interrupted);
// // intakeSequence.cancel();
// // }
// //
// // @Override
// // public boolean isFinished() {
// // return intakeSequence.isFinished();
// // }
// }

// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// package frc.robot.autos.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ScheduleCommand;
// import frc.robot.Constants.*;
// import frc.robot.helpers.DebugCommandBase;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.commands.IntakeIn;
// import frc.robot.subsystems.pivot.PivotIntake;
// import frc.robot.subsystems.pivot.commands.PivotSetAngle;
// import frc.robot.subsystems.pivot.commands.PivotSlam;
// import frc.robot.subsystems.pivot.commands.PivotSlamAndVoltage;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.shooter.commands.ShootSpeaker;

// public class IntakeSequence extends DebugCommandBase {
// Intake intakeSubsystem;
// PivotIntake pivotSubsystem;
// Shooter shooterSubsystem;

// Command pivotDown;
// Command suckkk;
// Command stow;
// Command spit;
// Command rumptumptumptump;
// Command intakeSequence;
// Command shootSpeaker2;

// public IntakeSequence(Intake intake, PivotIntake pivot, Shooter shoot) {
// intakeSubsystem = intake;
// pivotSubsystem = pivot;
// shooterSubsystem = shoot;
// addRequirements(pivotSubsystem, shooterSubsystem);
// }

// @Override
// public void initialize() {
// pivotDown = new PivotSetAngle(pivotSubsystem,
// PivotConstants.kPivotGroundAngleDeg).withTimeout(0.75);
// stow = new PivotSlamAndVoltage(pivotSubsystem).withTimeout(0.75);
// suckkk = new IntakeIn(intakeSubsystem);
// shootSpeaker2 = new ScheduleCommand(new ShootSpeaker(shooterSubsystem));

// intakeSequence = Commands.sequence(pivotDown, suckkk, Commands.parallel(stow,
// shootSpeaker2));
// // rumptumptumptump = new ShootSpeaker(shooterSubsystem);

// // intakeSequence =
// // Commands.sequence(pivotDown, suckkk, Commands.parallel(stow,
// // rumptumptumptump));
// intakeSequence.schedule();
// }

// @Override
// public void execute() {
// }

// @Override
// public void end(boolean interrupted) {
// super.end(interrupted);
// intakeSequence.cancel();
// }

// @Override
// public boolean isFinished() {
// return intakeSequence.isFinished();
// }
// }
