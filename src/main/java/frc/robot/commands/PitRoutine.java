// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import frc.robot.helpers.DebugCommandBase;

public class PitRoutine extends DebugCommandBase {
  // SwerveDrive swerve;
  // Climb climb;
  // Intake intake;
  // PivotIntake pivot;
  // Shooter shooter;
  // Command climbUp;
  // Command climbDown;

  // private final CommandXboxController driver = new CommandXboxController(0);

  // public PitRoutine(
  //     SwerveDrive swerve, Climb climb, Intake intake, PivotIntake pivot, Shooter shooter) {
  //   this.swerve = swerve;
  //   this.climb = climb;
  //   this.pivot = pivot;
  //   this.intake = intake;
  //   this.shooter = shooter;
  //   addRequirements(swerve, climb, pivot, intake, shooter);
  // }

  // @Override
  // public void initialize() {
  //   CommandScheduler.getInstance().cancelAll();
  //   LinkedList<Command> CommandList = new LinkedList<>();

  //   Command killAllCommands =
  //       new InstantCommand(
  //           () -> {
  //             CommandScheduler.getInstance().cancelAll();
  //           });

  //   // swerve

  //   if (Constants.FeatureFlags.kSwerveEnabled) {
  //     Command teleopSwerveBackward = new TeleopSwerve(swerve, () -> -1, () -> 0, () -> 0);
  //     Command teleopSwerveLeft = new TeleopSwerve(swerve, () -> 0, () -> -1, () -> 0);
  //     Command teleopSwerveRight = new TeleopSwerve(swerve, () -> 0, () -> 1, () -> 0);
  //     Command teleopSwerveForward = new TeleopSwerve(swerve, () -> 1, () -> 0, () -> 0);

  //     CommandList.add(teleopSwerveBackward);
  //     CommandList.add(teleopSwerveRight);
  //     CommandList.add(teleopSwerveLeft);
  //     CommandList.add(teleopSwerveForward);
  //     CommandList.add(new ZeroGyro(swerve));
  //   }

  //   // shooter
  //   if (Constants.FeatureFlags.kShooterEnabled) {
  //     Command shootSpeaker = new ShootSpeaker(shooter);
  //     Command shootAmp = new ShootAmp(shooter);
  //     Command shooterOff = new ShooterOff(shooter);
  //     CommandList.add(shootSpeaker);
  //     CommandList.add(shootAmp);
  //     CommandList.add(shooterOff);
  //   }

  //   // intake

  //   if (Constants.FeatureFlags.kIntakeEnabled) {
  //     Command intakeIn = new IntakeIn(intake);
  //     Command intakeOut = new IntakeOut(intake);
  //     CommandList.add(intakeIn);
  //     CommandList.add(intakeOut);
  //   }

  //   // pivot

  //   if (Constants.FeatureFlags.kPivotEnabled) {
  //     Command pivotDown = new PivotIntakeSetAngle(pivot, kPivotGroundAngleDeg).withTimeout(0.75);
  //     Command pivotSlam = new PivotIntakeSlamAndVoltage(pivot);

  //     CommandList.add(pivotDown);
  //     CommandList.add(pivotSlam);
  //   }

  //   // climb
  //   // if (Constants.FeatureFlags.kClimbEnabled) {
  //   // climbUp = new DownClimb(climb);
  //   // climbDown = new ReleaseClimbers(climb);
  //   // CommandList.add(climbUp);
  //   // CommandList.add(climbDown);
  //   // // }
  //   // if (Constants.FeatureFlags.kClimbEnabled) {
  //   // climbUp = new DownClimb(climb);
  //   // climbDown = new ReleaseClimbers(climb);
  //   // CommandList.add(climbUp);
  //   // CommandList.add(climbDown);
  //   // }

  //   // intakeSequence

  //   if (Constants.FeatureFlags.kIntakeEnabled
  //       && Constants.FeatureFlags.kPivotEnabled
  //       && Constants.FeatureFlags.kShooterEnabled) {
  //     //      Command intakeSequence = new IntakeSequence(intake, pivot, shooter, am);

  //     //      CommandList.add(intakeSequence);
  //   }

  //   Command nextCommandTrigger =
  //       new InstantCommand(
  //           () -> {
  //             if (!CommandList.isEmpty()) {
  //               Command nextCommand = CommandList.pop();
  //               nextCommand.schedule();
  //             }
  //           });
  //   driver.x().onTrue(nextCommandTrigger);
  //   driver.y().onTrue(killAllCommands);
  // }
}
