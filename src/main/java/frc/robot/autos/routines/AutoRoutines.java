// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos.routines;

import com.choreo.lib.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.pivotintake.PivotIntakeConstants;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.Util;

public class AutoRoutines {
  public static Command ampWing1Note(
      CommandSwerveDrivetrain swerve,
      Intake intake,
      Shooter shooter,
      PivotShooter pivotShooter,
      PivotIntake
          pivotIntake) { // This is an example on how to make a choreo auto, I have no clue if its
    // gonna work
    return Commands.sequence(
        swerve
            .runChoreoTraj(Choreo.getTrajectory("AmpSingleNote.1"))
            .alongWith(
                pivotIntake
                    .setPosition(PivotIntakeConstants.kPivotGroundPos)
                    .andThen(intake.intakeIn().andThen(pivotIntake.setPosition(0)))),
        shooter
            .setVelocity(
                ShooterConstants.kShooterSubwooferRPS,
                ShooterConstants.kShooterFollowerSubwooferRPS)
            .alongWith(
                pivotShooter
                    .setPosition(PivotShooterConstants.kSubWooferPreset)
                    .andThen(
                        intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage))));
  }

  public static Command center5Note(
      CommandSwerveDrivetrain swerve,
      Intake intake,
      Shooter shooter,
      PivotShooter pivotShooter,
      PivotIntake pivotIntake) {
    ChoreoTrajectory c1_c2 = Choreo.getTrajectory("C1-C2");
    ChoreoTrajectory c2_c3 = Choreo.getTrajectory("C2-C3");
    ChoreoTrajectory c1_center = Choreo.getTrajectory("C1-Center");
    ChoreoTrajectory c2_center = Choreo.getTrajectory("C2-Center");
    ChoreoTrajectory c3_center = Choreo.getTrajectory("C3-Center");
    ChoreoTrajectory center_c1 = Choreo.getTrajectory("Center-C1");
    ChoreoTrajectory center_w1 = Choreo.getTrajectory("Center-W1");
    ChoreoTrajectory center_w2 = Choreo.getTrajectory("Center-W2");
    ChoreoTrajectory center_w3 = Choreo.getTrajectory("Center-W3");
    ChoreoTrajectory w1_center = Choreo.getTrajectory("W1-Center");
    ChoreoTrajectory w2_center = Choreo.getTrajectory("W2-Center");
    ChoreoTrajectory w3_center = Choreo.getTrajectory("W3-Center");
    Trigger noteOuttaken =
        new Trigger(() -> !intake.isBeamBroken()).debounce(RoutineConstants.beamBreakDelay);

    return Commands.sequence(
        Commands.runOnce(
            () ->
                swerve.seedFieldRelative(
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                            == DriverStation.Alliance.Blue
                        ? center_w1.getInitialPose()
                        : center_w1.getFlippedInitialPose())),
        intake.intakeIn()
                .deadlineWith(
                        swerve.runChoreoTraj(center_w1),
                        pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos*PivotIntakeConstants.kPivotMotorGearing)),
        Commands.parallel(
                Commands.sequence(
                    swerve.runChoreoTraj(w1_center),
                    Commands.waitUntil(
                        () ->
                            Util.epsilonEquals(
                                pivotShooter.getPosition(),
                                PivotShooterConstants.kSubWooferPreset *PivotShooterConstants.kPivotMotorGearing,
                                5)),
                        intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)),

                shooter.setVelocity(
                    ShooterConstants.kShooterSubwooferRPS,
                    ShooterConstants.kShooterFollowerSubwooferRPS),
                pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing))
            .until(noteOuttaken),
            intake.intakeIn()
                    .deadlineWith(
                            pivotShooter.setPosition(0),
                            swerve.runChoreoTraj(center_w2),
                            pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos*PivotIntakeConstants.kPivotMotorGearing)),
        Commands.parallel(
                Commands.sequence(
                    swerve.runChoreoTraj(w2_center),
                    Commands.waitUntil(
                        () ->
                            Util.epsilonEquals(
                                pivotShooter.getPosition(),
                                PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing,
                                5)),
                        intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)),
                shooter.setVelocity(
                    ShooterConstants.kShooterSubwooferRPS,
                    ShooterConstants.kShooterFollowerSubwooferRPS),
                pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing))
            .until(noteOuttaken),
            intake.intakeIn()
                    .deadlineWith(
                            pivotShooter.setPosition(0),
                            swerve.runChoreoTraj(center_w3),
                            pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos*PivotIntakeConstants.kPivotMotorGearing)),
        Commands.parallel(
                Commands.sequence(
                    swerve.runChoreoTraj(w3_center),
                    Commands.waitUntil(
                        () ->
                            Util.epsilonEquals(
                                pivotShooter.getPosition(),
                                PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing,
                                5)),
                        intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)),

                shooter.setVelocity(
                    ShooterConstants.kShooterSubwooferRPS,
                    ShooterConstants.kShooterFollowerSubwooferRPS),
                pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing))
            .until(noteOuttaken),
            swerve.runChoreoTraj(center_c1).andThen(Commands.waitSeconds(1))
                            .deadlineWith(pivotShooter.setPosition(0),intake.intakeIn(), pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos*PivotIntakeConstants.kPivotMotorGearing)),
        Commands.either(
            Commands.parallel(
                    Commands.sequence(
                        swerve.runChoreoTraj(c1_center),
                        Commands.waitUntil(
                            () ->
                                Util.epsilonEquals(
                                    pivotShooter.getPosition(),
                                    PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing,
                                    5)),
                            intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)),

                    shooter.setVelocity(
                        ShooterConstants.kShooterSubwooferRPS,
                        ShooterConstants.kShooterFollowerSubwooferRPS),
                    pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing))
                .until(noteOuttaken),
            Commands.sequence(
                swerve
                    .runChoreoTraj(c1_c2)
                    .deadlineWith(
                            pivotShooter.setPosition(0),
                        pivotIntake
                            .setPosition(PivotIntakeConstants.kPivotGroundPos*PivotIntakeConstants.kPivotMotorGearing)
                            .alongWith(intake.intakeIn())),
                Commands.either(
                    Commands.parallel(
                            Commands.sequence(
                                swerve.runChoreoTraj(c2_center),
                                Commands.waitUntil(
                                    () ->
                                        Util.epsilonEquals(
                                            pivotShooter.getPosition(),
                                            PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing,
                                            5)),
                                    intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)),
                            shooter.setVelocity(
                                ShooterConstants.kShooterSubwooferRPS,
                                ShooterConstants.kShooterFollowerSubwooferRPS),
                            pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing))
                        .until(noteOuttaken),
                    Commands.sequence(
                        swerve
                            .runChoreoTraj(c2_c3)
                            .deadlineWith(
                                    pivotShooter.setPosition(0),
                                pivotIntake
                                    .setPosition(PivotIntakeConstants.kPivotGroundPos*PivotIntakeConstants.kPivotMotorGearing)
                                    .alongWith(intake.intakeIn())),
                        Commands.parallel(
                                Commands.sequence(
                                    swerve.runChoreoTraj(c3_center),
                                    Commands.waitUntil(
                                        () ->
                                            Util.epsilonEquals(
                                                pivotShooter.getPosition(),
                                                PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing,
                                                5)),
                                        intake.setPassthroughVoltage(
                                                IntakeConstants.kPassthroughIntakeVoltage)),

                                shooter.setVelocity(
                                    ShooterConstants.kShooterSubwooferRPS,
                                    ShooterConstants.kShooterFollowerSubwooferRPS),
                                pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset*PivotShooterConstants.kPivotMotorGearing))
                            .until(noteOuttaken)),
                    intake::isBeamBroken)),
            intake::isBeamBroken));
  }
}
