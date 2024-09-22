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

  public static Command boxAuto(CommandSwerveDrivetrain swerve) {
    ChoreoTrajectory boxPath = Choreo.getTrajectory("BoxPath");
    return Commands.sequence(AutoHelperCommands.resetPose(boxPath, swerve), swerve.runChoreoTraj(boxPath));
  }

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
        AutoHelperCommands.resetPose(center_w1, swerve),
        AutoHelperCommands.preLoad(pivotShooter, intake, shooter, noteOuttaken),
        AutoHelperCommands.intakeIn(intake, swerve, pivotIntake,pivotShooter,  center_w1),
        AutoHelperCommands.shootSubwoofer(
            intake, shooter, swerve, pivotShooter, w1_center, noteOuttaken),
        AutoHelperCommands.intakeIn(intake, swerve, pivotIntake,pivotShooter ,center_w2),
        AutoHelperCommands.shootSubwoofer(
            intake, shooter, swerve, pivotShooter, w2_center, noteOuttaken),
        AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter,center_w3),
        AutoHelperCommands.shootSubwoofer(
            intake, shooter, swerve, pivotShooter, w3_center, noteOuttaken),
        AutoHelperCommands.intakeInDeadlineTraj(intake, swerve, pivotIntake, pivotShooter,center_c1),
        Commands.either(
            AutoHelperCommands.shootSubwoofer(
                intake, shooter, swerve, pivotShooter, c1_center, noteOuttaken),
            Commands.sequence(
                AutoHelperCommands.intakeInDeadlineTraj(intake, swerve, pivotIntake, pivotShooter,c1_c2),
                Commands.either(
                    AutoHelperCommands.shootSubwoofer(
                        intake, shooter, swerve, pivotShooter, c2_center, noteOuttaken),
                    Commands.sequence(
                        AutoHelperCommands.intakeInDeadlineTraj(intake, swerve, pivotIntake,pivotShooter, c2_c3),
                        AutoHelperCommands.shootSubwoofer(
                            intake, shooter, swerve, pivotShooter, c3_center, noteOuttaken)),
                    intake::isBeamBroken)),
            intake::isBeamBroken));
  }

  private static class AutoHelperCommands {
    public static Command preLoad(
        PivotShooter pivotShooter, Intake intake, Shooter shooter, Trigger noteOuttaken) {
      return Commands.parallel(
              Commands.waitUntil(
                      () ->
                          Util.epsilonEquals(
                              pivotShooter.getPosition(),
                              PivotShooterConstants.kSubWooferPreset
                                  * PivotShooterConstants.kPivotMotorGearing,
                              5)).andThen(Commands.waitUntil(()->Util.epsilonEquals(shooter.getVelocity(), 60,5)))
                  .andThen(intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)),
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS),
              pivotShooter.setPosition(
                  PivotShooterConstants.kSubWooferPreset
                      * PivotShooterConstants.kPivotMotorGearing))
          .until(noteOuttaken).withTimeout(3);
    }

    public static Command resetPose(ChoreoTrajectory trajectory, CommandSwerveDrivetrain swerve) {
      return Commands.runOnce(
          () ->
              swerve.seedFieldRelative(
                  DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                          == DriverStation.Alliance.Blue
                      ? trajectory.getInitialPose()
                      : trajectory.getFlippedInitialPose()));
    }

    public static Command intakeInDeadlineTraj(
            Intake intake,
            CommandSwerveDrivetrain swerve,
            PivotIntake pivotIntake,
            PivotShooter pivotShooter,
            ChoreoTrajectory traj) {
      return swerve.runChoreoTraj(traj).andThen(Commands.waitSeconds(1))
              .deadlineWith(
                      intake.intakeIn(),
                      pivotShooter.setPosition(0),
                      pivotIntake.setPosition(
                              PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing)).withTimeout(3);
    }

    public static Command intakeIn(
        Intake intake,
        CommandSwerveDrivetrain swerve,
        PivotIntake pivotIntake,
        PivotShooter pivotShooter,
        ChoreoTrajectory traj) {
      return intake
          .intakeIn()
          .deadlineWith(
              swerve.runChoreoTraj(traj),
              pivotShooter.setPosition(0),
              pivotIntake.setPosition(
                  PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing)).withTimeout(3);
    }

    public static Command shootSubwoofer(
        Intake intake,
        Shooter shooter,
        CommandSwerveDrivetrain swerve,
        PivotShooter pivotShooter,
        ChoreoTrajectory traj,
        Trigger noteOuttaken) {
      return Commands.parallel(
              Commands.sequence(
                  swerve.runChoreoTraj(traj),
                  Commands.waitUntil(
                      () ->
                          Util.epsilonEquals(
                              pivotShooter.getPosition(),
                              PivotShooterConstants.kSubWooferPreset
                                  * PivotShooterConstants.kPivotMotorGearing,
                              5)),
                  intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)),
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS),
              pivotShooter.setPosition(
                  PivotShooterConstants.kSubWooferPreset
                      * PivotShooterConstants.kPivotMotorGearing))
          .until(noteOuttaken.debounce(1)).withTimeout(4);
    }
  }
}
