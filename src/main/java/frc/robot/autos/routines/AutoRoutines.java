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
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Util;

public class AutoRoutines {

  public static Command boxAuto(CommandSwerveDrivetrain swerve) {
    ChoreoTrajectory boxPath = Choreo.getTrajectory("BoxPath");
    return Commands.sequence(
        AutoHelperCommands.resetPose(boxPath, swerve), swerve.runChoreoTraj(boxPath));
  }

  public static Command ampWing1Note(
      CommandSwerveDrivetrain swerve,
      Intake intake,
      Shooter shooter,
      PivotShooter pivotShooter,
      PivotIntake pivotIntake) {
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

  public static Command sourceCenter2(
      CommandSwerveDrivetrain swerve,
      Intake intake,
      Shooter shooter,
      PivotShooter pivotShooter,
      PivotIntake pivotIntake,
      Vision vision) {
    ChoreoTrajectory source_c5 = Choreo.getTrajectory("Source-C5");
    ChoreoTrajectory c5_source = Choreo.getTrajectory("C5-Source");
    ChoreoTrajectory source_c4 = Choreo.getTrajectory("Source-C4");
    ChoreoTrajectory c4_source = Choreo.getTrajectory("C4-Source");
    Trigger noteOuttaken =
        new Trigger(() -> !intake.isBeamBroken()).debounce(RoutineConstants.beamBreakDelay);

    return Commands.sequence(
        AutoHelperCommands.resetPose(source_c5, swerve),
        AutoHelperCommands.preLoad(pivotShooter, intake, shooter, noteOuttaken),
        AutoHelperCommands.intakeIn(5, intake, swerve, pivotIntake, pivotShooter, source_c5),
        AutoHelperCommands.shootSubwoofer(
            intake, shooter, swerve, pivotShooter, c5_source, noteOuttaken),
        AutoHelperCommands.intakeIn(4, intake, swerve, pivotIntake, pivotShooter, source_c4),
        AutoHelperCommands.shootSubwoofer(
            intake, shooter, swerve, pivotShooter, c4_source, noteOuttaken));
  }

  public static Command center5Note2( // if we use this during comp we're cooked
      CommandSwerveDrivetrain swerve,
      Intake intake,
      Shooter shooter,
      PivotShooter pivotShooter,
      PivotIntake pivotIntake,
      Vision vision) {
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
    ChoreoTrajectory w1_w2 = Choreo.getTrajectory("W1-W2");
    ChoreoTrajectory w2_w3 = Choreo.getTrajectory("W2-W3");
    ChoreoTrajectory w3_c1 = Choreo.getTrajectory("W3-C1");
    Trigger noteOuttaken =
        new Trigger(() -> !intake.isBeamBroken()).debounce(RoutineConstants.beamBreakDelay);

    return Commands.sequence(
        AutoHelperCommands.resetPose(center_w1, swerve),
        AutoHelperCommands.preLoad(pivotShooter, intake, shooter, noteOuttaken),
        AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter, center_w2, 0.5),
        AutoHelperCommands.shootSubwoofer(
            intake, shooter, swerve, pivotShooter, w2_center, noteOuttaken),
        AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter, center_w1, 0.5),
        AutoHelperCommands.shootSubwoofer(
            intake, shooter, swerve, pivotShooter, w1_center, noteOuttaken),
        AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter, center_w3, 0.5),
        AutoHelperCommands.shootSubwoofer(
            intake, shooter, swerve, pivotShooter, w3_center, noteOuttaken),
        AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter, center_c1, 0.5),
        Commands.either(
            AutoHelperCommands.shootSubwoofer(
                intake, shooter, swerve, pivotShooter, c1_center, noteOuttaken),
            Commands.sequence(
                AutoHelperCommands.intakeInDeadlineTraj(
                    intake, swerve, pivotIntake, pivotShooter, c1_c2),
                Commands.either(
                    AutoHelperCommands.shootSubwoofer(
                        intake, shooter, swerve, pivotShooter, c2_center, noteOuttaken),
                    Commands.sequence(
                        AutoHelperCommands.intakeInDeadlineTraj(
                            intake, swerve, pivotIntake, pivotShooter, c2_c3),
                        AutoHelperCommands.shootSubwoofer(
                            intake, shooter, swerve, pivotShooter, c3_center, noteOuttaken)),
                    intake::isBeamBroken)),
            intake::isBeamBroken));
  }

  public static Command sourceMobility(
      CommandSwerveDrivetrain swerve,
      Intake intake,
      Shooter shooter,
      PivotShooter pivotShooter,
      PivotIntake pivotIntake) {

    ChoreoTrajectory amp_mobility = Choreo.getTrajectory("Source");
    Trigger noteOuttaken =
        new Trigger(() -> !intake.isBeamBroken()).debounce(RoutineConstants.beamBreakDelay);
    return Commands.sequence(
        AutoHelperCommands.resetPose(amp_mobility, swerve),
        AutoHelperCommands.preLoad(pivotShooter, intake, shooter, noteOuttaken),
        swerve.runChoreoTraj(amp_mobility));
  }

  public static Command ampMobility(
      CommandSwerveDrivetrain swerve,
      Intake intake,
      Shooter shooter,
      PivotShooter pivotShooter,
      PivotIntake pivotIntake) {

    ChoreoTrajectory amp_mobility = Choreo.getTrajectory("ampMobility");
    Trigger noteOuttaken =
        new Trigger(() -> !intake.isBeamBroken()).debounce(RoutineConstants.beamBreakDelay);
    return Commands.sequence(
        AutoHelperCommands.resetPose(amp_mobility, swerve),
        AutoHelperCommands.preLoad(pivotShooter, intake, shooter, noteOuttaken),
        Commands.waitSeconds(5),
        swerve.runChoreoTraj(amp_mobility));
  }

  public static Command center5Note( // if we use this during comp we're cooked
      CommandSwerveDrivetrain swerve,
      Intake intake,
      Shooter shooter,
      PivotShooter pivotShooter,
      PivotIntake pivotIntake,
      Vision vision) {
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
    ChoreoTrajectory w1_w2 = Choreo.getTrajectory("W1-W2");
    ChoreoTrajectory w2_w3 = Choreo.getTrajectory("W2-W3");
    ChoreoTrajectory w3_c1 = Choreo.getTrajectory("W3-C1");
    Trigger noteOuttaken =
        new Trigger(() -> !intake.isBeamBroken()).debounce(RoutineConstants.beamBreakDelay);

    return Commands.sequence(
        AutoHelperCommands.resetPose(center_w1, swerve),
        AutoHelperCommands.preLoad(pivotShooter, intake, shooter, noteOuttaken),
        AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter, center_w1, 0.5),
        Commands.either(
            AutoHelperCommands.shootSubwoofer(
                    intake, shooter, swerve, pivotShooter, w1_center, noteOuttaken)
                .andThen(
                    AutoHelperCommands.intakeIn(
                        intake, swerve, pivotIntake, pivotShooter, center_w2, 0.5)),
            AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter, w1_w2, 0.5),
            intake::isBeamBroken),
        Commands.either(
            AutoHelperCommands.shootSubwoofer(
                    intake, shooter, swerve, pivotShooter, w2_center, noteOuttaken)
                .andThen(
                    AutoHelperCommands.intakeIn(
                        intake, swerve, pivotIntake, pivotShooter, center_w3, 0.5)),
            AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter, w2_w3, 0.5),
            intake::isBeamBroken),
        Commands.either(
            AutoHelperCommands.shootSubwoofer(
                    intake, shooter, swerve, pivotShooter, w3_center, noteOuttaken)
                .andThen(
                    AutoHelperCommands.intakeIn(
                        intake, swerve, pivotIntake, pivotShooter, center_c1, 0.5)),
            AutoHelperCommands.intakeInDeadlineTraj(
                intake, swerve, pivotIntake, pivotShooter, w3_c1, 1),
            intake::isBeamBroken),
        Commands.either(
            AutoHelperCommands.shootSubwoofer(
                intake, shooter, swerve, pivotShooter, c1_center, noteOuttaken),
            Commands.sequence(
                AutoHelperCommands.intakeInDeadlineTraj(
                    intake, swerve, pivotIntake, pivotShooter, c1_c2),
                Commands.either(
                    AutoHelperCommands.shootSubwoofer(
                        intake, shooter, swerve, pivotShooter, c2_center, noteOuttaken),
                    Commands.sequence(
                        AutoHelperCommands.intakeInDeadlineTraj(
                            intake, swerve, pivotIntake, pivotShooter, c2_c3),
                        AutoHelperCommands.shootSubwoofer(
                            intake, shooter, swerve, pivotShooter, c3_center, noteOuttaken)),
                    intake::isBeamBroken)),
            intake::isBeamBroken));
  }

  public static Command noteDetectionRush(
      CommandSwerveDrivetrain swerve,
      Intake intake,
      PivotIntake pivotIntake,
      PivotShooter pivotShooter,
      Shooter shooter,
      Vision vision) {
    ChoreoTrajectory amp_c1 = Choreo.getTrajectory("Amp-C1");
    ChoreoTrajectory c1_c2_feed = Choreo.getTrajectory("C1-C2_feed");
    ChoreoTrajectory c1_center = Choreo.getTrajectory("C1-Center");
    ChoreoTrajectory c2_amp_preload = Choreo.getTrajectory("C2-Amp_preload");
    ChoreoTrajectory amp_preload_center = Choreo.getTrajectory("Amp_preload-center");
    Trigger noteOuttaken =
        new Trigger(() -> !intake.isBeamBroken()).debounce(RoutineConstants.beamBreakDelay);

    return Commands.sequence(
        AutoHelperCommands.resetPose(amp_c1, swerve),
        AutoHelperCommands.noteDetectionIntake(
            swerve, intake, pivotIntake, pivotShooter, vision, amp_c1),
        swerve.runChoreoTraj(c1_center));
  }

  public static Command ampFeed1Sub1Pre1(
      CommandSwerveDrivetrain swerve,
      Intake intake,
      PivotIntake pivotIntake,
      PivotShooter pivotShooter,
      Shooter shooter) {
    ChoreoTrajectory amp_c1 = Choreo.getTrajectory("Amp-C1");
    ChoreoTrajectory c1_c2 = Choreo.getTrajectory("C1-C2");
    ChoreoTrajectory c2_center = Choreo.getTrajectory("C2-Center_ampside");
    ChoreoTrajectory center_amp = Choreo.getTrajectory("Center_ampside-Amp_preload");
    ChoreoTrajectory amp_center = Choreo.getTrajectory("Amp_preload-Center");
    ChoreoTrajectory c2_amp_preload = Choreo.getTrajectory("C2-Amp_preload");
    ChoreoTrajectory amp_preload_center = Choreo.getTrajectory("Amp_preload-center");
    Trigger noteOuttaken =
        new Trigger(() -> !intake.isBeamBroken()).debounce(RoutineConstants.beamBreakDelay);

    return Commands.sequence(
        AutoHelperCommands.resetPose(amp_c1, swerve),
        // swerve.runChoreoTraj(amp_c1),
        // swerve.runChoreoTraj(c1_c2),
        // swerve.runChoreoTraj(c2_center),
        // swerve.runChoreoTraj(center_amp),
        // swerve.runChoreoTraj(amp_center));
        AutoHelperCommands.dropPreloadAndFeed(
            shooter, pivotIntake, pivotShooter, intake, swerve, amp_c1, noteOuttaken),
        AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter, c1_c2),
        Commands.either(
            Commands.sequence(
                AutoHelperCommands.shootSubwoofer(
                    intake, shooter, swerve, pivotShooter, c2_center, noteOuttaken),
                AutoHelperCommands.intakeIn(intake, swerve, pivotIntake, pivotShooter, center_amp),
                AutoHelperCommands.shootSubwoofer(
                    intake, shooter, swerve, pivotShooter, amp_center, noteOuttaken)),
            Commands.sequence(
                AutoHelperCommands.intakeIn(
                    intake, swerve, pivotIntake, pivotShooter, c2_amp_preload),
                AutoHelperCommands.shootSubwoofer(
                    intake, shooter, swerve, pivotShooter, amp_preload_center, noteOuttaken)),
            intake::isBeamBroken));
  }

  private static class AutoHelperCommands {

    public static Command noteDetectionIntake(
        CommandSwerveDrivetrain swerve,
        Intake intake,
        PivotIntake pivotIntake,
        PivotShooter pivotShooter,
        Vision vision,
        ChoreoTrajectory traj) {
      return intake
          .intakeIn()
          .deadlineWith(
              swerve.runChoreoTraj(traj).andThen(swerve.pidToNote(vision)),
              pivotShooter.setPosition(0),
              pivotIntake.setPosition(
                  PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing))
          .withTimeout(3);
    }

    public static Command dropPreloadAndFeed(
        Shooter shooter,
        PivotIntake pivotIntake,
        PivotShooter pivotShooter,
        Intake intake,
        CommandSwerveDrivetrain swerve,
        ChoreoTrajectory traj,
        Trigger noteOuttaken) {
      return swerve
          .runChoreoTraj(traj)
          .andThen(Commands.waitSeconds(1))
          .deadlineWith(
              intake
                  .setPassthroughVoltage(-IntakeConstants.kPassthroughIntakeVoltage)
                  .until(noteOuttaken.debounce(0.5))
                  .andThen(
                      Commands.parallel(
                          intake.setVoltage(
                              IntakeConstants.kIntakeIntakeVoltage,
                              IntakeConstants.kPassthroughIntakeVoltage),
                          pivotShooter.setPosition(
                              PivotShooterConstants.kFeederPreset
                                  * PivotShooterConstants.kPivotMotorGearing),
                          shooter.setVelocity(
                              ShooterConstants.kShooterFeederRPS,
                              ShooterConstants.kShooterFollowerFeederRPS),
                          pivotIntake.setPosition(
                              PivotIntakeConstants.kPivotGroundPos
                                  * PivotIntakeConstants.kPivotMotorGearing))));
    }

    public static Command feed(
        Shooter shooter,
        PivotIntake pivotIntake,
        PivotShooter pivotShooter,
        Intake intake,
        CommandSwerveDrivetrain swerve,
        ChoreoTrajectory traj) {
      return swerve
          .runChoreoTraj(traj)
          .andThen(Commands.waitSeconds(1))
          .deadlineWith(
              Commands.parallel(
                  intake.setIntakeVelocityPassthroughVoltage(
                      80, IntakeConstants.kPassthroughIntakeVoltage),
                  pivotShooter.setPosition(
                      PivotShooterConstants.kFeederPreset
                          * PivotShooterConstants.kPivotMotorGearing),
                  shooter.setVelocity(
                      ShooterConstants.kShooterFeederRPS,
                      ShooterConstants.kShooterFollowerFeederRPS),
                  pivotIntake.setPosition(
                      PivotIntakeConstants.kPivotGroundPos
                          * PivotIntakeConstants.kPivotMotorGearing)));
    }

    public static Command preLoad(
        PivotShooter pivotShooter, Intake intake, Shooter shooter, Trigger noteOuttaken) {
      return Commands.parallel(
              Commands.waitUntil(
                      () ->
                          Util.epsilonEquals(
                              pivotShooter.getPosition(),
                              PivotShooterConstants.kSubWooferPreset
                                  * PivotShooterConstants.kPivotMotorGearing,
                              5))
                  .andThen(
                      Commands.waitUntil(() -> Util.epsilonEquals(shooter.getVelocity(), 60, 5)))
                  .andThen(intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)),
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS),
              pivotShooter.setPosition(
                  PivotShooterConstants.kSubWooferPreset
                      * PivotShooterConstants.kPivotMotorGearing))
          .until(noteOuttaken)
          .withTimeout(3);
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
      return swerve
          .runChoreoTraj(traj)
          .andThen(Commands.waitSeconds(1))
          .deadlineWith(
              intake.intakeIn(),
              pivotShooter.setPosition(0),
              pivotIntake.setPosition(
                  PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing))
          .withTimeout(5);
    }

    public static Command intakeInDeadlineTraj(
        Intake intake,
        CommandSwerveDrivetrain swerve,
        PivotIntake pivotIntake,
        PivotShooter pivotShooter,
        ChoreoTrajectory traj,
        double waitSeconds) {
      return swerve
          .runChoreoTraj(traj)
          .andThen(Commands.waitSeconds(1))
          .deadlineWith(
              Commands.waitSeconds(waitSeconds).andThen(intake.intakeIn()),
              pivotShooter.setPosition(0),
              pivotIntake.setPosition(
                  PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing))
          .withTimeout(5);
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
                  PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing))
          .withTimeout(3);
    }

    public static Command intakeIn(
        double timeout,
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
                  PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing))
          .withTimeout(timeout);
    }

    public static Command intakeIn(
        Intake intake,
        CommandSwerveDrivetrain swerve,
        PivotIntake pivotIntake,
        PivotShooter pivotShooter,
        ChoreoTrajectory traj,
        double waitSeconds) {
      return Commands.waitSeconds(waitSeconds)
          .andThen(intake.intakeIn())
          .deadlineWith(
              swerve.runChoreoTraj(traj),
              pivotShooter.setPosition(0),
              pivotIntake.setPosition(
                  PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing))
          .withTimeout(3);
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
          .until(noteOuttaken.debounce(2))
          .withTimeout(5);
    }
  }
}
