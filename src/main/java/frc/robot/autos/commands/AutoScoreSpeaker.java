// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos.commands;

import static frc.robot.Constants.*;
import static frc.robot.Constants.FeatureFlags.kDebugEnabled;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autos.AutoConstants;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeOut;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ShootSpeaker;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Optional;

public class AutoScoreSpeaker extends DebugCommandBase {

  private final SwerveDrive swerveSubsystem;
  private final Shooter shooterSubsystem;
  private final Intake intakeSubsystem;

  private Pose2d scoringLocation;

  private Command autoScore;

  private Command moveToScoringLocation;
  private Command shootSpeaker;
  private Command intakeOut;

  private Command revShooter;

  public AutoScoreSpeaker(
      SwerveDrive swerveDrive, Shooter shooterSubsystem, Intake intakeSubsystem) {

    this.swerveSubsystem = swerveDrive;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(swerveSubsystem, shooterSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {

    Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == DriverStation.Alliance.Red) {
      scoringLocation = AutoConstants.kRedSpeakerLocation;
    } else if (ally.isPresent()) {
      scoringLocation = AutoConstants.kBlueSpeakerLocation;
    } else {
      scoringLocation = AutoConstants.kRedSpeakerLocation;
    }

    if (kDebugEnabled) {
      System.out.println("Scoring Location: " + scoringLocation);
      System.out.println("Starting Location: " + swerveSubsystem.getPose());
    }

    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    moveToScoringLocation = AutoBuilder.pathfindToPose(scoringLocation, constraints, 0.0, 0.0);
    //    shootSpeaker = new ShootSpeaker(shooterSubsystem);
    revShooter = new ShootSpeaker(shooterSubsystem);
    intakeOut = new IntakeOut(intakeSubsystem);

    autoScore = Commands.parallel(Commands.sequence(moveToScoringLocation, intakeOut), revShooter);

    autoScore.schedule();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    autoScore.cancel();
  }

  @Override
  public boolean isFinished() {

    return moveToScoringLocation.isFinished();
  }
}
