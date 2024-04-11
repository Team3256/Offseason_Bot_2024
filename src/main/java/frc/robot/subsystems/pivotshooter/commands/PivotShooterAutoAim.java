// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.autos.AutoConstants;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import java.util.Optional;
import java.util.function.Supplier;

public class PivotShooterAutoAim extends DebugCommandBase {
  PivotShooter pivotShooter;

  Supplier<Pose2d> targetPose;

  private Pose2d scoringLocation;

  public PivotShooterAutoAim(PivotShooter pivotShooter, Supplier<Pose2d> targetPose) {
    this.pivotShooter = pivotShooter;
    this.targetPose = targetPose;
    addRequirements(pivotShooter);
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
  }

  @Override
  public void execute() {
    Pose2d ourPose = targetPose.get();

    double distance = ourPose.getTranslation().getDistance(scoringLocation.getTranslation());
    pivotShooter.setDegrees(pivotShooter.pivotMotorData.get(distance));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
