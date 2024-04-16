// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter.commands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterConstants;
import java.util.Optional;

public class PivotShooterAutoAimSidEdition extends DebugCommandBase {
  PivotShooter pivotShooter;

  private int speakerId;
  private int speakerId2;

  public PivotShooterAutoAimSidEdition(PivotShooter pivotShooter) {
    this.pivotShooter = pivotShooter;
    addRequirements(pivotShooter);
  }

  @Override
  public void initialize() {
    Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == DriverStation.Alliance.Red) {
      speakerId = PivotShooterConstants.kSpeakerAprilTagRed;
      speakerId2 = PivotShooterConstants.kSpeakerBackupAprilTagRed;
    } else if (ally.isPresent()) {
      speakerId = PivotShooterConstants.kSpeakerAprilTagBlue;
      speakerId2 = PivotShooterConstants.kSpeakerBackupAprilTagBlue;
    } else {
      speakerId = PivotShooterConstants.kSpeakerAprilTagRed;
      speakerId2 = PivotShooterConstants.kSpeakerBackupAprilTagRed;
    }
  }

  @Override
  public void execute() {
    if (Limelight.getFiducialID("limelight") == speakerId) {
      var tx = Limelight.getTX("limelight");
      var ty = Limelight.getTY("limelight");

      pivotShooter.setDegrees(pivotShooter.pivotMotorDataNotGlobalPose.get(Math.hypot(tx, ty)));
    } else if (Limelight.getFiducialID("limelight") == speakerId2) {
      var tx = Limelight.getTX("limelight");
      var ty = Limelight.getTY("limelight");

      pivotShooter.setDegrees(pivotShooter.pivotMotorDataNotGlobalPose2.get(Math.hypot(tx, ty)));
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
