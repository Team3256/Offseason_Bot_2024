// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter.commands;

import static frc.robot.subsystems.pivotshooter.PivotingShooterConstants.kPivotMotorGearing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.autos.AutoConstants;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import java.util.Optional;

public class bruh extends DebugCommandBase {
  PivotShooter pivotShooter;

  private Pose2d lastLastRobotPose = new Pose2d();
  private Pose2d lastRobotPose = new Pose2d();

  private Pose2d scoringLocation;

  public bruh(PivotShooter pivotShooter) {
    this.pivotShooter = pivotShooter;
    addRequirements(pivotShooter);
  }

  @Override
  public void initialize() {
    Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == DriverStation.Alliance.Red) {
      scoringLocation = AutoConstants.kRedSpeakerLocation;
      LimelightHelpers.setPriorityTagID("limelight", 4);
    } else if (ally.isPresent()) {
      scoringLocation = AutoConstants.kBlueSpeakerLocation;
      LimelightHelpers.setPriorityTagID("limelight", 7);
    } else {
      scoringLocation = AutoConstants.kRedSpeakerLocation;
      LimelightHelpers.setPriorityTagID("limelight", 4);
    }
    //    if (!Limelight.hasValidTargets("limelight")) return;
    lastLastRobotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose;
    lastRobotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose;
  }

  @Override
  public void execute() {
    //    if (!Limelight.hasValidTargets("limelight")) return;
    Pose2d ourPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose;
    double lastDifferenceX = lastRobotPose.getX() - lastLastRobotPose.getX();
    double lastDifferenceY = lastRobotPose.getY() - lastLastRobotPose.getY();

    double offsetTX = ourPose.getX() + lastDifferenceX;
    double offsetTY = ourPose.getY() + lastDifferenceY;

    Pose2d offsetPose = new Pose2d(offsetTX, offsetTY, ourPose.getRotation());

    double distance = offsetPose.getTranslation().getDistance(scoringLocation.getTranslation());
    if (distance > 5.5) {
      return;
    }
    System.out.println("Distance For Monkeys: " + distance);
    //    distance = 1.3;
    pivotShooter.setDegrees(pivotShooter.pivotMotorData.get(distance) * kPivotMotorGearing);

    lastLastRobotPose = lastRobotPose;
    lastRobotPose = ourPose;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
