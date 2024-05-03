// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.robotviz;

import static frc.robot.robotviz.RobotVizConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FeatureFlags;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotViz {
  private SwerveDrive swerveSubsystem;
  private Shooter shooterSubsystem;
  private PivotIntake pivotIntakeSubsystem;
  private Intake intakeSubsystem;

  private Field2d field;
  private Mechanism2d robotCanvas;
  private MechanismLigament2d leftClimb;
  private MechanismLigament2d rightClimb;
  private MechanismLigament2d shooter;
  private MechanismLigament2d feeder;
  private MechanismLigament2d pivot;
  private MechanismLigament2d intake;

  public RobotViz(
      SwerveDrive swerveSubsystem,
      Shooter shooterSubsystem,
      Intake intakeSubsystem,
      PivotIntake pivotIntakeSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.pivotIntakeSubsystem = pivotIntakeSubsystem;
    // setup field2D
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    field.getObject("Robot").setPose(0, 0, new Rotation2d(0));
    // setup canvas
    robotCanvas = new Mechanism2d(kRobotSimWindowWidth, kRobotSimWindowHeight);
    SmartDashboard.putData("Robot Simviz", robotCanvas);
    MechanismRoot2d robotRoot = robotCanvas.getRoot("Robot Root", kRootX, kRootY);
    robotRoot.append(new MechanismLigament2d("Drive Chassis", botLength, 0, kLineWidth, orange));
    // left climb
    MechanismLigament2d leftClimbRoot =
        robotRoot.append(
            new MechanismLigament2d("Climb Left Root", kClimberX, 0, kLineWidth, orange));
    MechanismLigament2d leftClimbSpout =
        leftClimbRoot.append(
            new MechanismLigament2d("Climb Left Spout", kClimberSpoutHeight, 90, kLineWidth, blue));
    leftClimb =
        leftClimbSpout.append(new MechanismLigament2d("Climb Left", 0.05, 0, kLineWidth, green));
    // right climb
    MechanismLigament2d rightClimbRoot =
        robotRoot.append(
            new MechanismLigament2d(
                "Climb Right Root", kClimberX + kClimberSpacing, 0, kLineWidth, orange));
    MechanismLigament2d rightClimbSpout =
        rightClimbRoot.append(
            new MechanismLigament2d(
                "Climb Right Spout", kClimberSpoutHeight, 90, kLineWidth, blue));
    rightClimb =
        rightClimbSpout.append(new MechanismLigament2d("Climb Right", 0.05, 0, kLineWidth, green));
    // shooter
    MechanismLigament2d shooterRoot =
        robotRoot.append(new MechanismLigament2d("Shooter Root", shooterX, 0, kLineWidth, orange));
    MechanismLigament2d shooterSpout =
        shooterRoot.append(
            new MechanismLigament2d(
                "Shooter Spout", shooterOffset, shooterAngle.getDegrees(), kLineWidth, red));
    shooter =
        shooterSpout.append(
            new MechanismLigament2d("Shooter", shooterRadius, 90, kLineWidth, green));
    // feeder
    MechanismLigament2d feederSpout =
        shooterRoot.append(
            new MechanismLigament2d(
                "Feeder Spout", feederOffset, shooterAngle.getDegrees(), kLineWidth, red));
    feeder =
        feederSpout.append(new MechanismLigament2d("Feeder", feederRadius, 90, kLineWidth, green));
    // arm
    MechanismLigament2d pivotRoot =
        robotRoot.append(new MechanismLigament2d("Pivot Root", armX, 0, kLineWidth, orange));
    MechanismLigament2d pivotRootSpout =
        pivotRoot.append(new MechanismLigament2d("Pivot Root Spout", armY, 90, kLineWidth, blue));
    pivot =
        pivotRootSpout.append(new MechanismLigament2d("Pivot", armLength, 0, kLineWidth, white));
    MechanismLigament2d hand =
        pivot.append(new MechanismLigament2d("Hand", handLength, 90, kLineWidth, white));
    // intake
    intake = hand.append(new MechanismLigament2d("Intake", kIntakeRadius, 90, kLineWidth, green));
  }

  public void update(double dt) {
    // TODO: implement these when the subsystems are created
    if (FeatureFlags.kSwerveEnabled) {
      field.getObject("Robot").setPose(swerveSubsystem.getPose());
    }
    if (FeatureFlags.kClimbEnabled) {
      leftClimb.setLength(0);
      rightClimb.setLength(0);
    }
    if (FeatureFlags.kShooterEnabled) {
      shooter.setAngle(shooter.getAngle() + 10 * 360 * dt * shooterVelocitySimDamp);
      feeder.setAngle(0);
    }
    if (FeatureFlags.kPivotIntakeEnabled) {
      pivot.setAngle(pivot.getAngle());
    }
    if (FeatureFlags.kIntakeEnabled) {
      intake.setAngle(intake.getAngle() + intakeSubsystem.getIntakeVelocity() * 360 * dt);
    }
  }
}
