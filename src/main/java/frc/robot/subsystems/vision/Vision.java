// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Vision extends SubsystemBase {

  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged visionIOAutoLogged = new VisionIOInputsAutoLogged();

  private double lastLastCenterLimelightX = 0.0;
  private double lastLastCenterLimelightY = 0.0;
  private double lastCenterLimelightX = 0.0;
  private double lastCenterLimelightY = 0.0;
  private double centerLimelightX = 0.0;
  private double centerLimelightY = 0.0;

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(visionIOAutoLogged);

    lastLastCenterLimelightX = lastCenterLimelightX;
    lastLastCenterLimelightY = lastCenterLimelightY;
    lastCenterLimelightX = centerLimelightX;
    lastCenterLimelightY = centerLimelightY;
    centerLimelightX = visionIOAutoLogged.centerLimelightX;
    centerLimelightY = visionIOAutoLogged.centerLimelightY;
  }

  public double getCenterLimelightX() {
    return centerLimelightX;
  }

  public double getCenterLimelightY() {
    return centerLimelightY;
  }


  public boolean detectedNote() {
    return visionIOAutoLogged.detectedNote;
  }
  public double getLastCenterLimelightX() {
    return lastCenterLimelightX;
  }

  public double getLastCenterLimelightY() {
    return lastCenterLimelightY;
  }

  public double getLastLastCenterLimelightX() {
    return lastLastCenterLimelightX;
  }

  public double getLastLastCenterLimelightY() {
    return lastLastCenterLimelightY;
  }

  @AutoLogOutput
  public double getDistanceToNote() {
    return -(VisionConstants.noteLimelightHeightInches - VisionConstants.noteHeightInches)
        / Math.tan(
            Units.degreesToRadians(
                visionIOAutoLogged.noteLimelightY + VisionConstants.noteLimelightAngleDegrees));
  }

  public Pose2d getNotePose(Pose2d robotPose) {
    return robotPose
        .transformBy(VisionConstants.robotToCam)
        .transformBy(
            new Transform2d(
                new Translation2d(
                    Units.inchesToMeters(getDistanceToNote()),
                    robotPose
                        .getRotation()
                        .plus(Rotation2d.fromDegrees(visionIOAutoLogged.noteLimelightX))),
                robotPose
                    .getRotation()
                    .plus(Rotation2d.fromDegrees(visionIOAutoLogged.noteLimelightX))));
  }

  @AutoLogOutput
  public double getCompensatedCenterLimelightX() {
    return centerLimelightX + (lastCenterLimelightX - lastLastCenterLimelightX);
  }

  @AutoLogOutput
  public double getCompensatedCenterLimelightY() {
    return centerLimelightY + (lastCenterLimelightY - lastLastCenterLimelightY);
  }
}
