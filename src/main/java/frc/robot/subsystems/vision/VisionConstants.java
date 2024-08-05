// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class VisionConstants {
  public static final String noteDetectionLimelight = "limelight-note";
  public static final String centerLimelight = "limelight-speaker";

  public static final double noteLimelightAngleDegrees = -30.827; // TODO: idk tune
  public static final double noteLimelightHeightInches = 21; // TODO: idk tune
  public static final double noteHeightInches = 2; // TODO: idk tune

  public static final Transform2d robotToCam =
      new Transform2d(
          new Translation2d(0, 0), new Rotation2d(0)); // pretty sure its too small to matter
}
