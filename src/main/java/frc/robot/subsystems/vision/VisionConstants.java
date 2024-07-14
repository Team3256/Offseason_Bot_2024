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
  public static final String centerLimelight = "limelight";

  public static final double noteLimelightAngleDegrees = 0; //TODO: idk tune
  public static final double noteLimelightHeightInches = 10; //TODO: idk tune
  public static final double noteHeight = 2;

  public static final Transform2d robotToCam = new Transform2d(new Translation2d(0, 0), new Rotation2d(0)); // pretty sure its too small to matter

}
