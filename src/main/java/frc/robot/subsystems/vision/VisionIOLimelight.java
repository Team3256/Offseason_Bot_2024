// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.limelight.Limelight;

public class VisionIOLimelight implements VisionIO {

  public VisionIOLimelight() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.noteLimelightX = Limelight.getTX(VisionConstants.noteDetectionLimelight);
    inputs.noteLimelightY = Limelight.getTY(VisionConstants.noteDetectionLimelight);

    inputs.centerLimelightX = Limelight.getTX(VisionConstants.centerLimelight);
    inputs.centerLimelightY = Limelight.getTY(VisionConstants.centerLimelight);
  }
}
