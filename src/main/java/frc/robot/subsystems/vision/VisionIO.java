// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public double noteLimelightX = 0.0;
    public double noteLimelightY = 0.0;
    public boolean detectedNote = false;

    public double centerLimelightX = 0.0;
    public double centerLimelightY = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
