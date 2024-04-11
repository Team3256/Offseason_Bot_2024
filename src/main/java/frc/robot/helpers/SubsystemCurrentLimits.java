// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

public abstract interface SubsystemCurrentLimits {
  public static final boolean kSetCurrentLimits = false;
  public static final double kCurrentLimit = 0;
  public double kCurrentThreshold = 40;
  public double kCurrentThresholdTime = 0.1;
  public boolean kEnableCurrentLimit = true;

  public boolean kEnableStatorLimits = false;
  public double kStatorLimit = 10;
}
