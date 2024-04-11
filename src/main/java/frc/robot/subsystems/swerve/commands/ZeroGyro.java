// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import frc.robot.Constants;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ZeroGyro extends DebugCommandBase {
  SwerveDrive swerveDrive;

  public ZeroGyro(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void initialize() {
    super.initialize();
    swerveDrive.resetGyro();
    if (Constants.FeatureFlags.kResetHeadingOnZeroGyro) {
      System.out.println("kResetHeadingOnZeroGyro is TRUE -- resetting heading.");
      swerveDrive.zeroHeading();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
