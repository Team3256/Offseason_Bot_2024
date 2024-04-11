// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.pivot.PivotIntake;

public class PivotSlamAndVoltage extends SequentialCommandGroup {
  private final PivotIntake pivot;

  // position: 0 is ground, 1 is shooter
  public PivotSlamAndVoltage(PivotIntake pivot) {
    this.pivot = pivot;
    addRequirements(pivot);

    addCommands(
        new PivotSetAngle(pivot, 0).withTimeout(0.75),
        new PivotSlam(pivot, 0),
        new PivotStaticBrake(pivot));
  }
}
