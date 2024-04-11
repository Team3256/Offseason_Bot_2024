// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.pivotshooter.PivotShooter;

public class PivotShooterSlamAndVoltage extends SequentialCommandGroup {
  private final PivotShooter pivotShooter;

  // position: 0 is ground, 1 is shooter
  public PivotShooterSlamAndVoltage(PivotShooter pivot) {
    this.pivotShooter = pivot;
    addRequirements(pivotShooter);

    addCommands(
        new PivotShooterSetAngle(pivotShooter, 0).withTimeout(0.75),
        new PivotShooterSlam(pivotShooter, 0));
  }
}
