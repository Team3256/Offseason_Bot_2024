// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake.commands;

import frc.robot.helpers.DebugCommandBase;

public class PivotIntakeCalibrate extends DebugCommandBase {
  // private PivotIntake pivot;
  // private final int position;
  // private TimedBoolean isCurrentSpiking;

  // // position: 0 is ground, 1 is shooter
  // public PivotIntakeCalibrate(PivotIntake pivot, int position) {
  //   if (position != 0 && position != 1) {
  //     throw new IllegalArgumentException("Invalid position: " + position);
  //   }
  //   if (FeatureFlags.kDebugEnabled) {
  //     System.out.println("PivotSlam: " + position);
  //   }
  //   this.pivot = pivot;
  //   this.position = position;
  //   this.isCurrentSpiking =
  //       new TimedBoolean(pivot::isCurrentSpiking, 0.5); // Have we been spiking for 0.5 seconds?
  //   addRequirements(pivot);
  // }

  // @Override
  // public void initialize() {
  //   if (position == 0) {
  //     // slam to ground position
  //     pivot.setOutputVoltage(PivotIntakeConstants.kPivotSlamIntakeVoltage);
  //     System.out.println("pivot slam to ground");
  //   } else {
  //     // slam to shooter position
  //     pivot.setOutputVoltage(PivotIntakeConstants.kPivotSlamShooterVoltage);
  //     System.out.println("pivot slam to shooter");
  //   }
  // }

  // @Override
  // public void execute() {}

  // @Override
  // public void end(boolean interrupted) {
  //   if (interrupted) {
  //     System.out.println("interrupted");
  //   }
  //   super.end(interrupted);
  //   pivot.off();
  //   if (position == 1) {
  //     pivot.zero();
  //     if (FeatureFlags.kDebugEnabled) {
  //       System.out.println("Pivot Calibration finished -- zeroed");
  //     }
  //   }
  // }

  // @Override
  // public boolean isFinished() {
  //   isCurrentSpiking.update();
  //   return isCurrentSpiking.hasBeenTrueForThreshold();
  // }
}
