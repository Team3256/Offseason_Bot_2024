// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake.commands;

import frc.robot.helpers.DebugCommandBase;

public class PivotIntakeSlam extends DebugCommandBase {
  // private final PivotIntake pivot;
  // private final int position;

  // // position: 0 is ground, 1 is shooter
  // public PivotIntakeSlam(PivotIntake pivot, int position) {
  //   if (position != 0 && position != 1) {
  //     throw new IllegalArgumentException("Invalid position: " + position);
  //   }
  //   this.pivot = pivot;
  //   this.position = position;
  //   addRequirements(pivot);
  // }

  // @Override
  // public void initialize() {
  //   if (position == 0) {
  //     // slam to ground position
  //     pivot.setOutputVoltage(PivotIntakeConstants.kPivotSlamShooterVoltage);
  //     System.out.println("pivot slam to ground");
  //   } else {
  //     // slam to shooter position
  //     pivot.setOutputVoltage(PivotIntakeConstants.kPivotSlamIntakeVoltage);
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
  //   if (position == 0) {
  //     pivot.zero();
  //   }
  //   pivot.off();
  //   // pivot.setOffset(position);
  // }

  // @Override
  // public boolean isFinished() {
  //   // if (System.currentTimeMillis() - timeInit > 100
  //   // && (pivot.getIntakeCurrent() > 100 || pivot.getIntakeCurrent() - prevCurrent
  //   // >
  //   // PivotConstants.kCurrentThreshold
  //   // || Math.abs(pivot.getVoltage()) > PivotConstants.kVoltageThreshold
  //   // || Math.abs(pivot.getVelocity()) < PivotConstants.kVelocityThreshold)) {
  //   // return true;
  //   // }
  //   // prevCurrent = pivot.getIntakeCurrent();
  //   // return false;
  //   return pivot.getVelocityStalled() || pivot.getCurrent() > 10;
  //   // return pivot.getCurrent() > 10;
  // }
}
