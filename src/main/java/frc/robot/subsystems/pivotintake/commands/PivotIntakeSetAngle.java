// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake.commands;

import frc.robot.Constants;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.helpers.TimedBoolean;
import frc.robot.subsystems.pivotintake.PivotIntake;

import static frc.robot.subsystems.pivotintake.PivotIntakeConstants.kPivotMotorGearing;
import static frc.robot.subsystems.pivotintake.PivotIntakeConstants.kPivotPositionToleranceDeg;

import org.littletonrobotics.junction.Logger;

public class PivotIntakeSetAngle extends DebugCommandBase {
  private PivotIntake pivotIntake;
  private final double angleDeg;
  private TimedBoolean isMotorStalled;

  public PivotIntakeSetAngle(PivotIntake pivotIntake, double angleDeg) {
    this.pivotIntake = pivotIntake;
    this.angleDeg = angleDeg;
    this.isMotorStalled =
        new TimedBoolean(pivotIntake::isMotorStalled, 1.5); // We use a motor stall as a
    // fail-safe: see isFinished() for
    // more info.
    addRequirements(this.pivotIntake);
  }

  @Override
  public void initialize() {
    super.initialize();
    pivotIntake.setDegrees(angleDeg * kPivotMotorGearing);
    Logger.recordOutput("PivotSetAngle", angleDeg);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Set pivot at angle: " + angleDeg + " deg)");
    }
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("PivotSetAngleEnd", angleDeg);
    if (Constants.FeatureFlags.kDebugEnabled) {

      System.out.println(
          "Set pivot angle end: "
              + angleDeg
              + " deg) interrupted: "
              + interrupted
              + " current: "
              + pivotIntake.getDegrees()
              + " deg)");
    }
    super.end(interrupted);
    pivotIntake.off();
  }

  @Override
  public boolean isFinished() {
    // This is a fail-safe. If the current spikes for 1.5 seconds, we
    // stop the command, regardless of the setpoint.
    // Since we know that the chain can break, we wouldn't want that
    // to happen during a match.
    if (isMotorStalled.hasBeenTrueForThreshold()) {
      // Something went wrong. We stop the command, and print a message.
      System.out.println(
          "[PivotSetAngle] current over limit + velocity too slow for too long "
              + pivotIntake.getDegrees()
              + " DEG WHILE SETPOINT WAS "
              + angleDeg
              + " DEG");
      return true;
    }
    // System.out.println(
    // "current pivot position: " + Math.abs((pivotIntake.getDegrees() / 33) -
    // angleDeg));
    if (Math.abs((pivotIntake.getDegrees() / 33) - angleDeg) < kPivotPositionToleranceDeg) {
      // System.out.println("pivot reached shooting position");
      return true;
    } else return false;
  }
}
