// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter.commands;

import static frc.robot.subsystems.pivotshooter.PivotingShooterConstants.kSubWooferPreset;

import frc.robot.Constants;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.helpers.TimedBoolean;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotingShooterConstants;
import org.littletonrobotics.junction.Logger;

public class PivotShootSubwoofer extends DebugCommandBase {
  private PivotShooter pivotShooter;
  private final double angleDeg;
  private TimedBoolean isMotorStalled;

  public PivotShootSubwoofer(PivotShooter pivotShooter) {
    this.pivotShooter = pivotShooter;
    this.angleDeg = kSubWooferPreset;
    this.isMotorStalled =
        new TimedBoolean(pivotShooter::isMotorStalled, 1.5); // We use a motor stall as a
    // fail-safe: see isFinished() for
    // more info.
    addRequirements(this.pivotShooter);
  }

  @Override
  public void initialize() {
    super.initialize();
    pivotShooter.setDegrees(angleDeg * PivotingShooterConstants.kPivotMotorGearing);
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
          "Set pivot shooter angle end: "
              + angleDeg
              + " deg) interrupted: "
              + interrupted
              + " current: "
              + pivotShooter.getDegrees()
              + " deg)");
    }
    super.end(interrupted);
    pivotShooter.off();
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
          "[PivotShooterSetAngle] current over limit + velocity too slow for too long "
              + pivotShooter.getDegrees()
              + " DEG WHILE SETPOINT WAS "
              + angleDeg
              + " DEG");
      return true;
    }
    // System.out.println(
    // "current pivot position: " + Math.abs((pivotIntake.getDegrees() / 33) -
    // angleDeg));
    if (Math.abs((pivotShooter.getDegrees() / 1) - angleDeg)
        < PivotingShooterConstants.kPivotPositionToleranceDeg) {
      System.out.println("pivotShooter reached shooting position");
      return true;
    } else return false;
  }
}
