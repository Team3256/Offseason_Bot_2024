// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.utils.SinglePositionSubsystem;

public class PivotIntake extends SinglePositionSubsystem {
  public PivotIntake() {
    super(
        PivotIntakeConstants.kUseMotionMagic,
        PivotIntakeConstants.kCurrentThreshold,
        PivotIntakeConstants.kStallVelocityThreshold);
    super.configureRealHardware(
        PivotIntakeConstants.kPivotMotorID,
        NeutralModeValue.Brake,
        PivotIntakeConstants.kS,
        PivotIntakeConstants.kV,
        PivotIntakeConstants.kP,
        PivotIntakeConstants.kI,
        PivotIntakeConstants.kD,
        PivotIntakeConstants.motionMagicVelocity,
        PivotIntakeConstants.motionMagicAcceleration,
        PivotIntakeConstants.motionMagicJerk,
        PivotIntakeConstants.enableStatorLimit,
        PivotIntakeConstants.statorLimit);
    if (!RobotBase.isReal()) {
      super.configureSimHardware(
          PivotIntakeConstants.kNumPivotMotors,
          PivotIntakeConstants.kPivotMotorGearing,
          PivotIntakeConstants.jKgMetersSquared,
          PivotIntakeConstants.kPivotLength,
          PivotIntakeConstants.kPivotMinAngleDeg,
          PivotIntakeConstants.kPivotMaxAngleDeg,
          false);
    }
  }

  @Override
  public void off() {
    super.off();
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Pivot Intake off");
    }
  }

  @Override
  public void setOutputVoltage(double voltage) {
    super.setOutputVoltage(voltage);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Pivot Intake voltage set to: " + voltage);
    }
  }

  @Override
  public void setDegrees(double degrees) {
    super.setDegrees(degrees);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Pivot Intake set to: " + degrees);
    }
  }

  @Override
  public void zero() {
    super.zero();
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("[PivotIntake] Setting zero position to: " + getDegrees());
    }
  }
}
