// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.utils.SinglePositionSubsystem;

public class PivotShooter extends SinglePositionSubsystem {
  public InterpolatingDoubleTreeMap pivotMotorData = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap pivotMotorDataNotGlobalPose = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap pivotMotorDataNotGlobalPose2 =
      new InterpolatingDoubleTreeMap(); // for second april

  public PivotShooter() {
    super(
        PivotShooterConstants.kUseMotionMagic,
        PivotShooterConstants.kCurrentThreshold,
        PivotShooterConstants.kStallVelocityThreshold);
    super.configureRealHardware(
        PivotShooterConstants.kPivotMotorID,
        NeutralModeValue.Brake,
        PivotShooterConstants.kS,
        PivotShooterConstants.kV,
        PivotShooterConstants.kP,
        PivotShooterConstants.kI,
        PivotShooterConstants.kD,
        PivotShooterConstants.motionMagicVelocity,
        PivotShooterConstants.motionMagicAcceleration,
        PivotShooterConstants.motionMagicJerk,
        PivotShooterConstants.enableStatorLimit,
        PivotShooterConstants.statorLimit);
    if (!RobotBase.isReal()) {
      super.configureSimHardware(
          PivotShooterConstants.kNumPivotMotors,
          PivotShooterConstants.kPivotMotorGearing,
          PivotShooterConstants.jKgMetersSquared,
          PivotShooterConstants.kPivotLength,
          PivotShooterConstants.kPivotMinAngleDeg,
          PivotShooterConstants.kPivotMaxAngleDeg,
          false);
    }
    setupPivotShooterData();
  }

  private void setupPivotShooterData() {

    // pivotMotorData.put(0.051, kSubWooferPreset);
    // pivotMotorData.put(1.4, 6.25 / 138.333);
    // pivotMotorData.put(0.945, 5.8 / 138.333);
    // pivotMotorData.put(1.69, 6.5 / 138.333);
    // pivotMotorData.put(0.485, 5.25 / 138.333);
    pivotMotorData.put(19.771, PivotShooterConstants.kSubWooferPreset);
    // pivotMotorData.put(3.19, 5.6/138.333); // DO NOT use to interpolate for now
    // distance to speaker and then angle
  }

  private void setupPivotShooterDataNotGlobalPose() {
    pivotMotorDataNotGlobalPose.put(69.2, 2.1); // distance to speaker and then angle
    pivotMotorDataNotGlobalPose2.put(69.1, 69.2); // distance to speaker and then angle
  }

  @Override
  public void off() {
    super.off();
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Pivot off");
    }
  }

  @Override
  public void setOutputVoltage(double voltage) {
    super.setOutputVoltage(voltage);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Pivot Shooter voltage set to: " + voltage);
    }
  }

  @Override
  public void setDegrees(double degrees) {
    super.setDegrees(degrees);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Pivot Shooter set to: " + degrees);
    }
  }

  @Override
  public void zero() {
    super.zero();
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("[PivotShooter] Setting zero position to: " + getDegrees());
    }
  }
}
