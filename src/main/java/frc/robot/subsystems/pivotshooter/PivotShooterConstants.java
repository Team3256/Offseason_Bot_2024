// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class PivotShooterConstants {
  // public static final double kSubWooferPreset = (3.5 + 0.3) / 138.33; // idk if
  // this works
  public static final double kSubWooferPreset = 3.4 / 138.33; // 3.2
  public static final double kFeederPreset = 5.9 / 138.33;
  public static final double kAmpPreset = (4) / 138.33;
  public static final double kWingNoteCenterPreset = 5.8 / 138.33;
  public static final double kWingNoteSidePreset =
      5.5 / 138.33; // old value: 5.7 distance: -1.5 //old ish?: 5.4
  public static final double kWingNoteFarSidePreset = 0 / 138.33;
  public static final double kTrussSourceSidePreset = 6.7 / 138.33; // -10.6875
  public static final double kHalfWingPodiumPreset =
      6.55 / 138.33; // old value: 6.7 distance: -11.5275
  public static final double kPodiumLeftPreset = 6.5 / 138.33;
  public static final double kPodiumRPreset = 6 / 138.33;

  public static final int kPivotMotorID = 12;

  /* PID */
  public static final TrapezoidProfile.Constraints kPivotProfileContraints =
      new TrapezoidProfile.Constraints(16, 16);

  /* Tolerance/threshold */
  public static final double kPivotPositionToleranceDeg = 0.1; // 5deg for the pivot.
  public static final double kStallVelocityThreshold = 0.1;

  /* Physics/geometry */
  public static final double kPivotMotorGearing = 138.333; // 22 by 1
  public static final double kPivotLength = 0.2;
  public static final double kPivotMinAngleDeg = -90;
  public static final double kPivotMaxAngleDeg = 50;
  public static final double kPivotStartingAngleDeg = 0;
  public static final double jKgMetersSquared = 0.1; // for sim

  /* Preset */
  public static final double kPivotSlamIntakeVoltage = -5;
  public static final double kPivotSlamShooterVoltage = -2;

  /* Misc */
  public static final int kNumPivotMotors = 1;
  public static final boolean kUseFOC = false;
  public static boolean kUseMotionMagic = true; // idk
  public static double updateFrequency = 50.0;
  static double kPivotSlamStallCurrent = 50;

  public static final int kSpeakerAprilTagRed = 4;
  public static final int kSpeakerAprilTagBlue = 0;

  public static final int kSpeakerBackupAprilTagRed = 5;
  public static final int kSpeakerBackupAprilTagBlue = 1;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKP(10)
                  .withKI(0)
                  .withKD(0)
                  .withKG(1)
                  .withGravityType(GravityTypeValue.Arm_Cosine) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(400)
                  .withMotionMagicCruiseVelocity(50))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));
}
