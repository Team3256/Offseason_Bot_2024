// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class PivotIntakeConstants {
  /* CAN */
  public static final int kPivotMotorID = 34;

  /* PID */
  public static final TrapezoidProfile.Constraints kPivotProfileContraints =
      new TrapezoidProfile.Constraints(16, 16);

  /* Tolerance/threshold */

  /* Physics/geometry */
  public static final double kPivotMotorGearing = 36; // 22 by 1
  public static final double kPivotGroundPos = -5.6 / 16; // for sim

  /* Preset */
  public static final double kPivotSlamIntakeVoltage = -5;
  public static final double kPivotSlamShooterVoltage = 4;

  /* Misc */
  public static final boolean kUseFOC = false;
  public static boolean kUseMotionMagic = false;

  public static double updateFrequency = 50.0;

  protected static double kPivotSlamStallCurrent = 10.0;

  public static final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0.2)
                  .withKP(0.7)
                  .withKI(0)
                  .withKD(0)
                  .withKG(1)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
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
