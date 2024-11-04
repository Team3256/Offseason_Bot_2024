// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IntakeConstants {
  /* CAN */
  public static final int kIntakeMotorID = 33;
  public static final int kSecondaryIntakeMotorID = 999;

  public static final double kPassthroughIntakeVoltage = -8;
  public static final double kIntakeIntakeVoltage = 12;

  // Time of Flight constants
  public static final double kBeamBreakDelayTime = 0;

  public static final int kIntakeBeamBreakDIO = 1;
  public static int kPassthroughMotorID = 35;

  public static double updateFrequency = 50;
  public static boolean kIntakeMotionMagic = false;
  public static boolean kPassthroughMotionMagic = false;

  public static final TalonFXConfiguration intakeMotorConfig =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0.13).withKP(0.2).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(120)
                  .withMotionMagicCruiseVelocity(60)
                  .withMotionMagicJerk(1200))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80));
  public static final TalonFXConfiguration passthroughMotorConfig =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0).withKP(1).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(120)
                  .withMotionMagicCruiseVelocity(60)
                  .withMotionMagicJerk(1200))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80));
}
