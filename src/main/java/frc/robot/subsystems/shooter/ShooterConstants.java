// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ShooterConstants {
  /* Misc */
  public static final boolean kUseShooterMotionMagic = false;
  public static final boolean kUseFOC = true;
  /* CAN */
  public static int kShooterMotorID = 11;
  public static int kShooterMotorFollowerID = 23;
  /* PID */
  // Shooter
  public static MotorOutputConfigs motorOutputConfigs =
      new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive);
  public static TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0.39) // Original 0.145
                  // .withKA(1.48)// Original 0 only for feedforward, might not use
                  .withKP(0.4)
                  .withKI(0)
                  .withKD(0))
          .withMotorOutput(motorOutputConfigs)
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(100)
                  .withMotionMagicCruiseVelocity(300)
                  .withMotionMagicJerk(1600))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));
  public static TalonFXConfiguration followerMotorConfigs =
      motorConfigs.withMotorOutput(
          motorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive));

  public static double kShooterSpeakerRPS = 42;
  public static double kShooterFollowerSpeakerRPS = 45; // really 80

  public static double kShooterSubwooferRPS = 60;
  public static double kShooterFollowerSubwooferRPS = 70;

  public static double kShooterAmpRPS = 22.5; // BEFORE: 1200/60
  public static double kShooterFollowerAmpRPS = 22.5;

  public static double kShooterFeederRPS = 42;
  public static double kShooterFollowerFeederRPS = 45;

  /* Misc */
  public static double kShooterAngle = 10; // The fixed angle for the shooter (in degrees)
  // before: 1800/6
  public static double updateFrequency = 50.0;
  public static boolean kUseMotionMagic = false;

  public static NeutralModeValue neutralMode = NeutralModeValue.Brake;
  public static InvertedValue shooterInverted = InvertedValue.Clockwise_Positive;
  public static InvertedValue shooterFollowerInverted = InvertedValue.CounterClockwise_Positive;
}
