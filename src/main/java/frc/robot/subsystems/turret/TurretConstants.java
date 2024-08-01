// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class TurretConstants {

  public static int kCanCoderID = 0; // TODO: set id

  public static final CANcoderConfiguration canCoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0)); // TODO: set config
  public static double updateFrequency = 50.0;

  public static int kTurretMotorID = 0; // TODO: Set ID

  public static final TalonFXConfiguration motorConfigs = // TODO: Set configs
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0.05)
                  .withKP(25)
                  .withKI(0)
                  .withKD(0) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(100)
                  .withMotionMagicCruiseVelocity(100)
                  .withMotionMagicJerk(420))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(kCanCoderID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(1) // TODO: TUNE IMPORTANT
              );

  public static final int gearToothCountE1 = 1; // TODO: Set gear ratio
  public static final int gearToothCountE2 = 1; // TODO: Set gear ratio

  public static final boolean kUseMotionMagic = false;

  public static final double kForwardLimit = 69; // TODO: Set limit
  public static final double kReverseLimit = -69; // TODO: Set limit
}
