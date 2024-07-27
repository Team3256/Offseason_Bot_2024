// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class AmpBarConstants {
  public static final double kAmpBarVelocityThreshold = 1.5;

  public static final double kAmpBarCurrentThreshold = 5;
  public static final double kAmpBarAmpVoltage = 16; // 8
  public static final double kAmpBarStowVoltage = -16; // -4

  public static final double ampPosition = 2.834 / 10;
  public static final double stowPosition = 0;
  public static final int motorID = 10;
  public static double motorGearing = 10;

  public static double updateFrequency = 50;

  public static final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(20))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(400)
                  .withMotionMagicCruiseVelocity(50))
          .withSlot0(new Slot0Configs().withKS(0).withKV(0).withKP(3).withKI(0).withKD(0.2).withKG(1));
  public static boolean kUseMotionMagic = true;
}
