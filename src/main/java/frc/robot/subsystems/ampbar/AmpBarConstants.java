// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class AmpBarConstants {
  public static final double kAmpBarVelocityThreshold = 1.5;

  public static final double kAmpBarCurrentThreshold = 5;
  public static final double kAmpBarAmpVoltage = 16; // 8
  public static final double kAmpBarStowVoltage = -16; // -4

  public static final int kAmpBarMotorID = 10;
  public static int kNumAmpBarMotors = 1;
  public static double kAmpBarMotorGearing = 7; // i dont think we need this
  public static double jKgMetersSquared = 0.1;
  public static double kAmpBarLength = 5;
  public static double kAmpBarMinAngleDeg = 0;
  public static double kAmpBarMaxAngleDeg = 90;
  public static Double kStallVelocityThreshold = 0.1;

  public static double updateFrequency = 50;


  public static final TalonFXConfiguration motorConfig =
          new TalonFXConfiguration()
                  .withMotorOutput(
                          new MotorOutputConfigs()
                                  .withNeutralMode(NeutralModeValue.Brake)
                                  .withInverted(InvertedValue.Clockwise_Positive)
                  )
                  .withCurrentLimits(
                          new CurrentLimitsConfigs()
                                  .withStatorCurrentLimitEnable(true)
                                  .withStatorCurrentLimit(20)
                  );
}
