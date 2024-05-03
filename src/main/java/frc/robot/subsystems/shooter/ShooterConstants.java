// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  /* Misc */
  public static final boolean kUseShooterMotionMagic = false;
  public static final boolean kUseFOC = true;
  /* CAN */
  public static int kShooterMotorID = 11;
  public static int kShooterMotorFollowerID = 23;
  /* PID */
  // Shooter
  public static double kV = 0.39; // Original 0.145 TODO: Tune PID values
  public static double kA = 1.48; // Original 0 only for feedforward, might not use
  public static double kS = 0;
  public static double kP = 0.4;
  public static double kI = 0;
  public static double kD = 0;

  // unused,
  // public static double kShooterFollowerKV = 0.13; // TODO: Tune PID values
  // public static double kShooterFollowerKA = 0; // only for feedforward, might
  // not use
  // public static double kShooterFollowerKS = 0;
  // public static double kShooterFollowerKP = 0.5;
  // public static double kShooterFollowerKI = 0;
  // public static double kShooterFollowerKD = 0.001;
  // Shooter follower
  // Feeder

  public static double kShooterSpeakerRPS = 42; // TODO: Tune
  public static double kShooterFollowerSpeakerRPS = 45; // really 80

  public static double kShooterSubwooferRPS = 60;
  public static double kShooterFollowerSubwooferRPS = 70;

  public static double kShooterAmpRPS = 22.5; // BEFORE: 1200/60 TODO: Tune
  public static double kShooterFollowerAmpRPS = 22.5;

  public static double kShooterFeederRPS = 42;
  public static double kShooterFollowerFeederRPS = 45;

  /* Misc */
  public static double kShooterAngle = 10; // The fixed angle for the shooter (in degrees)
  // before: 1800/6
  public static double statorLimit = 60;
  public static boolean enableStatorLimit = true;
  public static double motionMagicJerk = 1600;
  public static double motionMagicAcceleration = 300;
  public static double motionMagicVelocity = 100;
  public static double updateFrequency = 50.0;
  public static boolean kUseMotionMagic = false;
}
