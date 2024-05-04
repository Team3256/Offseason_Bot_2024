// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

public final class ClimbConstants {

  public static final int kLeftClimbMotorID = 18;
  public static final boolean kUseClimbMotionMagic = false;
  public static final double gearRatio = 20; // needs to be tuned

  public static final double kClimbUpPosition = 81.6 / 20;

  public static final double kClimbDownPosition = 0.65;
  public static final double wheelRadius = 1;
  public static final double climbVelocity = 80;
  public static final double climbAcceleration = 400;
  public static final double climbJerk = 4000;

  public static final boolean enableStatorLimit = true;
  public static final double statorLimit = 60;
  public static final int kThresholdConstant = 16;

  public static final double kP = 1;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static double kCurrentThreshold = 4.5;

  // // What about the tension from the spring?
  // public static double gyroRollStableThreshold = 1; // 1 degree of error is
  // tolerated
  public static double motionMagicVelocity = 100;
  public static double motionMagicAcceleration = 100;
  public static double motionMagicJerk = 300;
}
