// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

public final class Constants {

  // We do not include the AdvKit in the main FeatureFlags class - since this is
  // in Robot.java and we'd prefer not to
  // have things break.

  /* FeatureFlags that are used in Robot.java */
  public static final boolean kEnableAdvKit = true;
  public static final boolean kEnableMonologue = false;
  public static final boolean kDisableSubsystemsOnDisableInit = true;

  // CAREFUL!
  public static final boolean kOverrideBrownOutVoltage = true;
  public static final double kOverridenBrownOutVoltage = 5.6;

  /* Drive configuration */
  public static final double stickDeadband = 0.1;
  public static final double rotationalDeadband = 0.12;
  public static final double azimuthStickDeadband = 0.3;
  // Logging
  public static final int kLogLinesBeforeFlush = 100;
  public static final boolean kMonologueFileOnly = false;
  public static final boolean kMonologueLazyLogging = false;

  public static final class FeatureFlags {

    // subsystems

    public static final boolean kIntakeEnabled = true;

    public static final boolean kShooterEnabled = true;
    public static final boolean kShooterRegenerativeBrakingEnabled = true;

    public static final boolean kSwerveEnabled = false;

    public static final boolean kPivotIntakeEnabled = true;

    public static final boolean kClimbEnabled = true;

    public static final boolean kAmpBarEnabled = true;

    // logging
    public static final boolean kTuningMode = false;
    public static final boolean kDebugEnabled = false;
    public static final boolean DebugCommandEnabled = false;
    public static final boolean kRobotVizEnabled = true && !Robot.isReal();
    public static final boolean kRumbleEnabled = true;

    public static boolean kPivotShooterEnabled = true;
  }
}
