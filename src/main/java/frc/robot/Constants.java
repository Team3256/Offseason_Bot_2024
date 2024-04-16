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
  public static final boolean kEnableOBlog = false;
  public static final boolean kDisableSubsystemsOnDisableInit = true;

  // CAREFUL!
  public static final boolean kOverrideBrownOutVoltage = true;
  public static final double kOverridenBrownOutVoltage = 5.6;

  /* General robot configruation */
  public static final double stickDeadband = 0.1;
  public static final double azimuthStickDeadband = 0.3;
  // Logging
  public static final int kLogLinesBeforeFlush = 100;

  public static final class FeatureFlags {
    // subsystems

    public static final boolean kEasterEggEnabled = false;

    public static final boolean kIntakeEnabled = true;

    public static final boolean kShooterEnabled = true;

    public static final boolean kSwerveEnabled = true;

    public static final boolean kPivotEnabled = true;

    public static final boolean kClimbEnabled = true;
    public static final boolean kLEDEnabled = true;

    public static final boolean kAmpBarEnabled = true;

    // logging
    public static final boolean kDebugEnabled = false;
    public static final boolean DebugCommandEnabled = false;
    public static final boolean kRobotVizEnabled = true && !Robot.isReal();

    // features

    public static final boolean kAutoAlignEnabled = false;

    public static final boolean kLocalizationEnabled = false;

    // public static final boolean kSwerveAccelerationLimitingEnabled = true;
    public static final boolean kSwerveUseVisionForPoseEst =
        true; // ummm probably not disabling this
    public static final boolean kLocalizationDataCollectionMode = false;
    public static final boolean kLocalizationStdDistanceBased = false;

    // TODO:Setup the Limelight offset in the CAD into the LL config.
    // Make sure the pose is CORRECT WITHOUT VISION BEFORE ENAVLINGTHIS!!!!
    public static final boolean kSwerveVelocityLimitingEnabled = false;
    public static final boolean kIntakeAutoScoreDistanceSensorOffset = false;
    public static final boolean kShuffleboardLayoutEnabled = true; // TODO: use this
    public static final boolean kGamePieceDetection = false;
    public static final boolean kUsePrefs = true;

    public static final boolean kPitRoutineEnabled = false;

    public static final boolean kCanTestEnabled = false;
    public static final boolean kResetHeadingOnZeroGyro = true;
    public static final boolean kQuadraticDrive = false;
    public static boolean kPivotShooterEnabled = true;
  }

  public static final class ShuffleboardConstants {
    public static final String kDriverTabName = "Driver";
    public static final String kOperatorTabName = "Operator";
    public static final String kIntakeLayoutName = "Intake";
    public static final String kSwerveLayoutName = "Swerve";
    public static final String kArmLayoutName = "Arm";
    public static final String kShooterLayoutName = "Shooter";
  }
}
