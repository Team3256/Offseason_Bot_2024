// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;

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
  public static final double rotationalDeadband = 0.1;
  public static final double azimuthStickDeadband = 0.3;
  // Logging
  public static final int kLogLinesBeforeFlush = 100;
  public static final boolean kMonologueFileOnly = false;
  public static final boolean kMonologueLazyLogging = false;

  public static final class FeatureFlags {
    // subsystems

    public static final boolean kEasterEggEnabled = false;

    public static final boolean kIntakeEnabled = true;

    public static final boolean kShooterEnabled = true;
    public static final boolean kShooterRegenerativeBrakingEnabled = true;

    public static final boolean kSwerveEnabled = true;

    public static final boolean kPivotIntakeEnabled = true;

    public static final boolean kClimbEnabled = true;
    public static final boolean kLEDEnabled = true;

    public static final boolean kAmpBarEnabled = true;

    // logging
    public static final boolean kTuningMode = false;
    public static final boolean kDebugEnabled = false;
    public static final boolean DebugCommandEnabled = false;
    public static final boolean kRobotVizEnabled = true && !Robot.isReal();

    // features
    public static final boolean kAutoAlignEnabled = true;
    public static final boolean kIntakeAutoScoreDistanceSensorOffset = false;
    public static final boolean kShuffleboardLayoutEnabled = true;
    public static final boolean kUsePrefs = true;

    public static final boolean kPitRoutineEnabled = false;

    public static final boolean kCanTestEnabled = false;
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

  public static final class FieldConstants {
    public static final ArrayList<Pose2d> kStagePosesBlue =
        new ArrayList<>() {
          {
            // Podium
            add(new Pose2d(2.657515287399292, 4.105274677276611, new Rotation2d(0)));
            // Amp side
            add(
                new Pose2d(
                    5.990447521209717, 6.171302795410156, new Rotation2d(-2.015216124571914)));
            // Etc
            add(
                new Pose2d(
                    6.10739278793335, 2.1172094345092773, new Rotation2d(2.118189174278151)));
          }
        };
    public static final ArrayList<Pose2d> kStagePosesRed =
        new ArrayList<>() {
          {
            // Podium
            add(new Pose2d(13.981689453125, 4.105274677276611, new Rotation2d(3.1415926536)));
            // Amp side
            add(
                new Pose2d(
                    10.512321472167969, 6.11283016204834, new Rotation2d(-1.0445000982232164)));
            // Etc
            add(
                new Pose2d(
                    10.492830276489258, 2.0977187156677246, new Rotation2d(1.0214219124306612)));
          }
        };
  }
}
