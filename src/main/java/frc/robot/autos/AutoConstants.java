// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class AutoConstants {
  // TODO: The below constants are used in the example auto, and must be tuned to
  // specific robot
  public static final double kMaxSpeedMetersPerSecond = 4.275;
  public static final double kDriveBaseRadiusMeters = SwerveConstants.driveBaseRadius;
  public static final double kPathGenerationEndGoalVelocity = 0.0;
  public static final double kPathGenerationRotationalDelay =
      0.01; // how long it should travel before rotating
  public static final double kMaxAccelerationMetersPerSecondSquared = 5;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

  public static final Pose2d kRedSpeakerLocation =
      new Pose2d(15.26, 5.56, Rotation2d.fromDegrees(-180));

  public static final Pose2d kBlueSpeakerLocation =
      new Pose2d(1.27, 5.56, Rotation2d.fromDegrees(180));

  public static final Pose2d kRedAmpLocation = new Pose2d(14.74, 7.48, Rotation2d.fromDegrees(90));

  public static final Pose2d kBlueAmpLocation = new Pose2d(1.81, 7.57, Rotation2d.fromDegrees(90));

  public static final double kSpeakerAlignmentThreshold = 0.2; // meters

  public static final double kPXController = 1;
  public static final double kPYController = 1;
  public static final double kPThetaController = 1;

  /* Constraint for the motion profilied robot angle controller */
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}
