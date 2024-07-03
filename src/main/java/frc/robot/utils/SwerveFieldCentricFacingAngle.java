// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveFieldCentricFacingAngle implements SwerveRequest {
  /**
   * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   */
  public double VelocityX = 0;
  /**
   * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   */
  public double VelocityY = 0;
  /**
   * The desired direction to face. 0 Degrees is defined as in the direction of the X axis. As a
   * result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
   */
  public Rotation2d TargetDirection = new Rotation2d();

  /** The allowable deadband of the request. */
  public double Deadband = 0;
  /** The rotational deadband of the request. */
  public double RotationalDeadband = 0;
  /**
   * The center of rotation the robot should rotate around. This is (0,0) by default, which will
   * rotate around the center of the robot.
   */
  public Translation2d CenterOfRotation = new Translation2d();

  /** The type of control request to use for the drive motor. */
  public SwerveModule.DriveRequestType DriveRequestType =
      SwerveModule.DriveRequestType.OpenLoopVoltage;
  /** The type of control request to use for the steer motor. */
  public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

  /**
   * The PID controller used to maintain the desired heading. Users can specify the PID gains to
   * change how aggressively to maintain heading.
   *
   * <p>This PID controller operates on heading radians and outputs a target rotational rate in
   * radians per second.
   */
  public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

  /** The perspective to use when determining which direction is forward. */
  public ForwardReference ForwardReference = SwerveRequest.ForwardReference.OperatorPerspective;

  public StatusCode apply(
      SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    double toApplyX = VelocityX;
    double toApplyY = VelocityY;
    Rotation2d angleToFace = TargetDirection;
    if (ForwardReference == SwerveRequest.ForwardReference.OperatorPerspective) {
      /* If we're operator perspective, modify the X/Y translation by the angle */
      Translation2d tmp = new Translation2d(toApplyX, toApplyY);
      tmp = tmp.rotateBy(parameters.operatorForwardDirection);
      toApplyX = tmp.getX();
      toApplyY = tmp.getY();
      /* And rotate the direction we want to face by the angle */
      angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
    }

    double rotationRate =
        HeadingController.calculate(
            parameters.currentPose.getRotation().getRadians(),
            angleToFace.getRadians(),
            parameters.timestamp);

    double toApplyOmega = rotationRate;
    if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
      toApplyX = 0;
      toApplyY = 0;
    }
    if (Math.abs(toApplyOmega) < RotationalDeadband) {
      toApplyOmega = 0;
    }

    ChassisSpeeds speeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                toApplyX, toApplyY, toApplyOmega, parameters.currentPose.getRotation()),
            parameters.updatePeriod);

    var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

    for (int i = 0; i < modulesToApply.length; ++i) {
      modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
    }

    return StatusCode.OK;
  }

  /**
   * Sets the velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   *
   * @param velocityX Velocity in the X direction, in m/s
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withVelocityX(double velocityX) {
    this.VelocityX = velocityX;
    return this;
  }

  /**
   * Sets the velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   *
   * @param velocityY Velocity in the Y direction, in m/s
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withVelocityY(double velocityY) {
    this.VelocityY = velocityY;
    return this;
  }

  /**
   * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X axis. As
   * a result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
   *
   * @param targetDirection Desired direction to face
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
    this.TargetDirection = targetDirection;
    return this;
  }

  /**
   * Sets the allowable deadband of the request.
   *
   * @param deadband Allowable deadband of the request
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withDeadband(double deadband) {
    this.Deadband = deadband;
    return this;
  }
  /**
   * Sets the rotational deadband of the request.
   *
   * @param rotationalDeadband Rotational deadband of the request
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
    this.RotationalDeadband = rotationalDeadband;
    return this;
  }
  /**
   * Sets the center of rotation of the request
   *
   * @param centerOfRotation The center of rotation the robot should rotate around.
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
    this.CenterOfRotation = centerOfRotation;
    return this;
  }

  /**
   * Sets the type of control request to use for the drive motor.
   *
   * @param driveRequestType The type of control request to use for the drive motor
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withDriveRequestType(
      SwerveModule.DriveRequestType driveRequestType) {
    this.DriveRequestType = driveRequestType;
    return this;
  }
  /**
   * Sets the type of control request to use for the steer motor.
   *
   * @param steerRequestType The type of control request to use for the steer motor
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withSteerRequestType(
      SwerveModule.SteerRequestType steerRequestType) {
    this.SteerRequestType = steerRequestType;
    return this;
  }

  public SwerveFieldCentricFacingAngle withHeadingController(
      PhoenixPIDController headingController) {
    this.HeadingController = headingController;
    return this;
  }
}
