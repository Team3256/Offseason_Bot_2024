// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.requests;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveFieldCentricFacingAngle extends SwerveRequest.FieldCentricFacingAngle
    implements SwerveRequest {

  /**
   * Sets the velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   *
   * @param velocityX Velocity in the X direction, in m/s
   * @return this request
   */
  @Override
  public SwerveFieldCentricFacingAngle withVelocityX(double velocityX) {
    super.VelocityX = velocityX;
    return this;
  }

  /**
   * Sets the velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   *
   * @param velocityY Velocity in the Y direction, in m/s
   * @return this request
   */
  @Override
  public SwerveFieldCentricFacingAngle withVelocityY(double velocityY) {
    super.VelocityY = velocityY;
    return this;
  }

  /**
   * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X axis. As
   * a result, a TargetDirection of 90 degrees will point along the Y axis, or to the left.
   *
   * @param targetDirection Desired direction to face
   * @return this request
   */
  @Override
  public SwerveFieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
    super.TargetDirection = targetDirection;
    return this;
  }

  /**
   * Sets the allowable deadband of the request.
   *
   * @param deadband Allowable deadband of the request
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withDeadband(double deadband) {
    super.Deadband = deadband;
    return this;
  }

  /**
   * Sets the rotational deadband of the request.
   *
   * @param rotationalDeadband Rotational deadband of the request
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
    super.RotationalDeadband = rotationalDeadband;
    return this;
  }

  /**
   * Sets the center of rotation of the request
   *
   * @param centerOfRotation The center of rotation the robot should rotate around.
   * @return this request
   */
  public SwerveFieldCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
    super.CenterOfRotation = centerOfRotation;
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
    super.DriveRequestType = driveRequestType;
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
    super.SteerRequestType = steerRequestType;
    return this;
  }

  public SwerveFieldCentricFacingAngle withHeadingController(
      PhoenixPIDController headingController) {
    super.HeadingController = headingController;
    return this;
  }
}
