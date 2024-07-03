// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.requests;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// josh: also a complete copy of com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds
// except for replacing DriveRequestType as Velocity instead of "OpenLoopVoltage"

/**
 * Accepts a generic ChassisSpeeds to apply to the drivetrain, but runs as
 * Velocity instead of
 * Voltage.
 */
public class SwerveApplyChassisSpeeds implements SwerveRequest {

    /** The chassis speeds to apply to the drivetrain. */
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    /** The center of rotation to rotate around. */
    public Translation2d CenterOfRotation = new Translation2d(0, 0);
    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.Velocity;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(
            SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        var states = parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
        }

        return StatusCode.OK;
    }

    /**
     * Sets the chassis speeds to apply to the drivetrain.
     *
     * @param speeds Chassis speeds to apply to the drivetrain
     * @return this request
     */
    public SwerveApplyChassisSpeeds withSpeeds(ChassisSpeeds speeds) {
        this.Speeds = speeds;
        return this;
    }

    /**
     * Sets the center of rotation to rotate around.
     *
     * @param centerOfRotation Center of rotation to rotate around
     * @return this request
     */
    public SwerveApplyChassisSpeeds withCenterOfRotation(Translation2d centerOfRotation) {
        this.CenterOfRotation = centerOfRotation;
        return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive
     *                         motor
     * @return this request
     */
    public SwerveApplyChassisSpeeds withDriveRequestType(
            SwerveModule.DriveRequestType driveRequestType) {
        this.DriveRequestType = driveRequestType;
        return this;
    }

    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer
     *                         motor
     * @return this request
     */
    public SwerveApplyChassisSpeeds withSteerRequestType(
            SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }
}
