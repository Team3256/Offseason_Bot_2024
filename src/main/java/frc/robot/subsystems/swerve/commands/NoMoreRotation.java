// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import static frc.robot.Constants.stickDeadband;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class NoMoreRotation extends DebugCommandBase {
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveSubsystem;
  private DoubleSupplier translationAxis;
  private DoubleSupplier strafeAxis;
  private PIDController rotateToSpeakerController;

  /** Driver control */
  public NoMoreRotation(
      SwerveDrive swerveSubsystem,
      DoubleSupplier translationAxis,
      DoubleSupplier strafeAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.rotateToSpeakerController =
        new PIDController(
            SwerveConstants.crosshairAngleKP,
            SwerveConstants.crosshairAngleKI,
            SwerveConstants.crosshairAngleKD);
    rotateToSpeakerController.setTolerance(0, SwerveConstants.crosshairTurnToleranceVel);
    rotateToSpeakerController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    System.out.println("nmr running");
    // Obtains axis values for x and y for translation command
    double yAxis = -translationAxis.getAsDouble();
    double xAxis = -strafeAxis.getAsDouble();

    // Safety area, insures that joystick movement will not be tracked within a
    // certain area,
    // prevents unintentional drifting
    yAxis = (Math.abs(yAxis) < stickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < stickDeadband) ? 0 : xAxis;

    translation = new Translation2d(yAxis, xAxis).times(maxTranslationalVelocity * 0.8);

    // Converts from coordinates to angle, sets joystick forward input as 0,
    // converts angle to
    // degrees

    // PID controller takes current robot position (getYaw) and compares to the
    // azimuth angle to
    // calculate error

    double rotationPIDOutput =
        rotateToSpeakerController.calculate(LimelightHelpers.getTX("limelight") / 2, 0.0);
    System.out.println("LimelightTX" + LimelightHelpers.getTX("limelight"));
    System.out.println("NMRRotationOutput " + rotationPIDOutput);
    //     Units.radiansToDegrees(rotationPIDOutput));
    Logger.recordOutput("NMRrotationOutput", rotationPIDOutput);
    Logger.recordOutput("NMRrotationOutputDeg", Units.radiansToDegrees(rotationPIDOutput));
    Logger.recordOutput("NMRrotationSetpoint", 0.0);

    translation = new Translation2d(yAxis, xAxis).times(maxTranslationalVelocity);
    rotationPIDOutput = MathUtil.clamp(rotationPIDOutput, -maxAngularVelocity, maxAngularVelocity);

    swerveSubsystem.drive(translation, rotationPIDOutput, fieldRelative, openLoop);
  }

  @Override
  public boolean isFinished() {
    // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
    // return rotateToSpeakerController.atSetpoint();
    return false;
  }
}
