// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import static frc.robot.Constants.azimuthStickDeadband;
import static frc.robot.Constants.stickDeadband;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.aziDrivekP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Azimuth extends DebugCommandBase {
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveSubsystem;
  private DoubleSupplier translationAxis;
  private DoubleSupplier strafeAxis;
  private DoubleSupplier rotationXAxis;
  private DoubleSupplier rotationYAxis;
  private DoubleSupplier setpointAngle;
  private BooleanSupplier manualRotating;
  private PIDController azimuthController;

  /** Driver control */
  public Azimuth(
      SwerveDrive swerveSubsystem,
      DoubleSupplier translationAxis,
      DoubleSupplier strafeAxis,
      DoubleSupplier rotationXAxis,
      DoubleSupplier rotationYAxis,
      DoubleSupplier setpointAngle,
      BooleanSupplier manualRotating,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationXAxis = rotationXAxis;
    this.rotationYAxis = rotationYAxis;
    this.setpointAngle = setpointAngle;
    this.manualRotating = manualRotating;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.azimuthController = new PIDController(aziDrivekP, aziDrivekI, aziDrivekD);
    azimuthController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    // Obtains axis values for x and y for translation command
    double yAxis = -translationAxis.getAsDouble();
    double xAxis = -strafeAxis.getAsDouble();

    // Gets position of joystick input on the x axis of the total stick area
    double rAxisX = rotationXAxis.getAsDouble();
    // Gets position of joystick input on the y axis of the total stick area
    double rAxisY = -rotationYAxis.getAsDouble();

    // Safety area, insures that joystick movement will not be tracked within a
    // certain area,
    // prevents unintentional drifting
    yAxis = (Math.abs(yAxis) < stickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < stickDeadband) ? 0 : xAxis;
    rAxisX = (Math.abs(rAxisX) < azimuthStickDeadband) ? 0 : rAxisX;
    rAxisY = (Math.abs(rAxisY) < azimuthStickDeadband) ? 0 : rAxisY;

    translation = new Translation2d(yAxis, xAxis).times(maxTranslationalVelocity);
    if (rAxisX == 0 && rAxisY == 0) {
      swerveSubsystem.drive(translation, 0, fieldRelative, openLoop);
      return;
    }

    // Converts from coordinates to angle, sets joystick forward input as 0,
    // converts angle to
    // degrees
    double azimuthAngle = setpointAngle.getAsDouble();

    // PID controller takes current robot position (getYaw) and compares to the
    // azimuth angle to
    // calculate error

    double rotationPIDOutput =
        azimuthController.calculate(swerveSubsystem.getHeading().getDegrees(), azimuthAngle);

    Logger.recordOutput("rotationOutput", rotationPIDOutput);
    Logger.recordOutput("rotationOutputDeg", Units.radiansToDegrees(rotationPIDOutput));
    Logger.recordOutput("rotationSetpoint", azimuthAngle);

    translation = new Translation2d(yAxis, xAxis).times(maxTranslationalVelocity);
    rotationPIDOutput = MathUtil.clamp(rotationPIDOutput, -maxAngularVelocity, maxAngularVelocity);

    swerveSubsystem.drive(translation, rotationPIDOutput, fieldRelative, openLoop);
  }

  @Override
  public boolean isFinished() {
    // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
    return azimuthController.atSetpoint();
  }
}
