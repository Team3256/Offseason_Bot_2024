// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.requests;

import static frc.robot.Constants.*;
import static frc.robot.RobotContainer.MaxAngularRate;
import static frc.robot.RobotContainer.MaxSpeed;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.function.DoubleSupplier;

// TODO: implement swerve request

public class Azimuth extends DebugCommandBase {
  private final CommandXboxController controller;

  private final CommandSwerveDrivetrain swerveSubsystem;
  private double angle;

  private final SwerveRequest.FieldCentric azimuthDrive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.stickDeadband * MaxSpeed)
          .withRotationalDeadband(azimuthStickDeadband * MaxAngularRate)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  /** Driver control */
  public Azimuth(
      CommandXboxController controller,
      CommandSwerveDrivetrain swerveSubsystem,
      DoubleSupplier anglePreset) {
    this.controller = controller;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.angle = anglePreset.getAsDouble();
    SwerveConstants.azimuthController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    // calculate error
    double rotationPIDOutput =
        SwerveConstants.azimuthController.calculate(
            swerveSubsystem.getPigeon2().getAngle(), angle, Timer.getFPGATimestamp());

    rotationPIDOutput = MathUtil.clamp(rotationPIDOutput, -(1.7 * Math.PI), (1.7 * Math.PI));

    double finalRotationPIDOutput = rotationPIDOutput;
    swerveSubsystem.applyRequest(
        () ->
            azimuthDrive
                .withVelocityX(controller.getLeftY() * MaxSpeed)
                .withVelocityY(controller.getLeftX() * MaxSpeed)
                .withRotationalRate(finalRotationPIDOutput * MaxAngularRate));
  }

  @Override
  public boolean isFinished() {
    return azimuthController.atSetpoint();
  }
}
