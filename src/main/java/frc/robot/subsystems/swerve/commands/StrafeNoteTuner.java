// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;

public class StrafeNoteTuner extends DebugCommandBase {
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveSubsystem;

  private PIDController strafePidController;

  /** Driver control */
  public StrafeNoteTuner(
      SwerveDrive swerveSubsystem,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;

    this.strafePidController = new PIDController(strafeNoteKP, strafeNoteKI, strafeNoteKD);
    strafePidController.setTolerance(strafeNoteTolerance, strafeNoteToleranceVel);
    strafePidController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    System.out.println("SAM NO MORE TRANSLATION FOR YOU");
    // Obtains axis values for x and y for translation command

    // Safety area, insures that joystick movement will not be tracked within a
    // certain area,
    // prevents unintentional drifting

    // Converts from coordinates to angle, sets joystick forward input as 0,
    // converts angle to
    // degrees

    // PID controller takes current robot position (getYaw) and compares to the
    // azimuth angle to
    // calculate error
    double compensatedMaxVelocity = maxTranslationalVelocity * 1;

    double strafePIDOutput = strafePidController.calculate(LimelightHelpers.getTX("limelight-note") * -1, 0.0);
    strafePIDOutput = MathUtil.clamp(strafePIDOutput, -compensatedMaxVelocity, compensatedMaxVelocity);
    translation = new Translation2d(0, strafePIDOutput);

    swerveSubsystem.drive(translation, 0, fieldRelative, openLoop);
  }

  @Override
  public boolean isFinished() {
    // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
    // return rotateToSpeakerController.atSetpoint();
    return false;
  }
}
