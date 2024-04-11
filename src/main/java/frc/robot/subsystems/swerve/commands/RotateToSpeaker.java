// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RotateToSpeaker extends PIDCommand {
  public RotateToSpeaker(SwerveDrive swerve) {
    super(
        new PIDController(
            SwerveConstants.crosshairAngleKP,
            SwerveConstants.crosshairAngleKI,
            SwerveConstants.crosshairAngleKD),
        () -> LimelightHelpers.getTX("limelight"),
        0.0,
        output -> swerve.setAngularVelocity(output),
        swerve);

    getController()
        .setTolerance(
            SwerveConstants.crosshairTurnTolerance, SwerveConstants.crosshairTurnToleranceVel);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
