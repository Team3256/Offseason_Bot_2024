// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RotateToNote extends PIDCommand {
  private final SwerveDrive swerve;

  public RotateToNote(SwerveDrive swerve) {
    super(
        new PIDController(
            SwerveConstants.noteTurnkP,
            SwerveConstants.noteTurnkI,
            SwerveConstants.noteTurnkD), // TODO: Tune PID so it doesn't act like a bang-bang
        () -> {
          double dx;
          double dy;
          if (Limelight.getPythonScriptData("limelight-bottom").length >= 2) {
            dx =
                SwerveConstants.noteTargetTurnX
                    - Limelight.getPythonScriptData("limelight-bottom")[0];
            dy =
                SwerveConstants.noteTargetTurnY
                    - Limelight.getPythonScriptData("limelight-bottom")[1];
          } else {
            dx = 0.9;
            dy = 0.3;
          }
          double degrees = -360 / (2 * Math.PI) * Math.atan(dx / dy);
          System.out.println("Degrees: " + degrees);
          return degrees;
        },
        0.0,
        (output) ->
            swerve.setAngularVelocity(output + Math.signum(output) * SwerveConstants.noteTurnkF),
        swerve);
    this.swerve = swerve;
    this.getController()
        .setTolerance(
            SwerveConstants.noteAngleTolerance, SwerveConstants.noteAngularVelocityTolerance);
  }

  @Override
  public void initialize() {
    super.initialize();
    System.out.println("Init: RotateToNote");
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println("End: RotateToNote (interrupted = " + interrupted + ")");
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
