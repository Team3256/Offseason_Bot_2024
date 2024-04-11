// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos.commands;

import static frc.robot.subsystems.swerve.SwerveConstants.noteDrivekF;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class MoveToNote extends PIDCommand {
  SwerveDrive swerveDrive;
  Intake intake;

  public MoveToNote(SwerveDrive swerve, Intake intake) {
    super(
        new PIDController(
            SwerveConstants.noteDrivekP, SwerveConstants.noteDrivekI, SwerveConstants.noteDrivekD),
        () -> {
          double dx;
          double dy;
          if (Limelight.getPythonScriptData("limelight-bottom").length >= 2) {
            dx =
                SwerveConstants.noteTargetDriveX
                    - Limelight.getPythonScriptData("limelight-bottom")[0];
            dy =
                SwerveConstants.noteTargetDriveY
                    - Limelight.getPythonScriptData("limelight-bottom")[1];
          } else {
            dx = 0.9;
            dy = 0.3;
          }
          double distance = -dy;
          System.out.println("Distance: " + distance);
          return distance;
        },
        0,
        (output) -> swerve.setVelocity(output + Math.signum(output) * noteDrivekF),
        swerve);
    this.m_controller.setTolerance(
        SwerveConstants.noteDistTolerance, SwerveConstants.noteVelocityTolerance);
    this.swerveDrive = swerve;
    this.intake = intake;
  }

  @Override
  public void initialize() {
    super.initialize();
    System.out.println("Init: MoveToNote");
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    swerveDrive.setVelocity(0);
    System.out.println("End: MoveToNote (interrupted = " + interrupted + ")");
  }

  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint() || intake.isBeamBroken();
  }
}
