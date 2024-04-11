// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import static frc.robot.Constants.FeatureFlags.kQuadraticDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends DebugCommandBase {
  private SwerveDrive swerveDrive;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  public TeleopSwerve(
      SwerveDrive swerveDrive,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal =
        MathUtil.applyDeadband(-translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(-strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal =
        MathUtil.applyDeadband(-rotationSup.getAsDouble(), Constants.stickDeadband);

    /* Drive */
    if (kQuadraticDrive) {
      swerveDrive.drive(
          new Translation2d(
              SwerveDrive.joystickToSpeed(translationVal, SwerveConstants.maxTranslationalVelocity),
              SwerveDrive.joystickToSpeed(strafeVal, SwerveConstants.maxTranslationalVelocity)),
          SwerveDrive.joystickToSpeed(rotationVal, SwerveConstants.maxAngularVelocity),
          true,
          true);
    } else {
      swerveDrive.drive(
          new Translation2d(translationVal, strafeVal)
              .times(SwerveConstants.maxTranslationalVelocity),
          rotationVal * SwerveConstants.maxAngularVelocity,
          true,
          true);
    }
  }
}
