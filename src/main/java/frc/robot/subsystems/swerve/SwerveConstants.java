// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;

public final class SwerveConstants {
  public static final PhoenixPIDController azimuthController =
      new PhoenixPIDController(8, 0, 0); // you shouldn't have a I or D value TODO: tune
  public static final PIDConstants autoRotationalController =
      new PIDConstants(6, azimuthController.getI(), azimuthController.getD()); // 6
  public static final PIDConstants autoTranslationalController = new PIDConstants(6, 0, 0); // 8

  public static final double azimuthEpsilon = 10.0; // TODO: tune

  public static final PIDController choreoTranslationController = new PIDController(6, 0, 0);
  public static final PIDController choreoRotationController =
      new PIDController(
          azimuthController.getP(), azimuthController.getI(), azimuthController.getD());
}
