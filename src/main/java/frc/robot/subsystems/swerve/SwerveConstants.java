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
      new PhoenixPIDController(3.65, 0, 0.55); // you shouldn't have a I or D value TODO: tune
  public static final PIDConstants autoRotationalController =
      new PIDConstants(6, azimuthController.getI(), azimuthController.getD()); // 6
  public static final PIDConstants autoTranslationalController = new PIDConstants(6, 0, 0); // 8

  public static final double azimuthEpsilon = 10.0; // TODO: tune

  public static final PIDController choreoTranslationController = new PIDController(6, 0, 0);
  public static final PIDController choreoRotationController =
      new PIDController(
          azimuthController.getP(), azimuthController.getI(), azimuthController.getD());

  public static final class AzimuthAngles {
    /* raw angles in degrees. positive angles mean left turns */

    public static final double aziAmpRed = 90 - 180;

    public static final double aziAmpBlue = -aziAmpRed;

    public static final double aziSubwooferFront = 0 - 180;

    public static final double aziSubwooferLeft = -30 - 180;

    public static final double aziSubwooferRight = -aziSubwooferLeft;

    public static final double aziSourceRed = 60 - 180;

    public static final double aziSourceBlue = -aziSourceRed;

    public static final double aziFeederRed = 45 - 180;

    public static final double aziFeederBlue = -aziFeederRed;

    public static final double aziCleanUp = 180 - 180;

    /* Timeout */
    public static final double aziCommandTimeOut = 1.5;
  }
}
