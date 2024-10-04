// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

public final class AzimuthConstants {
  /* raw angles in degrees. positive angles mean left turns */

  public static final double aziAmpRed = 90;

  public static final double aziAmpBlue = -aziAmpRed;

  public static final double aziSubwooferFront = 0;

  public static final double aziSubwooferLeft = -30;

  public static final double aziSubwooferRight = -aziSubwooferLeft;

  public static final double aziSourceRed = 60;

  public static final double aziSourceBlue = -aziSourceRed;

  public static final double aziFeederRed = 45;

  public static final double aziFeederBlue = -aziFeederRed;

  public static final double aziCleanUp = 180;

  /* Timeout */
  public static final double aziCommandTimeOut = 1.5;
}
