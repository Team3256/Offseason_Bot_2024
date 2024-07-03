// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.PIDConstants;
import java.util.List;

public final class SwerveConstants {
  public static final PhoenixPIDController azimuthController =
      new PhoenixPIDController(1, 0, 0); // you shouldn't have a I or D value TODO: tune
  public static final PIDConstants autoRotationalController =
      new PIDConstants(
          azimuthController.getP(),
          azimuthController.getI(),
          azimuthController.getD()); // should be the same
  public static final PIDConstants autoTranslationalController =
      new PIDConstants(1, 0, 0); // you shouldn't have a I or D value TODO: tune

  public static final List<Double> azimuthAngles = List.of(0.0, 90.0, 180.0, 270.0);
  public static final double azimuthEpsilon = 10.0; // TODO: tune
}
