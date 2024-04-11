// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb.helpers;

import frc.robot.subsystems.climb.ClimbConstants;

public class ClimbUnitConverter {
  private static final double gearRatio = ClimbConstants.gearRatio;
  private static final double wheelRadius = ClimbConstants.wheelRadius;

  // public MetersAndRotations(double gearRatio, double wheelRadius) {

  //   this.gearRatio = gearRatio;
  //   this.wheelRadius = wheelRadius;
  // }

  // public MetersAndRotations() {} // Unnecessary? I don't know

  // CTRE docs :)
  public static double convertMetersToRotations(double meters) {
    // double gearRatio = Constants.ClimbConstants.gearRatio;
    // double wheelRadius = Constants.ClimbConstants.wheelRadius;
    return (meters * gearRatio) / (wheelRadius * 2 * Math.PI);
  }

  // Algebra :)
  public static double convertRotationsToMeters(double rotations) {
    // double gearRatio = Constants.ClimbConstants.gearRatio;
    // double wheelRadius = Constants.ClimbConstants.wheelRadius;
    return (rotations * 2 * Math.PI * wheelRadius) / (gearRatio);
  }
}
