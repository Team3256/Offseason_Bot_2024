// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.robotviz;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotVizConstants {
  // --Visualization--
  // measurements are in meters
  // basic
  public static final double kLineWidth = 5;
  public static final double botLength = Units.inchesToMeters(33); // exact
  public static final double botHeight = Units.inchesToMeters(33);
  public static final double kRobotSimWindowWidth = Units.inchesToMeters(35);
  public static final double kRobotSimWindowHeight = Units.inchesToMeters(35);
  public static final double kRootX = 0;
  public static final double kRootY = 0;
  // intake
  public static final double kIntakeRadius = Units.inchesToMeters(2); // estimate
  // climber
  public static final double kClimberX = Units.inchesToMeters(13); // exact
  public static final double kClimberSpacing = Units.inchesToMeters(3); // spoof
  public static final double kClimberSpoutHeight = Units.inchesToMeters(17); // estimate
  // shooter
  public static final Rotation2d shooterAngle = Rotation2d.fromDegrees(180 - 55); // exact
  public static final double shooterX = Units.inchesToMeters(21); // estimate
  public static final double shooterOffset = Units.inchesToMeters(23); // estimate
  public static final double shooterRadius = Units.inchesToMeters(2); // estimate
  // feeder
  public static final double feederRadius = Units.inchesToMeters(2); // estimate
  public static final double feederOffset = Units.inchesToMeters(18); // estimate
  // arm
  public static final double armX = Units.inchesToMeters(28); // estimate
  public static final double armY = Units.inchesToMeters(5); // estimate
  public static final double armLength = Units.inchesToMeters(10); // exact
  public static final double handLength = Units.inchesToMeters(5); // exact
  // colors
  public static final Color8Bit orange = new Color8Bit(235, 137, 52);
  public static final Color8Bit red = new Color8Bit(255, 0, 0);
  public static final Color8Bit purple = new Color8Bit(255, 0, 255);
  public static final Color8Bit blue = new Color8Bit(0, 0, 255);
  public static final Color8Bit green = new Color8Bit(0, 255, 0);
  public static final Color8Bit white = new Color8Bit(255, 255, 255);

  // --Spoof--
  // IRL motor RPM is very high and we won't be able to understand the data we see. So, we slow
  // down the visualization of the motor speeds by a large constant factor.
  public static final double shooterVelocitySimDamp = 0.1; // TODO: tune
  public static final double intakeVelocitySimDamp = 0.1; // TODO: tune
  public static final double feederVelocitySimDamp = 0.1; // TODO: tune
}
