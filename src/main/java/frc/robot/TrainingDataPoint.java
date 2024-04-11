// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

// A helper class to store training data points for the robot's localization algorithm.
// Each data point contains the distance traveled by the robot, and the standard
// deviation of the robot's x and y translation and angle.
public class TrainingDataPoint {

  public double distance;
  public double stdDevXTranslation;
  public double stdDevYTranslation;
  public double stdDevAngle;

  public TrainingDataPoint(
      double distance, double stdDevXTranslation, double stdDevYTranslation, double stdDevAngle) {
    this.distance = distance;
    this.stdDevXTranslation = stdDevXTranslation;
    this.stdDevYTranslation = stdDevYTranslation;
    this.stdDevAngle = stdDevAngle;
  }
}
