// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

public final class LEDConstants {
  /* CAN */
  public static final int kLEDCANID = 40;
  public static final String kLEDCanBus = "mani";

  /* LED Configurations */
  public static final int kLEDLength = 256;
  public static final double kLEDSpeed = 0.5;

  public static final int kLEDWidth = 32;
  public static final int kLEDHeight = 8;

  public static final String[] based = {
      "B10000001111001111110011111011000",
      "B10000010000101000000010000010100",
      "B10000010000101000000010000010010",
      "B10000010000101111110010000010001",
      "B11111011111100000010011111010001",
      "B10001010000100000010010000010001",
      "B10001010000101111110010000010001",
      "B11011010000100000000011111011111"
  };
}
