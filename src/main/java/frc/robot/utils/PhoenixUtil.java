// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class PhoenixUtil {
  /**
   * checks the specified error code for issues
   *
   * @param errorCode error code
   * @param message message to print if error happens
   */
  public static void checkError(StatusCode errorCode, String message) {
    if (errorCode != StatusCode.OK) {
      DriverStation.reportError(message + " " + errorCode, false);
    }
  }

  public static boolean checkErrorAndRetry(Supplier<StatusCode> function, int numTries) {
    StatusCode code = function.get();
    int tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
      code = function.get();
      tries++;
    }
    if (code != StatusCode.OK) {
      DriverStation.reportError(
          "Failed to execute phoenix pro api call after " + numTries + " attempts", false);
      return false;
    }
    return true;
  }

  /**
   * checks the specified error code and throws an exception if there are any issues
   *
   * @param errorCode error code
   * @param message message to print if error happens
   */
  public static void checkErrorWithThrow(StatusCode errorCode, String message) {
    if (errorCode != StatusCode.OK) {
      throw new RuntimeException(message + " " + errorCode);
    }
  }

  public static boolean checkErrorAndRetry(Supplier<StatusCode> function) {
    return checkErrorAndRetry(function, 5);
  }
}
