// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
/// It's an enum as it's the easiest way to
/// make a singleton in Java.
/// Please also note that this class does not contain
/// Oblog initialization code or initialization code
/// for any other logging outlets

public enum WBLogger {
  INSTANCE;

  private Kattio io;
  private int linesPrinted = 0;

  private WBLogger() {
    this.io = new Kattio(System.in, System.out);
  }

  public static WBLogger getInstance() {
    return INSTANCE;
  }

  public <T> StatusSignal<T> monitorStatusSignal(
      StatusSignal<T> signal,
      int canDeviceID,
      String canDeviceBus,
      String itemName,
      String methodName) {
    Logger.recordOutput(
        itemName + " | " + canDeviceID + " | " + canDeviceBus,
        "Signal in "
            + signal.getName()
            + "( "
            + methodName
            + " ): "
            + signal.getStatus().toString()
            + " canDeviceID: "
            + canDeviceID
            + " canDeviceBus: "
            + canDeviceBus
            + " Device Name: "
            + signal.getName());
    if (signal.getStatus().isError()) {
      println("---- ERROR ----");
      println(
          "Error in "
              + signal.getName()
              + "( "
              + methodName
              + " )"
              + ": "
              + signal.getStatus().toString());
      println("canDeviceID: " + canDeviceID + " canDeviceBus: " + canDeviceBus);
      println("Device Name: " + signal.getName());
      println("---- ERROR ----");
    }
    return signal;
  }

  private void println(String message) {
    //    io.println(message);
    //    linesPrinted += 1;
    //    if (linesPrinted >= Constants.kLogLinesBeforeFlush) {
    //      io.flush();
    //      linesPrinted = 0;
    //    }
  }

  public void info(String message) {
    println("[INFO]: " + message);
  }

  public void warn(String message) {
    println("[WARNING]: " + message);
    DriverStation.reportWarning("[WARNING]: " + message, false);
  }

  public void error(String message) {
    println("[ERROR]: " + message);
    DriverStation.reportError("[ERROR]: " + message, false);
  }

  public void error(String message, ErrorCode errorCode) {
    println("[ERROR]: " + message + " | Error Code: " + errorCode);
    DriverStation.reportError("[ERROR]: " + message, false);
    println("Traceback (most recent call last)");
    for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
      println(ste + "\n");
    }
  }

  // getters and setters
}
