// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants.FeatureFlags;

public class StaticThreadChecker {
  // A piece of code that checks if the current thread is the main thread.
  // If it is, it will print a message and stop the thread to prevent drive lag.
  // If kDebugEnabled is true, it will throw a RuntimeException.

  public static void checkCurrentThread() {
    if (Thread.currentThread().getId() == 1) {
      String functionName = Thread.currentThread().getStackTrace()[2].getMethodName();
      System.out.println(functionName + " called from main thread; stopping to prevent drive lag!");
      if (FeatureFlags.kDebugEnabled) {
        throw new RuntimeException(
            functionName + " called from main thread; crashing since kDebugEnabled is true!");
      }
      return;
    }
  }
}
