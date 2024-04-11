// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.drivers.MonitoredTalonFX;

public class TalonUtil {

  public static boolean applyAndCheckConfiguration(
      MonitoredTalonFX talon, TalonFXConfiguration config, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (PhoenixUtil.checkErrorAndRetry(() -> talon.getConfigurator().apply(config))) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(talon, config)) {
          return true;
        } else {
          DriverStation.reportWarning(
              "Failed to verify config for talon ["
                  + talon.getDescription()
                  + "] (attempt "
                  + (i + 1)
                  + " of "
                  + numTries
                  + ")",
              false);
        }
      } else {
        DriverStation.reportWarning(
            "Failed to apply config for talon ["
                + talon.getDescription()
                + "] (attempt "
                + (i + 1)
                + " of "
                + numTries
                + ")",
            false);
      }
    }
    DriverStation.reportError(
        "Failed to apply config for talon after " + numTries + " attempts", false);
    return false;
  }

  public static boolean readAndVerifyConfiguration(
      MonitoredTalonFX talon, TalonFXConfiguration config) {
    TalonFXConfiguration readConfig = new TalonFXConfiguration();
    if (!PhoenixUtil.checkErrorAndRetry(() -> talon.getConfigurator().refresh(readConfig))) {
      // could not get config!
      DriverStation.reportWarning(
          "Failed to read config for talon [" + talon.getDescription() + "]", false);
      return false;
    } else if (!TalonConfigEquality.isEqual(config, readConfig)) {
      // configs did not match
      DriverStation.reportWarning(
          "Configuration verification failed for talon [" + talon.getDescription() + "]", false);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  public static boolean applyAndCheckConfiguration(
      MonitoredTalonFX talon, TalonFXConfiguration config) {
    boolean result = applyAndCheckConfiguration(talon, config, 5);

    if (!result) {
      DriverStation.reportWarning(
          "Failed to apply config for talon [" + talon.getDescription() + "]", false);
    }

    return result;
  }

  public enum StickyFault {
    BootDuringEnable,
    DeviceTemp,
    ForwardHardLimit,
    ForwardSoftLimit,
    Hardware,
    OverSupplyV,
    ProcTemp,
    ReverseHardLimit,
    ReverseSoftLimit,
    Undervoltage,
    UnstableSupplyV
  }

  public static void checkStickyFaults(String subsystemName, MonitoredTalonFX talon) {
    boolean[] faults = new boolean[StickyFault.values().length];
    faults[0] = talon.getStickyFault_BootDuringEnable().getValue();
    faults[1] = talon.getStickyFault_DeviceTemp().getValue();
    faults[2] = talon.getStickyFault_ForwardHardLimit().getValue();
    faults[3] = talon.getStickyFault_ForwardSoftLimit().getValue();
    faults[4] = talon.getStickyFault_Hardware().getValue();
    faults[5] = talon.getStickyFault_OverSupplyV().getValue();
    faults[6] = talon.getStickyFault_ProcTemp().getValue();
    faults[7] = talon.getStickyFault_ReverseHardLimit().getValue();
    faults[8] = talon.getStickyFault_ReverseSoftLimit().getValue();
    faults[9] = talon.getStickyFault_Undervoltage().getValue();
    faults[10] = talon.getStickyFault_UnstableSupplyV().getValue();

    for (int i = 0; i < faults.length; i++) {
      if (faults[i]) {
        DriverStation.reportError(
            subsystemName + ": Talon Fault! " + StickyFault.values()[i].toString(), false);
      }
    }

    talon.clearStickyFaults();
  }
}
