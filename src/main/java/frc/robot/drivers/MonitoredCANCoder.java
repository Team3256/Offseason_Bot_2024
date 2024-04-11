// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.utils.WBLogger;

public class MonitoredCANCoder extends CANcoder {
  private int canDeviceID;
  private String canDeviceBus;

  public MonitoredCANCoder(int deviceId, String canbus) {
    super(deviceId, canbus);
    this.canDeviceID = deviceId;
    this.canDeviceBus = canbus;
  }

  public MonitoredCANCoder(int deviceId) {
    super(deviceId);
    // Our DriveTrain can is on the "mani" bus.
    System.out.println(
        "MonitoredCANCoder: Using default canDeviceBus of 'rio' on device " + deviceId);
    this.canDeviceID = deviceId;
    this.canDeviceBus = "rio";
  }

  @Override
  public StatusSignal<Double> getAbsolutePosition() {
    return WBLogger.getInstance()
        .monitorStatusSignal(
            super.getAbsolutePosition(),
            canDeviceID,
            canDeviceBus,
            "MonitoredCANCoder",
            "MonitoredCANCoder.getAbsolutePosition()");
  }
}
