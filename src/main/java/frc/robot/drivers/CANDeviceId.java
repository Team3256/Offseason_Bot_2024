// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import java.util.Objects;

public class CANDeviceId {
  private final int mDeviceNumber;
  private final String mBus;

  public CANDeviceId(int deviceNumber, String bus) {
    mDeviceNumber = deviceNumber;
    mBus = bus;
  }

  // Use the default bus name (empty string).
  public CANDeviceId(int deviceNumber) {
    this(deviceNumber, "");
  }

  public int getDeviceNumber() {
    return mDeviceNumber;
  }

  public String getBus() {
    return mBus;
  }

  public boolean equals(CANDeviceId other) {
    return other.mDeviceNumber == mDeviceNumber && Objects.equals(other.mBus, mBus);
  }
}
