// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;
// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// package frc.robot.drivers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;

public class CANDeviceTester {

  /**
   * Helper method to test PDP
   *
   * @return Returns whether the PDP is online
   */
  public static boolean testPDP(PowerDistribution device) {
    double voltage = device.getVoltage();
    if (voltage == 0) System.out.println("PDP offline");
    return voltage != 0;
  }

  /**
   * @param device talon fx id to test
   * @return Returns whether all the TalonFXs are online
   */
  public static boolean testTalonFX(TalonFX device) {
    StatusSignal<Double> temp = device.getDeviceTemp();
    if (temp.getValue() == 0) System.out.println("TalonFX " + device.getDeviceID() + " offline");
    return temp.getValue() != 0;
  }

  /**
   * @param device pigeon to test
   * @return Returns whether the Pigeon is online
   */
  public static boolean testPigeon(Pigeon2 device) {
    StatusSignal<Double> temp = device.getTemperature();
    if (temp.getValue() == 0) System.out.println("Pigeon " + device.getDeviceID() + " offline");
    return temp.getValue() != 0;
  }

  /**
   * @param device spark max to test
   * @return Returns whether the SparkMax is online
   */

  /**
   * @param device CANCoder to test
   * @return Returns whether the CanCoder is online
   */
  public static boolean testCANCoder(CANcoder device) {
    StatusSignal<Double> voltage = device.getSupplyVoltage();
    if (voltage.getValue() == 0)
      System.out.println("CANCoder " + device.getDeviceID() + " offline");
    return voltage.getValue() != 0;
  }
}
