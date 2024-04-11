// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

public class XboxStalker {
  public static void stalk(CommandXboxController driver, CommandXboxController operator) {
    SmartDashboard.putBoolean("driverStatus", driver.getHID().getButtonCount() > 0);
    SmartDashboard.putBoolean("operatorStatus", operator.getHID().getButtonCount() > 0);
  }
}
