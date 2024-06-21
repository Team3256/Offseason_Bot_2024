// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double turretMotorVoltage = 0.0;
    public double turretMotorPosition = 0.0;
    public double turretMotorStatorCurrent = 0.0;
    public double turretMotorSupplyCurrent = 0.0;
    public double turretMotorTemperature = 0.0;
    public double turretMotorReferenceSlope = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setPosition(double position) {}

  public default void zero() {}

  public default void off() {}
}
