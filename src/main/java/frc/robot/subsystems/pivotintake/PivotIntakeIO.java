// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIntakeIO {
  @AutoLog
  public static class PivotIntakeIOInputs {
    public double pivotIntakeMotorVoltage = 0.0;
    public double pivotIntakeMotorPosition = 0.0;
    public double pivotIntakeMotorVelocity = 0.0;
    public double pivotIntakeMotorDegrees = 0.0;
    public double pivotIntakeMotorStatorCurrent = 0.0;
    public double pivotIntakeMotorSupplyCurrent = 0.0;
    public double pivotIntakeMotorTemperature = 0.0;
    public double pivotIntakeMotorReferenceSlope = 0.0;
  }

  public default void updateInputs(PivotIntakeIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double voltage) {}

  public default void off() {}

  public default void zero() {}
}
