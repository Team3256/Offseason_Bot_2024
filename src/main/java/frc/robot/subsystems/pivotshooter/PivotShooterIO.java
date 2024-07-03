// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.drivers.MonitoredTalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface PivotShooterIO {
  @AutoLog
  public static class PivotShooterIOInputs {
    public double pivotShooterMotorVoltage = 0.0;
    public double pivotShooterMotorVelocity = 0.0;
    public double pivotShooterMotorPosition = 0.0;
    public double pivotShooterMotorDegrees = 0.0;
    public double pivotShooterMotorStatorCurrent = 0.0;
    public double pivotShooterMotorSupplyCurrent = 0.0;
    public double pivotShooterMotorTemperature = 0.0;
    public double pivotShooterMotorReferenceSlope = 0.0;
  }

  public default void updateInputs(PivotShooterIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double voltage) {}

  public default MonitoredTalonFX getMotor() {
    return new MonitoredTalonFX(0);
  }

  public default VoltageOut getVoltageRequest() {
    return new VoltageOut(0);
  }

  public default void off() {}

  public default void zero() {}
}
