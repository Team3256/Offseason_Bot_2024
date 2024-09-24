// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.drivers.MonitoredTalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double climbMotorVoltage = 0.0;
    public double climbMotorPosition = 0.0;
    public double climbMotorVelocity = 0.0;
    public double climbMotorStatorCurrent = 0.0;
    public double climbMotorSupplyCurrent = 0.0;
    public double climbMotorTemperature = 0.0;
    public double climbMotorReferenceSlope = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double voltage) {}

  public default void off() {}

  public default void zero() {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getVoltageRequest() {
    return new VoltageOut(0);
  }
}
