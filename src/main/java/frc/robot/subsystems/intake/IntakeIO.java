// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.drivers.MonitoredTalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeMotorVoltage = 0.0;
    public double intakeMotorVelocity = 0.0;
    public double intakeMotorStatorCurrent = 0.0;
    public double intakeMotorSupplyCurrent = 0.0;
    public double intakeMotorTemperature = 0.0;
    public double intakeMotorReferenceSlope = 0.0;

    public double passthroughMotorVoltage = 0.0;
    public double passthroughMotorVelocity = 0.0;
    public double passthroughMotorStatorCurrent = 0.0;
    public double passthroughMotorSupplyCurrent = 0.0;
    public double passthroughMotorTemperature = 0.0;
    public double passthroughMotorReferenceSlope = 0.0;

    public boolean isBeamBroken = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeVoltage(double voltage) {}

  public default void setIntakeVelocity(double velocity) {}

  public default void setPassthroughVoltage(double voltage) {}

  public default void setPassthroughVelocity(double velocity) {}

  public default TalonFX getIntakeMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getIntakeVoltageRequest() {
    return new VoltageOut(0);
  }

  public default TalonFX getPassthroughMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getPassthroughVoltageRequest() {
    return new VoltageOut(0);
  }

  public default void off() {}

  public default boolean isBeamBroken() {
    return false;
  }
}
