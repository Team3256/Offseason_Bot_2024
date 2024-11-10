// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double leftClimbMotorVoltage = 0.0;
    public double leftClimbMotorPosition = 0.0;
    public double leftClimbMotorVelocity = 0.0;
    public double leftClimbMotorStatorCurrent = 0.0;
    public double leftClimbMotorSupplyCurrent = 0.0;
    public double leftClimbMotorTemperature = 0.0;
    public double leftClimbMotorReferenceSlope = 0.0;
    public double rightClimbMotorVoltage = 0.0;
    public double rightClimbMotorPosition = 0.0;
    public double rightClimbMotorVelocity = 0.0;
    public double rightClimbMotorStatorCurrent = 0.0;
    public double rightClimbMotorSupplyCurrent = 0.0;
    public double rightClimbMotorTemperature = 0.0;
    public double rightClimbMotorReferenceSlope = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {
  }

  public default void setPosition(double left, double right) {
  }

  public default void setVoltageLeft(double voltage) {
  }

  public default void setVoltageRight(double voltage) {
  }

  public default void off() {
  }

  public default void zero() {
  }

  public default void goToZeroLeft() {
  }

  public default void goToZeroRight() {
  }

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getVoltageRequest() {
    return new VoltageOut(0);
  }
}
