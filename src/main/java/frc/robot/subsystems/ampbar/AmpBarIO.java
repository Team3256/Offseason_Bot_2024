// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.drivers.MonitoredTalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface AmpBarIO {
  @AutoLog
  public static class AmpBarIOInputs {
    public double ampBarMotorVoltage = 0.0;
    public double ampBarMotorPosition = 0.0;
    public double ampBarMotorVelocity = 0.0;
    public double ampBarMotorStatorCurrent = 0.0;
    public double ampBarMotorSupplyCurrent = 0.0;
    public double ampBarMotorTemperature = 0.0;
  }

  public default void updateInputs(AmpBarIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default boolean isCurrentSpiking() {
    return false;
  }

  public default void off() {}

  public default void setPosition(double position) {}

  public default MonitoredTalonFX getMotor() {
    return new MonitoredTalonFX(0);
  }

  public default VoltageOut getVoltageRequest() {
    return new VoltageOut(0);
  }
}
