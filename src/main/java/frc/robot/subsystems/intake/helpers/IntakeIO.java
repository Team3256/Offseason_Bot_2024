// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.helpers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts = 8.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}
}
