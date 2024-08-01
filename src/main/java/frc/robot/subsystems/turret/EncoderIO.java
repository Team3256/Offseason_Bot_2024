// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {

  @AutoLog
  public static class EncoderIOInputs {
    public double encoderPositionDegrees = 0.0;
    public double encoderVelocity = 0.0;

    public Rotation2d turretMotorAbsolutePosition = new Rotation2d();
  }

  public default void updateInputs(EncoderIOInputs inputs) {}

  public default void zero() {}
}
