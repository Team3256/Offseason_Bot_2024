// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.kit;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double rate = 0.0;
    public Rotation3d position = new Rotation3d();
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public StatusSignal<Double> getYaw();

  public StatusSignal<Double> getAngularVelocity();

  public Pigeon2 getPigeon2();
}
