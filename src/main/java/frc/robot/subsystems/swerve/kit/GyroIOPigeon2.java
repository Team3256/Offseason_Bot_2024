// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.kit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> yawVelocity;

  public GyroIOPigeon2(int id, String canbusName) {
    this.pigeon = new Pigeon2(id, canbusName);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw = pigeon.getYaw().clone();
    yawVelocity = pigeon.getAngularVelocityZWorld().clone();
    yaw.setUpdateFrequency(20); // TODO: check if this is the right frequency
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.position = pigeon.getRotation3d();
    inputs.rate = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }

  @Override
  public StatusSignal<Double> getYaw() {
    return yaw;
  }

  @Override
  public StatusSignal<Double> getAngularVelocity() {
    return yawVelocity;
  }

  @Override
  public Pigeon2 getPigeon2() {
    return pigeon;
  }
}
