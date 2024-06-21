// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

public class EncoderIOCancoder implements EncoderIO {

  private final CANcoder encoder;

  private final StatusSignal<Double> encoderPosition;
  private final StatusSignal<Double> encoderVelocity;

  public EncoderIOCancoder() {
    encoder = new CANcoder(TurretConstants.kCanCoderID, "mani");
    var response = encoder.getConfigurator().apply(TurretConstants.canCoderConfig);

    if (!response.isOK()) {
      System.out.println(
          "CANcoder ID "
              + TurretConstants.kCanCoderID
              + " failed config with error "
              + response.getDescription());
    }

    encoderPosition = encoder.getPosition();
    encoderVelocity = encoder.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        TurretConstants.updateFrequency, encoderPosition, encoderVelocity);
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(EncoderIOInputs inputs) {
    inputs.encoderPosition = encoderPosition.getValueAsDouble();
    inputs.encoderVelocity = encoderVelocity.getValueAsDouble();
  }

  @Override
  public void zero() {
    encoder.setPosition(0.0);
  }
}
