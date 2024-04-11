// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.helpers;

import static frc.robot.subsystems.intake.IntakeConstants.kIntakeMotorID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intakeMotor = new TalonFX(kIntakeMotorID);

  private final StatusSignal<Double> intakeMotorPosition = intakeMotor.getPosition();
  private final StatusSignal<Double> intakeMotorVelocity = intakeMotor.getVelocity();
  private final StatusSignal<Double> intakeMotorAppliedVolts = intakeMotor.getMotorVoltage();

  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakeMotorPosition, intakeMotorVelocity, intakeMotorAppliedVolts);
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(intakeMotorPosition, intakeMotorVelocity, intakeMotorAppliedVolts);

    inputs.appliedVolts = intakeMotorAppliedVolts.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    intakeMotor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }
}
