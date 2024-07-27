// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class AmpBarIOTalonFX implements AmpBarIO {

  private final MonitoredTalonFX ampBarMotor = new MonitoredTalonFX(AmpBarConstants.kAmpBarMotorID);

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Double> ampBarMotorVoltage = ampBarMotor.getMotorVoltage();
  private final StatusSignal<Double> ampBarMotorVelocity = ampBarMotor.getVelocity();
  private final StatusSignal<Double> ampBarMotorPosition = ampBarMotor.getPosition();
  private final StatusSignal<Double> ampBarMotorStatorCurrent = ampBarMotor.getStatorCurrent();
  private final StatusSignal<Double> ampBarMotorSupplyCurrent = ampBarMotor.getSupplyCurrent();
  private final StatusSignal<Double> ampBarMotorTemperature = ampBarMotor.getDeviceTemp();

  private final VoltageOut voltageReq = new VoltageOut(0);

  public AmpBarIOTalonFX() {
    var motorConfig = AmpBarConstants.motorConfig;
    PhoenixUtil.checkErrorAndRetry(() -> ampBarMotor.getConfigurator().refresh(motorConfig));
    TalonUtil.applyAndCheckConfiguration(ampBarMotor, motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        AmpBarConstants.updateFrequency,
        ampBarMotorVoltage,
        ampBarMotorVelocity,
        ampBarMotorPosition,
        ampBarMotorStatorCurrent,
        ampBarMotorSupplyCurrent,
        ampBarMotorTemperature);
    ampBarMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(AmpBarIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        ampBarMotorVoltage,
        ampBarMotorVelocity,
        ampBarMotorPosition,
        ampBarMotorStatorCurrent,
        ampBarMotorSupplyCurrent,
        ampBarMotorTemperature);
    inputs.ampBarMotorVoltage = ampBarMotorVoltage.getValueAsDouble();
    inputs.ampBarMotorVelocity = ampBarMotorVelocity.getValueAsDouble();
    inputs.ampBarMotorPosition = ampBarMotorPosition.getValueAsDouble();
    inputs.ampBarMotorStatorCurrent = ampBarMotorStatorCurrent.getValueAsDouble();
    inputs.ampBarMotorSupplyCurrent = ampBarMotorSupplyCurrent.getValueAsDouble();
    inputs.ampBarMotorTemperature = ampBarMotorTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    ampBarMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    ampBarMotor.setControl(new NeutralOut());
  }

  @Override
  public boolean isCurrentSpiking() {
    return ampBarMotor.getStatorCurrent().getValueAsDouble() > AmpBarConstants.kAmpBarCurrentThreshold;
  }

  @Override
  public MonitoredTalonFX getMotor() {
    return ampBarMotor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }

  @Override
  public void setPosition(double position) {
    if (AmpBarConstants.kUseMotionMagic) {
      ampBarMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      ampBarMotor.setControl(positionRequest.withPosition(position));
    }
  }
}
