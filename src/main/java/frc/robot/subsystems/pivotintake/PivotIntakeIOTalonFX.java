// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class PivotIntakeIOTalonFX implements PivotIntakeIO {

  private final MonitoredTalonFX pivotIntakeMotor =
      new MonitoredTalonFX(PivotIntakeConstants.kPivotMotorID);
  final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Double> pivotIntakeMotorVoltage = pivotIntakeMotor.getMotorVoltage();
  private final StatusSignal<Double> pivotIntakeMotorPosition = pivotIntakeMotor.getPosition();
  private final StatusSignal<Double> pivotIntakeMotorStatorCurrent =
      pivotIntakeMotor.getStatorCurrent();
  private final StatusSignal<Double> pivotIntakeMotorSupplyCurrent =
      pivotIntakeMotor.getSupplyCurrent();
  private final StatusSignal<Double> pivotIntakeMotorTemperature = pivotIntakeMotor.getDeviceTemp();

  public PivotIntakeIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> pivotIntakeMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = PivotIntakeConstants.kS;
    motorConfig.Slot0.kV = PivotIntakeConstants.kV;
    motorConfig.Slot0.kP = PivotIntakeConstants.kP;
    motorConfig.Slot0.kI = PivotIntakeConstants.kI;
    motorConfig.Slot0.kD = PivotIntakeConstants.kD;
    motorConfig.MotionMagic.MotionMagicAcceleration = PivotIntakeConstants.motionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        PivotIntakeConstants.motionMagicAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = PivotIntakeConstants.motionMagicJerk;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = PivotIntakeConstants.enableStatorLimit;
    motorConfig.CurrentLimits.StatorCurrentLimit = PivotIntakeConstants.statorLimit;
    TalonUtil.applyAndCheckConfiguration(pivotIntakeMotor, motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        PivotIntakeConstants.updateFrequency,
        pivotIntakeMotorVoltage,
        pivotIntakeMotorPosition,
        pivotIntakeMotorStatorCurrent,
        pivotIntakeMotorSupplyCurrent,
        pivotIntakeMotorTemperature);
    pivotIntakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotIntakeMotorVoltage,
        pivotIntakeMotorPosition,
        pivotIntakeMotorStatorCurrent,
        pivotIntakeMotorSupplyCurrent,
        pivotIntakeMotorTemperature);
    inputs.pivotIntakeMotorVoltage = pivotIntakeMotorVoltage.getValueAsDouble();
    inputs.pivotIntakeMotorPosition = pivotIntakeMotorPosition.getValueAsDouble();
    inputs.pivotIntakeMotorDegrees =
        (inputs.pivotIntakeMotorPosition / PivotIntakeConstants.kPivotMotorGearing) * 360;
    inputs.pivotIntakeMotorStatorCurrent = pivotIntakeMotorStatorCurrent.getValueAsDouble();
    inputs.pivotIntakeMotorSupplyCurrent = pivotIntakeMotorSupplyCurrent.getValueAsDouble();
    inputs.pivotIntakeMotorTemperature = pivotIntakeMotorTemperature.getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    if (PivotIntakeConstants.kUseMotionMagic) {
      pivotIntakeMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      pivotIntakeMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    pivotIntakeMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    pivotIntakeMotor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    pivotIntakeMotor.setPosition(0);
  }
}
