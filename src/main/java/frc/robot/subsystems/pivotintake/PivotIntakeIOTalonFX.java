// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class PivotIntakeIOTalonFX implements PivotIntakeIO {
  private final TalonFX pivotIntakeMotor = new TalonFX(PivotIntakeConstants.kPivotMotorID);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Double> pivotIntakeMotorVoltage = pivotIntakeMotor.getMotorVoltage();
  private final StatusSignal<Double> pivotIntakeMotorVelocity = pivotIntakeMotor.getVelocity();
  private final StatusSignal<Double> pivotIntakeMotorPosition = pivotIntakeMotor.getPosition();
  private final StatusSignal<Double> pivotIntakeMotorStatorCurrent =
      pivotIntakeMotor.getStatorCurrent();
  private final StatusSignal<Double> pivotIntakeMotorSupplyCurrent =
      pivotIntakeMotor.getSupplyCurrent();
  private final StatusSignal<Double> pivotIntakeMotorTemperature = pivotIntakeMotor.getDeviceTemp();
  private final StatusSignal<Double> pivotIntakeMotorReferenceSlope =
      pivotIntakeMotor.getClosedLoopReferenceSlope();

  public PivotIntakeIOTalonFX() {
    var motorConfig = PivotIntakeConstants.motorConfig;
    PhoenixUtil.checkErrorAndRetry(() -> pivotIntakeMotor.getConfigurator().refresh(motorConfig));
    TalonUtil.applyAndCheckConfiguration(pivotIntakeMotor, motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        PivotIntakeConstants.updateFrequency,
        pivotIntakeMotorVoltage,
        pivotIntakeMotorVelocity,
        pivotIntakeMotorPosition,
        pivotIntakeMotorStatorCurrent,
        pivotIntakeMotorSupplyCurrent,
        pivotIntakeMotorTemperature,
        pivotIntakeMotorReferenceSlope);
    pivotIntakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotIntakeMotorVoltage,
        pivotIntakeMotorVelocity,
        pivotIntakeMotorPosition,
        pivotIntakeMotorStatorCurrent,
        pivotIntakeMotorSupplyCurrent,
        pivotIntakeMotorTemperature,
        pivotIntakeMotorReferenceSlope);
    inputs.pivotIntakeMotorVoltage = pivotIntakeMotorVoltage.getValueAsDouble();
    inputs.pivotIntakeMotorVelocity = pivotIntakeMotorVelocity.getValueAsDouble();
    inputs.pivotIntakeMotorPosition = pivotIntakeMotorPosition.getValueAsDouble();
    inputs.pivotIntakeMotorDegrees =
        (inputs.pivotIntakeMotorPosition / PivotIntakeConstants.kPivotMotorGearing) * 360;
    inputs.pivotIntakeMotorStatorCurrent = pivotIntakeMotorStatorCurrent.getValueAsDouble();
    inputs.pivotIntakeMotorSupplyCurrent = pivotIntakeMotorSupplyCurrent.getValueAsDouble();
    inputs.pivotIntakeMotorTemperature = pivotIntakeMotorTemperature.getValueAsDouble();
    inputs.pivotIntakeMotorReferenceSlope = pivotIntakeMotorReferenceSlope.getValueAsDouble();
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

  @Override
  public TalonFX getMotor() {
    return pivotIntakeMotor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
