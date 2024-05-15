// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class PivotShooterIOTalonFX implements PivotShooterIO {


  private final MonitoredTalonFX pivotShooterMotor =
      new MonitoredTalonFX(PivotShooterConstants.kPivotMotorID);
  final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Double> pivotShooterMotorVoltage = pivotShooterMotor.getMotorVoltage();
  private final StatusSignal<Double> pivotShooterMotorVelocity = pivotShooterMotor.getVelocity();
  private final StatusSignal<Double> pivotShooterMotorPosition = pivotShooterMotor.getPosition();
  private final StatusSignal<Double> pivotShooterMotorStatorCurrent =
      pivotShooterMotor.getStatorCurrent();
  private final StatusSignal<Double> pivotShooterMotorSupplyCurrent =
      pivotShooterMotor.getSupplyCurrent();
  private final StatusSignal<Double> pivotShooterMotorTemperature =
      pivotShooterMotor.getDeviceTemp();
  private final StatusSignal<Double> pivotShooterMotorReferenceSlope =
      pivotShooterMotor.getClosedLoopReferenceSlope();

  public PivotShooterIOTalonFX() {
    var motorConfig = PivotShooterConstants.motorConfigs;
    PhoenixUtil.checkErrorAndRetry(() -> pivotShooterMotor.getConfigurator().refresh(motorConfig));
    TalonUtil.applyAndCheckConfiguration(pivotShooterMotor, motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        PivotShooterConstants.updateFrequency,
        pivotShooterMotorVoltage,
        pivotShooterMotorVelocity,
        pivotShooterMotorPosition,
        pivotShooterMotorStatorCurrent,
        pivotShooterMotorSupplyCurrent,
        pivotShooterMotorTemperature,
        pivotShooterMotorReferenceSlope);
    pivotShooterMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotShooterMotorVoltage,
        pivotShooterMotorVelocity,
        pivotShooterMotorPosition,
        pivotShooterMotorStatorCurrent,
        pivotShooterMotorSupplyCurrent,
        pivotShooterMotorTemperature,
        pivotShooterMotorReferenceSlope);
    inputs.pivotShooterMotorVoltage = pivotShooterMotorVoltage.getValueAsDouble();
    inputs.pivotShooterMotorVelocity = pivotShooterMotorVelocity.getValueAsDouble();
    inputs.pivotShooterMotorPosition = pivotShooterMotorPosition.getValueAsDouble();
    inputs.pivotShooterMotorDegrees =
        (inputs.pivotShooterMotorPosition / PivotShooterConstants.kPivotMotorGearing) * 360;
    inputs.pivotShooterMotorStatorCurrent = pivotShooterMotorStatorCurrent.getValueAsDouble();
    inputs.pivotShooterMotorSupplyCurrent = pivotShooterMotorSupplyCurrent.getValueAsDouble();
    inputs.pivotShooterMotorTemperature = pivotShooterMotorTemperature.getValueAsDouble();
    inputs.pivotShooterMotorReferenceSlope = pivotShooterMotorReferenceSlope.getValueAsDouble();

  }

  @Override
  public void setPosition(double position) {
    if (PivotShooterConstants.kUseMotionMagic) {
      pivotShooterMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      pivotShooterMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    pivotShooterMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    pivotShooterMotor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    pivotShooterMotor.setPosition(0);
  }
}
