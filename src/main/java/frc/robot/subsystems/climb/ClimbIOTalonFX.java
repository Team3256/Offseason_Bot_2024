// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class ClimbIOTalonFX implements ClimbIO {

  private final MonitoredTalonFX climbMotor =
      new MonitoredTalonFX(ClimbConstants.kLeftClimbMotorID);
  final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Double> climbMotorVoltage = climbMotor.getMotorVoltage();
  private final StatusSignal<Double> climbMotorVelocity = climbMotor.getVelocity();
  private final StatusSignal<Double> climbMotorPosition = climbMotor.getPosition();
  private final StatusSignal<Double> climbMotorStatorCurrent = climbMotor.getStatorCurrent();
  private final StatusSignal<Double> climbMotorSupplyCurrent = climbMotor.getSupplyCurrent();
  private final StatusSignal<Double> climbMotorTemperature = climbMotor.getDeviceTemp();
  private final StatusSignal<Double> climbMotorReferenceSlope =
      climbMotor.getClosedLoopReferenceSlope();

  public ClimbIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> climbMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = ClimbConstants.kS;
    motorConfig.Slot0.kV = ClimbConstants.kV;
    motorConfig.Slot0.kP = ClimbConstants.kP;
    motorConfig.Slot0.kI = ClimbConstants.kI;
    motorConfig.Slot0.kD = ClimbConstants.kD;
    motorConfig.MotorOutput.NeutralMode = ClimbConstants.neutralMode;
    motorConfig.MotorOutput.Inverted = ClimbConstants.climbInverted;
    motorConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.motionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.motionMagicAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = ClimbConstants.motionMagicJerk;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = ClimbConstants.enableStatorLimit;
    motorConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.statorLimit;
    TalonUtil.applyAndCheckConfiguration(climbMotor, motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClimbConstants.updateFrequency,
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorStatorCurrent,
        climbMotorSupplyCurrent,
        climbMotorTemperature,
        climbMotorReferenceSlope);
    climbMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorStatorCurrent,
        climbMotorSupplyCurrent,
        climbMotorTemperature,
        climbMotorReferenceSlope);
    inputs.climbMotorVoltage = climbMotorVoltage.getValueAsDouble();
    inputs.climbMotorVelocity = climbMotorVelocity.getValueAsDouble();
    inputs.climbMotorPosition = climbMotorPosition.getValueAsDouble();
    inputs.climbMotorStatorCurrent = climbMotorStatorCurrent.getValueAsDouble();
    inputs.climbMotorSupplyCurrent = climbMotorSupplyCurrent.getValueAsDouble();
    inputs.climbMotorTemperature = climbMotorTemperature.getValueAsDouble();
    inputs.climbMotorReferenceSlope = climbMotorReferenceSlope.getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    if (ClimbConstants.kUseMotionMagic) {
      climbMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      climbMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    climbMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    climbMotor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    climbMotor.setPosition(0);
  }
}
