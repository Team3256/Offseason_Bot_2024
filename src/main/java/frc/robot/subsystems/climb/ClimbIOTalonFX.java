// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX leftClimbMotor = new TalonFX(ClimbConstants.kLeftClimbMotorID);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Double> climbMotorVoltage = leftClimbMotor.getMotorVoltage();
  private final StatusSignal<Double> climbMotorVelocity = leftClimbMotor.getVelocity();
  private final StatusSignal<Double> climbMotorPosition = leftClimbMotor.getPosition();
  private final StatusSignal<Double> climbMotorStatorCurrent = leftClimbMotor.getStatorCurrent();
  private final StatusSignal<Double> climbMotorSupplyCurrent = leftClimbMotor.getSupplyCurrent();
  private final StatusSignal<Double> climbMotorTemperature = leftClimbMotor.getDeviceTemp();
  private final StatusSignal<Double> climbMotorReferenceSlope =
      leftClimbMotor.getClosedLoopReferenceSlope();



  private final TalonFX rightClimbMotor = new TalonFX(ClimbConstants.kRightClimbMotorID);

  private final Follower rightClimbFollowReq = new Follower(leftClimbMotor.getDeviceID(), false);

  public ClimbIOTalonFX() {
    var leftClimbConfig = ClimbConstants.leftClimbConfig;
    PhoenixUtil.checkErrorAndRetry(() -> leftClimbMotor.getConfigurator().refresh(leftClimbConfig));
    TalonUtil.applyAndCheckConfiguration(leftClimbMotor, leftClimbConfig);

    var rightClimbConfig = ClimbConstants.righClimbConfig;
    PhoenixUtil.checkErrorAndRetry(() -> rightClimbMotor.getConfigurator().refresh(rightClimbConfig));
    TalonUtil.applyAndCheckConfiguration(rightClimbMotor, rightClimbConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClimbConstants.updateFrequency,
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorStatorCurrent,
        climbMotorSupplyCurrent,
        climbMotorTemperature,
        climbMotorReferenceSlope);
    leftClimbMotor.optimizeBusUtilization();
    rightClimbMotor.setControl(rightClimbFollowReq);
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
      leftClimbMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      leftClimbMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    leftClimbMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    leftClimbMotor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    leftClimbMotor.setPosition(0);
  }

  @Override
  public TalonFX getMotor() {
    return leftClimbMotor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
